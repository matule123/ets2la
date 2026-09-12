"""Independent ABI sentinels and fail-closed vehicle-profile contracts.

All dimensions here are SYNTHETIC fixtures, not profiles for real game trucks.
"""
from dataclasses import asdict, replace
import copy
import json
import math
import pickle
import struct
import time
import unittest
from unittest.mock import patch

from core.sdk.scs_sdk import SCSTelemetry
from core.sdk.vehicle_observation import capture_vehicle_observation, decode_vehicle_observation
from core.vehicle_profile import (
    VehicleProfileProvider, configuration_fingerprint, compile_catalog_entry,
    bind_envelope_vehicle, fixed_axle_geometry,
)
from core.swept_envelope import EnvelopeError
from tests.test_stage5a_swept_envelope import identity


def memory(trailers=0):
    """Offsets written independently of the production offset constants."""
    data = bytearray(32768)
    def put(fmt, address, *values):
        struct.pack_into('<'+fmt, data, address, *values)
    put('B', 0, 1)
    put('Q', 8, 1_000_000)
    put('Q', 16, 900_000)
    put('Q', 24, 800_000)
    put('6I', 40, 12, 1, 59, 1, 1, 18)
    put('I', 80, 4)
    data[2428:2432] = b'cab\0'
    data[2300:2306] = b'brand\0'
    data[2492:2497] = b'name\0'
    put('3f', 1664, 0, 1, 2.1)
    put('6d', 2200, 123, 17, -456, .25, .01, -.02)
    for i, (x,z) in enumerate(((-1,-1.7),(1,-1.7),(-1,2.1),(1,2.1))):
        put('f', 1676+4*i, x); put('f', 1740+4*i, .6); put('f', 1804+4*i, z)
        put('f', 752+4*i, .5)
        put('B', 1500+i, int(i<2)); put('B', 1516+i, 1)
        put('B', 1532+i, int(i>=2)); put('B', 1548+i, 0)
        put('B', 1590+i, 1)
        put('f', 1200+4*i, -.025 if i<2 else 0.)
    for slot in range(trailers):
        base = 6000+1560*slot
        put('B', base+80, 1); put('I', base+148, 2)
        data[base+920:base+924] = b'sem\0'
        data[base+984:base+990] = b'cargo\0'
        data[base+1048:base+1052] = b'box\0'
        data[base+1304:base+1311] = b'double\0'
        put('3f', base+664, 0, 1, -6)
        put('6d', base+872, 120-slot*8, 18, -449, .20+slot*.01, 0, 0)
        for i,x in enumerate((-1.,1.)):
            put('B', base+16+i, 1); put('B', base+64+i, 1)
            put('f', base+552+4*i, .5)
            put('f', base+676+4*i, x); put('f', base+740+4*i, .6)
            put('f', base+804+4*i, 2)
    return data


def observation(trailers=0, now=10.):
    return capture_vehicle_observation(memory(trailers), now)


def entry(o):
    bodies = []
    for a in o.articles:
        if a.attached:
            bodies.append({'slot': a.slot, 'width_m': 2.5,
                'front_m': 5. if a.slot == -1 else 9.,
                'rear_m': 1.2 if a.slot == -1 else 3.,
                'hitch_front_m': 0. if a.slot == -1 else 8.,
                'hitch_rear_m': 0. if a.slot == -1 else -1.,
                'axle_model': 'fixed_axle'})
    return {'schema_version': 1, 'source': 'SYNTHETIC FIXTURE ONLY',
        'evidence_sha256': 'a'*64, 'confirmed': True,
        'configuration_fingerprint': configuration_fingerprint(o), 'bodies': bodies,
        'limits': {'max_tyre_rad': .7, 'max_speed_mps': 2.,
                   'safety_margin_m': .15, 'uncertainty_m': .02}}


def provider(o):
    return VehicleProfileProvider({'schema_version': 1, 'profiles': [entry(o)]})


class VehicleObservationTests(unittest.TestCase):
    def test_revision12_offsets_units_and_all_ten_trailer_slots(self):
        o = observation(10)
        self.assertEqual(o.failure_reason, '')
        self.assertEqual(len(o.articles), 11)
        self.assertEqual((o.sdk_frame_us, o.simulated_us, o.render_us), (1000000,900000,800000))
        self.assertEqual(o.game_version, (1,59,1,1,18))
        cab = o.articles[0]
        self.assertEqual(cab.vehicle_id, 'cab')
        self.assertEqual(cab.position_m, (123,17,-456))
        self.assertAlmostEqual(cab.rotation_rad[0], math.pi/2)
        self.assertAlmostEqual(cab.rotation_rad[1], .01*math.tau)
        self.assertAlmostEqual(cab.rotation_rad[2], -.02*math.tau)
        self.assertAlmostEqual(cab.wheels[0].steering_right_rad, .025*math.tau)
        self.assertTrue(cab.wheels[0].on_ground)
        self.assertTrue(cab.wheels[2].powered)
        self.assertAlmostEqual(cab.wheels[0].position_m[1], .6)
        self.assertEqual(o.articles[-1].slot, 9)
        self.assertEqual(o.articles[-1].position_m, (48,18,-449))
        self.assertEqual(o.articles[-1].cargo_accessory_id, 'cargo')
        self.assertEqual(o.articles[-1].hook_local_m, (0,1,-6))
        self.assertTrue(o.stable_read)
        self.assertFalse(o.atomic)  # equal copies are NOT transaction proof

    def test_legacy_trailer_reader_rejects_negative_boolean_and_out_of_range_index(self):
        r = SCSTelemetry(); r.mm = memory(1)
        for index in (-1, 10, True, 1.5):
            with self.subTest(index=index):
                self.assertEqual(r.read_trailer(index), {})
        self.assertTrue(r.read_trailer(0)['attached'])

    def test_old_steering_values_are_not_changed_by_profile_read(self):
        r = SCSTelemetry(); r.mm = memory()
        struct.pack_into('<f', r.mm, 972, -.4)
        with patch('core.sdk.scs_sdk.logging'):
            raw = r.update()
        self.assertAlmostEqual(raw['truckFloat']['gameSteer'], -.4)
        self.assertEqual(raw['truckPlacement']['coordinateX'], 123)
        self.assertEqual(raw['vehicleObservation'].failure_reason, '')
        struct.pack_into('<I', r.mm, 40, 999)
        raw = r.update()
        self.assertAlmostEqual(raw['truckFloat']['gameSteer'], -.4)
        self.assertEqual(raw['vehicleObservation'].failure_reason, 'UNSUPPORTED_SDK_ABI')

    def test_bad_abi_counts_nonfinite_bool_strings_and_truncation(self):
        cases = [(40,'I',13,'UNSUPPORTED_SDK_ABI'),
                 (80,'I',15,'UNSUPPORTED_SDK_WHEEL_COUNT'),
                 (80,'I',0,'UNSUPPORTED_SDK_WHEEL_COUNT'),
                 (1676,'f',float('nan'),'NONFINITE_SDK_FIELD'),
                 (1500,'B',2,'INVALID_SDK_BOOLEAN'),
                 (752,'f',0,'INVALID_SDK_WHEEL_GEOMETRY'),
                 (2428,'B',255,'INVALID_SDK_STRING')]
        for address,fmt,value,reason in cases:
            with self.subTest(reason=reason):
                data = memory(); struct.pack_into('<'+fmt,data,address,value)
                self.assertEqual(capture_vehicle_observation(data,10).failure_reason, reason)
        self.assertEqual(capture_vehicle_observation(memory()[:200],10).failure_reason,
                         'TRUNCATED_SDK_VEHICLE_BLOCK')

    def test_detached_stale_slots_do_not_enter_chain(self):
        data = memory(2); data[7640] = 0  # trailer1 +80
        struct.pack_into('<f',data,7560+676,float('nan'))
        o = capture_vehicle_observation(data,10)
        self.assertEqual(o.failure_reason, '')
        self.assertFalse(o.articles[2].attached)
        self.assertEqual(o.articles[2].wheels, ())

    def test_bounded_torn_read_and_recovery(self):
        class Changing:
            def __init__(self, settle):
                self.calls = 0; self.data = memory(); self.settle = settle
            def __getitem__(self, key):
                self.calls += 1
                if not self.settle or self.calls < 3:
                    struct.pack_into('<Q', self.data, 8, self.calls)
                return self.data[key]
        bad = Changing(False)
        self.assertEqual(capture_vehicle_observation(bad,10).failure_reason,'SDK_VEHICLE_READ_CHANGED')
        self.assertEqual(bad.calls,3)
        good = Changing(True)
        self.assertEqual(capture_vehicle_observation(good,10).failure_reason,'')
        self.assertEqual(good.calls,3)


class VehicleProfileTests(unittest.TestCase):
    def test_missing_dimensions_are_unknown_not_track_width(self):
        o = observation(1); p = VehicleProfileProvider().update(o,10)
        self.assertEqual(p.observation_failure, '')
        self.assertEqual(p.model_failure,'MISSING_CONFIRMED_BODY_PROFILE')
        self.assertIsNone(p.model)
        self.assertFalse(p.live_maneuver_ready)
        self.assertEqual(fixed_axle_geometry(o.articles[1]).track_m, 2)
        self.assertAlmostEqual(p.axle_geometry[0].wheelbase_m,3.8,places=6)
        self.assertEqual(p.axle_geometry[1].hook_forward_m,8)
        self.assertEqual(p.geometry_failures, ('',''))

    def test_confirmed_adapter_zero_through_four_articles(self):
        for n in range(5):
            with self.subTest(trailers=n):
                o=observation(n); p=provider(o).update(o,10)
                self.assertEqual(p.model_failure,'')
                self.assertEqual(len(p.model.bodies),n+1)
                self.assertAlmostEqual(p.model.wheelbase_m,3.8,places=6)
                self.assertEqual(p.model.bodies[0].width_m,2.5)  # track is only 2
                self.assertFalse(p.live_maneuver_ready)
                self.assertEqual(pickle.loads(pickle.dumps(p)),p)
                json.dumps(asdict(p), allow_nan=False)

    def test_more_than_four_trailers_observed_but_not_promoted(self):
        o=observation(5); p=provider(o).update(o,10)
        self.assertEqual(p.observation_failure,'')
        self.assertEqual(p.model_failure,'UNSUPPORTED_ARTICLE_COUNT')

    def test_exact_static_configuration_hash_not_dynamic_pose_or_language(self):
        o=observation(1); a=o.articles[0]
        changed=replace(o,articles=(replace(a,position_m=(1,2,3),name='localized'),)+o.articles[1:])
        self.assertEqual(configuration_fingerprint(o), configuration_fingerprint(changed))
        changed=replace(o,articles=(replace(a,vehicle_id='other'),)+o.articles[1:])
        self.assertNotEqual(configuration_fingerprint(o), configuration_fingerprint(changed))

    def test_no_averaged_tandem_or_steered_trailer(self):
        for kind in ('tandem','steered','lifted','airborne','non_simulated','side_hitch'):
            with self.subTest(kind=kind):
                o=observation(1); a=o.articles[1]; wheels=list(a.wheels)
                if kind=='tandem':
                    wheels += [replace(w,index=w.index+2,position_m=(w.position_m[0],.6,3)) for w in wheels]
                if kind=='steered': wheels[0]=replace(wheels[0],steerable=True)
                if kind=='lifted': wheels[0]=replace(wheels[0],lift=.5)
                if kind=='airborne': wheels[0]=replace(wheels[0],on_ground=False)
                if kind=='non_simulated': wheels[0]=replace(wheels[0],simulated=False)
                a=replace(a,wheels=tuple(wheels),hook_local_m=(.2,1,-6) if kind=='side_hitch' else a.hook_local_m)
                o=replace(o,articles=(o.articles[0],a)+o.articles[2:])
                p=provider(o).update(o,10)
                self.assertIsNone(p.model)
                self.assertTrue(p.model_failure)

    def test_bad_profile_values_are_rejected_without_defaults(self):
        o=observation(1)
        for mutate in (
            lambda e:e.update(confirmed=False), lambda e:e.update(source=''),
            lambda e:e.update(evidence_sha256=''), lambda e:e['bodies'][0].update(width_m=float('nan')),
            lambda e:e['bodies'][0].update(width_m=True), lambda e:e['bodies'][1].update(hitch_front_m=7),
            lambda e:e['bodies'][0].update(axle_model='tandem_average'),
            lambda e:e['limits'].update(uncertainty_m=0), lambda e:e['limits'].update(max_tyre_rad=1.5),
            lambda e:e['limits'].update(max_speed_mps=10), lambda e:e['bodies'].pop(),
            lambda e:e.update(schema_version=True),
            lambda e:e['bodies'][0].update(width_m=1.9),
            lambda e:e['limits'].update(uncertainty_m=.0000001),
        ):
            e=entry(o); mutate(e)
            with self.assertRaises((EnvelopeError, ValueError)):
                compile_catalog_entry(e,o)

    def test_ambiguous_catalog_and_external_mutation(self):
        o=observation(); e=entry(o); catalog={'schema_version':1,'profiles':[e]}
        p=VehicleProfileProvider(catalog)
        e['bodies'][0]['width_m']=4
        self.assertEqual(p.update(o,10).model.bodies[0].width_m,2.5)
        bad=VehicleProfileProvider({'schema_version':1,'profiles':[e,copy.deepcopy(e)]})
        self.assertEqual(bad.update(o,10).model_failure,'AMBIGUOUS_VEHICLE_PROFILE')

    def test_stale_same_sdk_frame_not_refreshed_by_poll(self):
        o=observation(); owner=provider(o); first=owner.update(o,10)
        latest=owner.update(replace(o,captured_at=10.6),10.6)
        self.assertEqual(latest.observation_failure,'STALE_VEHICLE_OBSERVATION')
        self.assertEqual(latest.observed_at,10)
        self.assertNotEqual(latest.token,first.token)
        self.assertIsNone(latest.model)

    def test_session_configuration_pause_and_clock_invalidate(self):
        o=observation(1)
        for changed in (None, replace(o,paused=True), replace(o,active=False),
                        replace(o,sdk_frame_us=1), observation(0)):
            with self.subTest(change=changed and changed.sdk_frame_us):
                owner=provider(o); before=owner.update(o,10)
                after=owner.update(changed,10.1)
                self.assertNotEqual(before.token,after.token)
                restored=owner.update(replace(o,sdk_frame_us=2000000,captured_at=10.2),10.2)
                self.assertNotEqual(before.token,restored.token)
        owner=provider(o); before=owner.update(o,10)
        self.assertEqual(owner.update(o,9).observation_failure,'INVALID_VEHICLE_CAPTURE')
        self.assertNotEqual(before.token,provider(o).update(o,10).token)

    def test_trailer_gap_refused_not_renumbered(self):
        o=observation(2)
        o=replace(o,articles=(o.articles[0],replace(o.articles[1],attached=False))+o.articles[2:])
        self.assertEqual(VehicleProfileProvider().update(o,10).observation_failure,
                         'NONCONTIGUOUS_TRAILER_CHAIN')

    def test_lift_change_drops_cached_model_and_increments_token(self):
        o=observation(1); owner=provider(o); first=owner.update(o,10)
        a=o.articles[1]; wheels=(replace(a.wheels[0],lift=.5),a.wheels[1])
        changed=replace(o,sdk_frame_us=2000000,captured_at=10.1,
                        articles=(o.articles[0],replace(a,wheels=wheels))+o.articles[2:])
        new=owner.update(changed,10.1)
        self.assertNotEqual(first.token,new.token)
        self.assertEqual(new.model_failure,'UNSUPPORTED_LIFTED_OR_AIRBORNE_AXLE')
        self.assertIsNone(new.model)

    def test_route_binding_requires_fresh_profile_and_explicit_confirmation(self):
        o=observation(); owner=provider(o); p=owner.update(o,10); i=identity()
        with self.assertRaisesRegex(EnvelopeError,'UNPROVEN_FULL_ACCESSORY_CONFIGURATION'):
            bind_envelope_vehicle(p,p,i,i,10)
        b=bind_envelope_vehicle(p,p,i,i,10,p.token,'synthetic test configuration proof')
        self.assertEqual(b.vehicle,p.model)
        for field,value in (('revision',9),('intent','new'),('build','new'),('session','new'),
                            ('map_key','new'),('dataset','new'),('layer','bridge')):
            with self.subTest(field=field), self.assertRaisesRegex(EnvelopeError,'STALE_ENVELOPE_IDENTITY'):
                bind_envelope_vehicle(p,p,i,replace(i,**{field:value}),10,p.token,'proof')
        with self.assertRaisesRegex(EnvelopeError,'STALE_VEHICLE_OBSERVATION'):
            bind_envelope_vehicle(p,p,i,i,10.6,p.token,'proof')
        new=owner.update(None,10.1)
        with self.assertRaisesRegex(EnvelopeError,'STALE_VEHICLE_PROFILE'):
            bind_envelope_vehicle(p,new,i,i,10.1,p.token,'proof')

    def test_normalization_keeps_legacy_fields_and_marks_http_unavailable(self):
        from core.telemetry import Telemetry
        reader=SCSTelemetry(); reader.mm=memory(1)
        telemetry=Telemetry.__new__(Telemetry); telemetry.sdk_reader=reader
        with patch('core.telemetry.logging'):
            data=telemetry._normalize_sdk(reader.update())
            self.assertTrue(data['trailer']['attached'])
            self.assertEqual(data['trailer']['wheelTrackM'],2)
            self.assertEqual(data['truck']['x'],123)
            self.assertIn('vehicle_profile',data)
            http=telemetry._normalize_http({'truck':{'speed':0}})
        self.assertIsNone(http['vehicle_profile'].model)
        self.assertEqual(http['vehicle_profile'].observation_failure,'VEHICLE_OBSERVATION_UNAVAILABLE')

    def test_cross_frame_profile_is_not_associated_with_legacy_clock(self):
        from core.telemetry import Telemetry
        reader=SCSTelemetry(); reader.mm=memory()
        telemetry=Telemetry.__new__(Telemetry); telemetry.sdk_reader=reader
        raw=reader.update(); raw['time']+=1
        with patch('core.telemetry.logging'):
            profile=telemetry._normalize_sdk(raw)['vehicle_profile']
        self.assertEqual(profile.observation_failure,'SDK_PROFILE_FRAME_MISMATCH')

    def test_engine_publishes_same_profile_without_changing_legacy_payload(self):
        from core.engine import UltraPilotEngine
        engine=UltraPilotEngine.__new__(UltraPilotEngine)
        data={'truck':{'pose_valid':True,'speed':3,'gameSteer':.2},'raw':{'sdkActive':True}}
        before=engine._vehicle_telemetry_payload(data,10)
        o=observation(); profile=provider(o).update(o,10)
        after=engine._vehicle_telemetry_payload(dict(data,vehicle_profile=profile),10)
        self.assertIs(after['vehicle_profile_snapshot'],profile)
        self.assertEqual(before['vehicle_envelope_snapshot'],after['vehicle_envelope_snapshot'])
        self.assertEqual(before['truck_speed_ms'],after['truck_speed_ms'])
        self.assertNotIn('steering',after)

    def test_sdk_ground_height_is_not_silently_flattened_to_surface(self):
        o=observation(1); p=provider(o).update(o,10)
        self.assertEqual(p.observation.articles[0].position_m[1],17)
        self.assertEqual(p.observation.articles[1].position_m[1],18)
        self.assertNotEqual(p.observation.articles[0].rotation_rad[1],0)
        self.assertFalse(hasattr(p,'frame'))
        self.assertIn('UNPROVEN_GROUND_REFERENCE_AND_SURFACE',p.live_blockers)

    def test_empty_slots_are_not_a_valid_tractor_only_profile(self):
        o=replace(observation(),articles=())
        self.assertEqual(VehicleProfileProvider().update(o,10).observation_failure,
                         'INCOMPLETE_VEHICLE_SLOT_OBSERVATION')

    def test_inactive_zero_geometry_is_unavailable_not_a_default_vehicle(self):
        data=memory(); data[0]=0; struct.pack_into('<I',data,80,0)
        o=capture_vehicle_observation(data,10)
        self.assertEqual(o.failure_reason,'SDK_PAUSED_OR_INACTIVE')
        self.assertEqual(o.articles,())

    def test_unknown_observation_schema_cannot_reuse_profile(self):
        o=observation(); owner=provider(o); before=owner.update(o,10)
        after=owner.update(replace(o,schema_version=2),10.1)
        self.assertEqual(after.observation_failure,'UNSUPPORTED_VEHICLE_OBSERVATION_SCHEMA')
        self.assertIsNone(after.model)
        self.assertNotEqual(before.token,after.token)


if __name__ == '__main__':
    unittest.main()
