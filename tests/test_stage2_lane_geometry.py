"""Stage 2 regressions for continuous, authority-bound LanePath geometry."""

import math
import time
import unittest

from core.navigation.route import Route
from plugins.autopilot.main import navigation_command


def _authority(road_uid, direction=1, elevation=10):
    return ((road_uid, direction, 0, "", -1, ()), elevation)


class Stage2LaneGeometryTests(unittest.TestCase):
    def test_active_gps_never_falls_back_to_legacy_scalar_for_incomplete_packet(self):
        now = time.monotonic()
        snapshot = {
            "revision": 7,
            "navigation_intent_id": "intent",
            "route_build_id": "build",
            "source_game_session_id": "session",
            "source_map_key": "promods-1.59",
            "source_dataset_fingerprint": "fingerprint",
        }
        state = {
            "nav_steering": 0.42,
            "path_curve_signed_curvature": 0.03,
        }
        target, curvature, reason = navigation_command(
            state, snapshot, gps_active=True, now=now,
            packet={"calculation_packet_schema_version": 1,
                    "authority_valid": True, "computed_at": now})
        self.assertEqual((target, curvature), (0.0, 0.0))
        self.assertEqual(reason, "GPS steering packet is incomplete")

    def test_proven_lane_boundary_owns_its_directed_transition_segment(self):
        """Real replay switched identity before the next same-lane segment.

        At 75.562 s the new LaneId was already confirmed while its first
        same-authority segment began about one map sample farther ahead.  The
        immutable A->B trajectory edge is the only valid projection; omitting
        it generated the captured zero packet for two ticks.
        """
        first = _authority(5962819247758802322, elevation=61)
        second = _authority(5962819246299167667, elevation=62)
        points = [(0.0, 0.0, 0.0), (0.0, 0.1, -2.0),
                  (0.0, 0.2, -4.0), (0.0, 0.3, -6.0),
                  (0.0, 0.4, -8.0), (0.0, 0.5, -10.0)]
        point_authorities = [
            first, first, first, second, second, second]
        route = Route(points, point_authorities=point_authorities)
        command = route.steering(
            (0.10, -3.40), 0.0, 10.0,
            cross_track_error_m=0.10,
            control_authority={
                "lane_identity": second[0], "elevation_layer": second[1],
                "lane_width_m": 4.5, "revision": 7,
            })
        debug = route.last_steering_debug
        self.assertTrue(debug["authority_valid"], debug)
        self.assertEqual(debug["controller"], "frenet_bicycle")
        self.assertEqual(debug["tracking_segment_index"], 2)
        self.assertTrue(math.isfinite(command))

        previous_route = Route(points, point_authorities=point_authorities)
        previous_command = previous_route.steering(
            (0.10, -3.40), 0.0, 10.0,
            cross_track_error_m=0.10,
            control_authority={
                "lane_identity": first[0], "elevation_layer": first[1],
                "lane_width_m": 4.5, "revision": 7,
            })
        self.assertTrue(previous_route.last_steering_debug["authority_valid"])
        self.assertAlmostEqual(command, previous_command, places=12)

    def test_unproven_or_spatially_discontinuous_boundary_stays_fail_closed(self):
        cases = (
            # Opposite road directions without a prefab transition.
            (_authority(1, 1), _authority(2, -1), 2.0),
            # Same direction but a gap beyond the derivative safety bound.
            (_authority(1, 1), _authority(2, 1), 4.0),
        )
        for first, second, gap in cases:
            with self.subTest(first=first, second=second, gap=gap):
                points = [(0.0, 0.0, 0.0), (0.0, 0.0, -2.0),
                          (0.0, 0.0, -2.0-gap),
                          (0.0, 0.0, -4.0-gap)]
                route = Route(points, point_authorities=[
                    first, first, second, second])
                route.steering(
                    (0.0, -1.0), 0.0, 8.0,
                    cross_track_error_m=0.0,
                    control_authority={
                        "lane_identity": second[0],
                        "elevation_layer": second[1],
                        "lane_width_m": 4.5, "revision": 7,
                    })
                self.assertFalse(route.last_steering_debug["authority_valid"])

    def test_vertical_layer_gap_cannot_be_claimed_as_a_lane_transition(self):
        lower = _authority(10, 1, elevation=20)
        upper = _authority(11, 1, elevation=21)
        route = Route(
            [(0.0, 0.0, 0.0), (0.0, 0.0, -2.0),
             (0.0, 2.0, -4.0), (0.0, 2.0, -6.0)],
            point_authorities=[lower, lower, upper, upper])
        route.steering(
            (0.0, -1.0), 0.0, 8.0,
            cross_track_error_m=0.0,
            control_authority={
                "lane_identity": upper[0], "elevation_layer": upper[1],
                "lane_width_m": 4.5, "revision": 7,
            })
        self.assertFalse(route.last_steering_debug["authority_valid"])

    def test_all_captured_road_lane_boundaries_keep_a_complete_controller(self):
        captured = (
            (5962819239084995065, 51, 5962819241291168693, 53),
            (5962819239839939508, 58, 5962819249855954321, 60),
            (5962819247758802322, 61, 5962819246299167667, 62),
            (5962819247825894321, 63, 5962819242616568754, 64),
            (5962819235511417775, 66, 5962819251089054193, 66),
            (5962819265718787481, 67, 5962819263806183923, 68),
            (5962819237457574830, 71, 5962819257992878537, 71),
        )
        for first_uid, first_layer, second_uid, second_layer in captured:
            with self.subTest(second_uid=second_uid):
                first = _authority(first_uid, elevation=first_layer)
                second = _authority(second_uid, elevation=second_layer)
                # World Y is continuous even where the quantised layer label
                # changes, matching the proof used by the real LanePath.
                points = [(0.0, 0.0, 0.0), (0.0, 0.1, -2.0),
                          (0.0, 0.2, -4.0), (0.0, 0.3, -6.0),
                          (0.0, 0.4, -8.0), (0.0, 0.5, -10.0)]
                route = Route(points, point_authorities=[
                    first, first, first, second, second, second])
                route.steering(
                    (0.10, -3.40), 0.0, 10.0,
                    cross_track_error_m=0.10,
                    control_authority={
                        "lane_identity": second[0],
                        "elevation_layer": second[1],
                        "lane_width_m": 4.5, "revision": 7,
                    })
                debug = route.last_steering_debug
                self.assertTrue(debug["authority_valid"], debug)
                self.assertEqual(debug["controller"], "frenet_bicycle")


if __name__ == "__main__":
    unittest.main()
