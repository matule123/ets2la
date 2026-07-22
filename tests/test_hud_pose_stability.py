import math
import struct
import unittest

from core.hud import UltraPilotHUD
from core.sdk.scs_sdk import SCSTelemetry


class HudPoseStabilityTests(unittest.TestCase):
    def make_hud(self):
        hud = UltraPilotHUD.__new__(UltraPilotHUD)
        hud._display_truck_pos = None
        hud._display_truck_heading = None
        return hud

    def test_stationary_sdk_chatter_does_not_move_scene(self):
        hud = self.make_hud()
        first = hud._stabilize_display_pose((100.0, 200.0), 0.5, 0.0)
        second = hud._stabilize_display_pose((100.11, 199.91), 0.504, 0.0)
        self.assertEqual(first, second)

    def test_real_lateral_motion_is_preserved_outside_dead_band(self):
        hud = self.make_hud()
        hud._stabilize_display_pose((0.0, 0.0), 0.0, 40.0)
        (x, z), heading = hud._stabilize_display_pose((1.50, 0.0), 0.0, 40.0)
        self.assertAlmostEqual(x, 1.44, places=6)
        self.assertAlmostEqual(z, 0.0, places=6)
        self.assertAlmostEqual(heading, 0.0, places=6)

    def test_heading_wrap_is_stable(self):
        hud = self.make_hud()
        hud._stabilize_display_pose((0.0, 0.0), math.pi - 0.01, 30.0)
        _, heading = hud._stabilize_display_pose(
            (0.0, 0.1), -math.pi + 0.01, 30.0)
        error = (heading - (-math.pi + 0.01) + math.pi) % (2 * math.pi) - math.pi
        self.assertLess(abs(error), math.radians(0.13))

    def test_lane_match_keeps_model_on_same_lane_origin_while_moving(self):
        samples = (0.0, 0.35, 1.20, 2.35, -1.75)
        for lateral_error in samples:
            data = {
                "lane_revision": 12,
                "lane_match": {"lateral_error_m": lateral_error},
            }
            model_lateral = UltraPilotHUD._matched_ego_lateral(data)
            # The lane centre transformed into truck space is the negative of
            # LaneMatch's signed truck-from-lane error.
            lane_centre_lateral = -lateral_error
            self.assertAlmostEqual(model_lateral, lane_centre_lateral)

    def test_invalid_or_unmatched_lane_never_moves_model(self):
        self.assertEqual(UltraPilotHUD._matched_ego_lateral({
            "lane_revision": -1,
            "lane_match": {"lateral_error_m": 2.0},
        }), 0.0)
        self.assertEqual(UltraPilotHUD._matched_ego_lateral({
            "lane_revision": 2,
            "lane_match": {"lateral_error_m": float("nan")},
        }), 0.0)

    def test_live_trailer_heading_drives_articulation_across_wrap(self):
        data = {
            "trailer_attached": True,
            "heading": -math.pi + 0.05,
            "trailer_heading": math.pi - 0.10,
            "trailer_articulation": 0.0,
        }
        articulation = UltraPilotHUD._resolved_trailer_articulation(data)
        self.assertAlmostEqual(articulation, 0.15, places=6)

    def test_trailer_rotates_around_fixed_hitch_in_both_directions(self):
        straight_hinge, _straight_angle, straight_tail = (
            UltraPilotHUD._articulated_trailer_pose(0.8, 0.0))
        left_hinge, _left_angle, left_tail = (
            UltraPilotHUD._articulated_trailer_pose(0.8, math.radians(25.0)))
        right_hinge, _right_angle, right_tail = (
            UltraPilotHUD._articulated_trailer_pose(0.8, math.radians(-25.0)))
        self.assertEqual(straight_hinge, left_hinge)
        self.assertEqual(straight_hinge, right_hinge)
        self.assertNotAlmostEqual(left_tail[1], straight_tail[1])
        self.assertNotAlmostEqual(right_tail[1], straight_tail[1])
        self.assertLess((left_tail[1] - straight_tail[1])
                        * (right_tail[1] - straight_tail[1]), 0.0)
        for tail in (straight_tail, left_tail, right_tail):
            self.assertAlmostEqual(math.dist(straight_hinge, tail), 11.45,
                                   places=6)

    def test_sdk_reads_attached_after_all_eighty_trailer_flags(self):
        sdk = SCSTelemetry()
        sdk.mm = bytearray(sdk.mmap_size)
        base = sdk.TRAILER_BLOCK_START
        sdk.mm[base + 80] = 1
        sdk.mm[base + 81] = 0  # padding must not be read as attached
        for offset, value in ((872, 10.0), (880, 20.0), (888, 30.0),
                              (896, 0.25), (904, 0.0), (912, 0.0)):
            sdk.mm[base + offset:base + offset + 8] = struct.pack("d", value)
        trailer = sdk.read_trailer(0)
        self.assertTrue(trailer["attached"])
        self.assertEqual((trailer["worldX"], trailer["worldY"],
                          trailer["worldZ"]), (10.0, 20.0, 30.0))


if __name__ == "__main__":
    unittest.main()
