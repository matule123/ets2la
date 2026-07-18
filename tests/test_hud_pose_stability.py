import math
import unittest

from core.hud import UltraPilotHUD


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


if __name__ == "__main__":
    unittest.main()
