import math
import multiprocessing as mp
import struct
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

from core.ar_overlay import AROverlay
from core.camera import (
    CAMERA_MAPPING, CameraSnapshotProducer, camera_snapshot_reason,
    project_world_point, quaternion_to_euler,
)
from core.engine import UltraPilotEngine
from core.hud import UltraPilotHUD
from core.ipc.shared_state import SharedState
from core.navigation.runtime_preflight import build_runtime_preflight
from core.sdk.scs_sdk import SCSTelemetry
from core.sdk.game_utils import install_game_dlls
from sdk.plugin_sdk import CTL_BRAKE, CTL_STEERING, CTL_THROTTLE
from plugins.autopilot.main import lane_authority_rejection_reason
from tests.test_lane_authority_integration import Controller, State


def raw_camera(*, fov=90.0, x=10.0, y=3.0, z=20.0,
               tile_x=2, tile_z=-1, quaternion=(1.0, 0.0, 0.0, 0.0)):
    return (fov, x, y, z, tile_x, tile_z, *quaternion)


def viewport(width=1920, height=1080, x=100, y=50):
    return {"x": x, "y": y, "width": width, "height": height,
            "aspect": width / height, "hwnd": 123, "title": "Euro Truck Simulator 2"}


def producer(raw=None, view=None):
    raw = raw_camera() if raw is None else raw
    view = viewport() if view is None else view
    return CameraSnapshotProducer(lambda: raw, lambda: view)


def snapshot(raw=None, view=None, now=None, render_time=1_000_000):
    now = time.monotonic() if now is None else now
    return producer(raw, view).read(render_time, now, now)


class CameraSnapshotTests(unittest.TestCase):
    def test_scs_reader_exposes_render_time_from_zone_one_offset_24(self):
        reader = SCSTelemetry()
        reader.mm = bytearray(reader.mmap_size)
        struct.pack_into("?", reader.mm, 0, True)
        struct.pack_into("Q", reader.mm, 8, 111)
        struct.pack_into("Q", reader.mm, 16, 222)
        struct.pack_into("Q", reader.mm, 24, 333)
        data = reader.update()
        self.assertTrue(data["sdkActive"])
        self.assertEqual(data["time"], 111)
        self.assertEqual(data["simulatedTime"], 222)
        self.assertEqual(data["renderTime"], 333)

    def test_shipped_game_plugin_contains_camera_props_producer(self):
        data = (Path(__file__).parents[1] / "assets" / "ets2la_plugin.dll").read_bytes()
        self.assertIn("ETS2LACameraProps", data.decode("utf-16le", errors="ignore"))

    def test_bootloader_dll_check_does_not_copy_identical_file(self):
        with (mock.patch("core.sdk.game_utils.find_scs_games",
                         return_value=[r"C:\fake-game"]),
              mock.patch("core.sdk.game_utils.os.makedirs"),
              mock.patch("core.sdk.game_utils.os.path.exists",
                         return_value=True),
              mock.patch("filecmp.cmp", return_value=True),
              mock.patch("shutil.copy2") as copy):
            install_game_dlls(r"C:\fake-assets",
                              names=["ets2la_plugin.dll"])
        copy.assert_not_called()

    def test_tile_coordinates_become_absolute_xyz_and_metadata_is_explicit(self):
        snap = snapshot()
        self.assertTrue(snap["valid"], snap["failure_reason"])
        self.assertEqual(snap["source"], CAMERA_MAPPING)
        self.assertEqual(snap["position"], [1034.0, 3.0, -492.0])
        self.assertEqual(snap["tile"], [2, -1])
        self.assertEqual(snap["fov_convention"], "horizontal-degrees")
        self.assertEqual(snap["matrix_layout"], "row-major")
        self.assertEqual(snap["camera_axes"], "+X right, +Y up, -Z forward")
        self.assertEqual(len(snap["view_matrix"]), 16)
        self.assertEqual(len(snap["projection_matrix"]), 16)
        self.assertEqual(len(snap["view_projection"]), 16)

    def test_center_left_right_up_down_and_behind(self):
        now = time.monotonic()
        snap = snapshot(now=now)
        cx, cy, cz = snap["position"]
        center = project_world_point(snap, (cx, cy, cz - 20), now=now,
                                     telemetry_timestamp=now)
        left = project_world_point(snap, (cx - 4, cy, cz - 20), now=now,
                                   telemetry_timestamp=now)
        right = project_world_point(snap, (cx + 4, cy, cz - 20), now=now,
                                    telemetry_timestamp=now)
        up = project_world_point(snap, (cx, cy + 4, cz - 20), now=now,
                                 telemetry_timestamp=now)
        down = project_world_point(snap, (cx, cy - 4, cz - 20), now=now,
                                   telemetry_timestamp=now)
        behind = project_world_point(snap, (cx, cy, cz + 20), now=now,
                                     telemetry_timestamp=now)
        self.assertAlmostEqual(center[0], 960.0, places=5)
        self.assertAlmostEqual(center[1], 540.0, places=5)
        self.assertLess(left[0], center[0]); self.assertGreater(right[0], center[0])
        self.assertLess(up[1], center[1]); self.assertGreater(down[1], center[1])
        self.assertIsNone(behind)

    def test_4_3_16_9_and_21_9_use_real_aspect_without_fixed_4_3(self):
        positions = []
        for width, height in ((1600, 1200), (1920, 1080), (2520, 1080)):
            now = time.monotonic()
            snap = snapshot(view=viewport(width, height), now=now)
            cx, cy, cz = snap["position"]
            projected = project_world_point(
                snap, (cx + 5, cy + 2, cz - 20), now=now,
                telemetry_timestamp=now)
            positions.append((projected[0] / width, projected[1] / height))
        for position in positions[1:]:
            self.assertAlmostEqual(position[0], positions[0][0], places=6)
        # Horizontal FOV is fixed, therefore vertical normalized displacement
        # grows with aspect instead of pretending every viewport is 4:3.
        self.assertGreater(abs(positions[2][1] - 0.5),
                           abs(positions[1][1] - 0.5))
        self.assertGreater(abs(positions[1][1] - 0.5),
                           abs(positions[0][1] - 0.5))

    def test_yaw_pitch_and_roll_follow_camera_quaternion(self):
        angle = math.radians(30.0)
        half = angle * 0.5
        cases = (
            # normalized x controls ETS2LA yaw; stored component is qy.
            ((math.cos(half), 0.0, math.sin(half), 0.0),
             lambda c: (c[0] - math.sin(angle) * 20, c[1],
                        c[2] - math.cos(angle) * 20)),
            # normalized y controls pitch; stored component is qx.
            ((math.cos(half), math.sin(half), 0.0, 0.0),
             lambda c: (c[0], c[1] + math.sin(angle) * 20,
                        c[2] - math.cos(angle) * 20)),
            # roll does not move the optical axis.
            ((math.cos(half), 0.0, 0.0, math.sin(half)),
             lambda c: (c[0], c[1], c[2] - 20)),
        )
        for quaternion, optical_axis in cases:
            with self.subTest(quaternion=quaternion):
                now = time.monotonic()
                snap = snapshot(raw=raw_camera(quaternion=quaternion), now=now)
                point = optical_axis(snap["position"])
                projected = project_world_point(
                    snap, point, now=now, telemetry_timestamp=now)
                self.assertIsNotNone(projected)
                self.assertAlmostEqual(projected[0], 960.0, places=4)
                self.assertAlmostEqual(projected[1], 540.0, places=4)

    def test_matrix_projection_matches_original_ets2la_rotation_sequence(self):
        now = time.monotonic()
        snap = snapshot(raw=raw_camera(
            quaternion=(0.96, 0.08, 0.16, -0.10)), now=now)
        cx, cy, cz = snap["position"]
        target = (cx + 2.5, cy - 1.0, cz - 35.0)
        actual = project_world_point(
            snap, target, now=now, telemetry_timestamp=now)
        self.assertIsNotNone(actual)

        pitch, yaw, roll = quaternion_to_euler(snap["quaternion"])
        rel_x, rel_y, rel_z = (target[0] - cx, target[1] - cy, target[2] - cz)
        cos_yaw, sin_yaw = math.cos(-yaw), math.sin(-yaw)
        new_x = rel_x * cos_yaw + rel_z * sin_yaw
        new_z = rel_z * cos_yaw - rel_x * sin_yaw
        cos_pitch, sin_pitch = math.cos(-pitch), math.sin(-pitch)
        new_y = rel_y * cos_pitch - new_z * sin_pitch
        final_z = new_z * cos_pitch + rel_y * sin_pitch
        cos_roll, sin_roll = math.cos(-roll), math.sin(-roll)
        final_x = new_x * cos_roll - new_y * sin_roll
        final_y = new_y * cos_roll + new_x * sin_roll
        focal = 960.0 / math.tan(math.radians(snap["fov_horizontal_deg"]) / 2)
        expected_x = 1920.0 - ((final_x / final_z) * focal + 960.0)
        expected_y = (final_y / final_z) * focal + 540.0
        self.assertAlmostEqual(actual[0], expected_x, places=5)
        self.assertAlmostEqual(actual[1], expected_y, places=5)

    def test_stale_render_time_sync_invalid_quaternion_fov_and_nonfinite_fail(self):
        p = producer()
        first = p.read(100, 1.0, 1.0)
        self.assertTrue(first["valid"])
        stale = p.read(100, 1.6, 1.6)
        self.assertFalse(stale["valid"])
        self.assertIn("stale", stale["failure_reason"])
        unsynced = producer().read(100, 1.0, 1.3)
        self.assertFalse(unsynced["valid"])
        self.assertIn("not synchronized", unsynced["failure_reason"])
        for raw, reason in (
            (raw_camera(quaternion=(0, 0, 0, 0)), "zero length"),
            (raw_camera(quaternion=(math.nan, 0, 0, 0)), "NaN"),
            (raw_camera(fov=180.0), "FOV"),
        ):
            result = snapshot(raw=raw)
            self.assertFalse(result["valid"])
            self.assertIn(reason, result["failure_reason"])

    def test_camera_revision_increases_and_missing_viewport_is_invalid(self):
        p = producer()
        first = p.read(100, 1.0, 1.0)
        second = p.read(101, 1.01, 1.01)
        self.assertGreater(second["revision"], first["revision"])
        bad = producer(view=None)
        bad.viewport_provider = lambda: None
        result = bad.read(100, 1.0, 1.0)
        self.assertFalse(result["valid"])
        self.assertIn("viewport", result["failure_reason"])

    def test_missing_or_nonfinite_matrix_is_rejected(self):
        now = time.monotonic()
        snap = snapshot(now=now)
        snap["view_projection"] = [math.nan] + [0.0] * 15
        reason = camera_snapshot_reason(
            snap, now=now, telemetry_timestamp=now)
        self.assertIn("non-finite", reason)
        self.assertIsNone(project_world_point(
            snap, (0, 0, 0), now=now, telemetry_timestamp=now))


class CameraConsumerAndPreflightTests(unittest.TestCase):
    def test_nonfinite_runtime_points_are_rejected_by_hud_ar_and_autopilot(self):
        now = time.monotonic()
        route = {
            "revision": 2, "valid": True, "confidence": 0.95,
            "request_id": "r", "source_gps_uids": [1, 2],
            "points": [[0, 0, 0], [math.nan, 0, 10]],
            "display_points": [[0, 0, 0], [math.inf, 0, 10]],
        }
        state = State({
            "lane_trajectory": route, "lane_trajectory_revision": 2,
            "lane_trajectory_heartbeat": now, "game_route_node_uids": [1, 2],
            "nav_recalc_request": "r", "telemetry_valid": True,
        })
        hud = type("HUD", (), {"shared_state": state,
                                "_rear_cam_side": "off",
                                "_rear_cam_until": 0.0})()
        ar = type("AR", (), {"state": state})()
        self.assertEqual(UltraPilotHUD._read(hud)["nav_path"], [])
        self.assertEqual(AROverlay._current_display_points(ar), (-1, []))
        self.assertIn("non-finite",
                      lane_authority_rejection_reason(state, route, now))

    def test_end_to_end_same_lane_revision_and_xyz_are_projected(self):
        now = time.monotonic()
        camera = snapshot(raw=raw_camera(x=0, y=2, z=0, tile_x=0, tile_z=0),
                          now=now)
        state = State({
            "camera_snapshot": camera, "telemetry_timestamp": now,
            "telemetry_valid": True, "game_in_truck": True,
            "game_route_node_uids": [1, 2], "nav_recalc_request": "r1",
            "lane_trajectory_revision": 4, "lane_trajectory_heartbeat": now,
            "lane_trajectory": {
                "revision": 4, "valid": True, "confidence": 0.95,
                "request_id": "r1", "source_gps_uids": [1, 2],
                "points": [[0, 0, -10], [0, 0, -20]],
                "display_points": [[0, 0, -10], [0, 0, -20]],
            },
        })
        overlay = type("Projection", (), {"state": state})()
        revision, points = AROverlay._current_display_points(overlay)
        self.assertEqual(revision, 4)
        self.assertEqual(points, [[0, 0, -10], [0, 0, -20]])
        projected = AROverlay._project_world(overlay, points[0])
        self.assertIsNotNone(projected)

    def test_window_location_and_size_changes_move_overlay_without_offsets(self):
        state = State({"camera_snapshot": {"viewport": viewport(1280, 720, -1280, 20)}})
        overlay = type("Window", (), {
            "state": state, "_geometry": (0, 0, 100, 100),
            "x": lambda self: self._geometry[0],
            "y": lambda self: self._geometry[1],
            "width": lambda self: self._geometry[2],
            "height": lambda self: self._geometry[3],
            "setGeometry": lambda self, *value: setattr(self, "_geometry", value),
        })()
        AROverlay._sync_viewport(overlay)
        self.assertEqual(overlay._geometry, (-1280, 20, 1280, 720))

    def test_preflight_distinguishes_camera_hud_ar_and_autopilot(self):
        now = time.monotonic()
        camera = snapshot(now=now)
        route = {
            "revision": 8, "valid": True, "confidence": 0.72,
            "request_id": "x", "source_gps_uids": [1, 2],
            "points": [[0, 1, 0], [0, 2, 10]],
            "display_points": [[0, 1, 0], [0, 2, 10]],
        }
        state = State({
            "telemetry_valid": True, "telemetry_timestamp": now,
            "game_route_node_uids": [1, 2], "nav_recalc_request": "x",
            "lane_trajectory": route, "lane_trajectory_revision": 8,
            "lane_trajectory_heartbeat": now, "camera_snapshot": camera,
            "hud_navigation_readiness": {"ready": True},
            "ar_navigation_readiness": {"ready": True},
            "autopilot_navigation_readiness": {"ready": True},
        })
        result = build_runtime_preflight(state, now)
        self.assertTrue(result["display_ready"])
        self.assertTrue(result["ar_ready"])
        self.assertTrue(result["autopilot_ready"])
        state.set("camera_snapshot", {"valid": False,
                                      "failure_reason": "mapping missing"})
        result = build_runtime_preflight(state, now)
        self.assertTrue(result["display_ready"])
        self.assertFalse(result["ar_ready"])
        self.assertTrue(result["autopilot_ready"])

    def test_preflight_rejects_missing_target_map_telemetry_low_confidence_and_old_revision(self):
        now = time.monotonic()
        base_route = {
            "revision": 3, "valid": True, "confidence": 0.95,
            "request_id": "new", "source_gps_uids": [10, 20],
            "points": [[0, 0, 0], [0, 0, 10]],
            "display_points": [[0, 0, 0], [0, 0, 10]],
        }
        base = {
            "telemetry_valid": True, "telemetry_timestamp": now,
            "game_route_node_uids": [10, 20], "nav_recalc_request": "new",
            "lane_trajectory_revision": 3, "lane_trajectory_heartbeat": now,
            "lane_trajectory": base_route, "camera_snapshot": snapshot(now=now),
            "hud_navigation_readiness": {"ready": True},
            "ar_navigation_readiness": {"ready": True},
            "autopilot_navigation_readiness": {"ready": True},
        }
        mutations = (
            ({"game_route_node_uids": []}, "gps_target"),
            ({"telemetry_valid": False}, "telemetry"),
            ({"lane_trajectory": dict(base_route, valid=False,
                                      failure_reason="map missing")},
             "lane_trajectory"),
            ({"lane_trajectory": dict(base_route, confidence=0.719999)},
             "confidence"),
            ({"lane_trajectory_revision": 4}, "lane_trajectory"),
            ({"nav_recalc_request": "changed"}, "lane_trajectory"),
        )
        for mutation, failed_check in mutations:
            with self.subTest(failed_check=failed_check, mutation=mutation):
                state = State(dict(base, **mutation))
                result = build_runtime_preflight(state, now)
                self.assertFalse(result["checks"][failed_check]["ready"])
    def test_manual_disable_releases_immediately_and_clears_intents(self):
        engine = UltraPilotEngine.__new__(UltraPilotEngine)
        engine.shared_state = State({
            "autopilot_active": True,
            "autopilot_command": {"seq": 1, "enabled": False},
            CTL_STEERING: 0.8, CTL_THROTTLE: 1.0, CTL_BRAKE: 0.4,
        })
        engine.controller = Controller()
        engine.controller.release_count = 0
        def release_all():
            engine.controller.release_count += 1
            engine.controller.steering = 0.0
            engine.controller.throttle = 0.0
            engine.controller.brake = 0.0
        engine.controller.release_all = release_all
        engine._was_active = True
        engine._process_autopilot_command()
        self.assertFalse(engine.shared_state.get("autopilot_active"))
        self.assertEqual(engine.controller.release_count, 1)
        self.assertEqual((engine.shared_state.get(CTL_STEERING),
                          engine.shared_state.get(CTL_THROTTLE),
                          engine.shared_state.get(CTL_BRAKE)), (0.0, 0.0, 0.0))
        self.assertFalse(engine._was_active)

    def test_atomic_camera_snapshot_publication_never_accepts_mixed_revision(self):
        manager = None
        try:
            manager = mp.Manager()
            raw = manager.dict()
        except (OSError, PermissionError):
            raw = {}
        shared = SharedState(raw)
        shared.set("camera_snapshot", {"revision": 0, "position": [0, 0, 0]})
        failures = []

        def writer():
            for revision in range(1, 200):
                shared.set("camera_snapshot", {
                    "revision": revision,
                    "position": [revision, revision, revision],
                    "view_projection": [float(revision)] * 16,
                })

        thread = threading.Thread(target=writer)
        thread.start()
        while thread.is_alive():
            item = shared.get("camera_snapshot", {})
            revision = item.get("revision")
            if item.get("position") != [revision, revision, revision]:
                failures.append(item)
        thread.join()
        if manager is not None:
            manager.shutdown()
        self.assertEqual(failures, [])


if __name__ == "__main__":
    unittest.main()
