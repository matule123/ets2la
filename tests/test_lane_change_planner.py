import unittest
from dataclasses import replace

from core.navigation.lane_model import (
    LaneConnection, LaneId, LanePath, LanePoint, LaneSegment,
)
from core.navigation.lane_trajectory import (
    build_lane_trajectory, validate_lane_trajectory,
)
from core.navigation.navigation_intent import (
    NavigationBuildGuard, snapshot_matches_navigation_intent,
)
from core.navigation.road_network import RoadNetwork


class LaneChangeFixture:
    def __init__(self, length_m=220.0, gps_pair_index=0,
                 source_uids=(1, 2)):
        self.net = RoadNetwork()
        self.net.loaded = True
        self.length_m = float(length_m)
        self.gps_pair_index = int(gps_pair_index)
        self.start_uid, self.end_uid = source_uids
        self.left_id = LaneId(100, 1, 0)
        self.source_id = LaneId(100, 1, 1)
        self.right_id = LaneId(100, 1, 2)
        self.left = self.road_lane(
            self.left_id, -4.5, 0, left=None, right=self.source_id)
        self.source = self.road_lane(
            self.source_id, 0.0, 1,
            left=self.left_id, right=self.right_id)
        self.right = self.road_lane(
            self.right_id, 4.5, 2, left=self.source_id, right=None)

    def points(self, x, y=0.0):
        count = max(2, int(self.length_m // 2.0) + 1)
        return tuple(LanePoint(
            float(x), float(y), -self.length_m*index/(count-1),
            self.length_m*index/(count-1), 0.0, 0.0,
        ) for index in range(count))

    def road_lane(self, lane_id, x, raw_index, left, right, y=0.0,
                  direction=1, layer=0):
        return LaneSegment(
            lane_id, self.start_uid, self.end_uid, direction,
            lane_id.lane_index, 3, 4.5, "derived", layer, "three-lane",
            "traffic_lane.road.local", self.points(x, y),
            left_neighbor=left, right_neighbor=right,
            gps_uids=frozenset((self.start_uid, self.end_uid)),
            raw_lane_index=raw_index,
            gps_pair_index=self.gps_pair_index,
        )

    def downstream(self, target, lane_type="prefab", token="junction",
                   end_uid=3, lane_count=1):
        lane_id = LaneId(
            self.end_uid, 1, target.lane_index,
            token if lane_type in ("prefab", "roundabout") else None,
            7 if lane_type in ("prefab", "roundabout") else None,
            (7,) if lane_type in ("prefab", "roundabout") else (),
        )
        x = target.centerline[-1].x
        y = target.centerline[-1].y
        z = target.centerline[-1].z
        return LaneSegment(
            lane_id, self.end_uid, end_uid, 1, target.lane_index,
            lane_count, 4.5, "dataset", target.elevation_layer, None,
            lane_type, (
                LanePoint(x, y, z, heading=0.0),
                LanePoint(x, y, z-30.0, heading=0.0),
            ), connector_curve_indices=((7,) if token else ()),
            gps_uids=frozenset((self.end_uid, end_uid)),
            raw_lane_index=target.raw_lane_index,
            gps_pair_index=(self.gps_pair_index+1
                            if self.gps_pair_index >= 0 else 0),
        )

    def plan(self, target, *, speed=20.0, downstream=None,
             progress=0.0):
        downstream = downstream or self.downstream(target)
        return self.net._build_lane_change_segment(
            self.source, target, downstream, speed_mps=speed,
            start_progress_m=progress)


class SafeLaneChangePlannerTests(unittest.TestCase):
    def assert_valid_plan(self, fixture, target, speed=20.0,
                          downstream=None):
        downstream = downstream or fixture.downstream(target)
        transition, reason, details = fixture.plan(
            target, speed=speed, downstream=downstream)
        self.assertIsNotNone(transition, reason)
        self.assertEqual(reason, "")
        self.assertTrue(details["accepted"])
        connection = fixture.net._lane_connection(transition, downstream)
        transition = replace(
            transition, successors=(connection,))
        source_uids = (
            fixture.start_uid, fixture.end_uid, downstream.end_uid)
        path = fixture.net.connect_lane_sequence(
            (transition, downstream), source_uids,
            expected_first_gps_pair_index=fixture.gps_pair_index)
        self.assertTrue(path.valid, path.failure_reason)
        trajectory = build_lane_trajectory(path)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        validation = validate_lane_trajectory(trajectory)
        self.assertTrue(validation.valid, validation.failure_reason)
        self.assertEqual(validation.lane_change_count, 1)
        self.assertEqual(transition.lane_id,
                         transition.lane_change.source_lane_id)
        self.assertEqual(target.lane_id,
                         transition.lane_change.target_lane_id)
        return transition, trajectory, validation

    def test_safe_left_and_right_lane_changes_are_separate_transitions(self):
        for target_name in ("left", "right"):
            with self.subTest(target=target_name):
                fixture = LaneChangeFixture()
                target = getattr(fixture, target_name)
                original_source = fixture.source.centerline
                original_target = target.centerline
                transition, _trajectory, validation = self.assert_valid_plan(
                    fixture, target)
                self.assertEqual(transition.lane_change.direction, target_name)
                self.assertEqual(fixture.source.centerline, original_source)
                self.assertEqual(target.centerline, original_target)
                self.assertLessEqual(
                    transition.lane_change.max_lateral_accel_mps2, 0.90+1e-9)
                self.assertLess(validation.max_lane_change_curvature_slew,
                                0.020)

    def test_maneuver_length_depends_on_speed_width_accel_and_slew(self):
        fixture = LaneChangeFixture(length_m=300.0)
        slow, reason, _ = fixture.plan(fixture.left, speed=5.0)
        self.assertIsNotNone(slow, reason)
        fast, reason, _ = fixture.plan(fixture.left, speed=25.0)
        self.assertIsNotNone(fast, reason)
        self.assertGreater(fast.lane_change.required_length_m,
                           slow.lane_change.required_length_m)
        self.assertGreaterEqual(
            slow.lane_change.maneuver_length_m,
            fixture.source.width_m*8.0)

    def test_validation_rejects_curvature_slew_above_dedicated_limit(self):
        fixture = LaneChangeFixture()
        downstream = fixture.downstream(fixture.left)
        transition, _trajectory, _validation = self.assert_valid_plan(
            fixture, fixture.left, downstream=downstream)
        transition = replace(
            transition,
            lane_change=replace(
                transition.lane_change, max_curvature_slew=0.021),
        )
        path = fixture.net.connect_lane_sequence(
            (transition, downstream),
            (fixture.start_uid, fixture.end_uid, downstream.end_uid),
            expected_first_gps_pair_index=fixture.gps_pair_index,
        )
        validation = validate_lane_trajectory(path)
        self.assertFalse(validation.valid)
        self.assertIn("curvature-slew limit", validation.failure_reason)

    def test_insufficient_approach_is_fail_closed_with_metrics(self):
        fixture = LaneChangeFixture(length_m=30.0)
        transition, reason, details = fixture.plan(
            fixture.left, speed=0.0, progress=5.0)
        self.assertIsNone(transition)
        self.assertIn("LANE_CHANGE_INSUFFICIENT_APPROACH", reason)
        self.assertAlmostEqual(details["available_length_m"], 25.0, places=2)
        self.assertGreater(details["required_length_m"], 40.0)
        self.assertEqual(details["lateral_shift_m"], 4.5)

    def test_opposing_and_non_adjacent_lanes_are_rejected(self):
        fixture = LaneChangeFixture()
        opposing = replace(
            fixture.left, direction=-1,
            lane_id=LaneId(100, -1, 0))
        transition, reason, _ = fixture.plan(opposing)
        self.assertIsNone(transition)
        self.assertIn("OPPOSING_DIRECTION", reason)

        distant_id = LaneId(100, 1, 3)
        distant = fixture.road_lane(
            distant_id, -9.0, 3, left=None, right=fixture.source_id)
        transition, reason, _ = fixture.plan(distant)
        self.assertIsNone(transition)
        self.assertIn("NOT_ADJACENT", reason)

    def test_bridge_or_other_elevation_layer_is_rejected(self):
        fixture = LaneChangeFixture()
        upper = fixture.road_lane(
            fixture.left_id, -4.5, 0, left=None,
            right=fixture.source_id, y=12.0, layer=4)
        transition, reason, details = fixture.plan(upper)
        self.assertIsNone(transition)
        self.assertIn("ELEVATION_LAYER", reason)
        self.assertEqual(details["source_elevation_layer"], 0)
        self.assertEqual(details["target_elevation_layer"], 4)

    def test_prefab_and_roundabout_targets_are_both_confirmed(self):
        for lane_type in ("prefab", "roundabout"):
            with self.subTest(lane_type=lane_type):
                fixture = LaneChangeFixture()
                downstream = fixture.downstream(
                    fixture.left, lane_type=lane_type,
                    token=f"{lane_type}-token")
                transition, _, _ = self.assert_valid_plan(
                    fixture, fixture.left, downstream=downstream)
                self.assertEqual(transition.lane_change.prefab_token,
                                 f"{lane_type}-token")

    def test_merge_and_split_keep_directed_target_lane(self):
        for lane_count, expected in ((1, "merge"), (4, "split")):
            with self.subTest(expected=expected):
                fixture = LaneChangeFixture()
                downstream = fixture.downstream(
                    fixture.left, lane_type="road", token=None,
                    lane_count=lane_count)
                transition, _, _ = self.assert_valid_plan(
                    fixture, fixture.left, downstream=downstream)
                connection = fixture.net._lane_connection(
                    transition, downstream)
                self.assertEqual(connection.kind, expected)

    def test_rolling_prefix_lane_change_preserves_pair_and_repeated_uid_order(self):
        rolling = LaneChangeFixture(
            length_m=180.0, gps_pair_index=-1, source_uids=(1, 2))
        downstream = rolling.downstream(rolling.left, end_uid=3)
        transition, reason, _ = rolling.plan(
            rolling.left, speed=10.0, downstream=downstream,
            progress=20.0)
        self.assertIsNotNone(transition, reason)
        transition = replace(
            transition,
            successors=(rolling.net._lane_connection(
                transition, downstream),))
        path = rolling.net.connect_lane_sequence(
            (transition, downstream), (2, 3),
            expected_first_gps_pair_index=0)
        trajectory = build_lane_trajectory(path)
        self.assertTrue(trajectory.valid, trajectory.failure_reason)
        self.assertEqual(transition.lane_change.gps_pair_index, -1)

        repeated = LaneChangeFixture(
            length_m=180.0, gps_pair_index=2, source_uids=(1, 2))
        after = repeated.downstream(repeated.left, end_uid=3)
        after = replace(after, gps_pair_index=3)
        transition, reason, _ = repeated.plan(
            repeated.left, speed=10.0, downstream=after)
        self.assertIsNotNone(transition, reason)
        transition = replace(
            transition,
            successors=(repeated.net._lane_connection(
                transition, after),))
        path = repeated.net.connect_lane_sequence(
            (transition, after), (9, 9, 1, 2, 3),
            expected_first_gps_pair_index=2)
        trajectory = build_lane_trajectory(path)
        validation = validate_lane_trajectory(trajectory)
        self.assertTrue(validation.valid, validation.failure_reason)
        self.assertEqual((validation.first_gps_pair_index,
                          validation.last_gps_pair_index), (2, 3))

    def test_stale_callback_and_changed_intent_cannot_publish_transition(self):
        fixture = LaneChangeFixture()
        transition, _trajectory, _validation = self.assert_valid_plan(
            fixture, fixture.left)
        snapshot = {
            "revision": 7,
            "valid": True,
            "navigation_intent_id": "intent-a",
            "request_id": "intent-a",
            "source_gps_uids": [1, 2, 3],
            "lane_corridor": [{
                "lane_id": {"road_uid": transition.lane_id.road_uid},
                "lane_change": {
                    "source": transition.lane_change.source_lane_id.road_uid,
                    "target": transition.lane_change.target_lane_id.road_uid,
                },
            }],
        }
        self.assertTrue(snapshot_matches_navigation_intent(
            {"navigation_intent_id": "intent-a"}, snapshot))
        self.assertFalse(snapshot_matches_navigation_intent(
            {"navigation_intent_id": "intent-b"}, snapshot))

        guard = NavigationBuildGuard()
        token = guard.begin("intent-a", ((1, 2, 3),), "lane-change-build")
        self.assertTrue(guard.may_publish(token))
        guard.reset_intent("intent-a")
        self.assertFalse(guard.may_publish(token))


if __name__ == "__main__":
    unittest.main()
