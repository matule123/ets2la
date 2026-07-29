import unittest

from core.navigation.navigation_intent import (
    NavigationBufferClass,
    NavigationBuildGuard,
    NavigationIntentTracker,
    classify_navigation_buffer,
    destination_identity,
    ordered_suffix_prefix_overlap,
)


def items(uids, start_distance=50_000.0, step=100.0):
    return [
        {"uid": uid, "distance": start_distance - index * step}
        for index, uid in enumerate(uids)
    ]


class NavigationBufferClassificationTests(unittest.TestCase):
    def classify(self, old, new, **kwargs):
        return classify_navigation_buffer(old, new, **kwargs)[0]

    def test_exact_repeated_buffer(self):
        route = items([10, 11, 12, 13])
        self.assertEqual(
            self.classify(route, route), NavigationBufferClass.SAME_EXACT)

    def test_progressively_trimmed_prefix(self):
        route = items(range(100, 120))
        for count in range(1, 19):
            with self.subTest(count=count):
                result = classify_navigation_buffer(route, route[count:])
                self.assertEqual(result[0], NavigationBufferClass.ADVANCED_PREFIX)
                self.assertEqual(result[2], count)

    def test_extended_and_rolling_horizon(self):
        route = items(range(100, 130))
        self.assertEqual(
            self.classify(route[:10], route[:15]),
            NavigationBufferClass.EXTENDED_HORIZON)
        result = classify_navigation_buffer(route[:10], route[3:13])
        self.assertEqual(result[0], NavigationBufferClass.OVERLAPPING_CONTINUATION)
        self.assertEqual((result[1], result[2], result[3]), (7, 3, 3))

    def test_prefix_trim_and_end_extension(self):
        old = items(range(100, 120))
        new = items(range(105, 128), start_distance=49_500.0)
        result = classify_navigation_buffer(old, new)
        self.assertEqual(result[0], NavigationBufferClass.OVERLAPPING_CONTINUATION)
        self.assertEqual((result[1], result[2], result[3]), (15, 5, 8))

    def test_empty_buffer_is_temporary_and_removal_is_explicit(self):
        route = items(range(100, 106))
        self.assertEqual(
            self.classify(route, [], destination_present=True),
            NavigationBufferClass.TEMPORARILY_UNAVAILABLE)
        self.assertEqual(
            self.classify(route, [], destination_present=False),
            NavigationBufferClass.DESTINATION_REMOVED)

    def test_repeated_uids_use_order_and_distance_not_a_set(self):
        old = items([1, 2, 3, 2, 4, 5], start_distance=5_000.0)
        continuation = items([2, 4, 5, 6, 7], start_distance=4_700.0)
        self.assertEqual(ordered_suffix_prefix_overlap(old, continuation), 3)
        self.assertEqual(
            self.classify(old, continuation),
            NavigationBufferClass.OVERLAPPING_CONTINUATION)

        # The same UID occurrence on another lap has incompatible remaining
        # distances and therefore cannot prove progress through a roundabout.
        another_lap = items([2, 4, 5, 6], start_distance=2_000.0)
        self.assertEqual(ordered_suffix_prefix_overlap(old, another_lap), 0)
        self.assertEqual(
            self.classify(old, another_lap),
            NavigationBufferClass.TRUE_REROUTE)

    def test_real_destination_change_wins_even_with_partial_overlap(self):
        old = items(range(100, 110))
        new = items(range(105, 115), start_distance=49_500.0)
        result = classify_navigation_buffer(
            old, new, old_destination=("request", "A"),
            new_destination=("request", "B"))
        self.assertEqual(result[0], NavigationBufferClass.TRUE_REROUTE)

    def test_reroute_back_to_previous_road_is_not_forward_progress(self):
        old = items([10, 11, 12, 13, 14])
        rerouted = items([11, 12, 90, 10, 11], start_distance=49_500.0)
        self.assertEqual(
            self.classify(old, rerouted),
            NavigationBufferClass.TRUE_REROUTE)

    def test_session_map_and_dataset_changes(self):
        route = items(range(100, 105))
        for old, new in (
                ((1, "map", "data"), (2, "map", "data")),
                ((1, "map", "data"), (1, "other", "data")),
                ((1, "map", "data"), (1, "map", "other"))):
            with self.subTest(new=new):
                self.assertEqual(
                    self.classify(route, route, old_context=old,
                                  new_context=new),
                    NavigationBufferClass.SESSION_OR_DATASET_CHANGED)

    def test_terminal_destination_identity_is_not_truncated_horizon(self):
        self.assertIsNone(destination_identity("", items([1, 2, 3])))
        terminal = [
            {"uid": 1, "distance": 120.0},
            {"uid": 2, "distance": 30.0},
        ]
        self.assertEqual(destination_identity("", terminal),
                         ("terminal_uid", 2))

    def test_all_eight_classifications_are_deterministic(self):
        route = items(range(100, 108))
        cases = (
            (NavigationBufferClass.SAME_EXACT, route, route, {}),
            (NavigationBufferClass.ADVANCED_PREFIX, route, route[2:], {}),
            (NavigationBufferClass.EXTENDED_HORIZON,
             route[:5], route, {}),
            (NavigationBufferClass.OVERLAPPING_CONTINUATION,
             route, items(range(104, 112), start_distance=49_600.0), {}),
            (NavigationBufferClass.TEMPORARILY_UNAVAILABLE,
             route, [], {"destination_present": True}),
            (NavigationBufferClass.TRUE_REROUTE,
             route, items(range(900, 908)), {}),
            (NavigationBufferClass.DESTINATION_REMOVED,
             route, [], {"destination_present": False}),
            (NavigationBufferClass.SESSION_OR_DATASET_CHANGED,
             route, route, {"old_context": (1, "m", "d"),
                            "new_context": (2, "m", "d")}),
        )
        for expected, old, new, kwargs in cases:
            with self.subTest(expected=expected):
                results = [classify_navigation_buffer(
                    old, new, **kwargs) for _ in range(25)]
                self.assertTrue(all(result == results[0]
                                    for result in results))
                self.assertEqual(results[0][0], expected)

    def test_large_shared_prefix_and_same_destination_is_still_reroute(self):
        old = items(range(100, 120))
        rerouted = old[:17] + items([900, 901, 902], start_distance=48_300.0)
        result = classify_navigation_buffer(
            old, rerouted, old_destination=("city", "berlin"),
            new_destination=("city", "berlin"))
        self.assertEqual(result[0], NavigationBufferClass.TRUE_REROUTE)

    def test_same_destination_different_road_is_reroute(self):
        result = classify_navigation_buffer(
            items([10, 11, 12, 13, 14]),
            items([10, 11, 80, 81, 14]),
            old_destination=("city", "praha"),
            new_destination=("city", "praha"))
        self.assertEqual(result[0], NavigationBufferClass.TRUE_REROUTE)

    def test_multiple_roundabout_passes_need_matching_occurrence_progress(self):
        first_lap = items(
            [10, 20, 30, 20, 30, 40, 50], start_distance=8_000.0)
        next_window = items(
            [20, 30, 40, 50, 60], start_distance=7_700.0)
        other_lap = items(
            [20, 30, 40, 50, 60], start_distance=4_000.0)
        self.assertEqual(
            self.classify(first_lap, next_window),
            NavigationBufferClass.OVERLAPPING_CONTINUATION)
        self.assertEqual(
            self.classify(first_lap, other_lap),
            NavigationBufferClass.TRUE_REROUTE)


class NavigationIntentStateMachineTests(unittest.TestCase):
    def test_one_hundred_rolling_shifts_keep_one_intent(self):
        full_route = list(range(1_000, 1_140))
        tracker = NavigationIntentTracker()
        first = tracker.update(
            items(full_route[:20]), destination_present=True,
            context=(7, "promods-1.59", "fingerprint"))
        intent = first.intent_id
        intent_changes = 1
        full_builds = 1
        new_destination_events_during_progress = 0

        # Production refreshes geometry only when its validated covered UID
        # horizon is nearly consumed. This model uses the same one-third rule.
        covered_until = 60
        for offset in range(1, 101):
            window = items(
                full_route[offset:offset + 20],
                start_distance=50_000.0 - offset * 100.0)
            decision = tracker.update(
                window, destination_present=True,
                context=(7, "promods-1.59", "fingerprint"))
            self.assertEqual(decision.intent_id, intent)
            self.assertFalse(decision.intent_changed)
            self.assertIn(decision.classification, {
                NavigationBufferClass.OVERLAPPING_CONTINUATION,
                NavigationBufferClass.ADVANCED_PREFIX,
            })
            if covered_until - offset <= 20:
                full_builds += 1
                covered_until = min(len(full_route), offset + 60)
            intent_changes += int(decision.intent_changed)
            new_destination_events_during_progress += int(
                decision.intent_changed)

        self.assertEqual(intent_changes, 1)
        self.assertEqual(full_builds, 3)
        self.assertEqual(new_destination_events_during_progress, 0)

        reroute = tracker.update(
            items([full_route[100], 9_001, 9_002, 9_003]),
            destination_present=True, destination=("request", "new"),
            context=(7, "promods-1.59", "fingerprint"))
        self.assertEqual(reroute.classification,
                         NavigationBufferClass.TRUE_REROUTE)
        self.assertTrue(reroute.intent_changed)
        self.assertNotEqual(reroute.intent_id, intent)
        actual_destination_builds = 1 if reroute.intent_changed else 0
        self.assertEqual(actual_destination_builds, 1)
        full_builds += actual_destination_builds
        self.assertEqual(len({intent, reroute.intent_id}), 2)
        self.assertEqual(full_builds, 4)

    def test_temporary_unavailability_does_not_destroy_progress_state(self):
        tracker = NavigationIntentTracker()
        route = items(range(100, 110))
        initial = tracker.update(route, destination_present=True)
        missing = tracker.update([], destination_present=True)
        restored = tracker.update(route[2:], destination_present=True)
        self.assertEqual(missing.classification,
                         NavigationBufferClass.TEMPORARILY_UNAVAILABLE)
        self.assertEqual(restored.classification,
                         NavigationBufferClass.ADVANCED_PREFIX)
        self.assertEqual(initial.intent_id, missing.intent_id)
        self.assertEqual(initial.intent_id, restored.intent_id)

    def test_destination_removal_clears_intent(self):
        tracker = NavigationIntentTracker()
        tracker.update(items(range(100, 105)), destination_present=True)
        removed = tracker.update([], destination_present=False)
        self.assertEqual(removed.classification,
                         NavigationBufferClass.DESTINATION_REMOVED)
        self.assertIsNone(removed.intent_id)

    def test_target_change_during_temporary_empty_buffer_changes_intent_once(self):
        tracker = NavigationIntentTracker()
        initial = tracker.update(
            items(range(100, 110)), destination_present=True,
            destination=("request", "A"), context=(1, "map", "data"))
        unavailable = tracker.update(
            [], destination_present=True, destination=("request", "A"),
            context=(1, "map", "data"))
        changed = tracker.update(
            [], destination_present=True, destination=("request", "B"),
            context=(1, "map", "data"))
        restored = tracker.update(
            items(range(900, 910)), destination_present=True,
            destination=("request", "B"), context=(1, "map", "data"))

        self.assertEqual(unavailable.classification,
                         NavigationBufferClass.TEMPORARILY_UNAVAILABLE)
        self.assertEqual(changed.classification,
                         NavigationBufferClass.TRUE_REROUTE)
        self.assertNotEqual(changed.intent_id, initial.intent_id)
        self.assertEqual(restored.classification,
                         NavigationBufferClass.EXTENDED_HORIZON)
        self.assertEqual(restored.intent_id, changed.intent_id)
        self.assertFalse(restored.intent_changed)


class NavigationBuildGuardTests(unittest.TestCase):
    def test_duplicate_parallel_and_reverse_completion_are_rejected(self):
        guard = NavigationBuildGuard()
        first = guard.begin("intent", (1, 2), "build-old")
        self.assertIsNotNone(first)
        self.assertIsNone(guard.begin("intent", (1, 2), "duplicate"))
        self.assertIsNone(guard.begin("intent", (2, 3), "parallel"))

        guard.finish(first)
        newer = guard.begin("intent", (2, 3), "build-new")
        self.assertIsNotNone(newer)
        self.assertFalse(guard.may_publish(first))
        self.assertTrue(guard.may_publish(newer))

        # A late callback from the old build cannot evict or overwrite newer.
        guard.finish(first)
        self.assertTrue(guard.may_publish(newer))
        guard.finish(newer)
        self.assertFalse(guard.may_publish(newer))
        self.assertTrue(guard.input_completed("intent", (1, 2)))
        self.assertTrue(guard.input_completed("intent", (2, 3)))
        self.assertIsNone(guard.begin("intent", (1, 2), "repeat"))


if __name__ == "__main__":
    unittest.main()
