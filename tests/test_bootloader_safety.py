from bootloader import RestartGuard


def test_restart_guard_stops_tight_crash_loop():
    guard = RestartGuard(max_crashes=3, window_seconds=20)
    assert [guard.allow_restart("Engine", now=value)
            for value in (0, 1, 2, 3)] == [True, True, True, False]


def test_restart_guard_recovers_after_stable_window_and_is_per_process():
    guard = RestartGuard(max_crashes=2, window_seconds=10)
    assert guard.allow_restart("Engine", now=0)
    assert guard.allow_restart("Engine", now=1)
    assert not guard.allow_restart("Engine", now=2)
    assert guard.allow_restart("HUD", now=2)
    assert guard.allow_restart("Engine", now=20)
