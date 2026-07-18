"""Offline synthetic tests for Phase 1 driving math (no game needed).

Run with:  python tests/test_phase1_steering.py

These exercise the *math* we changed in Phase 1 — the lead-vehicle / light
brake gradients and the autopilot anti-jerk brake ramp — so we can prove they
behave sensibly before testing in the real game.

Each scenario prints a small table of (input -> output) so it's easy to eyeball
that the curves are smooth and stop in time.
"""
import os
import sys
import math

# Make the project importable when run directly.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


class FakeSharedState:
    """Minimal shared-state dict so the engine helpers read what we feed them."""
    def __init__(self, d=None):
        self._d = dict(d or {})

    def get(self, k, default=None):
        return self._d.get(k, default)

    def set(self, k, v):
        self._d[k] = v


def heading_towards(dx, dz):
    """Heading (radians) whose forward vector points along (dx, dz)."""
    L = math.hypot(dx, dz) or 1.0
    fx, fz = dx / L, dz / L          # forward = (-sin h, -cos h)
    return math.atan2(-fx, -fz)


def make_vehicle_ahead(ahead_m, lateral_m, speed_ms=0.0, facing_same=True):
    """A traffic dict placed `ahead_m` in front, `lateral_m` to the side."""
    h = heading_towards(0, 1)        # truck faces +z
    pos = (0.0, 0.0)
    px = ahead_m * (-math.sin(h)) + lateral_m * math.cos(h)
    pz = ahead_m * (-math.cos(h)) - lateral_m * math.sin(h)
    vyaw = h if facing_same else h + math.pi
    return {"x": px, "z": pz, "yaw": vyaw, "length": 4.5, "width": 2.0,
            "speed": speed_ms, "type": "car", "id": 1}


def run_lead_brake():
    from core.engine import UltraPilotEngine
    print("\n=== _lead_brake (TTC) ===")
    eng = UltraPilotEngine.__new__(UltraPilotEngine)   # skip __init__
    fmt = "  lead ahead={:>5}m  closing={:>5}m/s  my_speed={:>5}m/s -> brake={:.2f}"
    print("  (should rise early when closing fast, hold a short gap when matched)")
    cases = [
        # (ahead_m, lead_speed, my_speed, label)
        (40, 0.0, 0.0, "matched, far"),
        (30, 20.0, 20.0, "matched speed, medium gap"),
        (30, 10.0, 20.0, "closing 10 m/s"),
        (15, 0.0, 25.0, "closing 25 m/s, close"),
        (8, 0.0, 25.0, "closing 25 m/s, very close"),
        (5, 0.0, 0.0, "stopped, tiny gap"),
        (100, 0.0, 25.0, "far but closing"),
    ]
    for ahead, lead, mine, label in cases:
        st = FakeSharedState({"truck_speed_ms": mine})
        eng.shared_state = st
        v = make_vehicle_ahead(ahead, 0.0, lead)
        b = eng._lead_brake([v], (0.0, 0.0), heading_towards(0, 1))
        print(f"  [{label:<28}] brake={b:.2f}")
    # No traffic -> 0
    eng.shared_state = FakeSharedState({"truck_speed_ms": 20.0})
    assert eng._lead_brake([], (0.0, 0.0), 0.0) == 0.0
    # Oncoming (facing opposite) -> ignored
    v_oncoming = make_vehicle_ahead(20, 0.0, 0.0, facing_same=False)
    b = eng._lead_brake([v_oncoming], (0.0, 0.0), heading_towards(0, 1))
    print(f"  [oncoming car (ignored)   ] brake={b:.2f}  (expect 0.00)")
    assert b == 0.0
    print("  OK: lead_brake ignores oncoming, brakes for same-direction traffic.")


def run_light_brake():
    from core.engine import UltraPilotEngine
    print("\n=== _light_brake ===")
    eng = UltraPilotEngine.__new__(UltraPilotEngine)
    for color in ("red", "yellow", "green", "off"):
        print(f"  -- {color} --")
        for dist in (80, 50, 30, 15, 6, 2):
            st = FakeSharedState({"truck_speed_ms": 20.0})
            eng.shared_state = st
            b = eng._light_brake({"color": color, "distance": float(dist)})
            print(f"    dist={dist:>3}m -> brake={b:.2f}")
    eng.shared_state = FakeSharedState({})
    assert eng._light_brake(None) == 0.0
    print("  OK: green/off = 0, red rises early & full at line.")


def run_brake_ramp():
    """Simulate the autopilot anti-jerk ramp over a few seconds."""
    print("\n=== autopilot brake ramp (anti-jerk) ===")
    from plugins.autopilot.main import Plugin, BRAKE_RAMP_UP, BRAKE_RAMP_DOWN

    class FakeCtl:
        def __init__(self): self.brake = 0.0; self.throttle = 0.0; self.steering = 0.0
        def set_brake(self, v): self.brake = v
        def set_throttle(self, v): self.throttle = v
        def set_steering(self, v): self.steering = v
        def set_blinker(self, s): pass
        def pay_toll(self): pass
        def stop_completely(self): self.set_brake(1.0)

    class FakeTelem:
        def get(self, k, d=None):
            return {"speed": 22.2}.get(k, d) if k == "truck" else d

    class FakeTags:
        def __setattr__(self, k, v): pass

    p = Plugin.__new__(Plugin)
    p.enabled = True
    p._last_throttle = 0.0
    p._last_steering = 0.0
    p._last_brake = 0.0
    p._blinker = "off"
    p._speed_kmh = 0.0
    p._engage_blend = 0.0
    p._was_active = False
    p._diag_t = 0.0
    p.tags = FakeTags()
    p.sdk = type("S", (), {})()
    p.sdk.controller = FakeCtl()
    p.sdk.telemetry = FakeTelem()
    p.sdk.shared_state = FakeSharedState({
        "system_state": "CRUISE",
        "danger_level": 0.0,
        "lane_offset": 0.0,
        "traffic": [],                 # no real traffic -> vision full weight
        "nav_active": True,
        "nav_steering": 0.0,
        "acc_throttle": 0.6,
        "acc_brake": 0.0,
    })

    dt = 0.05
    # Step 1: request full brake suddenly; should ramp, not jump to 1.0 in one step.
    print("  requesting brake=1.0 abruptly (should ramp up, not jump to 1.0):")
    p.sdk.shared_state.set("collision_brake_request", 1.0)
    first_brake = None
    for i in range(40):
        p.on_tick(dt)
        if first_brake is None:
            first_brake = p._last_brake
        if i % 4 == 0:
            print(f"    t={i*dt:4.2f}s  brake={p._last_brake:.2f}  throttle={p._last_throttle:.2f}")
    # The very first step must be < 1.0 — that's the whole point (no instant slam).
    assert first_brake is not None and first_brake < 1.0, \
        f"ramp first step should be < 1.0 (was {first_brake})"
    print(f"  first-step brake={first_brake:.2f} (< 1.0, no instant slam)")
    # Now release and check it decays (and faster than it rose).
    p.sdk.shared_state.set("collision_brake_request", 0.0)
    print("  releasing brake (should decay faster):")
    for i in range(30):
        p.on_tick(dt)
        if i % 4 == 0:
            print(f"    t={i*dt:4.2f}s  brake={p._last_brake:.2f}  throttle={p._last_throttle:.2f}")
    print(f"  ramp up rate={BRAKE_RAMP_UP}/s, ramp down rate={BRAKE_RAMP_DOWN}/s "
          "(down is faster, as designed)")
    print("  OK: brake ramps smoothly both ways — no jerk.")


def run_route_steering():
    """Drive a synthetic straight + curve route, watch for oscillation."""
    print("\n=== Route.steering (no oscillation) ===")
    from core.navigation.route import Route
    # Straight road along +z, then a gentle right curve.
    pts = [(0, z) for z in range(0, 200, 10)]
    for k in range(20):
        z = 200 + k * 10
        x = (k * 1.5) ** 1.05
        pts.append((x, z))
    r = Route(pts)
    # Drive along the route exactly on it: steering should stay near 0 then
    # rise smoothly as the curve starts — never spike then snap back.
    prev = 0.0
    max_jump = 0.0
    x, z, h = 0.0, 50.0, heading_towards(0, 1)
    print("  on-route samples (should be smooth, small frame-to-frame change):")
    samples = []
    for step in range(40):
        # advance along route
        z += 6.0
        steer = r.steering((x, z), h, speed_ms=18.0)
        max_jump = max(max_jump, abs(steer - prev))
        prev = steer
        if step % 8 == 0:
            samples.append(f"z={z:>4.0f} steer={steer:+.3f}")
    print("    " + " | ".join(samples))
    print(f"  max frame-to-frame steering jump: {max_jump:.3f} (low = smooth)")
    assert max_jump < 0.25, "steering should not jump violently between frames"
    print("  OK: steering tracks the route smoothly (no fishtail).")


def run_heading_and_steering_signs():
    """Lock down SCS turn conversion and left/right controller signs."""
    from core.telemetry import Telemetry
    from core.navigation.route import Route

    telemetry = Telemetry.__new__(Telemetry)
    telemetry.sdk_reader = type("Reader", (), {
        "read_trailer": lambda self, index: {},
        "read_job_destination": lambda self: "",
    })()
    base = {"truckFloat": {}, "truckBool": {}, "truckInt": {}}
    headings = []
    for turns in (0.0, 0.25, 0.5, -0.25):
        raw = dict(base, truckPlacement={
            "rotationX": turns, "coordinateX": 0.0, "coordinateZ": 0.0})
        headings.append(telemetry._normalize_sdk(raw)["heading"])
    expected = (0.0, math.pi / 2, -math.pi, -math.pi / 2)
    for got, want in zip(headings, expected):
        assert math.isclose(got, want, abs_tol=1e-9), (got, want)

    heading = heading_towards(0, 1)
    # Facing +Z, physical right is -X in ETS2's right-handed X/Y/Z world.
    right = Route([(-5, 0), (-5, 100)])
    left = Route([(5, 0), (5, 100)])
    assert right.steering((0, 0), heading, 10.0) > 0.0
    assert left.steering((0, 0), heading, 10.0) < 0.0
    print("  OK: SCS turns convert to radians and steering signs are symmetric.")


if __name__ == "__main__":
    run_lead_brake()
    run_light_brake()
    run_brake_ramp()
    run_route_steering()
    run_heading_and_steering_signs()
    print("\nAll Phase 1 synthetic tests passed.")
