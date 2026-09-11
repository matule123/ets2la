"""Read-only Phase 4D cadence audit and fixed-clock counterfactual.

The counterfactual never invents Route targets: it replays each command only
when that calculation first reached the autopilot and holds that exact target
until the next proven packet.  Only the existing SteeringDynamics integration
clock changes from the recorded plugin cadence to 60 Hz.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core.steering_dynamics import SteeringDynamics
from core.steering_executor import STEERING_EXECUTION_HZ


def _finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def _distribution(values):
    values = sorted(float(value) for value in values
                    if value is not None and math.isfinite(float(value)))
    if not values:
        return {"count": 0}

    def percentile(fraction):
        index = min(len(values) - 1,
                    max(0, int(math.ceil(len(values) * fraction)) - 1))
        return values[index]

    return {
        "count": len(values),
        "minimum": values[0],
        "median": statistics.median(values),
        "p95": percentile(0.95),
        "p99": percentile(0.99),
        "maximum": values[-1],
        "mean": statistics.fmean(values),
    }


def _first_packet_arrivals(samples):
    packets = []
    seen = set()
    for row in samples:
        if not row.get("autopilot_active") or not row.get(
                "packet_binding_valid", False):
            continue
        sequence = row.get("calculation_sequence")
        frame = row.get("calculation_sdk_frame_us")
        if sequence is None:
            continue
        try:
            numeric_frame = int(frame or 0)
        except (TypeError, ValueError, OverflowError):
            numeric_frame = 0
        observation_key = (("frame", numeric_frame) if numeric_frame > 0
                           else ("sequence", int(sequence)))
        if observation_key in seen:
            continue
        target = _finite(row.get("controller_steer_raw"))
        timestamp = _finite(row.get("application_monotonic_s"))
        if target is None or timestamp is None:
            continue
        seen.add(observation_key)
        packets.append({
            "time": timestamp,
            "sequence": int(sequence),
            "frame": frame,
            "target": target,
            "speed_ms": abs(_finite(row.get("speed_kmh")) or 0.0) / 3.6,
            "curvature": _finite(row.get("local_k_per_m")) or 0.0,
        })
    return packets


def _recorded_output_metrics(samples):
    rows = []
    for row in samples:
        timestamp = _finite(row.get("application_monotonic_s"))
        output = _finite(row.get("engine_applied_steering"))
        if (timestamp is not None and output is not None
                and row.get("autopilot_active")):
            rows.append((timestamp, output))
    intervals = [b[0] - a[0] for a, b in zip(rows, rows[1:])
                 if 0.0 < b[0] - a[0] < 2.0]
    steps = [abs(b[1] - a[1]) for a, b in zip(rows, rows[1:])]
    return {
        "samples": len(rows),
        "write_interval_s": _distribution(intervals),
        "absolute_step": _distribution(steps),
    }


def _fixed_clock_counterfactual(packets, hz=STEERING_EXECUTION_HZ):
    if len(packets) < 2:
        return {"samples": 0}
    dt = 1.0 / float(hz)
    dynamics = SteeringDynamics()
    start = packets[0]["time"]
    end = packets[-1]["time"]
    packet_index = 0
    target = packets[0]
    now = start
    outputs = []
    rates = []
    accelerations = []
    while now <= end + 1e-12:
        while (packet_index + 1 < len(packets)
               and packets[packet_index + 1]["time"] <= now + 1e-12):
            packet_index += 1
            target = packets[packet_index]
        output = dynamics.update(
            target["target"], dt, speed_ms=target["speed_ms"],
            curvature_per_m=target["curvature"])
        outputs.append(output)
        rates.append(abs(dynamics.last_debug["rate_per_s"]))
        accelerations.append(abs(
            dynamics.last_debug["acceleration_per_s2"]))
        now += dt
    steps = [abs(current - previous)
             for previous, current in zip(outputs, outputs[1:])]
    return {
        "samples": len(outputs),
        "execution_hz": float(hz),
        "write_interval_s": _distribution([dt] * max(0, len(outputs) - 1)),
        "absolute_step": _distribution(steps),
        "absolute_rate_per_s": _distribution(rates),
        "absolute_acceleration_per_s2": _distribution(accelerations),
        "route_target_count": len(packets),
        "invented_route_targets": 0,
    }


def analyze(document):
    samples = list(document.get("samples", ()) or ())
    packets = _first_packet_arrivals(samples)
    calculation_intervals = [
        current["time"] - previous["time"]
        for previous, current in zip(packets, packets[1:])
        if 0.0 < current["time"] - previous["time"] < 2.0]
    frame_gaps = []
    for previous, current in zip(packets, packets[1:]):
        try:
            gap = (int(current["frame"]) - int(previous["frame"])) / 1e6
        except (TypeError, ValueError, OverflowError):
            continue
        if 0.0 < gap < 2.0:
            frame_gaps.append(gap)
    return {
        "schema_version": document.get("schema_version"),
        "identity": document.get("identity", {}),
        "sample_count": len(samples),
        "route_target_count": len(packets),
        "route_target_interval_s": _distribution(calculation_intervals),
        "route_sdk_frame_gap_s": _distribution(frame_gaps),
        "recorded": _recorded_output_metrics(samples),
        "fixed_60hz": _fixed_clock_counterfactual(packets),
        "method": (
            "Exact first-arrival Route targets; no interpolation, smoothing "
            "or synthesized geometry. Existing SteeringDynamics only."),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("replay", type=Path, nargs="+")
    args = parser.parse_args()
    results = []
    for path in args.replay:
        with path.open("r", encoding="utf-8") as stream:
            result = analyze(json.load(stream))
        result["path"] = str(path.resolve())
        results.append(result)
    print(json.dumps(results, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
