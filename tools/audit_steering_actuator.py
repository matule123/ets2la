"""Read-only identification of the SCS steering actuator from dense replays.

The tool aligns the first application sample for every new SDK frame. It never
fills missing frames, modifies settings, or feeds estimates back at runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys


ROOT = Path(__file__).resolve().parents[1]


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _summary(values):
    values = sorted(float(value) for value in values if _finite(value))
    if not values:
        return {"count": 0}
    quantiles = statistics.quantiles(values, n=20, method="inclusive")
    return {
        "count": len(values),
        "minimum": values[0],
        "p10": quantiles[1],
        "median": statistics.median(values),
        "p90": quantiles[17],
        "p95": quantiles[18],
        "maximum": values[-1],
    }


def _aligned_rows(document):
    rows = []
    observed_frames = set()
    for sample in document.get("samples", []):
        frame = sample.get("application_sdk_frame_us")
        wheel_values = [float(value) for value in
                        (sample.get("tyre_angles_rad") or [])
                        if _finite(value)]
        required = (frame, sample.get("engine_applied_steering"),
                    sample.get("game_steer_right"),
                    sample.get("speed_kmh"))
        if (frame in observed_frames or not wheel_values
                or not sample.get("autopilot_active", False)
                or not sample.get("packet_binding_valid", False)
                or not all(_finite(value) for value in required)):
            continue
        observed_frames.add(frame)
        rows.append({
            "frame_s": float(frame) / 1_000_000.0,
            "command": float(sample["engine_applied_steering"]),
            "game": float(sample["game_steer_right"]),
            "tyre_rad": statistics.fmean(wheel_values),
            "speed_kmh": float(sample["speed_kmh"]),
            "packet_age_s": sample.get("packet_age_s"),
            "frame_lag_s": (float(sample["frame_lag_us"]) / 1_000_000.0
                            if _finite(sample.get("frame_lag_us")) else None),
            "yaw_right_rad_s": sample.get("yaw_right_rad_s"),
        })
    return rows


def analyze_documents(documents, *, wheelbase_m=3.8):
    identities = [document.get("identity", {}) or {}
                  for document in documents]
    common = {key: {identity.get(key) for identity in identities}
              for key in ("source_game_session_id", "source_map_key",
                          "source_dataset_fingerprint")}
    for key in ("source_map_key", "source_dataset_fingerprint"):
        if len(common[key]) != 1:
            raise ValueError(f"replays do not share {key}")

    groups = []
    combined = []
    for document, identity in zip(documents, identities):
        rows = _aligned_rows(document)
        combined.extend(rows)
        groups.append({
            "identity": identity,
            "raw_sample_count": len(document.get("samples", [])),
            "unique_active_sdk_frames": len(rows),
        })
    frame_intervals = []
    tyre_per_game = []
    tyre_per_game_left = []
    tyre_per_game_right = []
    yaw_to_kinematic = []
    packet_ages = []
    frame_lags = []
    speed_bins = {"0_20": [], "20_30": [], "30_40": [],
                  "40_50": [], "50_65": [], "65_95": []}
    yaw_speed_bins = {key: [] for key in speed_bins}
    lag_errors = {lag: [] for lag in range(5)}
    dynamic_lag_errors = {lag: [] for lag in range(5)}

    for document in documents:
        rows = _aligned_rows(document)
        for index, row in enumerate(rows):
            game = row["game"]
            if abs(game) >= 0.05:
                ratio = row["tyre_rad"] / game
                tyre_per_game.append(ratio)
                (tyre_per_game_right if game > 0.0 else
                 tyre_per_game_left).append(ratio)
                speed = row["speed_kmh"]
                for label, low, high in (
                        ("0_20", 0, 20), ("20_30", 20, 30),
                        ("30_40", 30, 40), ("40_50", 40, 50),
                        ("50_65", 50, 65), ("65_95", 65, 95)):
                    if low <= speed < high:
                        speed_bins[label].append(ratio)
                        break
            speed_ms = row["speed_kmh"] / 3.6
            yaw = row["yaw_right_rad_s"]
            tyre = row["tyre_rad"]
            if (_finite(yaw) and speed_ms > 3.0 and abs(tyre) > 0.02):
                kinematic = math.tan(tyre) / float(wheelbase_m)
                observed = float(yaw) / speed_ms
                if abs(kinematic) > 1e-6 and observed * kinematic > 0.0:
                    yaw_ratio = observed / kinematic
                    yaw_to_kinematic.append(yaw_ratio)
                    speed = row["speed_kmh"]
                    for label, low, high in (
                            ("0_20", 0, 20), ("20_30", 20, 30),
                            ("30_40", 30, 40), ("40_50", 40, 50),
                            ("50_65", 50, 65), ("65_95", 65, 95)):
                        if low <= speed < high:
                            yaw_speed_bins[label].append(yaw_ratio)
                            break
            if _finite(row["packet_age_s"]):
                packet_ages.append(row["packet_age_s"])
            if _finite(row["frame_lag_s"]):
                frame_lags.append(row["frame_lag_s"])
            if index:
                dt = row["frame_s"] - rows[index - 1]["frame_s"]
                if 0.0 < dt < 0.2:
                    frame_intervals.append(dt)
            for lag in lag_errors:
                if index < max(1, lag):
                    continue
                if row["frame_s"] - rows[index - 1]["frame_s"] >= 0.2:
                    continue
                error = row["game"] - rows[index - lag]["command"]
                lag_errors[lag].append(error)
                previous_command = rows[index - lag - 1]["command"]
                if abs(rows[index - lag]["command"] - previous_command) > .002:
                    dynamic_lag_errors[lag].append(error)

    def rmse(values):
        return (math.sqrt(statistics.fmean(value * value for value in values))
                if values else None)

    lag_fit = {
        str(lag): {
            "rmse_input": rmse(lag_errors[lag]),
            "dynamic_rmse_input": rmse(dynamic_lag_errors[lag]),
            "count": len(lag_errors[lag]),
            "dynamic_count": len(dynamic_lag_errors[lag]),
        }
        for lag in lag_errors
    }
    best_lag = min(lag_errors, key=lambda lag: (
        float("inf") if not dynamic_lag_errors[lag]
        else rmse(dynamic_lag_errors[lag])))
    frame_summary = _summary(frame_intervals)
    command_delay = (best_lag * frame_summary.get("median", 0.0))
    observation_summary = _summary(frame_lags)
    observation_delay = observation_summary.get("median", 0.0)
    return {
        "groups": groups,
        "common_identity": {key: list(values)[0] if len(values) == 1
                            else sorted(str(value) for value in values)
                            for key, values in common.items()},
        "unique_active_sdk_frames": len(combined),
        "frame_interval_s": frame_summary,
        "packet_age_s": _summary(packet_ages),
        "calculation_to_application_frame_lag_s": observation_summary,
        "command_to_game_lag_fit": lag_fit,
        "best_command_lag_frames": best_lag,
        "identified_command_delay_s": command_delay,
        "identified_observation_delay_s": observation_delay,
        "identified_preview_horizon_s": command_delay + observation_delay,
        "tyre_rad_per_game_input": _summary(tyre_per_game),
        "tyre_rad_per_game_input_left": _summary(tyre_per_game_left),
        "tyre_rad_per_game_input_right": _summary(tyre_per_game_right),
        "tyre_rad_per_game_input_by_speed_kmh": {
            key: _summary(values) for key, values in speed_bins.items()},
        "yaw_curvature_to_bicycle_curvature": _summary(yaw_to_kinematic),
        "yaw_curvature_to_bicycle_curvature_by_speed_kmh": {
            key: _summary(values) for key, values in yaw_speed_bins.items()},
        "limitations": [
            "Values apply to the recorded truck/input setup, not every chassis.",
            "The SDK frame rate bounds sub-frame delay; missing ticks are not interpolated.",
            "Yaw includes slip and is diagnostic, never online calibration authority.",
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replays", nargs="+", type=Path)
    parser.add_argument("--wheelbase-m", type=float, default=3.8)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    documents = []
    hashes = []
    for path in args.replays:
        payload = path.read_bytes()
        documents.append(json.loads(payload.decode("utf-8")))
        hashes.append({"path": str(path),
                       "sha256": hashlib.sha256(payload).hexdigest()})
    result = analyze_documents(documents, wheelbase_m=args.wheelbase_m)
    result["inputs"] = hashes
    encoded = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output is not None:
        output = args.output.resolve()
        if not output.is_relative_to(ROOT):
            parser.error("output must stay in the UltraPilot workspace")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    return 0


if __name__ == "__main__":
    sys.exit(main())
