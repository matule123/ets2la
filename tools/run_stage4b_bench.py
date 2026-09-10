"""Phase 4B before/after benchmark with an independent nonlinear plant."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from steering_bench import path, run  # noqa: E402


OLD_COUPLED_PREVIEW_S = 0.45
PHASE4B_IDENTIFIED_PREVIEW_S = 0.134


def cases():
    for speed_kmh in (10, 30, 60, 90):
        yield (f"straight_{speed_kmh}",
               path([(0.0, 150.0), (0.0, 150.0)]),
               {"speed": speed_kmh / 3.6, "initial_cte": 0.5})
    for direction in (-1.0, 1.0):
        for radius, speed in ((250.0, 25.0), (83.0, 12.0),
                              (35.0, 7.4), (18.0, 5.4)):
            yield (f"R{int(radius)}_{int(direction):+d}",
                   path([(0.0, 55.0),
                         (direction / radius, max(100.0, radius * 1.2)),
                         (0.0, 100.0)]), {"speed": speed})
    yield ("S35", path([(0.0, 55.0), (-1.0 / 35.0, 75.0),
                         (1.0 / 35.0, 75.0), (0.0, 100.0)]),
           {"speed": 7.4})
    yield ("roundabout_R25",
           path([(0.0, 55.0), (-1.0 / 25.0, 25.0 * 6.283185307179586),
                 (0.0, 100.0)]), {"speed": 6.0})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if not output.is_relative_to(ROOT):
        parser.error("output must stay in the UltraPilot workspace")

    plants = {
        "identified_fast": {"lock_rad": 0.70, "lag": 0.01,
                            "transport": 0.067},
        "moderate_stress": {"lock_rad": 0.70, "lag": 0.25,
                            "transport": 0.10},
        "slow_stress": {"lock_rad": 0.70, "lag": 0.60,
                        "transport": 0.10},
    }
    result = {
        "plant_is_independent": True,
        "controller_lock_rad": 0.70,
        "reference_ahead_m": 2.1,
        "old_coupled_preview_s": OLD_COUPLED_PREVIEW_S,
        "phase4b_identified_preview_s": PHASE4B_IDENTIFIED_PREVIEW_S,
        "plants": plants,
        "cases": {},
        "gain_robustness": {},
    }
    for plant_name, plant in plants.items():
        result["cases"][plant_name] = {}
        for trailer in (False, True):
            mode = "trailer" if trailer else "cab"
            result["cases"][plant_name][mode] = {}
            for name, geometry, options in cases():
                row = {}
                for label, preview in (("before", OLD_COUPLED_PREVIEW_S),
                                       ("after",
                                        PHASE4B_IDENTIFIED_PREVIEW_S)):
                    metrics, _ = run(
                        geometry, controller_lock_rad=0.70,
                        controller_preview_s=preview,
                        noisy=True, jitter=True, trailer=trailer,
                        observation_ahead_m=2.1, controller_ahead_m=2.1,
                        **plant, **options)
                    row[label] = metrics
                result["cases"][plant_name][mode][name] = row

    # Calibration mismatch is visible, not adapted away. This matrix uses the
    # same identified timing and a fixed configured 0.70 rad/input.
    for gain in (0.60, 0.70, 0.95):
        key = f"plant_gain_{gain:.2f}"
        result["gain_robustness"][key] = {}
        for name, geometry, options in cases():
            metrics, _ = run(
                geometry, controller_lock_rad=0.70,
                controller_preview_s=PHASE4B_IDENTIFIED_PREVIEW_S,
                lock_rad=gain, lag=0.01, transport=0.067,
                noisy=True, jitter=True, trailer=False,
                observation_ahead_m=2.1, controller_ahead_m=2.1,
                **options)
            result["gain_robustness"][key][name] = metrics

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
