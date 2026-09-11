"""Compare Phase 4B/4C on the same independent plant and 2 m GPS geometry."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core.navigation.route import Route
from tests.steering_bench import path, run
from tools.run_steering_bench import factories_for_ref, real_speed


def gps_factory(base):
    # Keep an explicit production steering signature: run() discovers the
    # supported calibration arguments by signature, even for old git refs.
    import functools
    class Adapted(base):
        def __init__(self, points):
            authority = ((1, 1, 0, '', -1, ()), 50)
            base.__init__(self, points, point_authorities=[authority]*len(points))
            self.bench_authority = authority

        @functools.wraps(base.steering)
        def steering(self, *args, **kwargs):
            authority = dict(kwargs.get('control_authority') or {})
            authority.update(lane_identity=self.bench_authority[0], elevation_layer=50)
            kwargs['control_authority'] = authority
            return base.steering(self, *args, **kwargs)
    return Adapted


def cases():
    for kmh in (10, 30, 60, 90):
        yield f'straight_{kmh}', path([(0., 150.), (0., 150.)], step=2.), dict(
            speed=kmh/3.6, initial_cte=.5)
    for direction in (-1, 1):
        for radius, speed in ((18, 5.4), (35, 7.4), (83, 12.), (250, 25.)):
            yield f'R{radius}_{direction:+d}', path([
                (0., 55.), (direction/radius, max(100., radius*1.2)),
                (0., 180.)], step=2.), dict(speed=speed)
        yield f'S35_{direction:+d}', path([
            (0., 55.), (direction/35, 75.), (-direction/35, 75.),
            (0., 180.)], step=2.), dict(speed=7.4)
        yield f'roundabout_R25_{direction:+d}', path([
            (0., 55.), (direction/25, 25*math.tau),
            (0., 180.)], step=2.), dict(speed=6.)
    yield 'log_speed_R22', path([(0., 35.), (.045, 110.), (0., 180.)], step=2.), dict(
        speed_profile=real_speed())


def detailed_metrics(metrics, rows):
    result = dict(metrics)
    result['rejected_control_samples'] = sum(not r['authority_valid'] for r in rows)
    result['max_body_tracking_error_deg'] = math.degrees(max(
        abs(r['body_tracking_error_rad'] or 0.) for r in rows))
    result['monotone_output_sign_changes'] = sum(
        a['geometry_monotone'] and b['geometry_monotone']
        and a['out']*b['out'] < 0.
        for a, b in zip(rows, rows[1:]))
    middle = [r for r in rows if r['geometry_monotone']]
    if middle:
        result['monotone_rms_cte'] = math.sqrt(statistics.fmean(r['cte']**2 for r in middle))
        result['monotone_max_cte'] = max(abs(r['cte']) for r in middle)
        differences = []
        # Only genuinely adjacent ticks entirely inside a monotone footprint.
        for a, b in zip(rows, rows[1:]):
            if a['geometry_monotone'] and b['geometry_monotone']:
                differences.append((b['raw']-a['raw'])/b['dt'])
        result['monotone_raw_rate_rms'] = math.sqrt(statistics.fmean(
            d*d for d in differences)) if differences else 0.
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ref', default='7f3d365')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if not args.output.resolve().is_relative_to(ROOT):
        parser.error('output must stay in the repository')
    before_route, before_dynamics, hashes = factories_for_ref(args.ref)
    factories = {
        'before': dict(route_factory=gps_factory(before_route), dynamics_factory=before_dynamics),
        'after': dict(route_factory=gps_factory(Route)),
    }
    import hashlib
    production_hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in (ROOT/'core/navigation/route.py',
                                  ROOT/'core/lateral_controller.py',
                                  ROOT/'core/steering_dynamics.py')}
    result = dict(baseline_ref=args.ref, baseline_hashes=hashes,
                  production_hashes=production_hashes, path_step_m=2.,
                  oracle_window_m=[5., 45.], integration_step_max_s=.005,
                  controller_preview_s=.134, cases={}, calibrated_matrix={})
    for label, lag, transport in (('identified', .01, .067),
                                   ('moderate', .25, .10), ('slow', .60, .10)):
        for trailer in (False, True):
            mode = label+('_trailer' if trailer else '_cab')
            result['cases'][mode] = {}
            for name, data, options in cases():
                pair = {}
                for version, factory in factories.items():
                    metrics, rows = run(data, controller_lock_rad=.70,
                        controller_preview_s=.134, observation_ahead_m=2.1,
                        oracle_window_m=(5., 45.),
                        controller_ahead_m=2.1, lock_rad=.70,
                        lag=lag, transport=transport, noisy=True, jitter=True,
                        trailer=trailer, **options, **factory)
                    pair[version] = detailed_metrics(metrics, rows)
                result['cases'][mode][name] = pair
            print(mode, 'complete', flush=True)
    for gain in (.60, .95):
        for lag in (.25, .52):
            key = f'gain{gain}_lag{lag}+load0.08'
            result['calibrated_matrix'][key] = {}
            for name, data, options in cases():
                pair = {}
                for version, factory in factories.items():
                    metrics, rows = run(data, controller_lock_rad=gain,
                        controller_preview_s=.134, observation_ahead_m=2.1,
                        oracle_window_m=(5., 45.),
                        controller_ahead_m=2.1, lock_rad=gain, lag=lag,
                        transport=.10, noisy=True, jitter=True, trailer=True,
                        load=1., **options, **factory)
                    pair[version] = detailed_metrics(metrics, rows)
                result['calibrated_matrix'][key][name] = pair
            print(key, 'complete', flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(args.output)


if __name__ == '__main__':
    main()
