"""Same independent origin-aware plant before/after Phase 4A (no game writes)."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tests.steering_bench import run
from tools.run_steering_bench import cases, factories_for_ref, independent_matrix_cases


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ref')
    parser.add_argument('--matrix',action='store_true')
    parser.add_argument('--matched-calibration',action='store_true')
    args=parser.parse_args()
    if not args.output.resolve().is_relative_to(ROOT):
        parser.error('output must stay in workspace')
    route,dynamics,hashes=factories_for_ref(args.ref)
    factories={} if route is None else dict(route_factory=route,dynamics_factory=dynamics)
    results=dict(ref=args.ref or 'working-tree',baseline_hashes=hashes,
        production_hashes=(hashes or {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (ROOT/'core/lateral_controller.py',ROOT/'core/navigation/route.py',
                      ROOT/'core/steering_dynamics.py',ROOT/'core/vehicle_geometry.py')}),
        matrix_calibration=('matched_to_declared_plant' if args.matched_calibration
                            else 'fixed_0.78_with_0.60_to_0.95_plant_sweep'),
        observation_ahead_m=2.1,
        limitation='Analytical paths, independent cab-origin plant. Not a replay of unseen map points; not a trailer clearance proof.',
        cases={},matrix={})
    sets=[('cases',cases())]
    if args.matrix:
        sets.append(('matrix',independent_matrix_cases(args.matched_calibration)))
    for group,scenarios in sets:
        for name,data,options in scenarios:
            measured,rows=run(data,**options,**factories,observation_ahead_m=2.1)
            monotone=[r for r in rows if r['geometry_monotone']]
            measured['monotone_max_cte']=max((abs(r['cte']) for r in monotone),default=0.)
            results[group][name]=measured
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(results,indent=2),encoding='utf-8')
    for name,metrics in results['cases'].items():
        print(name, 'maxCTE',round(metrics['max_cte'],4),'RMS',round(metrics['rms_cte'],4),
              'tail',round(metrics['steady_cte'],4),'opposite',metrics['monotone_opposite_samples'])


if __name__=='__main__':
    main()
