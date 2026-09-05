"""Reproducible baseline/after report, without changing production or game data."""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import sys
import subprocess
import types

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'tests'))
from steering_bench import path,run


def real_speed():
    data=json.loads((ROOT/'tests/fixtures/steering-20260816.json').read_text(encoding='utf-8'))
    rows=[r for r in data['rows'] if '11:50:30'<=r['timestamp'][11:19]<='11:50:49' and r.get('nav')]
    ts=[r['time_s']-rows[0]['time_s']+8 for r in rows]
    def speed(t):
        j=max(0,min(len(ts)-2,bisect.bisect_right(ts,t)-1))
        f=max(0,min(1,(t-ts[j])/(ts[j+1]-ts[j])))
        return (rows[j]['speed']*(1-f)+rows[j+1]['speed']*f)/3.6
    return speed


def cases():
    for sign in (-1,1):
        for radius,speed in ((18,5.4),(35,7.4),(83,12.),(250,25.)):
            yield f'R{radius}_{sign}',path([(0,55),(sign/radius,radius*math.pi),(0,80)]),dict(speed=speed,noisy=True,jitter=True,trailer=True)
    yield 'S35',path([(0,55),(-1/35,75),(1/35,75),(0,80)]),dict(speed=7.4,noisy=True,jitter=True,trailer=True)
    yield 'roundabout',path([(0,55),(-1/25,25*math.tau),(0,80)]),dict(speed=6.,noisy=True,jitter=True,trailer=True)
    for kmh in (10,30,60,90):
        yield f'straight_{kmh}',path([(0,100),(0,100),(0,100)]),dict(speed=kmh/3.6,initial_cte=.5,noisy=True,jitter=True)
    for lag in (.32,.50):
        yield f'log_speed_R22_lag{lag}',path([(0,35),(.045,110),(0,80)]),dict(speed_profile=real_speed(),trailer=True,noisy=True,jitter=True,lag=lag)


def independent_matrix_cases(match_controller_calibration=False):
    """Plant sweep; none of these gain/response values comes from production."""
    geometries = (
        ('R250', path([(0,55),(1/250,250*1.2),(0,80)]), dict(speed=25.)),
        ('R83', path([(0,55),(-1/83,83*1.5),(0,80)]), dict(speed=12.)),
        ('R35', path([(0,55),(1/35,35*1.8),(0,80)]), dict(speed=7.4)),
        ('S35', path([(0,55),(-1/35,75),(1/35,75),(0,80)]), dict(speed=7.4)),
        ('roundabout_R25', path([(0,55),(-1/25,25*math.tau),(0,80)]), dict(speed=6.)),
        ('R18', path([(0,55),(1/18,18*math.pi),(0,80)]), dict(speed=5.4)),
        ('log_speed_R22', path([(0,35),(.045,110),(0,80)]),
         dict(speed_profile=real_speed())),
    )
    for lock in (.60, .78, .95):
        for response in (.25, .425, .60):
            for trailer in (False, True):
                for name, data, base in geometries:
                    options = dict(base, lock_rad=lock, lag=response,
                                   trailer=trailer, noisy=True, jitter=True)
                    if match_controller_calibration:
                        options['controller_lock_rad'] = lock
                    yield (f'{name}_gain{lock:.2f}_response{response:.3f}_'
                           f'{"trailer" if trailer else "cab"}', data, options)
                for kmh in (10, 30, 60, 90):
                    data = path([(0,100),(0,100),(0,100)])
                    yield (f'straight_{kmh}_gain{lock:.2f}_response{response:.3f}_'
                           f'{"trailer" if trailer else "cab"}', data,
                           dict(speed=kmh/3.6, initial_cte=.5,
                                lock_rad=lock, lag=response, trailer=trailer,
                                controller_lock_rad=(lock if
                                    match_controller_calibration else .78),
                                noisy=True, jitter=True))


def factories_for_ref(ref):
    if not ref:
        return None, None, {}
    loaded = {}
    factories = []
    for relpath, module_name, class_name in (
            ('core/navigation/route.py', '_audit_baseline_route', 'Route'),
            ('core/steering_dynamics.py', '_audit_baseline_dynamics',
             'SteeringDynamics')):
        source = subprocess.check_output(
            ['git', 'show', f'{ref}:{relpath}'], cwd=ROOT, text=True,
            encoding='utf-8')
        module = types.ModuleType(module_name)
        module.__file__ = f'{ref}:{relpath}'
        exec(compile(source, module.__file__, 'exec'), module.__dict__)
        factories.append(getattr(module, class_name))
        loaded[relpath] = hashlib.sha256(source.encode('utf-8')).hexdigest()
    return factories[0], factories[1], loaded


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ref', help='Run production controller/dynamics from a git ref')
    parser.add_argument('--matrix', action='store_true',
                        help='Also run the independent gain/response sweep')
    parser.add_argument('--quiet', action='store_true')
    parser.add_argument('--match-controller-calibration', action='store_true',
                        help='Set the explicit controller calibration to the independent plant gain')
    args=parser.parse_args()
    if not args.output.resolve().is_relative_to(ROOT):
        parser.error('Outputs must stay in the UltraPilot workspace')
    route_factory, dynamics_factory, ref_hashes = factories_for_ref(args.ref)
    paths = (ROOT/'core/navigation/route.py', ROOT/'core/steering_dynamics.py',
             ROOT/'core/lateral_controller.py', ROOT/'tests/steering_bench.py')
    result={'production_ref': args.ref or 'working-tree',
            'production_hashes':(ref_hashes or {
                str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in paths}),
            'plant':dict(full_lock_rad=.78,wheelbase_m=3.8,transport_s=.10,integration_step_s=.005),
            'limitation':'Log speed excitation on analytical R22; not the original map geometry or a reconstruction of unlogged 20 Hz poses.',
            'matrix_controller_calibration':(
                'matched_to_declared_plant' if args.match_controller_calibration
                else 'provisional_default_0.78'),
            'cases':{}, 'independent_matrix':{}}
    factories = {}
    if route_factory is not None:
        factories = dict(route_factory=route_factory,
                         dynamics_factory=dynamics_factory)
    for name,data,options in cases():
        report,rows=run(data,**options,**factories)
        result['cases'][name]=report
        if not args.quiet:
            print(name,json.dumps(report),flush=True)
    if args.matrix:
        for name, data, options in independent_matrix_cases(
                args.match_controller_calibration):
            report, _rows = run(data, **options, **factories)
            result['independent_matrix'][name] = report
            if not args.quiet:
                print(name, json.dumps(report), flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
