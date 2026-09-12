"""Offline 90-degree footprint benchmark; all surfaces/dimensions are fixtures.

The rear tractor axle follows an analytic circle at 2 m/s. A trailer begins
in steady articulation. This assesses that supplied path; it does not find a
safe path or model game steering delay. Output may only be written in the repo.
"""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

from core.swept_envelope import Pose, evaluate
from tests.test_stage5a_swept_envelope import vehicle, frame, surface, identity


def scenario(radius, right, length, width=2.5):
    v=vehicle(int(length>0))
    if length:
        v=replace(v,bodies=(v.bodies[0],replace(v.bodies[1],width_m=width,
                          front_m=length+1.,hitch_front_m=length)))
    sign=-1 if right else 1
    duration=radius*math.pi/2/2.
    count=math.ceil(duration/.02)
    frames=[]
    for i in range(count+1):
        h=sign*math.pi/2*i/count
        p=Pose(sign*radius*(math.cos(h)-1),0.,-sign*radius*math.sin(h),h)
        heads=(h-sign*math.asin(length/radius),) if length else ()
        frames.append(frame(v,duration*i/count,p,heads))
    # Exterior inscribed / island circumscribed: conservatively contained in
    # the ideal annulus, never permitting its island or widening its pavement.
    n=96
    def ring(r):
        return tuple((-sign*radius+r*math.cos(i*math.tau/n),
                       r*math.sin(i*math.tau/n)) for i in range(n))
    area=surface(ring(radius+2.25),(ring((radius-2.25)/math.cos(math.pi/n)),))
    result=evaluate(v,tuple(frames),area,identity())
    # Independent analytical body extrema relative to the turn centre.
    # At steady articulation its axle tangent is orthogonal to its radius.
    exact=[]
    for i,b in enumerate(v.bodies):
        r=math.sqrt(radius**2-length**2) if i else radius
        inner=r-b.width_m/2
        outer=math.hypot(r+b.width_m/2,max(b.front_m,b.rear_m))
        exact.append(min(inner-(radius-2.25),(radius+2.25)-outer))
    return dict(radius_m=radius,direction='right' if right else 'left',
        trailer_axle_distance_m=length,width_m=width,frames=len(frames),
        path_angle_deg=90.,duration_s=duration,
        accepted=result.accepted,reason=result.failure_reason,
        min_certified_clearance_m=result.minimum_clearance_m,
        limiting_body=result.limiting_body,
        analytic_static_clearance_m=min(exact),
        safety_margin_m=v.safety_margin_m,uncertainty_m=v.uncertainty_m,
        max_motion_pad_m=max(max(s.corner_motion_bounds_m) for s in result.steps))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args()
    output=args.output.resolve()
    if not output.is_relative_to(ROOT):
        parser.error('output must stay in the repository')
    started=time.perf_counter()
    rows=[]
    for radius in (18.,22.,25.,35.):
        for right in (False,True):
            for length in (0.,6.,8.,10.):
                row=scenario(radius,right,length,2.6 if length==10. else 2.5)
                rows.append(row)
                print(f'R{radius:g} {row["direction"]} trailer={length:g}: '
                      f'{row["reason"] or "CLEAR"} '
                      f'clearance={row["min_certified_clearance_m"]}',flush=True)
                if row['accepted']:
                    assert row['analytic_static_clearance_m'] > .15
    result=dict(scope='synthetic horizontal fixed-axle geometry, not game approval',
                elapsed_s=time.perf_counter()-started,cases=rows)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
