"""Same-surface centreline/planned full-vehicle occupancy comparison (offline)."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tests.maneuver_cases import junction,centreline_frames
from core.navigation.maneuver_planner import plan_maneuver,PlannerLimits
from core.navigation.drivable_surface import evaluate_on_confirmed_surface


def scenario(radius,right,length,half_width,outer=None,articles=1,angle=90):
    request=junction(radius,right,articles if length else 0,half_width,
                     outer_width=outer,trailer_length=length or 8.,angle=math.radians(angle))
    v,start,path,surface,ident=request
    baseline_frames=centreline_frames(request,radius,right,angle=math.radians(angle))
    baseline=evaluate_on_confirmed_surface(v,baseline_frames,surface,path,ident)
    started=time.perf_counter()
    plan=plan_maneuver(*request,limits=PlannerLimits(max_candidates=81))
    row=dict(radius_m=radius,right=right,article_count=len(v.bodies),
        trailer_length_m=length,half_width_m=half_width,outer_width_m=outer,
        turn_deg=angle,source_path_accepted=baseline.accepted,
        source_clearance_m=baseline.minimum_clearance_m,accepted=plan.accepted,
        failure_reason=plan.failure_reason,
        clearance_m=plan.envelope.minimum_clearance_m if plan.envelope else None,
        speed_mps=plan.speed_mps,samples=len(plan.samples),
        elapsed_s=time.perf_counter()-started,
        reasons=dict(Counter(row[1] for row in plan.diagnostics)),
        runtime_authorized=plan.runtime_authorized)
    if plan.accepted:
        row['metrics']=plan.diagnostics[-1][2]
        row['per_body_clearance_m']={body.name:min(step.clearance_m[i]
             for step in plan.envelope.steps) for i,body in enumerate(v.bodies)}
        row['entry_pose_error_m']=math.dist(
            (plan.samples[0].frame.poses[0].x,plan.samples[0].frame.poses[0].z),
            (start.poses[0].x,start.poses[0].z))
        end=plan.samples[-1].frame.poses[0];goal=path.points[-1]
        row['exit_pose_error_m']=math.hypot(end.x-goal.x,end.z-goal.z)
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--quick',action='store_true')
    args=parser.parse_args()
    if not args.output.resolve().is_relative_to(ROOT):
        parser.error('output must stay in the repository')
    specs=[]
    for radius in (18.,22.,25.,35.):
        for right in (False,True):
            for length in (0.,8.):
                specs.append((radius,right,length,2.25))
    specs += [(18.,right,8.,half) for right in (False,True) for half in (3.,3.25,3.5)]
    specs += [(22.,False,length,3.5) for length in (6.,10.)]
    if args.quick: specs=[(18.,False,8.,3.),(18.,False,8.,3.25)]
    rows=[]
    for spec in specs:
        row=scenario(*spec); rows.append(row)
        print(json.dumps(row),flush=True)
    if not args.quick:
        for kwargs in (dict(articles=2),dict(angle=60),dict(angle=110)):
            row=scenario(25.,False,6.,4.,**kwargs);rows.append(row)
            print(json.dumps(row),flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps({'scope':'synthetic offline fixed-axle model, no traffic permission',
                                      'cases':rows},indent=2)+'\n',encoding='utf8')


if __name__=='__main__': main()
