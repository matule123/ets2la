"""Read-only frame-bound SDK pose audit; estimates a turn centre, not calibration.

No interpolated SDK frames and no state written into the running application.
The estimate is observational: tyre slip/pitch can move the effective centre.
Runtime steering instead uses validated, static SDK axle positions.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def audit(path):
    blob=path.read_bytes()
    payload=json.loads(blob)
    rows=[]
    seen=set()
    for r in payload['samples']:
        frame=r.get('calculation_sdk_frame_us')
        if (r.get('autopilot_active') and r.get('packet_binding_valid')
                and frame and frame not in seen):
            seen.add(frame)
            rows.append(r)
    fits=[]
    identity_fields=('navigation_intent_id','route_build_id','revision',
        'source_game_session_id','source_map_key','source_dataset_fingerprint')
    for i,b in enumerate(rows):
        j=i+1
        while j<len(rows) and rows[j]['calculation_sdk_frame_us']-b['calculation_sdk_frame_us']<180000:
            j+=1
        if j==len(rows):
            continue
        c=rows[j]
        dt=(c['calculation_sdk_frame_us']-b['calculation_sdk_frame_us'])/1e6
        if (dt>.35 or c['lane_id']!=b['lane_id']
                or any(c.get(k)!=b.get(k) for k in identity_fields)):
            continue
        dh=(c['observation_heading_rad']-b['observation_heading_rad']+math.pi)%math.tau-math.pi
        h=b['observation_heading_rad']+dh/2
        vx,vz=((y-x)/dt for x,y in zip(b['observation_xz'],c['observation_xz']))
        forward=-math.sin(h)*vx-math.cos(h)*vz
        left=-math.cos(h)*vx+math.sin(h)*vz
        yaw=dh/dt
        if forward>3. and abs(yaw)>.03:
            fits.append((left/yaw,b['local_k_per_m']))
    result=dict(source=str(path.resolve()),sha256=hashlib.sha256(blob).hexdigest(),
        unique_active_sdk_poses=len(rows),
        identities=list({tuple(r.get(k) for k in identity_fields) for r in rows}),
        identity_fields=identity_fields, estimate_is_not_runtime_calibration=True)
    for name,subset in (('all',fits),('left',[r for r in fits if r[1]<-.01]),
                        ('right',[r for r in fits if r[1]>.01])):
        values=sorted(r[0] for r in subset)
        result[name]=dict(intervals=len(values),median_m=statistics.median(values),
            p10_m=values[len(values)//10],p90_m=values[9*len(values)//10]) if values else None
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replays',type=Path,nargs='+')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    result=[audit(path) for path in args.replays]
    output=json.dumps(result,indent=2)
    if args.output:
        root=Path(__file__).resolve().parents[1]
        if not args.output.resolve().is_relative_to(root):
            parser.error('output must stay in workspace')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(output,encoding='utf-8')
    else:
        print(output)
