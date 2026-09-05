"""Independent 20 Hz nonlinear vehicle bench (no production plant constants).

Only the controller/dynamics under test are imported from production. Paths
are analytical, integration uses <=5 ms plant steps and transport delay is
time based. A different tyre lock/wheelbase/delay must not silently update
when the controller's calibration is changed.
"""
from collections import deque
import bisect
import math
import statistics

from core.navigation.route import Route
from core.steering_dynamics import SteeringDynamics


def wrap(a):
    return (a + math.pi) % math.tau - math.pi


def path(sections, transition=8.0, step=.5):
    """Sections (right-positive curvature, metres), with linear entry ramps."""
    points = [(0., 0.)]
    ks = [0.]
    ss = [0.]
    x = z = s = 0.
    heading = math.pi
    previous = 0.
    boundaries = []
    for k, length in sections:
        local = 0.
        while local < length - 1e-9:
            ds = min(step, length-local)
            curvature = previous + (k-previous) * min(1., (local+ds/2)/max(transition, 1e-9))
            mid = heading-curvature*ds/2
            x -= math.sin(mid)*ds
            z -= math.cos(mid)*ds
            heading -= curvature*ds
            s += ds
            local += ds
            points.append((x,z))
            ks.append(curvature)
            ss.append(s)
        previous = k
        boundaries.append(s)
    return dict(points=points, curvature=ks, distance=ss, boundaries=boundaries)


def metrics(rows, end_curve):
    def diff(values):
        return [(b-a)/rows[i+1]['dt'] for i,(a,b) in enumerate(zip(values,values[1:]))]
    out = [r['out'] for r in rows]
    velocity = diff(out)
    accel = [(b-a)/rows[i+2]['dt'] for i,(a,b) in enumerate(zip(velocity,velocity[1:]))]
    jerk = [(b-a)/rows[i+3]['dt'] for i,(a,b) in enumerate(zip(accel,accel[1:]))]
    # Unwanted sign changes are tested where the immutable local curvature
    # actually has a definite direction; straight noise is quantified by RMS.
    opposite = [r for r in rows if abs(r['k'])>.003 and r['raw']*r['k'] < -1e-5]
    last_sign = 0
    last_curve = 0
    flips = 0
    for r in rows:
        curve = 1 if r['k']>.003 else -1 if r['k']<-.003 else 0
        sign = 1 if r['out']>.004 else -1 if r['out']<-.004 else 0
        if curve and curve == last_curve and sign and last_sign and sign!=last_sign:
            flips += 1
        if curve != last_curve:
            last_sign = 0
        if sign:
            last_sign = sign
        last_curve = curve
    tail=[r for r in rows if r['s'] > end_curve+15]
    exiting=[r for r in rows if r['s']>=end_curve]
    settle=None
    if exiting:
        unsettled=[r for r in exiting if abs(r['cte'])>.25 or abs(r['h'])>math.radians(1)]
        settle=(unsettled[-1]['t']-exiting[0]['t']) if unsettled else 0.
        if unsettled and unsettled[-1] is exiting[-1]:
            settle=None
    return dict(max_cte=max(abs(r['cte']) for r in rows),
                rms_cte=math.sqrt(statistics.fmean(r['cte']**2 for r in rows)),
                max_heading_deg=math.degrees(max(abs(r['h']) for r in rows)),
                max_step=max(map(abs,(b-a for a,b in zip(out,out[1:]))),default=0.),
                max_rate=max(map(abs,velocity),default=0.),
                max_accel=max(map(abs,accel),default=0.),
                max_jerk=max(map(abs,jerk),default=0.),
                unwanted_sign_changes=flips, opposite_samples=len(opposite),
                # The inclusive fields above retain the original baseline
                # definition. This additional geometric classification makes
                # entry/exit and S changes visible instead of calling every
                # reverse command a monotone-turn oscillation.
                monotone_opposite_samples=sum(r['geometry_monotone']
                    and r['raw']*r['k'] < -1e-5 for r in rows),
                trailer_max_cte=max((abs(r['trailer_cte']) for r in rows),default=0.),
                steady_cte=max((abs(r['cte']) for r in tail),default=0.),
                settling_s=settle, final_cte=rows[-1]['cte'],
                completed=rows[-1].get('completed',False),
                lost_lane=any(abs(r['cte'])>2.4 for r in rows))


def run(data, speed=12., *, lock_rad=.78, wheelbase=3.8, lag=.32, transport=.10,
        initial_cte=0., noisy=False, jitter=False, trailer=False, load=0.,
        speed_profile=None, max_time=180., route_factory=Route,
        dynamics_factory=SteeringDynamics, controller_lock_rad=.78):
    route=route_factory(data['points'])
    dynamics=dynamics_factory()
    x,z=data['points'][0]
    x+=initial_cte
    h=math.pi
    game=0.
    tr_h=h
    queue=deque()
    target=0.
    t=0.
    i=0
    rows=[]
    last_index=0
    while t<max_time:
        dt=(.05+(.007*math.sin(i*.7) if jitter else 0.))
        if jitter and i>0 and i%53==0:
            dt=.11
        v=float(speed if speed_profile is None else speed_profile(t))
        # Oracle is independent of Route's progress cache: search local true
        # path segments, preserving the same forward occurrence on loops.
        candidates=[]
        for j in range(max(0,last_index-10),min(len(data['points'])-1,last_index+90)):
            a,b=data['points'][j:j+2];dx,dz=b[0]-a[0],b[1]-a[1]
            f=max(0.,min(1.,((x-a[0])*dx+(z-a[1])*dz)/(dx*dx+dz*dz)))
            px,pz=a[0]+dx*f,a[1]+dz*f
            candidates.append(((x-px)**2+(z-pz)**2,j,f,dx,dz))
        _,j,f,dx,dz=min(candidates)
        last_index=j
        s=data['distance'][j]+f*(data['distance'][j+1]-data['distance'][j])
        length=math.hypot(dx,dz)
        cte=((x-data['points'][j][0])*dz-(z-data['points'][j][1])*dx)/length
        path_h=math.atan2(-dx,-dz)
        k=data['curvature'][j]
        # Inspect the complete preview + derivative footprint, independently
        # of the command. 8 m curvature support + 3.8 m trailer differential
        # support rounded up; 0.42 s is the identified nominal prediction.
        lo=bisect.bisect_left(data['distance'],max(0.,s-12.))
        hi=bisect.bisect_right(data['distance'],s+12.+v*.42)
        neighbourhood=data['curvature'][lo:hi]
        monotone=bool(abs(k)>.003 and neighbourhood
                      and max(neighbourhood)-min(neighbourhood)<1e-8)
        trailer_cte=0.
        measured=cte + (.04*math.sin(7*t)+.015*math.sin(31*t) if noisy else 0.)
        observed_h=h+(math.radians(.25)*math.sin(9*t) if noisy else 0.)
        envelope=None
        if trailer:
            envelope=dict(attached=True,position=(x+8*math.sin(tr_h),z+8*math.cos(tr_h)),
                          heading=tr_h,lane_width_m=4.5,tractor_altitude_m=45.,trailer_altitude_m=45.)
            tx,tz=envelope['position']
            distances=[]
            for n in range(max(0,j-45),min(len(data['points'])-1,j+5)):
                a,b=data['points'][n:n+2];ux,uz=b[0]-a[0],b[1]-a[1]
                f=max(0.,min(1.,((tx-a[0])*ux+(tz-a[1])*uz)/(ux*ux+uz*uz)))
                distances.append(((tx-a[0]-f*ux)**2+(tz-a[1]-f*uz)**2,
                    ((tx-a[0])*uz-(tz-a[1])*ux)/math.hypot(ux,uz)))
            trailer_cte=min(distances)[1]
        authority=dict(lane_identity='road' if s<data['boundaries'][0] else 'prefab',
                       revision=8 if s<data['boundaries'][0] else 9,
                       elevation_layer=45,lane_width_m=4.5)
        extra={}
        import inspect
        if 'vehicle_curvature_per_m' in inspect.signature(route.steering).parameters:
            extra['vehicle_curvature_per_m']=math.tan(lock_rad*game)/wheelbase
        if 'steering_lock_rad' in inspect.signature(route.steering).parameters:
            extra['steering_lock_rad']=controller_lock_rad
        raw=route.steering((x,z),observed_h,v,cross_track_error_m=measured,
                           vehicle_envelope=envelope,control_authority=authority,control_dt_s=dt,**extra)
        debug=route.last_steering_debug
        out=dynamics.update(raw,dt,speed_ms=v,curvature_per_m=debug.get('local_curvature',0.))
        queue.append((t+transport,out))
        # Continuous plant with exact first order actuator integration. The
        # loaded stress case increases tyre/chassis response by up to 80 ms.
        substeps=math.ceil(dt/.005)
        ds=dt/substeps
        for sub in range(substeps):
            while queue and queue[0][0]<=t+sub*ds+1e-10:
                _,target=queue.popleft()
            game+=(target-game)*(1-math.exp(-ds/(lag+.08*load)))
            yaw=-v*math.tan(lock_rad*game)/wheelbase
            mid=h+yaw*ds*.5
            x-=math.sin(mid)*v*ds;z-=math.cos(mid)*v*ds;h+=yaw*ds
            tr_h+=v/8*math.sin(h-tr_h)*ds
        completed=s>=data['distance'][-1]-5
        rows.append(dict(t=t,dt=dt,s=s,cte=cte,h=wrap(observed_h-path_h),k=k,raw=raw,
                         geometry_monotone=monotone,trailer_cte=trailer_cte,
                         out=out,game=game,ff=debug.get('feed_forward',0.),
                         fb=debug.get('feedback',0.),trailer=debug.get('trailer_envelope',{}).get('applied_offset_m',0.),
                         completed=completed))
        if completed or abs(cte)>8:
            break
        t+=dt;i+=1
    return metrics(rows,data['boundaries'][-2]),rows
