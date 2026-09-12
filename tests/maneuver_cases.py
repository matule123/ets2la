"""Synthetic authored road surfaces; never exported as actual game evidence."""
from dataclasses import replace
import math

from core.navigation.drivable_surface import build_confirmed_surface
from core.navigation.lane_model import LaneId, LanePoint, LaneSegment, LaneConnection, LanePath
from core.navigation.lane_trajectory import build_lane_trajectory
from core.swept_envelope import Identity, Body, Vehicle, Frame, Pose, articulated_poses, propagate
from tests.test_stage5b2_drivable_surface import record


def junction(radius=18., right=False, trailers=1, half_width=2.25,
             outer_width=None, trailer_length=8., approach=25., exit_length=45.,
             y=0., angle=math.pi/2):
    sign = -1 if right else 1
    outer = half_width if outer_width is None else outer_width
    lanes = (LaneId(101,1,0),LaneId(102,1,0,'synthetic_junction',0,(0,1,2)),LaneId(103,1,0))
    identity = Identity('synthetic',8,'build','session','synthetic_map','fixture',
                        'level',lanes,((1,2),(2,3),(3,4)))
    def arc(t,r=radius): return (sign*(-radius+r*math.cos(t)),-approach-r*math.sin(t))
    end_x,end_z = arc(angle)
    coords = [
        [(0.,-approach*i/20,0.) for i in range(21)],
        [(*arc(angle*i/32),sign*angle*i/32) for i in range(33)],
        [(end_x-sign*math.sin(angle)*exit_length*i/30,
          end_z-math.cos(angle)*exit_length*i/30,sign*angle) for i in range(31)],
    ]
    segments = tuple(LaneSegment(lanes[i],i+1,i+2,1,0,1,2*half_width,'derived',6,
        'synthetic','prefab' if i==1 else 'road',
        tuple(LanePoint(x,y,z,heading=h) for x,z,h in rows),
        successors=(LaneConnection(lanes[i+1],'prefab'),) if i<2 else (),
        gps_pair_index=i) for i,rows in enumerate(coords))
    path = build_lane_trajectory(LanePath(segments,(),(1,2,3,4),confidence=.99,
                                         valid=True,revision=8))
    assert path.valid,path.failure_reason
    # This is deliberately an authored synthetic test road, including long
    # entry/exit support for the full vehicle. It is not a production width buffer.
    outer_arc = [arc(angle*i/16,radius+outer) for i in range(17)]
    inner_arc = [arc(angle*i/16,radius-half_width) for i in range(17)]
    forward = (-sign*math.sin(angle),-math.cos(angle))
    extend = exit_length+10.
    exterior = [(sign*half_width,45.),(sign*half_width,-approach/2),*outer_arc,
        (outer_arc[-1][0]+forward[0]*extend,outer_arc[-1][1]+forward[1]*extend),
        (inner_arc[-1][0]+forward[0]*extend,inner_arc[-1][1]+forward[1]*extend),
        *inner_arc[::-1],(-sign*half_width,45.)]
    raw = record(path,identity,y=y,uncertainty=.02)
    raw['source'] = 'synthetic authored junction, not evidence for ETS2'
    raw['exterior_xyz'] = [[x,y,z] for x,z in exterior]
    surface = build_confirmed_surface(raw,path,identity,identity)
    assert not surface.failure_reason,surface.failure_reason
    tractor = Body('tractor',2.5,5.,1.2,0.,0.,'synthetic dimensions',True,'fixed_axle')
    bodies = (tractor,) + tuple(Body(f'trailer{i}',2.5,trailer_length+1.,2.,
        trailer_length,0.,'synthetic dimensions',True,'fixed_axle') for i in range(trailers))
    vehicle = Vehicle(bodies,3.8,.7,2.,.15,.02)
    first = path.points[0]
    start = Frame(0.,identity,articulated_poses(vehicle,
        Pose(first.x,first.y,first.z,first.heading),(first.heading,)*trailers))
    return vehicle,start,path,surface,identity


def centreline_frames(request,radius,right=False,approach=25.,exit_length=45.,angle=math.pi/2):
    """Independent piecewise constant bicycle inputs for the unshifted road.

    The instantaneous tyre changes are a geometric occupancy baseline only,
    not a proposed actuator-feasible maneuver.
    """
    v,start,_,_,_=request
    frames=[start]
    for length,delta in ((approach,0.),(radius*angle,
            (1 if right else -1)*math.atan(v.wheelbase_m/radius)),(exit_length,0.)):
        duration=length/2
        count=math.ceil(duration/.02)
        for _ in range(count):
            frames.append(propagate(v,frames[-1],2.,delta,duration/count))
    return tuple(frames)
