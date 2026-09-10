"""Static SDK chassis geometry, not an online steering calibration.

SCS local Z points backwards. Truck world placement is the chassis origin;
the non-steering axle is behind it at positive Z. Only a single unambiguous
fixed axle plane and a single front steering axle plane prove this bicycle
model. Tandem/rear-steer/load-distribution identification is a separate task.
"""
import math


def validate_reference_geometry(geometry):
    if not isinstance(geometry, dict) or geometry.get('valid') is not True:
        return 'unproven tractor reference geometry: ' + str(
            (geometry or {}).get('reason', 'missing SDK axle geometry')
            if isinstance(geometry, dict) else 'missing SDK axle geometry')
    try:
        if any(isinstance(geometry[key], bool) for key in ('wheelbase_m','reference_ahead_m')):
            return 'invalid tractor reference geometry'
        wheelbase=float(geometry['wheelbase_m'])
        ahead=float(geometry['reference_ahead_m'])
    except (KeyError, TypeError, ValueError, OverflowError):
        return 'invalid tractor reference geometry'
    if not (math.isfinite(wheelbase) and 2.0 <= wheelbase <= 8.0
            and math.isfinite(ahead) and 0.0 <= ahead <= wheelbase):
        return 'tractor reference geometry outside supported bicycle range'
    return ''


def sdk_reference_geometry(raw):
    """Derive origin-to-rear-axle distance and wheelbase from static SDK data.

    No tyre angle, yaw, steering command, trailer or previous-frame estimate
    participates. Missing/malformed or multi-axle data never defaults to 2.1 m.
    """
    def reject(reason):
        return dict(valid=False, source='sdk_wheel_positions', reason=reason)
    try:
        count=raw['truckWheelCount']
        if isinstance(count, bool) or int(count) != count or not 4 <= count <= 16:
            return reject('invalid truck wheel count')
        count=int(count)
        xs, zs, steering = (list(raw[key])[:count] for key in
            ('wheelPositionX', 'wheelPositionZ', 'wheelSteerable'))
        if any(len(values) != count for values in (xs,zs,steering)):
            return reject('incomplete axle geometry')
        xs=list(map(float,xs)); zs=list(map(float,zs))
        if not all(math.isfinite(v) for v in xs+zs):
            return reject('non-finite axle geometry')
        if not all(type(v) is bool for v in steering):
            return reject('invalid steerable flags')
        front=[i for i in range(count) if steering[i]]
        rear=[i for i in range(count) if not steering[i]]
        for group in (front,rear):
            if len(group)<2 or not (min(xs[i] for i in group)<-.3
                                    and max(xs[i] for i in group)>.3):
                return reject('axle lacks wheels on both sides')
            if max(zs[i] for i in group)-min(zs[i] for i in group)>.10:
                return reject('multiple axle planes require a validated chassis model')
            if abs(sum(xs[i] for i in group)/len(group))>.10:
                return reject('off-centre axle requires a validated chassis model')
            if not 1.0 <= max(xs[i] for i in group)-min(xs[i] for i in group) <= 3.5:
                return reject('axle track outside supported truck range')
        front_z=sum(zs[i] for i in front)/len(front)
        rear_z=sum(zs[i] for i in rear)/len(rear)
        geometry=dict(valid=True,source='sdk_wheel_positions',
            reference_ahead_m=rear_z, wheelbase_m=rear_z-front_z,
            front_axle_z_m=front_z, rear_axle_z_m=rear_z,
            wheel_count=count, wheel_position_x_m=xs, wheel_position_z_m=zs,
            wheel_steerable=steering)
        reason=validate_reference_geometry(geometry)
        return reject(reason) if reason else geometry
    except (KeyError, TypeError, ValueError, OverflowError):
        return reject('missing or malformed SDK axle geometry')
