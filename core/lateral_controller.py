"""Stateless, right-positive Frenet bicycle controller.

e [m] and h [rad] are LEFT-positive errors. Curvature [1/m], road-wheel
angle [rad] and the returned normalized SDK command are RIGHT-positive.
See STEERING_AUDIT.md for identification limits, equations and closed-loop
tests with an independent plant. No moving average, integrator or turn latch.
"""
import math

from core.steering_calibration import (
    DEFAULT_TYRE_ANGLE_PER_INPUT_RAD as REFERENCE_LOCK_RAD,
    MIN_TYRE_ANGLE_PER_INPUT_RAD as MIN_STEERING_LOCK_RAD,
    MAX_TYRE_ANGLE_PER_INPUT_RAD as MAX_STEERING_LOCK_RAD,
)

WHEELBASE_M = 3.8

# Feedback pole placement is a controller design term, not measured actuator
# latency. Before Phase 4B the same 0.45 s value was also used as the path
# preview horizon. Real frame-bound data disproves that coupling: command to
# road-wheel transport is about one SDK frame while this longer spatial length
# remains necessary for high-speed closed-loop damping.
FEEDBACK_RESPONSE_S = 0.45
# Critical-damping spatial length: two wheelbases rounded to the 2 m map
# sampling scale.  The response distance weakens feedback at speed without
# pretending that the current yaw rate will remain constant through the whole
# actuator delay.
# This is deliberately a controller design value. Dense actuator measurements
# identify preview timing separately in core.steering_calibration.
FEEDBACK_LENGTH_BASE_M = 8.0
FRENET_DAMPING_RATIO = 1.10
FRENET_LATERAL_GAIN = 0.85
# Slightly overdamped Frenet feedback.  A damping ratio above one prevents a
# large initial CTE and same-side heading error from crossing the centreline
# with residual yaw; it changes the geometric pole placement, not actuator
# smoothing.
def solve(curvature, preview_curvature, cte_m, heading_error_rad, speed_ms,
          vehicle_curvature_per_m=None,
          response_s=FEEDBACK_RESPONSE_S, steering_lock_rad=REFERENCE_LOCK_RAD,
          reference_ahead_m=0.0, wheelbase_m=WHEELBASE_M):
    """One curvature demand, followed by exactly one inverse bicycle mapping.

    For the legacy rear-axle reference (a=0):
    e_dot=v*sin(h), h_dot=v*k*cos(h)/(1+k*e)-v*k_vehicle.
    With preview==k and constant speed its small-error feedback is
    e_ddot + 2*zeta*v/ell*e_dot + gain*(v/ell)^2*e = 0.
    Preview changes only the feed-forward along confirmed geometry, never the
    tangent or the point to which CTE is measured.
    For a>0 use the chassis-origin kinematics and circle equilibrium below;
    this is a local tracking law, not an articulated swept-envelope planner.
    """
    values=(curvature, preview_curvature, cte_m, heading_error_rad, speed_ms)
    if not all(math.isfinite(float(v)) for v in values):
        return 0.0, {"valid": False, "reason": "non-finite lateral input"}
    k,kp,e,h,v=map(float,values)
    from core.vehicle_geometry import validate_reference_geometry
    reason=validate_reference_geometry(dict(valid=True,
        reference_ahead_m=reference_ahead_m, wheelbase_m=wheelbase_m))
    if reason:
        return 0.0, {"valid": False, "reason": reason}
    a, wheelbase=float(reference_ahead_m),float(wheelbase_m)
    if not (math.isfinite(response_s) and 0.0 <= response_s <= 1.0
            and math.isfinite(steering_lock_rad)
            and MIN_STEERING_LOCK_RAD <= steering_lock_rad
                <= MAX_STEERING_LOCK_RAD):
        return 0.0, {"valid": False, "reason": "invalid actuator calibration"}
    # Road-wheel curvature is retained as actuator diagnostics only.  Feeding
    # it back into this geometric target created a second, faster actuator
    # loop in front of SteeringDynamics: while the tyres lagged it increased
    # the target, then removed that increase as the next SDK wheel sample
    # arrived.  On the real constant bend this produced the observed
    # 0.1134 -> 0.1232 -> 0.1105 command impulse despite stable path geometry
    # and improving Frenet error.  Vehicle motion already enters this one
    # controller through the current pose/CTE/heading; physical command
    # execution belongs solely to SteeringDynamics and the game.
    measured_vehicle_curvature = None
    if vehicle_curvature_per_m is not None:
        measured_vehicle_curvature=float(vehicle_curvature_per_m)
        if not math.isfinite(measured_vehicle_curvature):
            return 0.0, {"valid": False, "reason": "non-finite vehicle curvature"}
    # One deterministic spatial feedback length.  It must not change merely
    # because road-wheel telemetry is present or momentarily unavailable.
    # The chassis plus the configured actuator response is the physical
    # distance travelled before a new wheel angle takes effect; 8 m remains
    # the conservative lower bound identified by the independent plant.  This
    # is geometry/speed based and has no temporal memory or dependence on
    # delayed tyre samples.
    length=max(FEEDBACK_LENGTH_BASE_M,
               wheelbase+abs(v)*response_s)

    # The observed SDK chassis origin is a metres AHEAD of the rear axle.
    # Its exact kinematics are:
    # e_dot = v*(sin(h)-a*k_vehicle*cos(h))
    # s_dot = v*(cos(h)+a*k_vehicle*sin(h))/(1+k*e).
    # On a centred circle body heading therefore differs from the path's
    # tangent by asin(a*k); h==0 is NOT the correct equilibrium. Feeding this
    # necessary body angle into rear-axle heading feedback creates a sustained
    # ~2 m inward CTE. Derive the body reference from local path geometry, not
    # delayed tyre/yaw observations (which previously created an actuator loop).
    denominator=1+k*e
    if denominator <= .10 or abs(a*k/denominator) >= 1.0:
        return 0.0, {"valid": False, "reason": "unreachable chassis reference circle"}
    body_reference=math.asin(a*k/denominator)
    control_heading=(h-body_reference+math.pi)%math.tau-math.pi

    def control_at(error_m, error_heading):
        cosine=math.cos(error_heading)
        denominator=1+k*error_m
        if cosine <= .10 or denominator <= .10:
            return None
        rear_radius_squared=denominator**2-(a*kp)**2
        if rear_radius_squared <= .01:
            return None
        # Cab circle R -> rear axle circle sqrt(R^2-a^2). With a=0
        # this is exactly the historical rear-axle controller.
        foundation=kp*cosine/math.sqrt(rear_radius_squared)
        heading=(2*FRENET_DAMPING_RATIO
                 * math.tan(error_heading)/length)
        lateral=(FRENET_LATERAL_GAIN * error_m
                 / (length*length*cosine))
        return foundation+heading+lateral,foundation,heading,lateral

    control=control_at(e,control_heading)
    if control is None:
        return 0.0, {"valid": False, "reason": "invalid Frenet frame"}
    demand,foundation,heading,lateral=control
    # CTE and heading already are the current observed Frenet state.  The old
    # algebraic fixed-point predictor treated its own candidate as an applied
    # tyre curvature and changed a measured -1.55 m CTE into a fictitious
    # +1.42 m.  Transport is represented by curvature preview; actuator motion
    # is represented once by SteeringDynamics and the game.
    predicted_e,predicted_h=e,h
    observation_weight=0.0
    angle=math.atan(wheelbase*demand)
    base_angle=math.atan(wheelbase*foundation)
    lateral_angle=math.atan(wheelbase*(foundation+lateral))-base_angle
    heading_angle=angle-base_angle-lateral_angle
    return angle/steering_lock_rad, dict(
        valid=True, curvature_foundation_per_m=foundation,
        curvature_heading_per_m=heading, curvature_cte_per_m=lateral,
        curvature_demand_per_m=demand, feedback_length_m=length,
        feed_forward=base_angle/steering_lock_rad,
        cte_feedback=lateral_angle/steering_lock_rad,
        heading_feedback=heading_angle/steering_lock_rad,
        feed_forward_angle_rad=base_angle,
        cte_feedback_angle_rad=lateral_angle,
        heading_feedback_angle_rad=heading_angle,
        steering_angle_rad=angle, lock_rad=steering_lock_rad,
        frenet_heading_error_rad=h, frenet_cte_m=e,
        body_reference_heading_rad=body_reference,
        body_tracking_error_rad=control_heading,
        reference_ahead_m=a, wheelbase_m=wheelbase,
        measured_vehicle_curvature_per_m=(
            measured_vehicle_curvature),
        curvature_observation_weight=observation_weight,
        predicted_frenet_cte_m=predicted_e,
        predicted_frenet_heading_error_rad=predicted_h,
        trailer_target_m=0.0)


def offset_frame(curvature, curvature_derivative, target_m,
                 target_derivative, target_second_derivative):
    """Differential geometry of a tractor swept-path reference, not map edits.

    r_target(s)=r_lane(s)+E(s)*left_normal(s). Its tangent and curvature must
    change together; adding E only to CTE gives a moving lateral target with
    no matching heading/feed-forward, particularly when exiting a tight bend.
    """
    k,dk,e,de,dde=(float(v) for v in (curvature,curvature_derivative,target_m,
                                     target_derivative,target_second_derivative))
    a=1+k*e
    q2=a*a+de*de
    tangent=math.atan2(de,a)
    tangent_derivative=(a*dde-de*(dk*e+k*de))/q2
    return tangent,(k-tangent_derivative)/math.sqrt(q2)
