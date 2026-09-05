"""Stateless, right-positive Frenet bicycle controller.

e [m] and h [rad] are LEFT-positive errors. Curvature [1/m], road-wheel
angle [rad] and the returned normalized SDK command are RIGHT-positive.
See STEERING_AUDIT.md for identification limits, equations and closed-loop
tests with an independent plant. No moving average, integrator or turn latch.
"""
import math


WHEELBASE_M = 3.8
# Equivalent lock identified from same-LaneId quasi-steady 2026-08-16 samples:
# 0.705..0.809 rad for the 3.8 m reference chassis (median 0.788). This is NOT
# an SDK universal constant. Physical wheel telemetry is logged independently
# so another chassis/input configuration can be identified rather than hidden
# by a permanent feedback offset. The former 0.28 was a turning-radius design
# choice, incorrectly treated as the game's actual input-to-tyre conversion.
# This is a conservative *provisional* default for the one truck/input setup
# represented by the 2026-08-16 log.  Runtime callers must pass the value from
# settings explicitly; keeping the default here is useful for pure geometry
# tools and backwards-compatible unit tests, not a claim that every ETS2
# chassis has the same input-to-tyre conversion.
REFERENCE_LOCK_RAD = 0.78
MIN_STEERING_LOCK_RAD = 0.60
MAX_STEERING_LOCK_RAD = 0.95
GAME_RESPONSE_S = 0.32
TRANSPORT_S = 0.10
ACTUATION_PREVIEW_S = GAME_RESPONSE_S + TRANSPORT_S
# Critical-damping spatial length: two wheelbases rounded to the 2 m map
# sampling scale. With a measured yaw predictor, add one wheelbase to the
# response distance at road speed. Without yaw, use a slower feedback length
# (twice the nominal delay) rather than pretending a predicted pose exists.
# The old ~1 Hz log cannot identify sub-frame transport/lag separately; these
# nominal model values are stress-tested at longer delay, not measured maxima.
FEEDBACK_LENGTH_BASE_M = 8.0
FEEDBACK_RESPONSE_S = 2 * ACTUATION_PREVIEW_S


def solve(curvature, preview_curvature, cte_m, heading_error_rad, speed_ms,
          trailer_offset_m=0.0, vehicle_curvature_per_m=None,
          response_s=ACTUATION_PREVIEW_S, steering_lock_rad=REFERENCE_LOCK_RAD):
    """One curvature demand, followed by exactly one inverse bicycle mapping.

    e_dot=v*sin(h), h_dot=v*k*cos(h)/(1+k*e)-v*k_vehicle.
    Without prediction, with preview==k, constant speed and no trailer offset the commanded
    curvature gives e_ddot + 2*v/ell*e_dot + (v/ell)^2*e = 0 exactly.
    Preview changes only the feed-forward along confirmed geometry, never the
    tangent or the point to which CTE is measured.
    """
    values=(curvature, preview_curvature, cte_m, heading_error_rad,
            speed_ms, trailer_offset_m)
    if not all(math.isfinite(float(v)) for v in values):
        return 0.0, {"valid": False, "reason": "non-finite lateral input"}
    k,kp,e,h,v,offset=map(float,values)
    if not (math.isfinite(response_s) and 0.0 <= response_s <= 1.0
            and math.isfinite(steering_lock_rad)
            and MIN_STEERING_LOCK_RAD <= steering_lock_rad
                <= MAX_STEERING_LOCK_RAD):
        return 0.0, {"valid": False, "reason": "invalid actuator calibration"}
    # Predict errors through the identified command transport+tyre response
    # using the CURRENT physical yaw curvature. This is a plant predictor,
    # not a stored/filtered target. The same Frenet equations used by the
    # feedback proof advance pose and direction together.
    if vehicle_curvature_per_m is not None:
        kv=float(vehicle_curvature_per_m)
        if not math.isfinite(kv):
            return 0.0, {"valid": False, "reason": "non-finite vehicle curvature"}
        duration=response_s
        substeps=8
        dt=duration/substeps
        for index in range(substeps):
            predicted_k=k+(kp-k)*(index+.5)/substeps
            if 1+predicted_k*e <= .10:
                return 0.0, {"valid": False, "reason": "invalid predicted Frenet frame"}
            hd=v*(predicted_k*math.cos(h)/(1+predicted_k*e)-kv)
            mid_h=h+hd*dt*.5
            e+=v*math.sin(mid_h)*dt
            h+=hd*dt
    cosine=math.cos(h)
    denominator=1+k*e
    if cosine <= .10 or denominator <= .10:
        return 0.0, {"valid": False, "reason": "invalid Frenet frame"}
    length=(max(FEEDBACK_LENGTH_BASE_M, WHEELBASE_M+abs(v)*response_s)
            if vehicle_curvature_per_m is not None else
            FEEDBACK_LENGTH_BASE_M+abs(v)*FEEDBACK_RESPONSE_S)
    foundation=kp*cosine/denominator
    heading=2*math.tan(h)/length
    lateral=(e+offset)/(length*length*cosine)
    demand=foundation+heading+lateral
    angle=math.atan(WHEELBASE_M*demand)
    base_angle=math.atan(WHEELBASE_M*foundation)
    lateral_angle=math.atan(WHEELBASE_M*(foundation+lateral))-base_angle
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
        trailer_target_m=-offset)


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
