"""Stateless, right-positive Frenet bicycle controller.

e [m] and h [rad] are LEFT-positive errors. Curvature [1/m], road-wheel
angle [rad] and the returned normalized SDK command are RIGHT-positive.
See STEERING_AUDIT.md for identification limits, equations and closed-loop
tests with an independent plant. No moving average, integrator or turn latch.
"""
import math


WHEELBASE_M = 3.8
# Equivalent lock identified from the latest same-LaneId quasi-steady
# 2026-09-06 samples: 0.698..0.704 rad for the current truck/input setup. This is NOT
# an SDK universal constant. Physical wheel telemetry is logged independently
# so another chassis/input configuration can be identified rather than hidden
# by a permanent feedback offset. The former 0.28 was a turning-radius design
# choice, incorrectly treated as the game's actual input-to-tyre conversion.
# This is a conservative *provisional* default for the one truck/input setup
# represented by the 2026-09-06 log.  Runtime callers must pass the value from
# settings explicitly; keeping the default here is useful for pure geometry
# tools and backwards-compatible unit tests, not a claim that every ETS2
# chassis has the same input-to-tyre conversion.
REFERENCE_LOCK_RAD = 0.70
MIN_STEERING_LOCK_RAD = 0.60
MAX_STEERING_LOCK_RAD = 0.95
GAME_RESPONSE_S = 0.32
TRANSPORT_S = 0.10
ACTUATION_PREVIEW_S = GAME_RESPONSE_S + TRANSPORT_S
# Critical-damping spatial length: two wheelbases rounded to the 2 m map
# sampling scale.  The response distance weakens feedback at speed without
# pretending that the current yaw rate will remain constant through the whole
# actuator delay.
# The old ~1 Hz log cannot identify sub-frame transport/lag separately; these
# nominal model values are stress-tested at longer delay, not measured maxima.
FEEDBACK_LENGTH_BASE_M = 8.0
FEEDBACK_RESPONSE_S = 2 * ACTUATION_PREVIEW_S
# Below this spatial curvature, wheel/yaw observations are not distinguishable
# from the corrective transients seen on the 2026-09-06 straight (R > 500 m).
# The continuous squared ratio below has no threshold, latch or temporal state.
CURVATURE_OBSERVABILITY_FLOOR_PER_M = 1.0 / 500.0
PREDICTOR_SUBSTEPS = 8
PREDICTOR_ITERATIONS = 5


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
    # Vehicle curvature is a delayed physical response, not a known future
    # input.  The former predictor held one sample constant for the complete
    # response horizon and thereby turned ordinary lag into positive feedback.
    if vehicle_curvature_per_m is not None:
        kv=float(vehicle_curvature_per_m)
        if not math.isfinite(kv):
            return 0.0, {"valid": False, "reason": "non-finite vehicle curvature"}
    length=(max(FEEDBACK_LENGTH_BASE_M,
                WHEELBASE_M+abs(v)*response_s)
            if vehicle_curvature_per_m is not None else
            FEEDBACK_LENGTH_BASE_M+abs(v)*FEEDBACK_RESPONSE_S)

    def control_at(error_m, error_heading):
        cosine=math.cos(error_heading)
        denominator=1+k*error_m
        if cosine <= .10 or denominator <= .10:
            return None
        foundation=kp*cosine/denominator
        heading=2*math.tan(error_heading)/length
        lateral=(error_m+offset)/(length*length*cosine)
        return foundation+heading+lateral,foundation,heading,lateral

    control=control_at(e,h)
    if control is None:
        return 0.0, {"valid": False, "reason": "invalid Frenet frame"}
    demand,foundation,heading,lateral=control
    predicted_e,predicted_h=e,h
    observation_weight=0.0
    if vehicle_curvature_per_m is not None and response_s > 0.0:
        # The path supplies observability: on a confirmed bend physical wheel
        # curvature is useful actuator-state evidence; on a geometrically
        # straight road its small alternating residual is old command response.
        signal=max(abs(k),abs(kp))
        floor=CURVATURE_OBSERVABILITY_FLOOR_PER_M
        observation_weight=signal*signal/(signal*signal+floor*floor)
        measured_state=(foundation
                        +observation_weight*(kv-foundation))
        dt=response_s/PREDICTOR_SUBSTEPS
        transport=min(TRANSPORT_S,response_s)
        plant_tau=max(GAME_RESPONSE_S,response_s)
        original_e,original_h=e,h
        # Fixed-point solution of command -> first-order tyre response ->
        # predicted Frenet error -> command.  Relaxation is internal numerical
        # convergence within this tick, never a filter across ticks.
        for _iteration in range(PREDICTOR_ITERATIONS):
            predicted_e,predicted_h=original_e,original_h
            valid_prediction=True
            for index in range(PREDICTOR_SUBSTEPS):
                elapsed=(index+.5)*dt
                path_k=k+(kp-k)*(index+.5)/PREDICTOR_SUBSTEPS
                if elapsed <= transport:
                    vehicle_k=measured_state
                else:
                    vehicle_k=(demand+(measured_state-demand)*math.exp(
                        -(elapsed-transport)/plant_tau))
                denominator=1+path_k*predicted_e
                if denominator <= .10:
                    valid_prediction=False
                    break
                heading_rate=v*(path_k*math.cos(predicted_h)/denominator
                                -vehicle_k)
                midpoint_heading=predicted_h+heading_rate*dt*.5
                predicted_e+=v*math.sin(midpoint_heading)*dt
                predicted_h+=heading_rate*dt
            if not valid_prediction:
                return 0.0, {
                    "valid": False,
                    "reason": "invalid predicted Frenet frame",
                }
            predicted_control=control_at(predicted_e,predicted_h)
            if predicted_control is None:
                return 0.0, {
                    "valid": False,
                    "reason": "invalid predicted Frenet frame",
                }
            predicted_demand=predicted_control[0]
            demand=.5*demand+.5*predicted_demand
        final_control=control_at(predicted_e,predicted_h)
        demand,foundation,heading,lateral=final_control
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
        measured_vehicle_curvature_per_m=(
            float(vehicle_curvature_per_m)
            if vehicle_curvature_per_m is not None else None),
        curvature_observation_weight=observation_weight,
        predicted_frenet_cte_m=predicted_e,
        predicted_frenet_heading_error_rad=predicted_h,
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
