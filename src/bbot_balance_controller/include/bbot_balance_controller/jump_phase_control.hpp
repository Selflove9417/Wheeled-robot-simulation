#pragma once
#include <algorithm>
#include <cmath>

namespace bbot_jump {
struct PitchReference { double angle; double rate; };

// Integrate a smooth rate decay. After the finite lead, both references hold.
inline PitchReference thrust_pitch_reference(double initial, double initial_rate,
                                             double lead_duration, double elapsed) {
    if (lead_duration <= 0.0) return {initial, 0.0};
    const double u = std::clamp(elapsed / lead_duration, 0.0, 1.0);
    const double rate_scale = 1.0 - 3.0*u*u + 2.0*u*u*u;
    const double integrated = u - u*u*u + 0.5*u*u*u*u;
    return {initial + initial_rate*lead_duration*integrated, initial_rate*rate_scale};
}

// Once the nominal stroke ends, a force-controlled extension must not silently
// turn into a zero-speed servo. Keep only the safe extension component; travel
// protection and the terminal braking blend still reduce it before a hard stop.
inline double thrust_extension_velocity(double measured, double speed_limit,
                                         double extension_sign, double travel_scale) {
    return extension_sign * std::clamp(extension_sign*measured,0.0,speed_limit) *
        std::clamp(travel_scale,0.0,1.0);
}

// Start unloading before the sampled COM speed reaches the ballistic target.
// The synchronized COM estimate arrives at about 50 Hz, while the force ramp
// needs 40 ms to decay.  Waiting for 95% therefore adds another complete
// ground-reaction impulse and overshoots the requested apex.  The 86% lead
// lets that already-commanded impulse carry the COM to the target.  A later
// speed drop must not restart thrust and drive the knee into its travel limit.
class ThrustRelease {
    double start_=-1.0;
    double force_=0.0;
    double brake_=0.0;
public:
    void reset() { start_=-1.0; force_=0.0; brake_=0.0; }
    bool active() const { return start_>=0.0; }
    void update(double now, double phase, double velocity, double target, double force) {
        // phase>0 means the attitude gate has opened and the propulsive stroke
        // has actually started.  The lead threshold covers one estimator
        // sample plus the finite unloading ramp; it is deliberately separate
        // from the 95% takeoff-speed latch used for event reporting.
        if (!active() && std::isfinite(now) && target>0.0 && phase>0.0 &&
            velocity>=0.70*target) { start_=now; force_=std::max(0.0,force); }
    }
    double blend(double now) const {
        if (!active()) return 0.0;
        const double u=std::clamp((now-start_)/0.040,0.0,1.0);
        return u*u*u*(10.0-15.0*u+6.0*u*u);
    }
    double brake_blend(double now, double candidate) {
        if (!active()) return candidate;
        brake_=std::max({brake_,candidate,blend(now)});
        return brake_;
    }
    double force_limit(double now) const { return force_*(1.0-blend(now)); }
};

// The speed event precedes geometric airborne confirmation by several samples.
// Once unloading has started, keeping the grounded brake disabled lets the leg
// run into its hard stop during that confirmation gap. Brake residual extension
// after either event; before both events the propulsive stroke is untouched.
inline double thrust_brake_factor(bool wheels_airborne, bool speed_release_active,
                                  double requested_blend) {
    return (wheels_airborne || speed_release_active)
        ? std::clamp(requested_blend, 0.0, 1.0)
        : 0.0;
}

// Opt-in diagnostic: isolate the short negative hip command at the end of
// THRUST without weakening knee travel braking. Never increases hip effort.
inline double thrust_terminal_hip_command(double requested, bool enabled,
                                          bool release_active, bool geometry_valid,
                                          double previous_clearance, bool imu_fresh,
                                          double raw_pitch_rate) {
    if (!enabled || !release_active || !geometry_valid || !imu_fresh ||
        previous_clearance < 0.010 || raw_pitch_rate < 0.50)
        return requested;
    return std::max(0.0, requested);
}

inline double ballistic_time_to_height(double height, double velocity,
                                       double target_height) {
    if (!std::isfinite(height) || !std::isfinite(velocity) ||
        !std::isfinite(target_height)) return 0.0;
    const double dz = std::max(0.0, height - target_height);
    const double discriminant = std::max(0.0, velocity * velocity + 2.0 * 9.81 * dz);
    return std::max(0.0, (velocity + std::sqrt(discriminant)) / 9.81);
}

struct WheelFirstTarget {
    double knee_position;
    double knee_velocity;
    bool active;
};

// A wheel-first constraint cannot instantaneously move a leg that is already
// outside the landing cone. Projecting the complete error into one reference
// sample creates exactly the joint-velocity reversal and body impulse that the
// constraint is meant to prevent. Apply only bounded position/rate corrections;
// the admissible landing trajectory remains responsible for reaching the safe
// terminal geometry before contact.
inline WheelFirstTarget bounded_wheel_first_target(
    double hip_position, double knee_position,
    double hip_velocity, double knee_velocity,
    double body_pitch, double body_pitch_rate,
    double shank_zero, double shank_limit,
    double max_position_correction = 0.03,
    double max_velocity_correction = 1.0) {
    if (!std::isfinite(hip_position) || !std::isfinite(knee_position) ||
        !std::isfinite(hip_velocity) || !std::isfinite(knee_velocity) ||
        !std::isfinite(body_pitch) || !std::isfinite(body_pitch_rate) ||
        !std::isfinite(shank_zero) || !std::isfinite(shank_limit) ||
        shank_limit <= 0.0) {
        return {knee_position, knee_velocity, false};
    }
    const double shank = shank_zero + hip_position + knee_position - body_pitch;
    const double safe = std::clamp(shank, -shank_limit, shank_limit);
    if (std::abs(safe - shank) <= 1e-9)
        return {knee_position, knee_velocity, false};

    const double position_step = std::clamp(
        safe - shank, -std::abs(max_position_correction),
        std::abs(max_position_correction));
    double projected_velocity = knee_velocity;
    const double shank_rate = hip_velocity + knee_velocity - body_pitch_rate;
    const bool moving_outward =
        (shank > shank_limit && shank_rate > 0.0) ||
        (shank < -shank_limit && shank_rate < 0.0);
    if (moving_outward) {
        const double zero_shank_rate = body_pitch_rate - hip_velocity;
        projected_velocity += std::clamp(
            zero_shank_rate - knee_velocity,
            -std::abs(max_velocity_correction),
            std::abs(max_velocity_correction));
    }
    return {knee_position + position_step, projected_velocity, true};
}

inline bool touchdown_capture_lost(double pitch_error, double rate, double capture) {
    return std::abs(capture)>0.16 && std::abs(pitch_error)>0.06 && pitch_error*rate>0.0;
}

// Once both the COM and support have crossed through zero velocity, continuing
// to chase a large configuration-dependent COM offset drives the wheels into
// a full reverse overshoot. The independent recapture guard restores the full
// law if the torso subsequently leaves the recoverable region.
inline bool landing_pitch_unrecoverable(double body_pitch, double body_rate,
                                        double soft_limit = 0.60,
                                        double hard_limit = 0.90) {
    if (!std::isfinite(body_pitch) || !std::isfinite(body_rate)) return true;
    if (std::abs(body_pitch) >= hard_limit) return true;
    return std::abs(body_pitch) > soft_limit && body_pitch * body_rate > 0.05;
}

// In REVERSE_BRAKE, the support wheels decelerate or back up to damp torso motion.
// If the centroidal capture state diverges backward below threshold in valid,
// fresh observations, authority must be returned to CATCH before the box pitch
// itself suffers large irreversible divergence.
inline bool reverse_brake_capture_diverged(bool capture_valid,
                                           double obs_age,
                                           double capture_state,
                                           double capture_threshold = -0.16,
                                           double max_obs_age = 0.080) {
    if (!capture_valid || !std::isfinite(capture_state)) return false;
    if (!std::isfinite(obs_age) || obs_age < 0.0 || obs_age > max_obs_age) return false;
    return capture_state < capture_threshold;
}

// Braking the rapidly extending knees while the thrust force is unloading
// injects an equal-and-opposite pitch impulse into the upper body.  Feed a
// bounded fraction of that *positive* knee braking torque through the hips;
// ordinary extension torque (negative in this mechanism) is handled by the
// existing velocity feedforward and must not be mirrored here.
inline double thrust_knee_brake_reaction_compensation(
    double left_knee_torque, double right_knee_torque,
    double gain = 0.40, double limit = 8.0) {
    if (!std::isfinite(left_knee_torque) || !std::isfinite(right_knee_torque) ||
        !std::isfinite(gain) || !std::isfinite(limit) || gain <= 0.0 || limit <= 0.0) {
        return 0.0;
    }
    const double mean_brake_torque = 0.5 * (
        std::max(0.0, left_knee_torque) + std::max(0.0, right_knee_torque));
    return std::clamp(gain * mean_brake_torque, 0.0, limit);
}

inline bool touchdown_reverse_brake_ready(double com_velocity,
                                          double support_velocity,
                                          double capture_state,
                                          double balance_angle,
                                          double balance_rate) {
    return std::isfinite(com_velocity) && std::isfinite(support_velocity) &&
        std::isfinite(capture_state) && std::isfinite(balance_angle) &&
        std::isfinite(balance_rate) &&
        com_velocity < -0.02 && support_velocity < -0.02 &&
        capture_state > -0.12 &&
        balance_angle > -0.10 &&
        balance_rate > -0.35;
}

// Positive knee torque flexes the leg.  During touchdown, an impact rebound
// can send q rapidly toward the negative (extension) hard stop while the
// vertical-force feedforward is still requesting negative torque.  Override
// only that outward component inside the soft zone; compression/flexion is
// left untouched.
inline double landing_knee_extension_guard(double position, double velocity,
                                           double requested_torque,
                                           double torque_limit = 60.0) {
    if (!std::isfinite(position) || !std::isfinite(velocity) ||
        !std::isfinite(requested_torque) || !std::isfinite(torque_limit) ||
        torque_limit <= 0.0) return 0.0;
    constexpr double warning_position = -1.30;
    constexpr double soft_limit = -1.45;
    if (velocity >= 0.0 || position > warning_position)
        return std::clamp(requested_torque, -torque_limit, torque_limit);
    const double zone = std::clamp(
        (warning_position - position) / (warning_position - soft_limit), 0.0, 1.0);
    const double braking = zone * (10.0 * (-velocity)) +
        180.0 * std::max(0.0, soft_limit - position);
    return std::clamp(std::max(requested_torque, braking),
                      -torque_limit, torque_limit);
}

inline double landing_common_hip_damping(double left_velocity,
                                         double right_velocity,
                                         double gain = 1.0,
                                         double limit = 12.0) {
    if (!std::isfinite(left_velocity) || !std::isfinite(right_velocity) ||
        !std::isfinite(gain) || gain < 0.0 ||
        !std::isfinite(limit) || limit <= 0.0) return 0.0;
    return std::clamp(-gain * 0.5 * (left_velocity + right_velocity),
                      -limit, limit);
}

struct LandingBufferProfile {
    double target_height;
    double duration;
};

// A long wheel-first landing must not immediately reuse the deep-crouch
// buffer target that was tuned for the old folded airborne pose.  Limit the
// first grounded compression stroke and size its smoothstep duration from the
// requested travel (smoothstep's peak rate is 1.5 * travel / duration).
inline LandingBufferProfile landing_buffer_profile(
    double touchdown_height, double minimum_height, double standing_height,
    double max_compression = 0.17, double max_target_rate = 0.50,
    double minimum_duration = 0.18) {
    if (!std::isfinite(touchdown_height) ||
        !std::isfinite(minimum_height) ||
        !std::isfinite(standing_height) ||
        !std::isfinite(max_compression) || max_compression <= 0.0 ||
        !std::isfinite(max_target_rate) || max_target_rate <= 0.0 ||
        !std::isfinite(minimum_duration) || minimum_duration <= 0.0) {
        return {minimum_height, minimum_duration};
    }
    const double lower = std::min(minimum_height, touchdown_height);
    const double upper = std::max(lower, std::min(standing_height, touchdown_height));
    const double target = std::clamp(
        touchdown_height - max_compression, lower, upper);
    const double travel = std::max(0.0, touchdown_height - target);
    return {target, std::max(minimum_duration,
                             1.5 * travel / max_target_rate)};
}

inline double touchdown_catch_limit(double pitch_error, double rate, double height) {
    const double omega=std::sqrt(9.81/std::clamp(height,0.15,0.55));
    const double capture=pitch_error+rate/omega;
    // Only widen the saturation envelope for a diverging body. No wheel-speed
    // based ratchet or accumulating command reference is introduced.
    return 0.90 + (pitch_error*rate>0.0 ?
        std::clamp(2.0*(std::abs(capture)-0.12),0.0,0.60) : 0.0);
}

inline bool touchdown_release_ready(double pitch_error, double rate, double capture,
                                     double wheel_velocity, double wheel_command) {
    return std::abs(pitch_error)<0.10 && std::abs(rate)<0.35 && std::abs(capture)<0.12 &&
        std::abs(wheel_velocity)<0.15 && std::abs(wheel_command)<0.20;
}

// Signed wheel command convention: negative command rolls the robot forward.
// Keep attitude feedback active for the entire grounded stroke, including overrun.
inline double thrust_ground_wheel_target(double forward_feedforward, double pitch_error,
                                         double rate_error, double kp, double kd) {
    return std::clamp(-forward_feedforward + 0.037*(kp*pitch_error + kd*rate_error),
                      -1.50, 1.50);
}

// Asymmetric rate limiter for ground wheel commands in THRUST.
// Signed convention: cmd < 0 is forward roll.
// When target < current, the controller is commanding further forward acceleration (limited by max_accel).
// When target > current, the controller is commanding deceleration/braking towards zero/positive (limited by max_decel).
inline double thrust_wheel_rate_limit(double target, double current, double dt,
                                      double max_accel = 8.0, double max_decel = 24.0) {
    const double valid_dt = std::max(dt, 0.001);
    const double max_step_accel = max_accel * valid_dt;
    const double max_step_decel = max_decel * valid_dt;
    const double delta = target - current;
    // delta < 0: accelerating forward (command becoming more negative) -> lower bound is -max_step_accel
    // delta > 0: decelerating/braking (command becoming more positive) -> upper bound is +max_step_decel
    const double clamped_delta = std::clamp(delta, -max_step_accel, max_step_decel);
    return current + clamped_delta;
}

inline double landing_wheel_blend(double previous, bool fresh_geometry,
                                  bool descending, double clearance) {
    if (!fresh_geometry || !descending || !std::isfinite(clearance)) return previous;
    const double u = std::clamp((0.080-clearance)/0.060, 0.0, 1.0);
    return std::max(previous, u*u*(3.0-2.0*u));
}

inline double catch_wheel_target(double pitch_error, double rate, double forward_velocity,
                                 double kp, double kd, double scale, double limit=0.90) {
    const double p = 0.55*kp*pitch_error;
    double d = 0.35*kd*rate;
    if (std::abs(pitch_error)>0.08 && p*d<0.0)
        d=std::clamp(d,-0.45*std::abs(p),0.45*std::abs(p));
    const double velocity = std::copysign(std::max(0.0,std::abs(forward_velocity)-0.03),forward_velocity);
    return std::clamp(scale*(p+d)+1.20*velocity,-limit,limit);
}

inline bool touchdown_capture_settled(double pitch_error, double rate, double capture) {
    // Large positive rate is forward divergence, not proof of recovery from backward lean.
    return std::abs(pitch_error)<0.10 && std::abs(rate)<0.35 && std::abs(capture)<0.12;
}

// Capture AND sustained ground hold: r = COM_x - axle_x.
// Request axle_v = 1.25*COM_v + omega*r. Do not switch it off when capture ends.
// For v_dot=omega²*r this gives r_dot=-omega*r-0.25*v: a critically damped
// capture AND stop (double pole -omega/2). Pure velocity matching would leave
// constant translation and never satisfy the low-speed release condition.
// COM_v is an independent
// world-pose estimate: never integrate wheel speed into a growing reference.
// Joint wheel velocity is relative to the shank. With +X joint axes and +Y
// forward, axle_v = -R*(wheel_rate + hip_rate + knee_rate - pitch_rate).
inline double centroidal_catch_target(double com_velocity, double forward,
                                      double height, double shank_rate,
                                      double wheel_radius, double speed_limit,
                                      double velocity_reference=0.0) {
    if (!std::isfinite(com_velocity) || !std::isfinite(forward) ||
        !std::isfinite(height) || height<=0 || !std::isfinite(shank_rate) ||
        !std::isfinite(wheel_radius) || wheel_radius<=0 ||
        !std::isfinite(speed_limit) || speed_limit<=0 ||
        !std::isfinite(velocity_reference)) return 0.0;
    const double omega=std::sqrt(9.81/std::clamp(height,0.15,0.55));
    // r_dot=-omega*r-0.25*(v-v_ref). At r=0,v=v_ref the axle follows v_ref;
    // v_ref=0 reproduces the validated capture/stop law exactly.
    return std::clamp(-1.25*com_velocity+0.25*velocity_reference-
                      omega*forward-wheel_radius*shank_rate,
                      -speed_limit,speed_limit);
}

inline double ground_drive_reference(double previous, double requested, double dt,
                                     double limit, double rate) {
    if (!std::isfinite(previous)) previous=0.0;
    if (!std::isfinite(requested)) requested=0.0;
    if (!std::isfinite(dt) || dt<=0) return previous;
    return previous+std::clamp(std::clamp(requested,-limit,limit)-previous,
                              -rate*std::min(dt,.020),rate*std::min(dt,.020));
}

// During a force-driven stroke, the nominal position trajectory is a lower
// extension bound, not a reason to pull an already extended knee back. Keep
// ALL velocity damping and restore bilateral P for braking/travel protection.
inline double thrust_knee_position_error(double desired, double measured,
                                         bool propelling) {
    const double error=desired-measured;
    return propelling ? std::min(0.0,error) : error; // extension is q_knee < 0
}

// A quiet torso and small wheel spin do not establish COM equilibrium while
// the legs extend. Require fresh world translation and centroidal attitude too.
inline bool centroidal_hold_ready(bool valid, double lean, double rate, double velocity) {
    // The balanced flat-ground configuration has a small, repeatable COM-to-axle
    // offset even when torso pitch and translation are essentially zero.  A
    // 0.03 rad zero-centred gate rejected that physical equilibrium forever.
    // Keep this tighter than the touchdown catch gate, but include the measured
    // standing equilibrium while still rejecting a clearly diverging lean.
    return valid && std::isfinite(lean) && std::isfinite(rate) && std::isfinite(velocity) &&
        std::abs(lean)<=0.06 && std::abs(rate)<=0.15 && std::abs(velocity)<=0.08;
}
} // namespace bbot_jump
