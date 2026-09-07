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

// Reaching launch speed ends the propulsive pulse once. A later speed drop
// must not restart thrust and drive the knee into its travel limit.
class ThrustRelease {
    double start_=-1.0;
    double force_=0.0;
    double brake_=0.0;
public:
    void reset() { start_=-1.0; force_=0.0; brake_=0.0; }
    bool active() const { return start_>=0.0; }
    void update(double now, double phase, double velocity, double target, double force) {
        if (!active() && std::isfinite(now) && target>0.0 && phase>=0.75 &&
            velocity>=0.95*target) { start_=now; force_=std::max(0.0,force); }
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

inline bool touchdown_capture_lost(double pitch_error, double rate, double capture) {
    return std::abs(capture)>0.16 && std::abs(pitch_error)>0.06 && pitch_error*rate>0.0;
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
    return valid && std::isfinite(lean) && std::isfinite(rate) && std::isfinite(velocity) &&
        std::abs(lean)<=0.03 && std::abs(rate)<=0.15 && std::abs(velocity)<=0.08;
}
} // namespace bbot_jump
