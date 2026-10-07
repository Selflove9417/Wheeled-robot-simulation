#pragma once

#include <algorithm>
#include <cmath>
#include "bbot_balance_controller/flight_joint_pd.hpp"

namespace bbot_jump {

// Independent, timestamped 10 ms filter for the torso loop. Do not change the
// established gyro filtering used by airborne/wheel control. At 100 Hz the old
// alpha=.15 gyro lags a contact-induced direction reversal by several samples.
class TorsoImuObserver {
    double stamp_=-1.0, period_=0.010;
    double rate_=0.0, fy_=0.0, fz_=9.81;
public:
    void update(double stamp, double rate, double fy, double fz) {
        if (!std::isfinite(stamp) || stamp<=0.0 || !std::isfinite(rate) ||
            !std::isfinite(fy) || !std::isfinite(fz)) return;
        if (stamp==stamp_) return;
        const double dt=stamp-stamp_;
        if (stamp_<0.0 || dt<=0.0 || dt>0.080) {
            rate_=rate; fy_=fy; fz_=fz; period_=0.010;
        } else {
            const double alpha=1.0-std::exp(-dt/0.010);
            rate_+=alpha*(rate-rate_); fy_+=alpha*(fy-fy_); fz_+=alpha*(fz-fz_);
            period_+=0.2*(std::clamp(dt,0.005,0.030)-period_);
        }
        stamp_=stamp;
    }
    bool fresh(double now) const {
        return std::isfinite(now) && stamp_>=0.0 && now>=stamp_ && now-stamp_<=0.080;
    }
    double stamp() const { return stamp_; }
    double period() const { return period_; }
    double rate() const { return rate_; }
    double fy() const { return fy_; }
    double fz() const { return fz_; }
};

struct TorsoPitchTorque {
    double force_feedforward=0.0; // per hip
    double feedback=0.0;         // per hip
    double command=0.0;          // bounded mean of the two hip torques
};

// Touchdown convergence may run only after the support controller has fully
// switched to Effort.  This includes a first Position-mode jump after touchdown.
inline bool touchdown_effort_support_active(bool effort_mode_active,
                                           bool leg_mode_switch_pending) {
    return effort_mode_active && !leg_mode_switch_pending;
}

// Keep the asymmetric impact correction until the short impact window and
// centroidal capture state are quiet. The controller latches this entry test.
inline bool touchdown_torso_convergence_ready(bool effort_support_active,
                                               double touchdown_elapsed,
                                               bool fresh_world_com,
                                               double com_lean,
                                               double com_velocity,
                                               double body_pitch_error) {
    return effort_support_active && std::isfinite(touchdown_elapsed) &&
        touchdown_elapsed >= 0.50 && fresh_world_com &&
        std::isfinite(com_lean) && std::abs(com_lean) <= 0.10 &&
        std::isfinite(com_velocity) && std::abs(com_velocity) <= 0.35 &&
        std::isfinite(body_pitch_error) && std::abs(body_pitch_error) <= 0.15;
}

inline double touchdown_torso_convergence_blend(double elapsed_since_latch,
                                                  double duration=0.20) {
    if (!std::isfinite(elapsed_since_latch) || !std::isfinite(duration) || duration<=0.0)
        return 0.0;
    const double u=std::clamp(elapsed_since_latch/duration,0.0,1.0);
    return u*u*(3.0-2.0*u);
}

inline TorsoPitchTorque blend_torso_pitch_torque(const TorsoPitchTorque & impact,
                                                  const TorsoPitchTorque & balance,
                                                  double blend) {
    TorsoPitchTorque out;
    if (!std::isfinite(blend)) return out;
    const double b=std::clamp(blend,0.0,1.0);
    out.force_feedforward=b*balance.force_feedforward;
    out.feedback=(1.0-b)*impact.command+b*balance.feedback;
    // Blend already bounded endpoint commands. The diagnostic component sum
    // may differ from command when the balance endpoint saturated at its cap.
    out.command=(1.0-b)*impact.command+b*balance.command;
    return out;
}

// The IMU sits at the box COM (<0.04 mm offset in current URDF), with axes
// aligned to base_link. f is measured specific force, not world acceleration.
// Box moment balance about its COM, pitch=-roll:
//   I_COM*pitch_ddot = tau_HL+tau_HR - m*(r_z*f_y-r_y*f_z).
// r is hip->box COM: (Y=.00761282, Z=.12396677). This accounts for both gravity
// and translational contact forces without adding a second J^T F hip command.
// The requested box feedback is discretized once using box inertia, not the
// free-leg Schur inertia, and clipped only after that solve.
inline TorsoPitchTorque torso_pitch_torque(
    double pitch, double reference, double rate, double fy, double fz,
    double body_mass, double kp, double kd, double horizon, double per_hip_limit)
{
    TorsoPitchTorque out;
    if (!std::isfinite(pitch) || !std::isfinite(reference) || !std::isfinite(rate) ||
        !std::isfinite(fy) || !std::isfinite(fz) || !std::isfinite(body_mass) || body_mass<=0 ||
        !std::isfinite(kp) || !std::isfinite(kd) || kp<0 || kd<0 ||
        !std::isfinite(horizon) || !std::isfinite(per_hip_limit) || per_hip_limit<=0) return out;
    const double h=std::clamp(horizon,0.030,0.080);
    const double inertia=0.159013*body_mass/14.0;
    out.force_feedforward=0.5*body_mass*(0.12396677*fy-0.00761282*fz);
    out.feedback=0.5*inertia/(inertia+h*kd+h*h*kp)*
        (-kp*(pitch-reference)-(kd+h*kp)*rate);
    out.command=std::clamp(out.force_feedforward+out.feedback,-per_hip_limit,per_hip_limit);
    return out;
}

// Reserve the common hip mode for the box. Leg support and shape feedback
// contribute only to the left/right difference; neither can cancel box PD.
// Limit the difference before output clipping, preserving the requested mean.
inline JointVector allocate_torso_hips(const JointVector & leg_torque,
                                      double torso_per_hip, double hip_limit) {
    JointVector result=leg_torque;
    const double common=std::clamp(torso_per_hip,-hip_limit,hip_limit);
    const double room=hip_limit-std::abs(common);
    const double difference=std::clamp(0.5*(leg_torque[0]-leg_torque[2]),-room,room);
    result[0]=common+difference;
    result[2]=common-difference;
    return result;
}

/**
 * @brief 计算着陆缓冲期单髋姿态恢复力矩（含非对称阻尼）
 *
 * 控制律约定:
 *   tau_body_per_hip = -0.5 * (p_term + d_term)
 * 其中:
 *   p_term = kp * pitch_err
 *   d_term = kd * pitch_rate
 *
 * 非对称阻尼机制:
 *   当机身仍明显倾斜 (|pitch_err| > neutral_threshold, 默认 0.10 rad)
 *   且正在朝向零误差回正 (p_term * d_term < 0) 时，
 *   阻尼项会严重抵消比例回正力矩甚至导致提前刹停后倒。
 *   因此在该区域限制阻尼项最多抵消比例项的 max_opposing_ratio (默认 45%):
 *     |d_term| <= max_opposing_ratio * |p_term|
 *   当机身接近中立区 (|pitch_err| in [neutral_lower, neutral_upper]) 时，
 *   阻尼限制平滑连续过渡，直至进入深中立区 (|pitch_err| <= neutral_lower) 恢复完整阻尼。
 *
 * @param pitch_err 俯仰角误差 pitch - pitch_ref (rad)
 * @param pitch_rate 俯仰角速度 (rad/s)
 * @param kp 姿态刚度增益 (默认 55.0 Nm/rad)
 * @param kd 姿态阻尼增益 (默认 15.0 Nm*s/rad)
 * @param max_limit 单髋力矩限幅 (默认 20.0 Nm)
 * @param neutral_lower 完全恢复完整阻尼的下阈值 (默认 0.06 rad)
 * @param neutral_upper 开始逐渐过渡阻尼的上阈值 (默认 0.25 rad)
 * @param max_opposing_ratio 回正过程中深倾角阻尼项最大允许抵消比例 (默认 0.45)
 * @return double 单髋姿态控制力矩 (Nm)
 */
inline double touchdown_torso_pitch_torque(
    double pitch_err,
    double pitch_rate,
    double kp = 55.0,
    double kd = 15.0,
    double max_limit = 20.0,
    double neutral_lower = 0.06,
    double neutral_upper = 0.25,
    double max_opposing_ratio = 0.45)
{
    if (!std::isfinite(pitch_err) || !std::isfinite(pitch_rate) ||
        !std::isfinite(kp) || kp < 0.0 ||
        !std::isfinite(kd) || kd < 0.0 ||
        !std::isfinite(max_limit) || max_limit <= 0.0 ||
        !std::isfinite(neutral_lower) || !std::isfinite(neutral_upper) ||
        neutral_lower < 0.0 || neutral_lower >= neutral_upper ||
        !std::isfinite(max_opposing_ratio) || max_opposing_ratio < 0.0)
    {
        return 0.0;
    }

    const double p_term = kp * pitch_err;
    const double d_raw = kd * pitch_rate;
    double d_term = d_raw;

    // 非对称阻尼：当机身正在朝向零误差回正时 (p_term 与 d_raw 异号)
    if (p_term * d_raw < 0.0)
    {
        const double err_mag = std::abs(pitch_err);
        const double max_opposing_d = max_opposing_ratio * std::abs(p_term);
        const double d_clamped = std::clamp(d_raw, -max_opposing_d, max_opposing_d);

        if (err_mag >= neutral_upper)
        {
            // 深倾角区：保持严格非对称限制
            d_term = d_clamped;
        }
        else if (err_mag <= neutral_lower)
        {
            // 深中立区：完全恢复 100% 阻尼
            d_term = d_raw;
        }
        else
        {
            // 过渡区：在 [neutral_lower, neutral_upper] 采用平滑 Hermite 插值
            const double u = (neutral_upper - err_mag) / (neutral_upper - neutral_lower);
            const double s = u * u * (3.0 - 2.0 * u); // 0 (at upper) -> 1 (at lower)
            d_term = (1.0 - s) * d_clamped + s * d_raw;
        }
    }

    const double raw_tau = -0.5 * (p_term + d_term);
    return std::clamp(raw_tau, -max_limit, max_limit);
}

/**
 * @brief 触地缓冲期髋关节预测软限位与主动回位保护
 *
 * 针对机身姿态调整中髋关节大摆角接近/撞击 1.57 rad 机械硬限位的风险：
 * 1. 采用前向预瞻（predict_time = 0.025 s）：q_pred = position + predict_time * max(0.0, velocity)；
 * 2. 安全区（q <= warning 且 q_pred <= warning）：完全放行请求力矩；
 * 3. 明确远离限位运动（velocity <= recede_vel_thresh 且 requested_torque <= 0）：完全解除保护放行负力矩；
 * 4. 处于警戒区或正向逼近时（q_eval > warning_position）：
 *    - 严禁施加正向驱动力矩；
 *    - 施加主动负向回位弹簧与阻尼力矩，即使在撞击后速度为零时也能强制拉回；
 *    - 软上限（hard_limit = 1.45 rad）处回位力矩打满最大限幅。
 *
 * @param position 当前髋关节位置 (rad)
 * @param velocity 当前髋关节速度 (rad/s)
 * @param requested_torque 当前期望髋关节力矩 (Nm)
 * @param torque_limit 最大力矩限幅 (默认 20.0 Nm)
 * @param warning_position 警戒位置 (默认 1.20 rad)
 * @param hard_limit 目标软上限 (默认 1.45 rad)
 * @param predict_time 前向预测时间 (默认 0.025 s)
 * @param recede_vel_thresh 离开限位速度门限 (默认 -0.05 rad/s)
 * @return double 经过软限位与回位保护后的髋关节力矩指令 (Nm)
 */
inline double touchdown_hip_soft_limit_guard(
    double position,
    double velocity,
    double requested_torque,
    double torque_limit = 20.0,
    double warning_position = 1.20,
    double hard_limit = 1.45,
    double predict_time = 0.025,
    double recede_vel_thresh = -0.05)
{
    if (!std::isfinite(position) || !std::isfinite(velocity) ||
        !std::isfinite(requested_torque) || !std::isfinite(torque_limit) ||
        torque_limit <= 0.0 || !std::isfinite(warning_position) ||
        !std::isfinite(hard_limit) || warning_position >= hard_limit ||
        !std::isfinite(predict_time) || predict_time <= 0.0)
    {
        return 0.0;
    }

    const double q_pred = position + predict_time * std::max(0.0, velocity);

    // 1. 安全区：当前位置与预测位置均在警戒线以内，完全放行
    if (position <= warning_position && q_pred <= warning_position)
    {
        return std::clamp(requested_torque, -torque_limit, torque_limit);
    }

    // 2. 明确离开限位运动（已退至软上限以内、速度为负且控制器正在请求负向拉回力矩）：完全解除保护放行
    if (position <= hard_limit && velocity <= recede_vel_thresh && requested_torque <= 0.0)
    {
        return std::clamp(requested_torque, -torque_limit, torque_limit);
    }

    // 3. 处于警戒区或正向逼近：严禁正向力矩，并施加主动负向回位弹簧与阻尼
    const double k_spring = torque_limit / (hard_limit - warning_position);
    // 弹簧回位力基于真实位置超出门限程度（若未越界则为0）
    const double spring_mag = (position > warning_position)
        ? std::clamp(k_spring * (position - warning_position), 0.0, torque_limit)
        : 0.0;
    // 预瞻阻尼基于预测侵入程度与正向速度
    const double pred_penetration = std::max(0.0, q_pred - warning_position);
    const double zone = std::clamp(pred_penetration / (hard_limit - warning_position), 0.0, 1.0);
    const double damping_mag = std::clamp(zone * 2.0 * std::max(0.0, velocity), 0.0, torque_limit);
    const double total_brake_mag = std::clamp(spring_mag + damping_mag, 0.0, torque_limit);
    const double braking_torque = -total_brake_mag;

    // 必须禁止正力矩，且至少输出负向回位力矩
    const double allowable_torque = std::min(requested_torque, braking_torque);
    return std::clamp(allowable_torque, -torque_limit, torque_limit);
}

} // namespace bbot_jump
