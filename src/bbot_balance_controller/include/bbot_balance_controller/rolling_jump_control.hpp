#pragma once
#include <algorithm>
#include <cmath>

namespace bbot_jump
{
// 工具函数
inline double clamp_value(double value, double min_value, double max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

inline double deadband(double value, double threshold)
{
    if (std::abs(value) < threshold)
        return 0.0;
    return (value > 0.0) ? (value - threshold) : (value + threshold);
}

// 本地日志显示2 s时仍在正常加速；保留严格准入条件，给准备过程4 s预算。
constexpr double kDefaultPreJumpTimeout = 4.0;

inline bool rolling_prepare_ready(double velocity, double target, double ramped_target,
                                  double pitch_error, double pitch_rate,
                                  double velocity_tolerance, double pitch_tolerance,
                                  double rate_limit)
{
    return std::abs(ramped_target - target) <= 0.001 &&
        std::abs(velocity - target) <= velocity_tolerance &&
        std::abs(pitch_error) <= pitch_tolerance && std::abs(pitch_rate) <= rate_limit;
}

inline double rolling_reference_blend(double initial, double final, double progress)
{
    const double s = std::clamp(progress, 0.0, 1.0);
    return initial + (final - initial) * s * s * (3.0 - 2.0 * s);
}

inline double rolling_abort_wheel_command(double target, double previous, double dt)
{
    const double bounded = std::clamp(target, -1.20, 1.20);
    const double step = 5.0 * std::clamp(dt, 0.0, 0.05);
    return previous + std::clamp(bounded - previous, -step, step);
}

/**
 * @brief 计算空中姿态反作用轮速度控制目标
 *
 * 物理坐标系约定:
 * - 轮子正转前进产生负轮速 (wl < 0)
 * - 机身前倾为正 (pitch > 0), 后仰为负 (pitch < 0)
 * - 机身后仰 (pitch_err < 0, gyro < 0) 时，轮子需要向后加速 (正向轮加速度)
 *   通过动量守恒产生机身前倾反作用力矩 (ddot{pitch} > 0)。
 * - 因此 air_wheel_sign 默认必须为 -1.0。
 * - ATTITUDE_ARREST 阶段推地结束膝关节制动产生负向俯仰冲击，前馈反作用轮命令
 *   必须与反馈同向 (当 air_wheel_sign=-1.0 时为正向轮速前馈)。
 *
 * @param baseline_wheel_speed 离地时锁存的轮速基准 (m/s)
 * @param air_pitch_err 俯仰角误差 pitch - pitch_ref (rad)
 * @param air_pitch_rate_err 俯仰角速度误差 pitch_rate - pitch_rate_ref (rad/s)
 * @param knee_speed_mag 膝关节平均速度幅值 0.5 * (|knee_l| + |knee_r|) (rad/s)
 * @param elapsed 当前空中子阶段经历时间 (s)
 * @param is_attitude_arrest 是否处于 ATTITUDE_ARREST 阶段
 * @param air_wheel_sign 反作用轮反馈符号映射 (默认 -1.0)
 * @param k_air_p 比例增益 (默认 0.50)
 * @param k_air_d 微分增益 (默认 0.45)
 * @param speed_limit 轮速绝对值限幅 (默认 1.40 m/s)
 * @param arrest_ff_out 可选输出: 前馈补偿分量 (m/s)
 * @param raw_target_out 可选输出: 限幅前的原始指令 (m/s)
 * @return double 限幅后的轮速目标 (m/s)
 */
inline double flight_wheel_target(
    double baseline_wheel_speed,
    double air_pitch_err,
    double air_pitch_rate_err,
    double knee_speed_mag,
    double elapsed,
    bool is_attitude_arrest,
    double air_wheel_sign = -1.0,
    double k_air_p = 0.50,
    double k_air_d = 0.45,
    double speed_limit = 1.40,
    double * arrest_ff_out = nullptr,
    double * raw_target_out = nullptr)
{
    const double air_p_term = k_air_p * deadband(air_pitch_err, 0.02);
    const double air_d_term = k_air_d * deadband(air_pitch_rate_err, 0.08);

    double arrest_wheel_ff = 0.0;
    if (is_attitude_arrest)
    {
        const double speed_ff = clamp_value((knee_speed_mag - 4.0) / 6.0, 0.0, 1.0);
        const double time_ff = 1.0 - clamp_value(elapsed / 0.10, 0.0, 1.0);
        arrest_wheel_ff = -air_wheel_sign * 0.35 * speed_ff * time_ff;
    }

    const double raw_target = baseline_wheel_speed + arrest_wheel_ff +
                              air_wheel_sign * (air_p_term + air_d_term);

    if (arrest_ff_out != nullptr)
    {
        *arrest_ff_out = arrest_wheel_ff;
    }
    if (raw_target_out != nullptr)
    {
        *raw_target_out = raw_target;
    }

    return clamp_value(raw_target, -speed_limit, speed_limit);
}
} // namespace bbot_jump
