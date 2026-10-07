// 轮腿高度调度线性 MPC 平衡控制器 ROS 2 节点。
//
// 1. 状态定义与符号约定：
//    - 误差状态：X = [x - x_ref, x_dot - v_ref, pitch - theta_eq(H), pitch_rate]^T
//    - 传感器符号：pitch = -imu.roll, pitch_rate = -imu.omega_x, x = -R * avg_wheel_pos
//    - 轮电机力矩：tau_each = -0.5 * u（负力矩驱动机器人向前）
//
// 2. 核心控制架构：
//    - rebuild_model(H): 每控制周期按当前腿高 H 动态更新离散模型 (Ad, Bd)、终端代价 P 与平衡角 theta_eq
//      * 稳定化初值：从增益表插值获取 Riccati 迭代初始策略；若求解失败，安全沿用上一拍模型
//    - Frozen Scheduling (冻结调度): 在单周期 N=20 步预测时域内假设腿高 H 保持常数
//    - 安全保护链：包含启动姿态保持触地、周期 tick 保护、倾角硬安全门限（超限急停）与 MPC 四级降级链
//
// 3. ROS 2 通信接口：
//    - 订阅：/imu, /joint_states, /cmd_vel, /target_height, /robot_mode
//    - 发布：/wheel_effort_controller/commands, /leg_position_controller/commands

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "bbot_balance_controller/keyboard_reader.h"
#include "bbot_balance_controller/linear_mpc.hpp"
#include "bbot_balance_controller/lqr_plant_model.hpp"
#include "bbot_kinematics/kinematics.hpp"

using namespace std::chrono_literals;

namespace
{
  constexpr std::size_t kGainTableSize = 5;

  // 与 adaptive_lqr_balance_controller.cpp:300-312（legacy_safe 配置）相同的
  // 增益表，含 y_com / z_com 列。此处仅用于两件事：标称平衡角 theta_eq(H) 和
  // Riccati 迭代的稳定化初值。MPC 增益不取自该表。
  struct GainRow
  {
    double height;
    double k_x;
    double k_x_dot;
    double k_theta;
    double k_theta_dot;
    double y_com;
    double z_com;
  };

  constexpr std::array<GainRow, kGainTableSize> kGainTable{{{0.3000, -5.622712, -42.666506, -156.508986, -35.846746, -0.0276264, 0.3592154},
                                                            {0.3500, -5.791278, -43.976682, -169.544125, -39.563240, -0.0258066, 0.4016316},
                                                            {0.4000, -5.931603, -45.087129, -181.972969, -43.390225, -0.0234056, 0.4443432},
                                                            {0.4500, -6.052411, -46.061241, -193.948669, -47.349665, -0.0203401, 0.4872441},
                                                            {0.5000, -6.157159, -46.922551, -205.518217, -51.430573, -0.0164269, 0.5302655}}};

  double clamp_value(double value, double lo, double hi)
  {
    return std::max(lo, std::min(hi, value));
  }

  double low_pass(double value, double previous, double alpha)
  {
    return alpha * value + (1.0 - alpha) * previous;
  }

  // adaptive_lqr_balance_controller.cpp 先插值 y_com 和 z_com，再取 -atan2，
  // 因此平衡角是插值后质心的几何量，而非两个角度的插值。此处保持逐位一致。
  double row_equilibrium_pitch(const GainRow &row)
  {
    return -std::atan2(row.y_com, row.z_com);
  }

  GainRow interpolate_row(double height)
  {
    if (height <= kGainTable.front().height)
    {
      return kGainTable.front();
    }
    if (height >= kGainTable.back().height)
    {
      return kGainTable.back();
    }
    for (std::size_t index = 0; index + 1 < kGainTable.size(); ++index)
    {
      const GainRow &low = kGainTable[index];
      const GainRow &high = kGainTable[index + 1];
      if (height >= low.height && height <= high.height)
      {
        const double ratio = (height - low.height) / (high.height - low.height);
        const auto lerp = [ratio](double a, double b)
        { return a + (b - a) * ratio; };
        return GainRow{height, lerp(low.k_x, high.k_x), lerp(low.k_x_dot, high.k_x_dot),
                       lerp(low.k_theta, high.k_theta), lerp(low.k_theta_dot, high.k_theta_dot),
                       lerp(low.y_com, high.y_com), lerp(low.z_com, high.z_com)};
      }
    }
    return kGainTable[2];
  }
} // namespace

class LinearMpcBalanceController : public rclcpp::Node
{
public:
  LinearMpcBalanceController()
      : Node("linear_mpc_balance_controller")
  {
    const auto &robot = kinematics_.get_params();

    wheel_radius_ = robot.wheel_radius;

    // ---- 机构几何与腿高参数 ------------------------------------------------
    // H: 轮轴中心到髋关节的竖直距离 [m]
    height_min_ = declare_parameter<double>("height.hip_axle_min", robot.L_MIN);
    height_max_ = declare_parameter<double>("height.hip_axle_max", robot.L_MAX);
    base_to_hip_height_ = declare_parameter<double>("height.base_to_hip", 0.07);
    target_height_ = clamp_value(declare_parameter<double>("target_height", 0.40), height_min_,
                                 height_max_);
    startup_height_ = clamp_value(
        declare_parameter<double>("height.startup_hip_axle", 0.36), height_min_, height_max_);
    // 启动保持时间：维持初始腿高让车轮先平稳接地，延时结束后才允许调节腿高
    startup_hold_time_ = declare_parameter<double>("height.startup_hold_time", 2.0);
    leg_transition_speed_ = declare_parameter<double>("leg_transition_speed", 0.05);
    if (!std::isfinite(height_min_) || !std::isfinite(height_max_) ||
        !std::isfinite(base_to_hip_height_) || height_min_ >= height_max_ ||
        base_to_hip_height_ < 0.0 || !std::isfinite(startup_hold_time_) ||
        startup_hold_time_ < 0.0 || !std::isfinite(leg_transition_speed_) ||
        leg_transition_speed_ < 0.0)
    {
      throw std::invalid_argument("invalid hip-axle height mapping parameters");
    }
    current_height_ = startup_height_;

    // ---- 控制周期 (5 ms / 200 Hz) ----------------------------------------
    period_s_ = declare_parameter<double>("control.period_s", 0.005);
    if (std::abs(period_s_ - 0.005) > 1.0e-9)
    {
      RCLCPP_WARN(get_logger(),
                  "control.period_s = %.6f s differs from the 0.005 s period of the deployed "
                  "controllers; the model is discretised at the configured value.",
                  period_s_);
    }

    // ---- 状态低通滤波参数 -------------------------------------------------
    pitch_rate_alpha_ = declare_parameter<double>("filter.pitch_rate_alpha", 0.10); // 角速度低通滤波器 越小越平滑
    x_dot_alpha_ = declare_parameter<double>("filter.x_dot_alpha", 0.05);           // 线速度低通滤波器 越小越平滑

    // ---- 执行器力矩与姿态保护限幅 -----------------------------------------
    wheel_torque_max_ = declare_parameter<double>("mpc.wheel_torque_max", robot.wheel_torque_max);
    total_torque_max_ = declare_parameter<double>("mpc.total_torque_max", 2.0 * wheel_torque_max_);
    max_pitch_error_ = declare_parameter<double>("max_pitch_error", 0.50);

    // ---- MPC 目标函数与约束参数 -------------------------------------------
    // Q 对角权重 [x, x_dot, pitch, pitch_rate]
    q_diagonal_ = declare_parameter<std::vector<double>>(
        "mpc.q", std::vector<double>{100.0, 5000.0, 3000.0, 1200.0});
    // 控制加权 R：标称值 8.0（兼顾抗扰刚度与力矩裕量）
    cost_r_ = declare_parameter<double>("mpc.r", 8.0);
    horizon_ = declare_parameter<int>("mpc.horizon", 20);                             // 预测步数 N
    theta_band_limit_ = declare_parameter<double>("mpc.theta_error_limit", 0.20);     // Pitch 软约束
    theta_band_enabled_ = declare_parameter<bool>("mpc.use_theta_band", true);        // 是否启用俯仰误差带
    theta_slack_weight_ = declare_parameter<double>("mpc.theta_slack_weight", 1.0e5); // slack线性惩罚
    max_qp_iterations_ = declare_parameter<int>("mpc.max_qp_iterations", 300);

    // ---- 标称平衡角来源设置 -----------------------------------------------
    // "table": 增益表质心插值；"geometric": 连续动力学模型几何值；"override": 固定调试值
    theta_source_ = declare_parameter<std::string>("mpc.theta_eq_source", "table");
    theta_eq_override_ = declare_parameter<double>("mpc.theta_eq", 0.0);

    // ---- 期望速度与斜坡平滑 -----------------------------------------------
    walk_speed_ = declare_parameter<double>("joystick.walk_speed", 0.3);
    speed_ramp_time_ = declare_parameter<double>("joystick.speed_ramp_time", 1.0);

    log_path_ = declare_parameter<std::string>("log_path",
                                               "/home/admin/bbot_ws_new/src/bbot_balance_controller/src/data_logs/mpc/mpc_balance_log.csv");
    log_enabled_ = declare_parameter<bool>("log_enabled", true);

    // 模型在第一拍的 current_height_（即 height.startup_hip_axle）上建立，之后
    // 每个控制周期按同一个 current_height_ 重建。
    prepare_cost_and_qp_config();                  // 把 ROS 参数翻译成 LinearMpc 能使用的数据结构。
    model_ready_ = rebuild_model(current_height_); // 输入H，输出 Ad/Bd/P/K ，相当于重新配置MPC
    if (!model_ready_)
    {
      RCLCPP_FATAL(get_logger(),
                   "the initial model at H=%.3f m is not usable; wheel torque stays zero.",
                   current_height_);
    }
    if (theta_source_ == "geometric")
    {
      theta_eq_ = geometric_theta_eq_;
    }
    else if (theta_source_ == "override")
    {
      theta_eq_ = theta_eq_override_;
    }
    else
    {
      theta_source_ = "table";
      theta_tracks_commanded_height_ = true;
      theta_eq_ = row_equilibrium_pitch(interpolate_row(current_height_));
    }
    const GainRow row = interpolate_row(current_height_);
    RCLCPP_INFO(get_logger(), "theta_eq source=%s -> %.6f rad (table %.6f, geometric %.6f, "
                              "override %.6f)",
                theta_source_.c_str(), theta_eq_, row_equilibrium_pitch(row),
                geometric_theta_eq_, theta_eq_override_);

    // ---- 接口 -----------------------------------------------------------
    wheel_effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/wheel_effort_controller/commands", 10);
    leg_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/leg_position_controller/commands", 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu", 10,
                                                          std::bind(&LinearMpcBalanceController::imu_callback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
                                                                         std::bind(&LinearMpcBalanceController::joint_state_callback, this, std::placeholders::_1));
    // 真值位姿仅用于验证。话题与字段名同 bbot_velocity_jump_controller.cpp:560-570；
    // 不进入控制律，只追加到 CSV，便于事后将编码器推算的 x 与仿真位姿对比。
    ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/model/bbot/odometry", 10,
        std::bind(&LinearMpcBalanceController::ground_truth_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
                                                                  [this](const geometry_msgs::msg::Twist::SharedPtr msg)
                                                                  {
                                                                    if (std::abs(msg->angular.z) > 1.0e-3)
                                                                    {
                                                                      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                                                                           "[MPC] angular.z=%.3f ignored: phase one commands equal wheel torque, no yaw control",
                                                                                           msg->angular.z);
                                                                    }
                                                                    target_speed_const_ = msg->linear.x;
                                                                  });
    robot_mode_sub_ = create_subscription<std_msgs::msg::String>("/robot_mode", 10,
                                                                 [this](const std_msgs::msg::String::SharedPtr msg)
                                                                 {
                                                                   handle_command(msg->data);
                                                                 });
    // 与 adaptive_lqr_balance_controller.cpp:247-252 逐字一致：话题 /target_height、
    // 消息 std_msgs/msg/Float64、按 [height.hip_axle_min, height.hip_axle_max] 限幅，
    // 只改写目标高度；current_height_ 仍由 update_height() 以 leg_transition_speed_
    // 平滑逼近，平衡轮转矩不受腿部运动影响。
    target_height_sub_ = create_subscription<std_msgs::msg::Float64>("/target_height", 10,
                                                                     [this](const std_msgs::msg::Float64::SharedPtr msg)
                                                                     {
                                                                       const double requested = clamp_value(msg->data, height_min_, height_max_);
                                                                       if (std::abs(requested - target_height_) < 1.0e-12)
                                                                       {
                                                                         return;
                                                                       }
                                                                       target_height_ = requested;
                                                                       RCLCPP_INFO(get_logger(), "[MPC] target_height -> %.3f m (hip axle above axle)",
                                                                                   target_height_);
                                                                     });
    command_sub_ = create_subscription<std_msgs::msg::String>("/linear_mpc/command", 10,
                                                              [this](const std_msgs::msg::String::SharedPtr msg)
                                                              {
                                                                handle_command(msg->data);
                                                              });

    if (log_enabled_)
    {
      open_log_file();
    }

    start_time_ = now();
    last_time_ = start_time_;
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s_)),
        std::bind(&LinearMpcBalanceController::control_loop, this));

    print_summary(row);
  }

  ~LinearMpcBalanceController() override
  {
    publish_wheel_torque(0.0);
    if (log_file_.is_open())
    {
      log_file_.close();
    }
  }

private:
  // 代价权重与 QP 结构只由参数决定，构造时确定一次；高度调度只更换 Ad/Bd/P/K。
  void prepare_cost_and_qp_config()
  {
    model_q_ = Eigen::Matrix4d::Zero();
    if (q_diagonal_.size() == 4)
    {
      for (int index = 0; index < 4; ++index)
      {
        model_q_(index, index) = q_diagonal_[index];
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "mpc.q must have 4 entries (state order x, x_dot, theta, "
                                 "theta_dot); using 100/5000/3000/1200.");
      model_q_ = Eigen::Matrix4d::Zero();
      model_q_ << 100.0, 0.0, 0.0, 0.0, 0.0, 5000.0, 0.0, 0.0, 0.0, 0.0, 3000.0, 0.0, 0.0, 0.0,
          0.0, 1200.0;
    }

    mpc_config_.horizon = horizon_;
    mpc_config_.total_torque_limit = total_torque_max_;
    mpc_config_.wheel_torque_limit = wheel_torque_max_;
    mpc_config_.theta_error_limit = theta_band_limit_;
    mpc_config_.theta_slack_weight = theta_slack_weight_;
    mpc_config_.use_theta_band = theta_band_enabled_;
    mpc_config_.max_qp_iterations = max_qp_iterations_;
  }

  // 每拍高度调度模型动态重建：根据当前髋-轴高度 H 重新推导离散模型与 Riccati 终端代价。
  // 1. 增益表插值提供 DARE 稳定化初始策略种子与标称平衡角；
  // 2. 策略迭代求解 DARE 获取终端代价 P 与 LQR 增益 K；
  // 3. 若求解失败，自动沿用上一拍有效模型，确保控制量输出不中断；
  // 4. 调用 mpc_.configure() 刷新预测矩阵与优化问题。
  bool rebuild_model(double height)
  {
    const auto rebuild_start = std::chrono::steady_clock::now();
    if (!std::isfinite(height))
    {
      return false;
    }

    Eigen::Matrix4d A;
    Eigen::Vector4d B;
    bbot_balance_controller::lqr_plant::continuous_matrices(
        height, kinematics_.get_params(), A, B, &suspended_);
    Eigen::Matrix4d ad;
    Eigen::Vector4d bd;
    bbot_balance_controller::lqr_plant::zoh_discretize(period_s_, A, B, ad, bd);
    const double geometric_theta =
        bbot_balance_controller::lqr_plant::equilibrium_pitch(suspended_);

    const GainRow row = interpolate_row(height);
    Eigen::RowVector4d seed;
    seed << row.k_x, row.k_x_dot, row.k_theta, row.k_theta_dot;
    Eigen::Matrix4d p;
    Eigen::RowVector4d gain;
    int iterations = 0;
    double residual = 0.0;
    const bool solved = bbot_balance_controller::lqr_plant::solve_dare(
        ad, bd, model_q_, cost_r_, seed, p, gain, 100, iterations, residual);
    if (!solved || !ad.allFinite() || !bd.allFinite() || !p.allFinite())
    {
      // 容错降级：若 DARE 求解失败（缺少 P 有限时域不稳定），沿用上一拍模型保证控制安全
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "[MPC] Riccati iteration rejected H=%.4f m (solved=%d, residual=%.3e); keeping the "
                           "previous model at %.4f m",
                           height, static_cast<int>(solved), residual, model_height_);
      return false;
    }

    if (!mpc_.configure(ad, bd, model_q_, cost_r_, p, gain, mpc_config_))
    {
      // 到这里 Ad/Bd/Q/P 都已确认为有限、R 与限值也在参数检查里过检，configure
      // 只可能因固定配置本身被拒；此时不能带着半成品预测矩阵继续跑。
      model_ready_ = false;
      RCLCPP_FATAL(get_logger(),
                   "MPC configuration rejected at H=%.4f m (horizon=%d, r=%.4f, limits=%.2f/%.2f); "
                   "wheel torque stays zero.",
                   height, horizon_, cost_r_, total_torque_max_,
                   wheel_torque_max_);
      return false;
    }

    model_ad_ = ad;
    model_bd_ = bd;
    model_p_ = p;
    lqr_gain_ = gain;
    geometric_theta_eq_ = geometric_theta;
    model_height_ = height;
    closed_loop_radius_ =
        bbot_balance_controller::lqr_plant::spectral_radius(model_ad_ - model_bd_ * lqr_gain_);
    model_rebuild_us_ = std::chrono::duration<double, std::micro>(
                            std::chrono::steady_clock::now() - rebuild_start)
                            .count();
    return true;
  }

  void print_summary(const GainRow &row)
  {
    RCLCPP_INFO(get_logger(), "==============================================");
    RCLCPP_INFO(get_logger(), "Linear MPC balance controller (phase 2, height scheduled).");
    RCLCPP_INFO(get_logger(), "model rebuilt every %.0f ms at current_height; allowed hip-axle "
                              "range [%.3f, %.3f] m, target %.3f m",
                1000.0 * period_s_, height_min_, height_max_,
                target_height_);
    RCLCPP_INFO(get_logger(), "COM of suspended body: y=%.6f m z=%.6f m, length=%.6f m",
                suspended_.y_com, suspended_.z_com, suspended_.length);
    RCLCPP_INFO(get_logger(), "period=%.4f s (%.0f Hz), horizon N=%d (=%.3f s preview)",
                period_s_, 1.0 / period_s_, horizon_, horizon_ * period_s_);
    RCLCPP_INFO(get_logger(), "Q=[%.1f %.1f %.1f %.1f] R=%.3f, terminal cost=DARE, rho_cl=%.6f",
                model_q_(0, 0), model_q_(1, 1), model_q_(2, 2), model_q_(3, 3), cost_r_, closed_loop_radius_);
    RCLCPP_INFO(get_logger(), "DARE gain K=[%.4f %.4f %.4f %.4f]  (deployed adaptive table row "
                              "H=%.2f: [%.4f %.4f %.4f %.4f])",
                lqr_gain_(0), lqr_gain_(1), lqr_gain_(2), lqr_gain_(3),
                row.height, row.k_x, row.k_x_dot, row.k_theta, row.k_theta_dot);
    if (model_ready_)
    {
      const Eigen::RowVector4d &gain = mpc_.first_move_gain();
      RCLCPP_INFO(get_logger(), "MPC first-move gain=[%.4f %.4f %.4f %.4f]", gain(0), gain(1),
                  gain(2), gain(3));
    }
    RCLCPP_INFO(get_logger(), "theta_eq=%.6f rad (source=%s), pitch band=%.3f rad %s, "
                              "hard gate=%.2f rad",
                theta_eq_, theta_source_.c_str(), theta_band_limit_,
                theta_band_enabled_ ? "on" : "off", max_pitch_error_);
    RCLCPP_INFO(get_logger(), "startup: hip-axle %.3f m -> target %.3f m at %.3f m/s after a "
                              "%.2f s hold, base_link %.3f -> %.3f m",
                current_height_, target_height_,
                leg_transition_speed_, startup_hold_time_, base_link_height(),
                target_height_ + base_to_hip_height_ + wheel_radius_);
    RCLCPP_INFO(get_logger(), "torque: total |u|<=%.2f Nm, per wheel |tau|<=%.2f Nm, split -0.5*u",
                mpc_.torque_limit(), wheel_torque_max_);
    RCLCPP_INFO(get_logger(), "log: %s", log_path_.c_str());
    RCLCPP_INFO(get_logger(), "keys: W/S speed +/- | Space reset x_ref | B resume | X emergency");
    RCLCPP_INFO(get_logger(), "==============================================");
  }

  void handle_command(const std::string &data)
  {
    if (data == "emergency" || data == "x" || data == "X")
    {
      control_enabled_ = false;
      publish_wheel_torque(0.0);
      RCLCPP_WARN(get_logger(), "[MPC] emergency stop, wheel torque zeroed");
    }
    else if (data == "balance" || data == "b" || data == "B")
    {
      control_enabled_ = true;
      target_x_ = x_;
      RCLCPP_INFO(get_logger(), "[MPC] balance resumed, x_ref=%.3f m", target_x_);
    }
    else if (data == "reset_position" || data == " ")
    {
      target_x_ = x_;
      RCLCPP_INFO(get_logger(), "[MPC] position reference reset: x_ref=%.3f m", target_x_);
    }
    else if (data.rfind("step_position:", 0) == 0)
    {
      // 位置阶跃测试：参考位置移动一个带符号的偏移量，由调节器把
      // x_error 拉回零。
      try
      {
        const double offset = std::stod(data.substr(std::strlen("step_position:")));
        target_x_ += offset;
        RCLCPP_INFO(get_logger(), "[MPC] position reference stepped by %.3f m -> x_ref=%.3f m",
                    offset, target_x_);
      }
      catch (const std::exception &error)
      {
        RCLCPP_WARN(get_logger(), "[MPC] malformed step_position command: %s", error.what());
      }
    }
    else if (data == "stop_moving")
    {
      target_speed_const_ = 0.0;
      target_x_ = x_;
      RCLCPP_INFO(get_logger(), "[MPC] speed reference zeroed, x_ref=%.3f m", target_x_);
    }
  }

  void process_keyboard()
  {
    const std::string key = keyboard_.read_sequence();
    if (key.empty())
    {
      return;
    }
    if (key == "w" || key == "W")
    {
      target_speed_const_ = walk_speed_;
    }
    else if (key == "s" || key == "S")
    {
      target_speed_const_ = -walk_speed_;
    }
    else if (key == " ")
    {
      handle_command("reset_position");
    }
    else if (key == "b" || key == "B")
    {
      handle_command("balance");
    }
    else if (key == "x" || key == "X")
    {
      handle_command("emergency");
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                      msg->orientation.w);
    double roll;
    double unused_pitch;
    double unused_yaw;
    tf2::Matrix3x3(q).getRPY(roll, unused_pitch, unused_yaw);

    // 轮轴沿 X 方向：矢状面倾角即 IMU 的 roll，物理上前倾对应负 roll，
    // 因此 pitch > 0 表示前倾（adaptive_lqr_balance_controller.cpp:488）。
    pitch_ = -roll;
    pitch_rate_raw_ = -msg->angular_velocity.x;
    if (!pitch_rate_filter_init_)
    {
      pitch_rate_filt_ = pitch_rate_raw_;
      pitch_rate_filter_init_ = true;
    }
    else
    {
      pitch_rate_filt_ = low_pass(pitch_rate_raw_, pitch_rate_filt_, pitch_rate_alpha_);
    }
    pitch_rate_ = pitch_rate_filt_;
    imu_received_ = true;
    last_imu_time_ = now();
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    bool has_004 = false;
    bool has_007 = false;
    for (std::size_t index = 0; index < msg->name.size(); ++index)
    {
      if (msg->name[index] == "link_004_joint")
      {
        if (index < msg->position.size())
        {
          wheel_004_pos_ = msg->position[index];
        }
        if (index < msg->velocity.size())
        {
          wheel_004_vel_ = msg->velocity[index];
        }
        has_004 = true;
      }
      else if (msg->name[index] == "link_007_joint")
      {
        if (index < msg->position.size())
        {
          wheel_007_pos_ = msg->position[index];
        }
        if (index < msg->velocity.size())
        {
          wheel_007_vel_ = msg->velocity[index];
        }
        has_007 = true;
      }
    }
    if (!(has_004 && has_007))
    {
      return;
    }

    if (!wheel_origin_set_)
    {
      wheel_004_origin_ = wheel_004_pos_;
      wheel_007_origin_ = wheel_007_pos_;
      wheel_origin_set_ = true;
      target_x_ = 0.0;
    }

    // 轮子负向转动使机器人前进，因此速度与位移估计都带前导负号
    // （adaptive_lqr_balance_controller.cpp:552-566）。
    x_dot_raw_ = -wheel_radius_ * 0.5 * (wheel_004_vel_ + wheel_007_vel_);
    if (!x_dot_filter_init_)
    {
      x_dot_filt_ = x_dot_raw_;
      x_dot_filter_init_ = true;
    }
    else
    {
      x_dot_filt_ = low_pass(x_dot_raw_, x_dot_filt_, x_dot_alpha_);
    }
    x_dot_ = x_dot_filt_;

    x_ = -wheel_radius_ * 0.5 *
         ((wheel_004_pos_ - wheel_004_origin_) + (wheel_007_pos_ - wheel_007_origin_));
    last_joint_time_ = now();
  }

  void ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double z = msg->pose.pose.position.z;
    if (!std::isfinite(x) || !std::isfinite(y))
    {
      return;
    }
    ground_truth_x_ = x;
    ground_truth_y_ = y;
    // base_link 离地真值：用来独立核对腿部是否真的走到了指令高度。
    ground_truth_z_ = z;
    ground_truth_received_ = true;
  }

  double base_link_height() const
  {
    return current_height_ + base_to_hip_height_ + wheel_radius_;
  }

  // 限速的髋-轴高度过渡，与 adaptive_lqr_balance_controller.cpp:618-631 相同。
  void update_height(double dt)
  {
    const double step = std::max(0.0, leg_transition_speed_) * dt;
    if (current_height_ < target_height_)
    {
      current_height_ = std::min(current_height_ + step, target_height_);
    }
    else if (current_height_ > target_height_)
    {
      current_height_ = std::max(current_height_ - step, target_height_);
    }
  }

  void publish_leg_pose()
  {
    // bbot_kinematics 的 IK 期望 base_link 离地高度，内部会减去
    // (base_to_hip + wheel_radius)，与 adaptive_lqr_balance_controller.cpp:635
    // 的约定一致。
    const auto solution = kinematics_.inverse_kinematics(base_link_height(), 0.0);
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {solution.theta_hip, solution.theta_knee, solution.theta_hip,
                solution.theta_knee};
    leg_pub_->publish(msg);
  }

  void publish_wheel_torque(double tau_each)
  {
    std_msgs::msg::Float64MultiArray msg;
    // /wheel_effort_controller 关节顺序为 [link_004_joint, link_007_joint]
    // （bbot_controllers.yaml）；直行平衡两轮等转矩，与转矩输入控制器的
    // publish_wheel_torque() 一致。
    msg.data = {tau_each, tau_each};
    wheel_effort_pub_->publish(msg);
  }

  void control_loop()
  {
    process_keyboard();

    const rclcpp::Time current = now();
    // 与 adaptive_lqr_balance_controller.cpp:668-691 相同的节拍保护：墙钟
    // 定时器可能比仿真推进更频繁地触发，离散模型绝不能按虚构的 dt 步进。
    if (last_control_sim_time_.nanoseconds() != 0)
    {
      const double sim_elapsed = (current - last_control_sim_time_).seconds();
      if (sim_elapsed < 0.0)
      {
        RCLCPP_WARN(get_logger(),
                    "[MPC] simulation clock rollback (%.4f s); resetting time baselines", sim_elapsed);
        last_control_sim_time_ = current;
        last_time_ = current;
        return;
      }
      if (sim_elapsed < 0.0045)
      {
        return;
      }
    }
    raw_dt_ = last_time_.nanoseconds() != 0 ? (current - last_time_).seconds() : period_s_;
    last_time_ = current;
    last_control_sim_time_ = current;
    used_dt_ = raw_dt_;
    if (used_dt_ <= 0.0001 || used_dt_ > 0.05)
    {
      used_dt_ = period_s_;
    }
    const double dt = used_dt_;

    const double elapsed = (current - start_time_).seconds();

    // 1. 反馈就绪前保持启动姿态并输出零转矩，使腿部先平稳触地建立支撑
    if (!imu_received_ || !wheel_origin_set_ || !control_enabled_ || !model_ready_)
    {
      publish_leg_pose();
      publish_wheel_torque(0.0);
      log_line(elapsed, dt, 0.0, 0.0, 0.0, 0.0, "disabled", 0, 0.0);
      return;
    }

    // 2. 启动延时结束后，才开始向目标高度动态调节腿长
    balance_ready_elapsed_ += dt;
    if (balance_ready_elapsed_ >= startup_hold_time_)
    {
      update_height(dt);
    }
    publish_leg_pose();

    // 3. 冻结调度 (Frozen Scheduling)：
    //    在当前拍 N=20 预测时域内假设腿高 H 恒定，域内不预测腿长变化。
    //    每控制周期按当前腿高重新求解 (Ad, Bd, P, theta_eq)；若 DARE 求解失败则自动沿用上一拍模型。
    const bool model_rebuilt = rebuild_model(current_height_);
    last_rebuild_ok_ = model_rebuilt;
    if (model_rebuilt)
    {
      if (theta_tracks_commanded_height_)
      {
        theta_eq_ = row_equilibrium_pitch(interpolate_row(current_height_));
      }
      else if (theta_source_ == "geometric")
      {
        theta_eq_ = geometric_theta_eq_;
      }
    }

    // 4. 姿态硬安全门限：俯仰角误差过大时立即关停控制器输出零转矩，防止失控或翻倒后飞车
    if (std::abs(pitch_ - theta_eq_) > max_pitch_error_)
    {
      control_enabled_ = false;
      publish_wheel_torque(0.0);
      RCLCPP_ERROR(get_logger(),
                   "[MPC] pitch error %.3f rad exceeds the %.3f rad safety limit; controller disabled.",
                   pitch_ - theta_eq_, max_pitch_error_);
      log_line(elapsed, dt, 0.0, 0.0, pitch_ - theta_eq_, pitch_rate_, "safety_disabled", 0, 0.0);
      return;
    }

    // 5. 速度指令斜坡平滑与参考位置积分
    if (speed_ramp_time_ > 1.0e-3)
    {
      const double step = dt / speed_ramp_time_;
      if (target_speed_smoothed_ < target_speed_const_)
      {
        target_speed_smoothed_ = std::min(target_speed_const_, target_speed_smoothed_ + step);
      }
      else if (target_speed_smoothed_ > target_speed_const_)
      {
        target_speed_smoothed_ = std::max(target_speed_const_, target_speed_smoothed_ - step);
      }
    }
    else
    {
      target_speed_smoothed_ = target_speed_const_;
    }
    target_x_ += target_speed_smoothed_ * dt;

    // 6. 构造系统误差状态向量 X = [x_err, x_dot_err, pitch_err, pitch_rate]^T
    Eigen::Vector4d state;
    state << (x_ - target_x_), (x_dot_ - target_speed_smoothed_), (pitch_ - theta_eq_), pitch_rate_;

    // 7. 求解 MPC 并下发双轮力矩指令 (tau_each = -0.5 * u)
    bbot_balance_controller::LinearMpcDiagnostics diagnostics;
    const double u = mpc_.solve(state, diagnostics);
    publish_wheel_torque(diagnostics.tau_each);

    log_line(elapsed, dt, state(0), state(1), state(2), state(3),
             bbot_balance_controller::mpc_fallback_name(diagnostics.fallback_stage), 1, u, diagnostics);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 250,
                         "[MPC] H=%.2f tgt=%.2f model=%.2f rebuild=%.0fus x_err=%.3f x_dot=%.3f v_ref=%.3f "
                         "the=%.4f eq=%.4f err=%.4f u=%.2fNm tau=%.2fNm N=%d it=%d t=%.0fus stage=%s%s%s",
                         current_height_, target_height_, model_height_, model_rebuild_us_, state(0), x_dot_,
                         target_speed_smoothed_, pitch_, theta_eq_, state(2), u, diagnostics.tau_each, horizon_,
                         diagnostics.qp_iterations, diagnostics.qp_solve_time_us,
                         bbot_balance_controller::mpc_fallback_name(diagnostics.fallback_stage),
                         diagnostics.total_torque_saturated ? " SAT" : "",
                         diagnostics.theta_band_binding ? " BAND" : "");
  }

  void open_log_file()
  {
    const std::filesystem::path path(log_path_);
    if (path.has_parent_path())
    {
      std::error_code ec;
      std::filesystem::create_directories(path.parent_path(), ec);
    }
    log_file_.open(log_path_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open())
    {
      RCLCPP_ERROR(get_logger(), "Could not open MPC log: %s", log_path_.c_str());
      return;
    }
    log_file_ << "time,dt,stage,control_enabled,height,base_link_height,"
              << "x,x_ref,x_error,x_dot,v_ref,v_error,pitch,pitch_rate,theta_eq,theta_error,"
              << "u_mpc,u_lqr_fallback,tau_each,total_torque_max,wheel_torque_max,"
              << "total_torque_saturated,wheel_torque_saturated,"
              << "theta_band_enabled,theta_band_limit,theta_band_binding,"
              << "theta_pred_max,theta_pred_min,theta_slack_upper,theta_slack_lower,"
              << "solver_status,solver_iterations,solver_time_us,working_set_size,warm_start,"
              << "kkt_primal_residual,kkt_dual_residual,kkt_stationarity,kkt_complementarity,"
              << "objective,gt_pose_x,gt_pose_y,gt_valid"
              << ",target_height,model_height,model_rebuild_us,model_rebuild_failed,gt_pose_z"
              << "\n";
    log_file_ << std::fixed << std::setprecision(7);
  }

  void log_line(double time, double dt, double x_error_value, double v_error_value,
                double theta_error, double pitch_rate_value, const std::string &stage, int enabled, double u,
                const bbot_balance_controller::LinearMpcDiagnostics &diagnostics =
                    bbot_balance_controller::LinearMpcDiagnostics())
  {
    if (!log_enabled_ || !log_file_.is_open())
    {
      return;
    }
    log_file_ << time << ',' << dt << ',' << stage << ',' << enabled << ','
              << current_height_ << ',' << base_link_height() << ','
              << x_ << ',' << target_x_ << ',' << x_error_value << ',' << x_dot_ << ','
              << target_speed_smoothed_ << ',' << v_error_value << ','
              << pitch_ << ',' << pitch_rate_value << ',' << theta_eq_ << ',' << theta_error << ','
              << u << ',' << diagnostics.lqr_u << ',' << diagnostics.tau_each << ','
              << total_torque_max_ << ',' << wheel_torque_max_ << ','
              << static_cast<int>(diagnostics.total_torque_saturated) << ','
              << static_cast<int>(diagnostics.wheel_torque_saturated) << ','
              << static_cast<int>(theta_band_enabled_) << ',' << theta_band_limit_ << ','
              << static_cast<int>(diagnostics.theta_band_binding) << ','
              << diagnostics.theta_predicted_max << ',' << diagnostics.theta_predicted_min << ','
              << diagnostics.theta_slack_upper << ',' << diagnostics.theta_slack_lower << ','
              << static_cast<int>(diagnostics.qp_status) << ',' << diagnostics.qp_iterations << ','
              << diagnostics.qp_solve_time_us << ',' << diagnostics.working_set_size << ','
              << static_cast<int>(diagnostics.qp_warm_start_used) << ','
              << diagnostics.qp_primal_residual << ',' << diagnostics.qp_dual_residual << ','
              << diagnostics.qp_stationarity << ',' << diagnostics.qp_complementarity << ','
              << diagnostics.objective << ','
              << ground_truth_x_ << ',' << ground_truth_y_ << ','
              << static_cast<int>(ground_truth_received_) << ','
              << target_height_ << ',' << model_height_ << ',' << model_rebuild_us_ << ','
              << static_cast<int>(!last_rebuild_ok_) << ',' << ground_truth_z_ << "\n";
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_height_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_effort_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leg_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  KeyboardReader keyboard_;
  bbot_kinematics::Kinematics kinematics_;
  bbot_balance_controller::LinearMpc mpc_;

  rclcpp::Time start_time_;
  rclcpp::Time last_time_;
  rclcpp::Time last_control_sim_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_joint_time_;
  double raw_dt_ = 0.005;
  double used_dt_ = 0.005;

  bool imu_received_ = false;
  bool wheel_origin_set_ = false;
  bool control_enabled_ = true;
  bool model_ready_ = false;
  bool theta_band_enabled_ = true;
  bool pitch_rate_filter_init_ = false;
  bool x_dot_filter_init_ = false;

  double pitch_ = 0.0;
  double pitch_rate_ = 0.0;
  double pitch_rate_raw_ = 0.0;
  double pitch_rate_filt_ = 0.0;
  double pitch_rate_alpha_ = 0.10;

  double wheel_004_pos_ = 0.0;
  double wheel_007_pos_ = 0.0;
  double wheel_004_vel_ = 0.0;
  double wheel_007_vel_ = 0.0;
  double wheel_004_origin_ = 0.0;
  double wheel_007_origin_ = 0.0;

  double x_ = 0.0;
  double x_dot_ = 0.0;
  double x_dot_raw_ = 0.0;
  double x_dot_filt_ = 0.0;
  double x_dot_alpha_ = 0.05;

  double target_x_ = 0.0;
  // 真值（世界系）。哪根世界轴是前进方向由日志数据判断而非假设，
  // 因此两个候选轴都记录。
  double ground_truth_x_ = 0.0;
  double ground_truth_y_ = 0.0;
  double ground_truth_z_ = 0.0;
  bool ground_truth_received_ = false;
  double target_speed_const_ = 0.0;
  double target_speed_smoothed_ = 0.0;
  double walk_speed_ = 0.3;
  double speed_ramp_time_ = 1.0;

  double current_height_ = 0.36;
  double target_height_ = 0.40;
  double startup_height_ = 0.36;
  double height_min_ = 0.30;
  double height_max_ = 0.50;
  double base_to_hip_height_ = 0.07;
  double startup_hold_time_ = 2.0;
  double leg_transition_speed_ = 0.05;
  double balance_ready_elapsed_ = 0.0;
  double wheel_radius_ = 0.07;

  double period_s_ = 0.005;
  int horizon_ = 20;
  double cost_r_ = 1.0;
  std::vector<double> q_diagonal_;
  double theta_band_limit_ = 0.20;
  double theta_slack_weight_ = 1.0e5;
  int max_qp_iterations_ = 300;
  double theta_eq_ = 0.0;
  std::string theta_source_ = "table";
  bool theta_tracks_commanded_height_ = false;
  double theta_eq_override_ = 0.0; // 仅 mpc.theta_eq_source=override 时使用
  double geometric_theta_eq_ = 0.0;
  double max_pitch_error_ = 0.50;
  double wheel_torque_max_ = 10.0;
  double total_torque_max_ = 20.0;
  double closed_loop_radius_ = 0.0;

  Eigen::Matrix4d model_ad_ = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d model_q_ = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d model_p_ = Eigen::Matrix4d::Zero();
  Eigen::Vector4d model_bd_ = Eigen::Vector4d::Zero();
  Eigen::RowVector4d lqr_gain_ = Eigen::RowVector4d::Zero();
  bbot_balance_controller::lqr_plant::SuspendedBody suspended_;
  bbot_balance_controller::LinearMpcConfig mpc_config_;
  // 高度调度状态：model_height_ 是 Ad/Bd/P/QP 实际所在的高度，与 current_height_
  // 分开记录，重建失败时两者会短暂不一致并可从日志直接核对。
  double model_height_ = 0.0;
  double model_rebuild_us_ = 0.0;
  bool last_rebuild_ok_ = true;

  std::string log_path_;
  bool log_enabled_ = true;
  std::ofstream log_file_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinearMpcBalanceController>());
  rclcpp::shutdown();
  return 0;
}
