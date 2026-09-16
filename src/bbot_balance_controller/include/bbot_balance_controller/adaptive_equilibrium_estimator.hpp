#ifndef BBOT_BALANCE_CONTROLLER__ADAPTIVE_EQUILIBRIUM_ESTIMATOR_HPP_
#define BBOT_BALANCE_CONTROLLER__ADAPTIVE_EQUILIBRIUM_ESTIMATOR_HPP_

#include <algorithm>
#include <cmath>

namespace bbot_balance_controller
{

  // ============================================================
  // 自适应平衡点估计器的状态机阶段
  //
  // 两阶段自适应流程：
  // WaitCoarse -> ApplyCoarse -> WaitFine -> ApplyFine
  //            -> Verify -> Hold
  //
  // 粗估计：允许机器人存在较小运动，快速获得质心偏移初值
  // 精估计：机器人接近静止后进一步提高估计精度
  // Verify ：验证补偿后机器人位置是否已经稳定
  // Hold   ：保持当前补偿值，必要时重新进入精估计
  // ============================================================
  enum class AdaptivePhase
  {
    WaitCoarse = 0,  // 等待满足粗估计条件，并采集一段稳定数据
    ApplyCoarse = 1, // 缓慢应用粗估计得到的质心偏移
    WaitFine = 2,    // 使用更严格条件进行精细估计
    ApplyFine = 3,   // 缓慢应用精细估计结果
    Verify = 4,      // 验证补偿效果
    Hold = 5,        // 补偿完成，保持当前平衡点

    // 旧版本名称的兼容别名
    WaitCapture = WaitCoarse,
    ApplyTarget = ApplyCoarse,
  };

  // ============================================================
  // 自适应平衡点估计器参数
  // ============================================================
  struct AdaptiveEquilibriumConfig
  {
    // ----------------------------------------------------------
    // 第一阶段：快速粗估计
    // ----------------------------------------------------------

    // 连续满足粗估计条件的时间
    double early_capture_time{1.0};

    // 粗估计窗口内 observed_com_offset 允许的最大波动范围 [m]
    double early_observation_window_range_max{0.00010};

    // 粗估计时允许的最大车体纵向速度 [m/s]
    double early_speed_max{0.080};

    // 粗估计时允许的最大纵向加速度
    double early_accel_threshold{0.015};

    // 是否启用“粗估计 + 精估计”两阶段模式
    double early_pitch_rate_threshold{0.010};
    bool two_stage_enabled{true};

    // ----------------------------------------------------------
    // 第二阶段：高精度准静态估计
    // ----------------------------------------------------------

    // 精估计连续采样时间 [s]
    double capture_time{1.0};

    // 精估计窗口内 observed_com_offset 最大允许波动 [m]
    double observation_window_range_max{0.00010};

    // equivalent_com_offset 最大应用速度 [m/s]
    // 避免平衡点发生突变
    double apply_rate_max{0.0010};

    // 目标偏移允许误差
    double target_tolerance{0.00001};

    // Verify 阶段连续验证时间 [s]
    double verify_time{0.50};

    // 精估计允许的最大纵向速度 [m/s]
    double speed_safety_max{0.015};

    // 精估计允许的最大加速度
    double accel_threshold{0.005};

    // 精估计允许的最大 pitch_rate [rad/s]
    double fast_pitch_rate_threshold{0.01};

    // 精估计时允许的最大控制力矩
    // 力矩过大意味着机器人仍处于动态恢复过程
    double fast_torque_threshold{0.05};

    // Hold / Verify 中触发重新估计的质心偏移残差阈值 [m]
    double reacquire_threshold{0.00005};

    // ----------------------------------------------------------
    // 公共滤波及安全参数
    // ----------------------------------------------------------

    // x_error、x_dot 和 observed_com_offset 的一阶低通时间常数 [s]
    double filter_time_constant{0.50};

    // 加速度低通滤波时间常数 [s]
    double accel_filter_time_constant{0.10};

    // 判断机器人位置是否已经稳定的误差死区 [m]
    double position_error_deadband{0.005};

    // 质心偏移估计残差死区 [m]
    double observation_deadband{0.00005};

    // 自适应方向符号
    // +1 表示按照当前推导方向修正
    double adaptation_sign{1.0};

    // equivalent_com_offset 允许范围 [m]
    double offset_min{-0.050};
    double offset_max{0.050};

    // 自适应过程中允许的最大高度变化率
    // 升降腿时不允许更新平衡点
    double height_rate_threshold{0.01};

    // 力矩占额定范围的比例限制
    double torque_ratio_threshold{0.75};

    // 允许进行自适应估计的最大俯仰角误差 [rad]
    double pitch_error_threshold{0.12};

    // ----------------------------------------------------------
    // 旧版本遗留参数
    //
    // 为兼容现有 launch / YAML 参数文件而保留，
    // 当前 fast-capture 状态机并不使用这些参数。
    // ----------------------------------------------------------
    double averaging_time{2.0};
    double cooldown_time{4.0};
    double correction_fraction{0.50};
    double offset_step_max{0.00025};
    double speed_threshold{0.001};
    double position_window_range_max{0.001};
    double pitch_rate_threshold{0.25};
    double torque_threshold{0.25};
  };

  // ============================================================
  // 每个控制周期传入估计器的数据
  // ============================================================
  struct AdaptiveEquilibriumInput
  {
    // 相对于初始参考位置的位置误差 [m]
    double x_error{0.0};

    // 机器人纵向速度 [m/s]
    double x_dot{0.0};

    // IMU 测得的实际俯仰角 [rad]
    double pitch{0.0};

    // 当前 pitch 与当前平衡角之间的误差 [rad]
    double pitch_error{0.0};

    // IMU 俯仰角速度 [rad/s]
    double pitch_rate{0.0};

    // 当前腿长/机器人高度变化速度
    double height_rate{0.0};

    // 当前控制器输出力矩
    double torque{0.0};

    // 当前控制力矩相对于最大允许力矩的比例
    double torque_ratio{0.0};

    // 名义模型中的质心纵向位置 [m]
    double nominal_com_y{0.0};

    // 名义模型中的质心高度 [m]
    double nominal_com_z{0.0};

    // 是否允许自适应估计
    bool enabled{true};
  };

  // ============================================================
  // 自适应估计器当前状态及诊断量
  // ============================================================
  struct AdaptiveEquilibriumState
  {
    // 当前真正已经应用到平衡点计算中的质心偏移 [m]
    double equivalent_com_offset{0.0};

    // 最新一次估计得到、正在逐渐逼近的目标偏移 [m]
    double target_com_offset{0.0};

    // ----------------------------------------------------------
    // 滤波后的运动状态
    // ----------------------------------------------------------
    double filtered_x_error{0.0};
    double filtered_x_dot{0.0};
    double filtered_x_accel{0.0};

    // ----------------------------------------------------------
    // 根据实际 pitch 反算出的质心偏移
    // ----------------------------------------------------------

    // 当前瞬时反算结果
    double observed_com_offset{0.0};

    // 低通滤波后的反算结果
    double filtered_observed_com_offset{0.0};

    // 目标观测值与当前已应用偏移之间的残差
    double correction_error{0.0};

    // 每周期实际应用的偏移增量 [m]
    double offset_step{0.0};

    // 实际偏移调整速度 [m/s]
    double apply_rate{0.0};

    // ----------------------------------------------------------
    // 状态机诊断信息
    // ----------------------------------------------------------

    // 当前稳定采样累计时间
    double stable_time{0.0};

    // 当前采样窗口完成比例：0~1
    double window_progress{0.0};

    // 当前采样窗口内 observed_com_offset 最大值与最小值之差
    double observation_window_range{0.0};

    // Verify 阶段完成比例：0~1
    double verify_progress{0.0};

    // ----------------------------------------------------------
    // 兼容旧版本日志字段
    // ----------------------------------------------------------
    double window_position_range{0.0};
    double cooldown_remaining{0.0};

    // 当前状态机阶段
    AdaptivePhase phase{AdaptivePhase::WaitCoarse};

    // 当前是否满足采样门控条件
    bool gate_open{false};

    // 当前质心偏移观测是否有效
    bool observation_valid{false};

    // 本周期是否发生自适应更新
    bool updated{false};

    // 本周期目标质心偏移是否更新
    bool target_updated{false};
  };

  class AdaptiveEquilibriumEstimator
  {
  public:
    explicit AdaptiveEquilibriumEstimator(
        const AdaptiveEquilibriumConfig &config = AdaptiveEquilibriumConfig())
        : config_(config)
    {
      // 对配置参数进行合法性检查，
      // 防止出现负时间常数、负阈值等无效配置
      sanitize_config();
    }

    // ==========================================================
    // 重置整个自适应估计器
    //
    // equivalent_com_offset:
    //   可以指定一个初始质心偏移，否则默认从 0 开始。
    // ==========================================================
    void reset(double equivalent_com_offset = 0.0)
    {
      // 清空所有状态
      state_ = AdaptiveEquilibriumState{};

      // 初始偏移限制在允许范围内
      state_.equivalent_com_offset = std::clamp(
          equivalent_com_offset, config_.offset_min, config_.offset_max);

      // 初始目标值等于当前值
      state_.target_com_offset = state_.equivalent_com_offset;

      // 每次 reset 都重新从粗估计阶段开始
      state_.phase = AdaptivePhase::WaitCoarse;

      reset_capture_window();

      verify_duration_ = 0.0;
      hold_reacquire_duration_ = 0.0;

      // 各滤波器重新初始化
      position_filter_initialized_ = false;
      observation_filter_initialized_ = false;
      acceleration_filter_initialized_ = false;

      previous_filtered_x_dot_ = 0.0;
    }

    // ==========================================================
    // 自适应估计器主更新函数
    //
    // 每个控制周期调用一次：
    //   1. 更新滤波状态
    //   2. 检查 enabled
    //   3. 根据当前 phase 执行对应状态机逻辑
    //
    // 返回当前 AdaptiveEquilibriumState
    // ==========================================================
    const AdaptiveEquilibriumState &update(
        double dt, const AdaptiveEquilibriumInput &input)
    {
      // 清除“本周期事件”标志
      state_.updated = false;
      state_.target_updated = false;
      state_.offset_step = 0.0;
      state_.apply_rate = 0.0;

      // dt 非法时保持现有状态，不进行任何更新
      if (!(dt > 0.0) || !std::isfinite(dt))
      {
        return state_;
      }

      // 首先进行位置、速度、加速度以及质心偏移观测的滤波
      update_filters(dt, input);

      // 外部禁止自适应时：
      // 清空采样窗口，并重新回到 WaitCoarse
      if (!input.enabled)
      {
        reset_capture_window();
        verify_duration_ = 0.0;
        hold_reacquire_duration_ = 0.0;
        state_.phase = AdaptivePhase::WaitCoarse;
        state_.gate_open = false;
        state_.verify_progress = 0.0;
        return state_;
      }

      // 根据当前阶段调用不同的状态处理函数
      switch (state_.phase)
      {
      case AdaptivePhase::WaitCoarse:
        update_wait_coarse(dt, input);
        break;

      case AdaptivePhase::ApplyCoarse:
        update_apply_coarse(dt);
        break;

      case AdaptivePhase::WaitFine:
        update_wait_fine(dt, input);
        break;

      case AdaptivePhase::ApplyFine:
        update_apply_fine(dt);
        break;

      case AdaptivePhase::Verify:
        update_verify(dt, input);
        break;

      case AdaptivePhase::Hold:
        update_hold(dt, input);
        break;
      }

      return state_;
    }

    // 获取当前估计器状态
    const AdaptiveEquilibriumState &state() const
    {
      return state_;
    }

    // ==========================================================
    // 根据名义质心位置 + 在线估计质心偏移计算平衡角
    //
    //             -(y_nom + Delta_y)
    // theta_eq = atan2(----------------)
    //                     z_nom
    //
    // 等价写法：
    // theta_eq = -atan2(y_nom + Delta_y, z_nom)
    //
    // equivalent_com_offset 就是在线估计的 Delta_y。
    // ==========================================================
    static double equilibrium_pitch(
        double nominal_com_y,
        double nominal_com_z,
        double equivalent_com_offset)
    {
      // 防止 z 太小造成数值问题
      const double z = std::max(0.05, nominal_com_z);

      return -std::atan2(
          nominal_com_y + equivalent_com_offset, z);
    }

  private:
    // ==========================================================
    // 死区函数
    //
    // |value| <= width 时输出 0，
    // 超过死区以后，仅保留超出的部分。
    // ==========================================================
    static double apply_deadband(double value, double width)
    {
      if (std::abs(value) <= width)
      {
        return 0.0;
      }

      return std::copysign(std::abs(value) - width, value);
    }

    // ==========================================================
    // 第一阶段的较宽松门控条件
    //
    // 用于粗估计。
    // 允许机器人有一定小速度，但不能处于明显动态过程中。
    // ==========================================================
    bool early_gate(const AdaptiveEquilibriumInput &input) const
    {
      return state_.observation_valid &&

             // 纵向速度不能过大
             std::abs(input.x_dot) <= config_.early_speed_max &&

             // 加速度不能过大
             std::abs(state_.filtered_x_accel) <= config_.early_accel_threshold &&

             // 俯仰运动必须足够慢
             std::abs(input.pitch_rate) <= config_.early_pitch_rate_threshold &&

             // 升降过程中禁止估计
             std::abs(input.height_rate) <= config_.height_rate_threshold &&

             // 当前姿态必须处于基本稳定范围
             std::abs(input.pitch_error) <= config_.pitch_error_threshold;
    }

    // ==========================================================
    // 第二阶段严格门控
    //
    // 精估计不仅要求速度、加速度、pitch_rate 很小，
    // 还要求控制器力矩较小。
    //
    // 目的：
    // 只有机器人真正接近准静态平衡时，
    // 当前 pitch 才可以用来反推真实质心位置。
    // ==========================================================
    bool strict_gate(const AdaptiveEquilibriumInput &input) const
    {
      return state_.observation_valid &&
             std::abs(input.x_dot) <= config_.speed_safety_max &&
             std::abs(state_.filtered_x_accel) <= config_.accel_threshold &&
             std::abs(input.pitch_rate) <= config_.fast_pitch_rate_threshold &&
             std::abs(input.height_rate) <= config_.height_rate_threshold &&
             std::abs(input.torque) <= config_.fast_torque_threshold &&
             std::abs(input.torque_ratio) <= config_.torque_ratio_threshold &&
             std::abs(input.pitch_error) <= config_.pitch_error_threshold;
    }

    // 当前 fast_gate 与 strict_gate 完全相同
    bool fast_gate(const AdaptiveEquilibriumInput &input) const
    {
      return strict_gate(input);
    }

    // ==========================================================
    // WaitCoarse：第一阶段粗估计
    //
    // 工作流程：
    // 1. 判断 early_gate / strict_gate
    // 2. 连续采集 observed_com_offset
    // 3. 检查采样期间数据波动是否足够小
    // 4. 对整个采样窗口求时间平均
    // 5. 得到 target_com_offset
    // 6. 进入 ApplyCoarse
    // ==========================================================
    void update_wait_coarse(
        double dt,
        const AdaptiveEquilibriumInput &input)
    {
      // 两阶段模式使用相对宽松的 early_gate；
      // 单阶段模式直接使用 strict_gate
      const bool gate =
          config_.two_stage_enabled ? early_gate(input) : strict_gate(input);

      // 根据模式选择观察时间
      const double capture_time =
          config_.two_stage_enabled ? config_.early_capture_time : config_.capture_time;

      // 根据模式选择观察窗口最大允许波动范围
      const double window_range_max =
          config_.two_stage_enabled ? config_.early_observation_window_range_max : config_.observation_window_range_max;

      state_.gate_open = gate;
      state_.verify_progress = 0.0;

      // 条件一旦不满足，当前连续采样作废
      if (!gate)
      {
        reset_capture_window();
        return;
      }

      // 第一个有效采样点：
      // 初始化窗口最大值和最小值
      if (capture_duration_ <= 0.0)
      {
        observation_window_min_ =
            state_.filtered_observed_com_offset;

        observation_window_max_ =
            state_.filtered_observed_com_offset;
      }
      else
      {
        // 后续不断更新窗口中的最大/最小值
        observation_window_min_ = std::min(
            observation_window_min_,
            state_.filtered_observed_com_offset);

        observation_window_max_ = std::max(
            observation_window_max_,
            state_.filtered_observed_com_offset);
      }

      // 观察窗口内偏移估计的波动范围
      state_.observation_window_range =
          observation_window_max_ - observation_window_min_;

      // 兼容旧日志字段
      state_.window_position_range =
          state_.observation_window_range;

      // 如果这段时间内质心偏移估计波动太大，
      // 说明机器人还没有真正稳定，放弃当前窗口重新开始
      if (state_.observation_window_range > window_range_max)
      {
        reset_capture_window();
        state_.gate_open = false;
        return;
      }

      // 对 filtered_observed_com_offset 进行时间积分，
      // 最终用于计算整个稳定窗口的平均质心偏移
      observation_integral_ +=
          state_.filtered_observed_com_offset * dt;

      capture_duration_ += dt;

      state_.stable_time = capture_duration_;

      // 用于日志显示当前采样进度
      state_.window_progress =
          std::min(1.0, capture_duration_ / capture_time);

      // 观察时间还没达到要求，继续采集
      if (capture_duration_ < capture_time)
      {
        return;
      }

      // ========================================================
      // 稳定采样完成
      //
      // 对整个时间窗口内的质心偏移观测求平均
      // ========================================================
      const double mean_observation =
          observation_integral_ / capture_duration_;

      // 当前观测结果与已经应用的质心补偿之间还有多少差值
      const double residual =
          mean_observation - state_.equivalent_com_offset;

      // 差值小于 observation_deadband 时认为无需继续修正
      state_.correction_error =
          std::abs(residual) <= config_.observation_deadband ? 0.0 : residual;

      // 已经基本无需修正
      if (state_.correction_error == 0.0)
      {
        reset_capture_window();

        if (config_.two_stage_enabled)
        {
          // 两阶段模式：继续进入精估计
          state_.phase = AdaptivePhase::WaitFine;
        }
        else
        {
          // 单阶段模式：检查位置误差
          state_.phase =
              std::abs(state_.filtered_x_error) <=
                      config_.position_error_deadband
                  ? AdaptivePhase::Hold
                  : AdaptivePhase::Verify;
        }

        state_.gate_open = false;
        return;
      }

      // ========================================================
      // 生成新的目标质心偏移
      //
      // adaptation_sign = +1 时：
      //
      // target
      // = current + (observed - current)
      // ≈ observed
      // ========================================================
      state_.target_com_offset = std::clamp(
          state_.equivalent_com_offset +
              config_.adaptation_sign * state_.correction_error,
          config_.offset_min,
          config_.offset_max);

      state_.target_updated = true;
      state_.updated = true;

      // 开始逐渐应用新的质心偏移
      state_.phase = AdaptivePhase::ApplyCoarse;

      reset_capture_window();
      state_.gate_open = false;
    }

    // ==========================================================
    // ApplyCoarse：缓慢应用粗估计得到的目标质心偏移
    //
    // 不允许 equivalent_com_offset 瞬间跳变，
    // 而是以 apply_rate_max 限制变化速度。
    // ==========================================================
    void update_apply_coarse(double dt)
    {
      state_.gate_open = false;
      state_.verify_progress = 0.0;

      // 距离目标偏移还有多少
      const double error =
          state_.target_com_offset -
          state_.equivalent_com_offset;

      // 当前周期允许改变的最大偏移量
      const double max_step =
          config_.apply_rate_max * dt;

      // 当前周期是否可以直接到达目标
      const bool reaches_target =
          std::abs(error) <= max_step;

      // 限制单周期变化量
      state_.offset_step =
          reaches_target ? error : std::clamp(error, -max_step, max_step);

      // 更新当前真正使用的质心偏移
      state_.equivalent_com_offset = std::clamp(
          state_.equivalent_com_offset + state_.offset_step,
          config_.offset_min,
          config_.offset_max);

      // 记录实际修改速度
      state_.apply_rate =
          state_.offset_step / dt;

      // 到达目标以后进入下一阶段
      if (reaches_target)
      {
        state_.equivalent_com_offset =
            state_.target_com_offset;

        if (config_.two_stage_enabled)
        {
          // 粗估计完成 -> 开始精估计
          state_.phase = AdaptivePhase::WaitFine;
        }
        else
        {
          // 单阶段模式直接验证
          state_.phase = AdaptivePhase::Verify;
          verify_duration_ = 0.0;
        }

        reset_capture_window();
      }
    }

    // ==========================================================
    // WaitFine：精细质心偏移估计
    //
    // 原理基本与 WaitCoarse 相同，
    // 但使用 strict_gate，要求机器人更加接近静止状态。
    // ==========================================================
    void update_wait_fine(
        double dt,
        const AdaptiveEquilibriumInput &input)
    {
      const bool gate = strict_gate(input);

      state_.gate_open = gate;
      state_.verify_progress = 0.0;

      // 不满足严格准静态条件，重新开始采样
      if (!gate)
      {
        reset_capture_window();
        return;
      }

      // 初始化 / 更新观察窗口最大最小值
      if (capture_duration_ <= 0.0)
      {
        observation_window_min_ =
            state_.filtered_observed_com_offset;

        observation_window_max_ =
            state_.filtered_observed_com_offset;
      }
      else
      {
        observation_window_min_ = std::min(
            observation_window_min_,
            state_.filtered_observed_com_offset);

        observation_window_max_ = std::max(
            observation_window_max_,
            state_.filtered_observed_com_offset);
      }

      // 当前观察窗口中的偏移波动
      state_.observation_window_range =
          observation_window_max_ -
          observation_window_min_;

      state_.window_position_range =
          state_.observation_window_range;

      // 数据波动过大，说明仍然不是可靠静态状态
      if (
          state_.observation_window_range >
          config_.observation_window_range_max)
      {
        reset_capture_window();
        state_.gate_open = false;
        return;
      }

      // 积分并累计观察时间
      observation_integral_ +=
          state_.filtered_observed_com_offset * dt;

      capture_duration_ += dt;

      state_.stable_time = capture_duration_;

      state_.window_progress =
          std::min(
              1.0,
              capture_duration_ / config_.capture_time);

      // 尚未达到规定采样时间
      if (capture_duration_ < config_.capture_time)
      {
        return;
      }

      // 精估计阶段的时间平均质心偏移
      const double mean_observation =
          observation_integral_ / capture_duration_;

      // 与当前已应用补偿比较
      const double residual =
          mean_observation -
          state_.equivalent_com_offset;

      state_.correction_error =
          std::abs(residual) <=
                  config_.observation_deadband
              ? 0.0
              : residual;

      // 如果已经基本不需要修正
      if (state_.correction_error == 0.0)
      {
        reset_capture_window();

        // 如果位置误差也足够小，则直接 Hold；
        // 否则进入 Verify 继续观察
        state_.phase =
            std::abs(state_.filtered_x_error) <=
                    config_.position_error_deadband
                ? AdaptivePhase::Hold
                : AdaptivePhase::Verify;

        verify_duration_ = 0.0;
        state_.gate_open = false;
        return;
      }

      // 更新精细阶段新的目标质心偏移
      state_.target_com_offset = std::clamp(
          state_.equivalent_com_offset +
              config_.adaptation_sign *
                  state_.correction_error,
          config_.offset_min,
          config_.offset_max);

      state_.target_updated = true;
      state_.updated = true;

      // 进入精细补偿应用阶段
      state_.phase = AdaptivePhase::ApplyFine;

      reset_capture_window();
      state_.gate_open = false;
    }

    // ==========================================================
    // ApplyFine：限速应用精估计结果
    // ==========================================================
    void update_apply_fine(double dt)
    {
      state_.gate_open = false;
      state_.verify_progress = 0.0;

      const double error =
          state_.target_com_offset -
          state_.equivalent_com_offset;

      const double max_step =
          config_.apply_rate_max * dt;

      const bool reaches_target =
          std::abs(error) <= max_step;

      state_.offset_step =
          reaches_target ? error : std::clamp(error, -max_step, max_step);

      state_.equivalent_com_offset = std::clamp(
          state_.equivalent_com_offset + state_.offset_step,
          config_.offset_min,
          config_.offset_max);

      state_.apply_rate =
          state_.offset_step / dt;

      // 精补偿完成以后开始验证
      if (reaches_target)
      {
        state_.equivalent_com_offset =
            state_.target_com_offset;

        state_.phase = AdaptivePhase::Verify;
        verify_duration_ = 0.0;
      }
    }

    // ==========================================================
    // Verify：验证当前质心补偿是否已经足够准确
    //
    // 判断两个问题：
    //
    // 1. 机器人位置是否已经处于允许死区？
    //    是 -> Hold
    //
    // 2. 如果位置仍有误差，质心估计是否仍存在明显残差？
    //    是 -> 重新 WaitFine
    //
    // 否则继续 Verify，不随意改变平衡点。
    // ==========================================================
    void update_verify(
        double dt,
        const AdaptiveEquilibriumInput &input)
    {
      const bool gate = strict_gate(input);

      state_.gate_open = gate;
      state_.window_progress = 0.0;

      // 验证期间一旦机器人再次运动，
      // 验证计时重新开始
      if (!gate)
      {
        verify_duration_ = 0.0;
        state_.verify_progress = 0.0;
        return;
      }

      verify_duration_ += dt;

      state_.verify_progress =
          std::min(
              1.0,
              verify_duration_ / config_.verify_time);

      // 必须持续稳定 verify_time 才进行判断
      if (verify_duration_ < config_.verify_time)
      {
        return;
      }

      verify_duration_ = 0.0;
      state_.verify_progress = 0.0;
      state_.gate_open = false;

      // 当前观测质心偏移与已经应用偏移之间的残差
      const double residual =
          state_.filtered_observed_com_offset -
          state_.equivalent_com_offset;

      // 位置误差已经足够小，认为自适应成功
      if (
          std::abs(state_.filtered_x_error) <=
          config_.position_error_deadband)
      {
        state_.phase = AdaptivePhase::Hold;
        hold_reacquire_duration_ = 0.0;
      }

      // 位置仍然有偏移，而且观测出来的质心补偿仍存在残差，
      // 则重新进行精估计
      else if (
          std::abs(residual) >
          config_.reacquire_threshold)
      {
        state_.phase =
            config_.two_stage_enabled ? AdaptivePhase::WaitFine : AdaptivePhase::WaitCoarse;

        reset_capture_window();
      }

      // 质心估计已经基本一致，但位置还没有完全稳定，
      // 此时不要继续修改质心，继续等待验证
      else
      {
        state_.phase = AdaptivePhase::Verify;
      }
    }

    // ==========================================================
    // Hold：保持当前自适应结果
    //
    // 正常情况下不再修改 equivalent_com_offset。
    //
    // 如果之后：
    //   1. 机器人满足严格静态条件
    //   2. 位置重新出现明显误差
    //   3. observed_com_offset 与当前补偿又出现明显残差
    //
    // 并持续 verify_time，
    // 则认为质量分布可能发生变化，重新进行估计。
    // ==========================================================
    void update_hold(
        double dt,
        const AdaptiveEquilibriumInput &input)
    {
      const double residual =
          state_.filtered_observed_com_offset -
          state_.equivalent_com_offset;

      const bool needs_reacquire =
          strict_gate(input) &&

          // 位置偏移重新超过允许范围
          std::abs(state_.filtered_x_error) >
              config_.position_error_deadband &&

          // 质心偏移估计也显示当前补偿已经不正确
          std::abs(residual) >
              config_.reacquire_threshold;

      state_.gate_open = needs_reacquire;
      state_.window_progress = 0.0;
      state_.verify_progress = 0.0;

      // 必须连续满足重估条件，
      // 防止单次噪声导致状态机跳转
      if (needs_reacquire)
      {
        hold_reacquire_duration_ += dt;
      }
      else
      {
        hold_reacquire_duration_ = 0.0;
      }

      if (
          hold_reacquire_duration_ >=
          config_.verify_time)
      {
        hold_reacquire_duration_ = 0.0;

        state_.phase =
            config_.two_stage_enabled ? AdaptivePhase::WaitFine : AdaptivePhase::WaitCoarse;

        reset_capture_window();
        state_.gate_open = false;
      }
    }

    // ==========================================================
    // 更新所有滤波量以及根据 pitch 反算等效质心偏移
    //
    // 这是整个估计器最核心的数学部分。
    // ==========================================================
    void update_filters(
        double dt,
        const AdaptiveEquilibriumInput &input)
    {
      // --------------------------------------------------------
      // 一阶低通滤波系数
      //
      // alpha = 1 - exp(-dt / tau)
      //
      // tau = filter_time_constant
      // --------------------------------------------------------
      const double alpha =
          1.0 -
          std::exp(
              -dt / config_.filter_time_constant);

      // --------------------------------------------------------
      // 对位置误差和速度进行低通滤波
      // --------------------------------------------------------
      if (!position_filter_initialized_)
      {
        state_.filtered_x_error = input.x_error;
        state_.filtered_x_dot = input.x_dot;

        previous_filtered_x_dot_ = input.x_dot;

        position_filter_initialized_ = true;
      }
      else
      {
        state_.filtered_x_error +=
            alpha *
            (input.x_error -
             state_.filtered_x_error);

        state_.filtered_x_dot +=
            alpha *
            (input.x_dot -
             state_.filtered_x_dot);
      }

      // --------------------------------------------------------
      // 根据滤波后的速度差分计算纵向加速度
      //
      // a = (v_k - v_{k-1}) / dt
      // --------------------------------------------------------
      const double raw_filtered_accel =
          (state_.filtered_x_dot -
           previous_filtered_x_dot_) /
          dt;

      previous_filtered_x_dot_ =
          state_.filtered_x_dot;

      // 加速度采用单独的低通时间常数
      const double accel_alpha =
          1.0 -
          std::exp(
              -dt /
              config_.accel_filter_time_constant);

      if (!acceleration_filter_initialized_)
      {
        state_.filtered_x_accel =
            raw_filtered_accel;

        acceleration_filter_initialized_ = true;
      }
      else
      {
        state_.filtered_x_accel +=
            accel_alpha *
            (raw_filtered_accel -
             state_.filtered_x_accel);
      }

      // --------------------------------------------------------
      // 检查计算质心偏移所需数据是否合法
      // --------------------------------------------------------
      state_.observation_valid =
          std::isfinite(input.pitch) &&
          std::isfinite(input.nominal_com_y) &&
          std::isfinite(input.nominal_com_z) &&
          input.nominal_com_z > 0.05;

      if (!state_.observation_valid)
      {
        return;
      }

      // ========================================================
      // 核心公式：
      //
      // 已知理论平衡关系：
      //
      // theta =
      //   -atan2(y_nom + Delta_y, z_nom)
      //
      // 对 Delta_y 反解：
      //
      // Delta_y =
      //   -z_nom * tan(theta) - y_nom
      //
      // 因此可以根据机器人实际稳定时的 pitch
      // 反推出等效纵向质心偏移。
      // ========================================================
      state_.observed_com_offset =
          -input.nominal_com_z *
              std::tan(input.pitch) -
          input.nominal_com_y;

      // 检查反算结果是否有效
      state_.observation_valid =
          std::isfinite(
              state_.observed_com_offset);

      if (!state_.observation_valid)
      {
        return;
      }

      // --------------------------------------------------------
      // 对 observed_com_offset 进行低通滤波，
      // 避免 IMU 噪声直接进入自适应平衡点。
      // --------------------------------------------------------
      if (!observation_filter_initialized_)
      {
        state_.filtered_observed_com_offset =
            state_.observed_com_offset;

        observation_filter_initialized_ = true;
      }
      else
      {
        state_.filtered_observed_com_offset +=
            alpha *
            (state_.observed_com_offset -
             state_.filtered_observed_com_offset);
      }
    }

    // ==========================================================
    // 清空当前稳定观测窗口
    //
    // 注意：
    // 这里只清除“当前采样窗口”，
    // 不会清除已经学习到的 equivalent_com_offset。
    // ==========================================================
    void reset_capture_window()
    {
      capture_duration_ = 0.0;
      observation_integral_ = 0.0;

      observation_window_min_ = 0.0;
      observation_window_max_ = 0.0;

      state_.stable_time = 0.0;
      state_.window_progress = 0.0;

      state_.observation_window_range = 0.0;

      // 旧日志兼容字段
      state_.window_position_range = 0.0;
    }

    // ==========================================================
    // 参数安全处理
    //
    // 防止 YAML / launch 中给出非法负值。
    // 时间常数和采样时间至少保持一个很小的正值。
    // ==========================================================
    void sanitize_config()
    {
      config_.early_capture_time =
          std::max(
              1.0e-3,
              config_.early_capture_time);

      config_.early_observation_window_range_max =
          std::max(
              0.0,
              config_.early_observation_window_range_max);

      config_.early_speed_max =
          std::max(
              0.0,
              config_.early_speed_max);

      config_.early_accel_threshold =
          std::max(
              0.0,
              config_.early_accel_threshold);

      config_.early_pitch_rate_threshold =
          std::max(
              0.0,
              config_.early_pitch_rate_threshold);

      config_.capture_time =
          std::max(
              1.0e-3,
              config_.capture_time);

      config_.observation_window_range_max =
          std::max(
              0.0,
              config_.observation_window_range_max);

      config_.apply_rate_max =
          std::max(
              0.0,
              config_.apply_rate_max);

      config_.target_tolerance =
          std::max(
              0.0,
              config_.target_tolerance);

      config_.verify_time =
          std::max(
              1.0e-3,
              config_.verify_time);

      config_.speed_safety_max =
          std::max(
              0.0,
              config_.speed_safety_max);

      config_.accel_threshold =
          std::max(
              0.0,
              config_.accel_threshold);

      config_.fast_pitch_rate_threshold =
          std::max(
              0.0,
              config_.fast_pitch_rate_threshold);

      config_.fast_torque_threshold =
          std::max(
              0.0,
              config_.fast_torque_threshold);

      config_.reacquire_threshold =
          std::max(
              0.0,
              config_.reacquire_threshold);

      config_.filter_time_constant =
          std::max(
              1.0e-3,
              config_.filter_time_constant);

      config_.accel_filter_time_constant =
          std::max(
              1.0e-3,
              config_.accel_filter_time_constant);

      config_.position_error_deadband =
          std::max(
              0.0,
              config_.position_error_deadband);

      config_.observation_deadband =
          std::max(
              0.0,
              config_.observation_deadband);

      config_.height_rate_threshold =
          std::max(
              0.0,
              config_.height_rate_threshold);

      config_.torque_ratio_threshold =
          std::max(
              0.0,
              config_.torque_ratio_threshold);

      config_.pitch_error_threshold =
          std::max(
              0.0,
              config_.pitch_error_threshold);

      // 如果最小值和最大值写反了，自动交换
      if (config_.offset_min > config_.offset_max)
      {
        std::swap(
            config_.offset_min,
            config_.offset_max);
      }
    }

    // ==========================================================
    // 成员变量
    // ==========================================================

    // 参数配置
    AdaptiveEquilibriumConfig config_;

    // 当前估计器状态
    AdaptiveEquilibriumState state_;

    // 当前稳定采样窗口累计时间
    double capture_duration_{0.0};

    // filtered_observed_com_offset 的时间积分
    // 用于最终求稳定窗口平均值
    double observation_integral_{0.0};

    // 当前观察窗口的最小 / 最大质心偏移
    double observation_window_min_{0.0};
    double observation_window_max_{0.0};

    // Verify 阶段累计稳定时间
    double verify_duration_{0.0};

    // Hold 阶段中“需要重新估计”条件持续时间
    double hold_reacquire_duration_{0.0};

    // 用于速度差分求加速度
    double previous_filtered_x_dot_{0.0};

    // 各滤波器是否已经获得第一个有效数据
    bool position_filter_initialized_{false};
    bool observation_filter_initialized_{false};
    bool acceleration_filter_initialized_{false};
  };

} // namespace bbot_balance_controller

#endif // BBOT_BALANCE_CONTROLLER__ADAPTIVE_EQUILIBRIUM_ESTIMATOR_HPP_