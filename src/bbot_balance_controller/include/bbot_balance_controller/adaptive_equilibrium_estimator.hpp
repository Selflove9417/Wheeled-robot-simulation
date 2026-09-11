#ifndef BBOT_BALANCE_CONTROLLER__ADAPTIVE_EQUILIBRIUM_ESTIMATOR_HPP_
#define BBOT_BALANCE_CONTROLLER__ADAPTIVE_EQUILIBRIUM_ESTIMATOR_HPP_

#include <algorithm>
#include <cmath>

namespace bbot_balance_controller
{

enum class AdaptivePhase
{
  WaitCoarse = 0,
  ApplyCoarse = 1,
  WaitFine = 2,
  ApplyFine = 3,
  Verify = 4,
  Hold = 5,

  // Aliases for backward compatibility
  WaitCapture = WaitCoarse,
  ApplyTarget = ApplyCoarse,
};

struct AdaptiveEquilibriumConfig
{
  // Stage 1: Early coarse capture parameters
  double early_capture_time{1.0};
  double early_observation_window_range_max{0.00010};
  double early_speed_max{0.080};
  double early_accel_threshold{0.015};
  double early_pitch_rate_threshold{0.010};
  bool two_stage_enabled{true};

  // Stage 2: Precision quasi-static capture parameters
  double capture_time{1.0};
  double observation_window_range_max{0.00010};
  double apply_rate_max{0.0010};
  double target_tolerance{0.00001};
  double verify_time{0.50};
  double speed_safety_max{0.015};
  double accel_threshold{0.005};
  double fast_pitch_rate_threshold{0.01};
  double fast_torque_threshold{0.05};
  double reacquire_threshold{0.00005};

  double filter_time_constant{0.50};
  double accel_filter_time_constant{0.10};
  double position_error_deadband{0.005};
  double observation_deadband{0.00005};
  double adaptation_sign{1.0};
  double offset_min{-0.050};
  double offset_max{0.050};
  double height_rate_threshold{0.01};
  double torque_ratio_threshold{0.75};
  double pitch_error_threshold{0.12};

  // Legacy parameters are retained so existing launch/config files remain
  // accepted. They are not used by the fast-capture state machine.
  double averaging_time{2.0};
  double cooldown_time{4.0};
  double correction_fraction{0.50};
  double offset_step_max{0.00025};
  double speed_threshold{0.001};
  double position_window_range_max{0.001};
  double pitch_rate_threshold{0.25};
  double torque_threshold{0.25};
};

struct AdaptiveEquilibriumInput
{
  double x_error{0.0};
  double x_dot{0.0};
  double pitch{0.0};
  double pitch_error{0.0};
  double pitch_rate{0.0};
  double height_rate{0.0};
  double torque{0.0};
  double torque_ratio{0.0};
  double nominal_com_y{0.0};
  double nominal_com_z{0.0};
  bool enabled{true};
};

struct AdaptiveEquilibriumState
{
  // Applied compensation and the most recently captured target [m].
  double equivalent_com_offset{0.0};
  double target_com_offset{0.0};

  double filtered_x_error{0.0};
  double filtered_x_dot{0.0};
  double filtered_x_accel{0.0};
  double observed_com_offset{0.0};
  double filtered_observed_com_offset{0.0};
  double correction_error{0.0};

  // Per-cycle applied offset increment [m] and rate [m/s].
  double offset_step{0.0};
  double apply_rate{0.0};

  double stable_time{0.0};
  double window_progress{0.0};
  double observation_window_range{0.0};
  double verify_progress{0.0};

  // Backward-compatible diagnostics. window_position_range mirrors the new
  // observation range; cooldown is always zero in the new state machine.
  double window_position_range{0.0};
  double cooldown_remaining{0.0};

  AdaptivePhase phase{AdaptivePhase::WaitCoarse};
  bool gate_open{false};
  bool observation_valid{false};
  bool updated{false};
  bool target_updated{false};
};

class AdaptiveEquilibriumEstimator
{
public:
  explicit AdaptiveEquilibriumEstimator(
    const AdaptiveEquilibriumConfig & config = AdaptiveEquilibriumConfig())
  : config_(config)
  {
    sanitize_config();
  }

  void reset(double equivalent_com_offset = 0.0)
  {
    state_ = AdaptiveEquilibriumState{};
    state_.equivalent_com_offset = std::clamp(
      equivalent_com_offset, config_.offset_min, config_.offset_max);
    state_.target_com_offset = state_.equivalent_com_offset;
    state_.phase = AdaptivePhase::WaitCoarse;
    reset_capture_window();
    verify_duration_ = 0.0;
    hold_reacquire_duration_ = 0.0;
    position_filter_initialized_ = false;
    observation_filter_initialized_ = false;
    acceleration_filter_initialized_ = false;
    previous_filtered_x_dot_ = 0.0;
  }

  const AdaptiveEquilibriumState & update(
    double dt, const AdaptiveEquilibriumInput & input)
  {
    state_.updated = false;
    state_.target_updated = false;
    state_.offset_step = 0.0;
    state_.apply_rate = 0.0;

    if (!(dt > 0.0) || !std::isfinite(dt)) {
      return state_;
    }

    update_filters(dt, input);

    if (!input.enabled) {
      reset_capture_window();
      verify_duration_ = 0.0;
      hold_reacquire_duration_ = 0.0;
      state_.phase = AdaptivePhase::WaitCoarse;
      state_.gate_open = false;
      state_.verify_progress = 0.0;
      return state_;
    }

    switch (state_.phase) {
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

  const AdaptiveEquilibriumState & state() const
  {
    return state_;
  }

  static double equilibrium_pitch(
    double nominal_com_y, double nominal_com_z, double equivalent_com_offset)
  {
    const double z = std::max(0.05, nominal_com_z);
    return -std::atan2(nominal_com_y + equivalent_com_offset, z);
  }

private:
  static double apply_deadband(double value, double width)
  {
    if (std::abs(value) <= width) {
      return 0.0;
    }
    return std::copysign(std::abs(value) - width, value);
  }

  bool early_gate(const AdaptiveEquilibriumInput & input) const
  {
    return state_.observation_valid &&
      std::abs(input.x_dot) <= config_.early_speed_max &&
      std::abs(state_.filtered_x_accel) <= config_.early_accel_threshold &&
      std::abs(input.pitch_rate) <= config_.early_pitch_rate_threshold &&
      std::abs(input.height_rate) <= config_.height_rate_threshold &&
      std::abs(input.pitch_error) <= config_.pitch_error_threshold;
  }

  bool strict_gate(const AdaptiveEquilibriumInput & input) const
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

  bool fast_gate(const AdaptiveEquilibriumInput & input) const
  {
    return strict_gate(input);
  }

  void update_wait_coarse(double dt, const AdaptiveEquilibriumInput & input)
  {
    const bool gate = config_.two_stage_enabled ? early_gate(input) : strict_gate(input);
    const double capture_time =
      config_.two_stage_enabled ? config_.early_capture_time : config_.capture_time;
    const double window_range_max =
      config_.two_stage_enabled ? config_.early_observation_window_range_max : config_.observation_window_range_max;

    state_.gate_open = gate;
    state_.verify_progress = 0.0;
    if (!gate) {
      reset_capture_window();
      return;
    }

    if (capture_duration_ <= 0.0) {
      observation_window_min_ = state_.filtered_observed_com_offset;
      observation_window_max_ = state_.filtered_observed_com_offset;
    } else {
      observation_window_min_ = std::min(
        observation_window_min_, state_.filtered_observed_com_offset);
      observation_window_max_ = std::max(
        observation_window_max_, state_.filtered_observed_com_offset);
    }

    state_.observation_window_range =
      observation_window_max_ - observation_window_min_;
    state_.window_position_range = state_.observation_window_range;
    if (state_.observation_window_range > window_range_max) {
      reset_capture_window();
      state_.gate_open = false;
      return;
    }

    observation_integral_ += state_.filtered_observed_com_offset * dt;
    capture_duration_ += dt;
    state_.stable_time = capture_duration_;
    state_.window_progress = std::min(1.0, capture_duration_ / capture_time);
    if (capture_duration_ < capture_time) {
      return;
    }

    const double mean_observation = observation_integral_ / capture_duration_;
    const double residual = mean_observation - state_.equivalent_com_offset;
    state_.correction_error =
      std::abs(residual) <= config_.observation_deadband ? 0.0 : residual;

    if (state_.correction_error == 0.0) {
      reset_capture_window();
      if (config_.two_stage_enabled) {
        state_.phase = AdaptivePhase::WaitFine;
      } else {
        state_.phase =
          std::abs(state_.filtered_x_error) <= config_.position_error_deadband ?
          AdaptivePhase::Hold : AdaptivePhase::Verify;
      }
      state_.gate_open = false;
      return;
    }

    state_.target_com_offset = std::clamp(
      state_.equivalent_com_offset +
      config_.adaptation_sign * state_.correction_error,
      config_.offset_min, config_.offset_max);
    state_.target_updated = true;
    state_.updated = true;
    state_.phase = AdaptivePhase::ApplyCoarse;
    reset_capture_window();
    state_.gate_open = false;
  }

  void update_apply_coarse(double dt)
  {
    state_.gate_open = false;
    state_.verify_progress = 0.0;
    const double error = state_.target_com_offset - state_.equivalent_com_offset;
    const double max_step = config_.apply_rate_max * dt;
    const bool reaches_target = std::abs(error) <= max_step;
    state_.offset_step = reaches_target ? error : std::clamp(error, -max_step, max_step);
    state_.equivalent_com_offset = std::clamp(
      state_.equivalent_com_offset + state_.offset_step,
      config_.offset_min, config_.offset_max);
    state_.apply_rate = state_.offset_step / dt;

    if (reaches_target) {
      state_.equivalent_com_offset = state_.target_com_offset;
      if (config_.two_stage_enabled) {
        state_.phase = AdaptivePhase::WaitFine;
      } else {
        state_.phase = AdaptivePhase::Verify;
        verify_duration_ = 0.0;
      }
      reset_capture_window();
    }
  }

  void update_wait_fine(double dt, const AdaptiveEquilibriumInput & input)
  {
    const bool gate = strict_gate(input);
    state_.gate_open = gate;
    state_.verify_progress = 0.0;
    if (!gate) {
      reset_capture_window();
      return;
    }

    if (capture_duration_ <= 0.0) {
      observation_window_min_ = state_.filtered_observed_com_offset;
      observation_window_max_ = state_.filtered_observed_com_offset;
    } else {
      observation_window_min_ = std::min(
        observation_window_min_, state_.filtered_observed_com_offset);
      observation_window_max_ = std::max(
        observation_window_max_, state_.filtered_observed_com_offset);
    }

    state_.observation_window_range =
      observation_window_max_ - observation_window_min_;
    state_.window_position_range = state_.observation_window_range;
    if (state_.observation_window_range > config_.observation_window_range_max) {
      reset_capture_window();
      state_.gate_open = false;
      return;
    }

    observation_integral_ += state_.filtered_observed_com_offset * dt;
    capture_duration_ += dt;
    state_.stable_time = capture_duration_;
    state_.window_progress = std::min(1.0, capture_duration_ / config_.capture_time);
    if (capture_duration_ < config_.capture_time) {
      return;
    }

    const double mean_observation = observation_integral_ / capture_duration_;
    const double residual = mean_observation - state_.equivalent_com_offset;
    state_.correction_error =
      std::abs(residual) <= config_.observation_deadband ? 0.0 : residual;

    if (state_.correction_error == 0.0) {
      reset_capture_window();
      state_.phase =
        std::abs(state_.filtered_x_error) <= config_.position_error_deadband ?
        AdaptivePhase::Hold : AdaptivePhase::Verify;
      verify_duration_ = 0.0;
      state_.gate_open = false;
      return;
    }

    state_.target_com_offset = std::clamp(
      state_.equivalent_com_offset +
      config_.adaptation_sign * state_.correction_error,
      config_.offset_min, config_.offset_max);
    state_.target_updated = true;
    state_.updated = true;
    state_.phase = AdaptivePhase::ApplyFine;
    reset_capture_window();
    state_.gate_open = false;
  }

  void update_apply_fine(double dt)
  {
    state_.gate_open = false;
    state_.verify_progress = 0.0;
    const double error = state_.target_com_offset - state_.equivalent_com_offset;
    const double max_step = config_.apply_rate_max * dt;
    const bool reaches_target = std::abs(error) <= max_step;
    state_.offset_step = reaches_target ? error : std::clamp(error, -max_step, max_step);
    state_.equivalent_com_offset = std::clamp(
      state_.equivalent_com_offset + state_.offset_step,
      config_.offset_min, config_.offset_max);
    state_.apply_rate = state_.offset_step / dt;

    if (reaches_target) {
      state_.equivalent_com_offset = state_.target_com_offset;
      state_.phase = AdaptivePhase::Verify;
      verify_duration_ = 0.0;
    }
  }

  void update_verify(double dt, const AdaptiveEquilibriumInput & input)
  {
    const bool gate = strict_gate(input);
    state_.gate_open = gate;
    state_.window_progress = 0.0;
    if (!gate) {
      verify_duration_ = 0.0;
      state_.verify_progress = 0.0;
      return;
    }

    verify_duration_ += dt;
    state_.verify_progress = std::min(1.0, verify_duration_ / config_.verify_time);
    if (verify_duration_ < config_.verify_time) {
      return;
    }

    verify_duration_ = 0.0;
    state_.verify_progress = 0.0;
    state_.gate_open = false;
    const double residual =
      state_.filtered_observed_com_offset - state_.equivalent_com_offset;
    if (std::abs(state_.filtered_x_error) <= config_.position_error_deadband) {
      state_.phase = AdaptivePhase::Hold;
      hold_reacquire_duration_ = 0.0;
    } else if (std::abs(residual) > config_.reacquire_threshold) {
      state_.phase = config_.two_stage_enabled ?
        AdaptivePhase::WaitFine : AdaptivePhase::WaitCoarse;
      reset_capture_window();
    } else {
      // Position has not yet settled, so verify again without changing target.
      state_.phase = AdaptivePhase::Verify;
    }
  }

  void update_hold(double dt, const AdaptiveEquilibriumInput & input)
  {
    const double residual =
      state_.filtered_observed_com_offset - state_.equivalent_com_offset;
    const bool needs_reacquire = strict_gate(input) &&
      std::abs(state_.filtered_x_error) > config_.position_error_deadband &&
      std::abs(residual) > config_.reacquire_threshold;

    state_.gate_open = needs_reacquire;
    state_.window_progress = 0.0;
    state_.verify_progress = 0.0;
    if (needs_reacquire) {
      hold_reacquire_duration_ += dt;
    } else {
      hold_reacquire_duration_ = 0.0;
    }

    if (hold_reacquire_duration_ >= config_.verify_time) {
      hold_reacquire_duration_ = 0.0;
      state_.phase = config_.two_stage_enabled ?
        AdaptivePhase::WaitFine : AdaptivePhase::WaitCoarse;
      reset_capture_window();
      state_.gate_open = false;
    }
  }

  void update_filters(double dt, const AdaptiveEquilibriumInput & input)
  {
    const double alpha = 1.0 - std::exp(-dt / config_.filter_time_constant);
    if (!position_filter_initialized_) {
      state_.filtered_x_error = input.x_error;
      state_.filtered_x_dot = input.x_dot;
      previous_filtered_x_dot_ = input.x_dot;
      position_filter_initialized_ = true;
    } else {
      state_.filtered_x_error += alpha * (input.x_error - state_.filtered_x_error);
      state_.filtered_x_dot += alpha * (input.x_dot - state_.filtered_x_dot);
    }

    const double raw_filtered_accel =
      (state_.filtered_x_dot - previous_filtered_x_dot_) / dt;
    previous_filtered_x_dot_ = state_.filtered_x_dot;
    const double accel_alpha =
      1.0 - std::exp(-dt / config_.accel_filter_time_constant);
    if (!acceleration_filter_initialized_) {
      state_.filtered_x_accel = raw_filtered_accel;
      acceleration_filter_initialized_ = true;
    } else {
      state_.filtered_x_accel +=
        accel_alpha * (raw_filtered_accel - state_.filtered_x_accel);
    }

    state_.observation_valid =
      std::isfinite(input.pitch) &&
      std::isfinite(input.nominal_com_y) &&
      std::isfinite(input.nominal_com_z) &&
      input.nominal_com_z > 0.05;
    if (!state_.observation_valid) {
      return;
    }

    state_.observed_com_offset =
      -input.nominal_com_z * std::tan(input.pitch) - input.nominal_com_y;
    state_.observation_valid = std::isfinite(state_.observed_com_offset);
    if (!state_.observation_valid) {
      return;
    }

    if (!observation_filter_initialized_) {
      state_.filtered_observed_com_offset = state_.observed_com_offset;
      observation_filter_initialized_ = true;
    } else {
      state_.filtered_observed_com_offset +=
        alpha * (state_.observed_com_offset - state_.filtered_observed_com_offset);
    }
  }

  void reset_capture_window()
  {
    capture_duration_ = 0.0;
    observation_integral_ = 0.0;
    observation_window_min_ = 0.0;
    observation_window_max_ = 0.0;
    state_.stable_time = 0.0;
    state_.window_progress = 0.0;
    state_.observation_window_range = 0.0;
    state_.window_position_range = 0.0;
  }

  void sanitize_config()
  {
    config_.early_capture_time = std::max(1.0e-3, config_.early_capture_time);
    config_.early_observation_window_range_max =
      std::max(0.0, config_.early_observation_window_range_max);
    config_.early_speed_max = std::max(0.0, config_.early_speed_max);
    config_.early_accel_threshold = std::max(0.0, config_.early_accel_threshold);
    config_.early_pitch_rate_threshold =
      std::max(0.0, config_.early_pitch_rate_threshold);

    config_.capture_time = std::max(1.0e-3, config_.capture_time);
    config_.observation_window_range_max =
      std::max(0.0, config_.observation_window_range_max);
    config_.apply_rate_max = std::max(0.0, config_.apply_rate_max);
    config_.target_tolerance = std::max(0.0, config_.target_tolerance);
    config_.verify_time = std::max(1.0e-3, config_.verify_time);
    config_.speed_safety_max = std::max(0.0, config_.speed_safety_max);
    config_.accel_threshold = std::max(0.0, config_.accel_threshold);
    config_.fast_pitch_rate_threshold =
      std::max(0.0, config_.fast_pitch_rate_threshold);
    config_.fast_torque_threshold = std::max(0.0, config_.fast_torque_threshold);
    config_.reacquire_threshold = std::max(0.0, config_.reacquire_threshold);
    config_.filter_time_constant = std::max(1.0e-3, config_.filter_time_constant);
    config_.accel_filter_time_constant =
      std::max(1.0e-3, config_.accel_filter_time_constant);
    config_.position_error_deadband = std::max(0.0, config_.position_error_deadband);
    config_.observation_deadband = std::max(0.0, config_.observation_deadband);
    config_.height_rate_threshold = std::max(0.0, config_.height_rate_threshold);
    config_.torque_ratio_threshold = std::max(0.0, config_.torque_ratio_threshold);
    config_.pitch_error_threshold = std::max(0.0, config_.pitch_error_threshold);
    if (config_.offset_min > config_.offset_max) {
      std::swap(config_.offset_min, config_.offset_max);
    }
  }

  AdaptiveEquilibriumConfig config_;
  AdaptiveEquilibriumState state_;

  double capture_duration_{0.0};
  double observation_integral_{0.0};
  double observation_window_min_{0.0};
  double observation_window_max_{0.0};
  double verify_duration_{0.0};
  double hold_reacquire_duration_{0.0};
  double previous_filtered_x_dot_{0.0};
  bool position_filter_initialized_{false};
  bool observation_filter_initialized_{false};
  bool acceleration_filter_initialized_{false};
};

}  // namespace bbot_balance_controller

#endif  // BBOT_BALANCE_CONTROLLER__ADAPTIVE_EQUILIBRIUM_ESTIMATOR_HPP_
