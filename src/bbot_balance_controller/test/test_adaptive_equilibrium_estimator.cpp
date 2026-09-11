#include <cassert>
#include <cmath>

#include "bbot_balance_controller/adaptive_equilibrium_estimator.hpp"

using bbot_balance_controller::AdaptiveEquilibriumConfig;
using bbot_balance_controller::AdaptiveEquilibriumEstimator;
using bbot_balance_controller::AdaptiveEquilibriumInput;
using bbot_balance_controller::AdaptivePhase;

namespace
{

AdaptiveEquilibriumConfig fast_test_config()
{
  AdaptiveEquilibriumConfig config;
  config.two_stage_enabled = false;
  config.early_capture_time = 0.10;
  config.capture_time = 0.10;
  config.observation_window_range_max = 0.00010;
  config.apply_rate_max = 0.020;
  config.target_tolerance = 1.0e-8;
  config.verify_time = 0.05;
  config.speed_safety_max = 1.0;
  config.accel_threshold = 10.0;
  config.fast_pitch_rate_threshold = 1.0;
  config.fast_torque_threshold = 1.0;
  config.reacquire_threshold = 0.00010;
  config.filter_time_constant = 0.01;
  config.accel_filter_time_constant = 0.01;
  config.position_error_deadband = 0.005;
  config.observation_deadband = 0.0;
  config.offset_min = -0.02;
  config.offset_max = 0.02;
  return config;
}

AdaptiveEquilibriumInput input_for_offset(double offset)
{
  AdaptiveEquilibriumInput input;
  input.x_error = 0.10;
  input.nominal_com_y = -0.02;
  input.nominal_com_z = 0.20;
  input.pitch = -std::atan2(input.nominal_com_y + offset, input.nominal_com_z);
  return input;
}

}  // namespace

int main()
{
  constexpr double dt = 0.01;
  auto config = fast_test_config();
  AdaptiveEquilibriumEstimator estimator(config);
  auto input = input_for_offset(0.01);
  input.x_error = 0.001;  // Capture is allowed before the 5 mm task error develops.

  // A complete trustworthy window locks the observed target without applying
  // the full compensation as a discontinuous jump.
  for (int index = 0; index < 9; ++index) {
    estimator.update(dt, input);
  }
  assert(estimator.state().phase == AdaptivePhase::WaitCapture);
  assert(std::abs(estimator.state().equivalent_com_offset) < 1.0e-12);

  bool target_locked = false;
  for (int index = 0; index < 3; ++index) {
    const auto & state = estimator.update(dt, input);
    if (state.target_updated) {
      target_locked = true;
      break;
    }
  }
  assert(target_locked);
  assert(estimator.state().phase == AdaptivePhase::ApplyTarget);
  assert(std::abs(estimator.state().target_com_offset - 0.01) < 1.0e-12);
  assert(std::abs(estimator.state().equivalent_com_offset) < 1.0e-12);

  // Continuous application must obey the configured rate on every cycle.
  double previous = estimator.state().equivalent_com_offset;
  while (estimator.state().phase == AdaptivePhase::ApplyTarget) {
    estimator.update(dt, input);
    const double increment = estimator.state().equivalent_com_offset - previous;
    assert(std::abs(increment) <= config.apply_rate_max * dt + 1.0e-12);
    previous = estimator.state().equivalent_com_offset;
  }
  assert(estimator.state().phase == AdaptivePhase::Verify);
  assert(std::abs(estimator.state().equivalent_com_offset - 0.01) < 1.0e-12);

  // Once position performance is inside the task deadband, verification ends
  // in HOLD and the learned compensation is retained.
  input.x_error = 0.0;
  for (int index = 0; index < 8; ++index) {
    estimator.update(dt, input);
  }
  assert(estimator.state().phase == AdaptivePhase::Hold);
  assert(std::abs(estimator.state().equivalent_com_offset - 0.01) < 1.0e-12);

  // A changing observation invalidates a partially collected capture window.
  estimator.reset();
  input = input_for_offset(0.002);
  for (int index = 0; index < 5; ++index) {
    estimator.update(dt, input);
  }
  assert(estimator.state().window_progress > 0.0);
  input.pitch = -std::atan2(input.nominal_com_y + 0.004, input.nominal_com_z);
  estimator.update(dt, input);
  assert(estimator.state().window_progress == 0.0);
  assert(estimator.state().observation_window_range == 0.0);

  // Acceleration is a capture gate even when absolute speed is below the broad
  // safety limit.
  auto accel_config = fast_test_config();
  accel_config.accel_threshold = 0.005;
  AdaptiveEquilibriumEstimator accel_estimator(accel_config);
  input = input_for_offset(0.002);
  input.x_dot = 0.0;
  accel_estimator.update(dt, input);
  input.x_dot = 0.010;
  accel_estimator.update(dt, input);
  assert(!accel_estimator.state().gate_open);
  assert(accel_estimator.state().phase == AdaptivePhase::WaitCapture);
  assert(accel_estimator.state().window_progress == 0.0);

  const double adapted_pitch = AdaptiveEquilibriumEstimator::equilibrium_pitch(
    input.nominal_com_y, input.nominal_com_z, 0.01);
  const double nominal_pitch = AdaptiveEquilibriumEstimator::equilibrium_pitch(
    input.nominal_com_y, input.nominal_com_z, 0.0);
  assert(adapted_pitch < nominal_pitch);

  estimator.reset();
  assert(estimator.state().phase == AdaptivePhase::WaitCapture);
  assert(std::abs(estimator.state().equivalent_com_offset) < 1.0e-12);

  // Two-stage state machine verification:
  // WaitCoarse -> ApplyCoarse -> WaitFine -> ApplyFine -> Verify -> Hold
  auto two_stage_config = fast_test_config();
  two_stage_config.two_stage_enabled = true;
  AdaptiveEquilibriumEstimator ts_estimator(two_stage_config);
  assert(ts_estimator.state().phase == AdaptivePhase::WaitCoarse);

  input = input_for_offset(0.01);
  input.x_error = 0.001;
  // Accumulate coarse capture
  for (int index = 0; index < 12; ++index) {
    ts_estimator.update(dt, input);
  }
  assert(ts_estimator.state().phase == AdaptivePhase::ApplyCoarse);
  assert(std::abs(ts_estimator.state().target_com_offset - 0.01) < 1.0e-12);

  // Apply coarse compensation until target reached
  while (ts_estimator.state().phase == AdaptivePhase::ApplyCoarse) {
    ts_estimator.update(dt, input);
  }
  assert(ts_estimator.state().phase == AdaptivePhase::WaitFine);
  assert(std::abs(ts_estimator.state().equivalent_com_offset - 0.01) < 1.0e-12);

  // Introduce a slight shift for precision refinement: 0.01 -> 0.012
  input = input_for_offset(0.012);
  input.x_error = 0.001;
  bool fine_locked = false;
  for (int index = 0; index < 30; ++index) {
    ts_estimator.update(dt, input);
    if (ts_estimator.state().phase == AdaptivePhase::ApplyFine) {
      fine_locked = true;
      break;
    }
  }
  assert(fine_locked);
  assert(ts_estimator.state().phase == AdaptivePhase::ApplyFine);
  assert(std::abs(ts_estimator.state().target_com_offset - 0.012) < 1.0e-3);

  while (ts_estimator.state().phase == AdaptivePhase::ApplyFine) {
    ts_estimator.update(dt, input);
  }
  assert(ts_estimator.state().phase == AdaptivePhase::Verify);
  assert(std::abs(ts_estimator.state().equivalent_com_offset - ts_estimator.state().target_com_offset) < 1.0e-12);

  input.x_error = 0.0;
  for (int index = 0; index < 8; ++index) {
    ts_estimator.update(dt, input);
  }
  assert(ts_estimator.state().phase == AdaptivePhase::Hold);

  return 0;
}
