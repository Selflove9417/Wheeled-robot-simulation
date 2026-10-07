#pragma once

#include <utility>
#include "bbot_balance_controller/native_command_state.hpp"

namespace bbot_jump {

// ECS1 has the NCS1 field layout, but is sourced exclusively from direct
// Physics before/after CSV records plus the actual BeforePhysics ForceCmd.
// NCS1 ECM packets must never be accepted on this control-state path.
class GroundEngineStateWire {
 public:
  static bool parse(std::string_view wire, NativeCommandState &out) {
    out = {};
    if (wire.substr(0, 5) != "ECS1,") return false;
    std::string compatible(wire);
    compatible.replace(0, 4, "NCS1");
    if (!NativeCommandStateWire::parse(compatible, out) || out.dt_ns != 1'000'000) {
      out = {};
      return false;
    }
    return true;
  }
};

// Both clocks must be fresh. A paused or delayed /clock must not make an old
// sensor packet look current; select the receipt belonging to the chosen step.
class GroundEngineStateHistory {
 public:
  bool push(const NativeCommandState &state, int64_t receipt_steady_ns) {
    const bool has_previous = !receipts_.empty();
    const bool continuous = !has_previous ||
        (last_iteration_ != std::numeric_limits<uint64_t>::max() &&
         last_stamp_ns_ <= std::numeric_limits<int64_t>::max() - kExpectedStepNs &&
         state.iteration == last_iteration_ + 1 &&
         state.stamp_ns == last_stamp_ns_ + kExpectedStepNs);
    if (receipt_steady_ns < 0 || state.dt_ns != kExpectedStepNs || !continuous ||
        (has_previous && receipt_steady_ns < receipts_.back().second) ||
        !history_.push(state)) {
      clear();
      return false;
    }
    receipts_.emplace_back(state.iteration, receipt_steady_ns);
    last_iteration_ = state.iteration;
    last_stamp_ns_ = state.stamp_ns;
    if (receipts_.size() > NativeCommandStateHistory::kCapacity) receipts_.pop_front();
    return true;
  }
  bool snapshot(int64_t control_ns, int64_t now_steady_ns,
                NativeCommandState &out) const {
    if (!history_.snapshot(control_ns, out)) return false;
    for (auto it = receipts_.rbegin(); it != receipts_.rend(); ++it) {
      if (it->first != out.iteration) continue;
      if (now_steady_ns >= it->second &&
          now_steady_ns - it->second <= NativeCommandStateHistory::kMaxAgeNs)
        return true;
      out = {};
      return false;
    }
    out = {};
    return false;
  }
  void clear() {
    history_.clear();
    receipts_.clear();
    last_iteration_ = 0;
    last_stamp_ns_ = 0;
  }
  size_t size() const { return history_.size(); }
 private:
  static constexpr int64_t kExpectedStepNs = 1'000'000;
  NativeCommandStateHistory history_;
  std::deque<std::pair<uint64_t, int64_t>> receipts_;
  uint64_t last_iteration_{0};
  int64_t last_stamp_ns_{0};
};

// Sticky validation for the physics-rate contact stream. The controller calls
// observe for every callback (not only at its slower control tick), so a brief
// contact loss cannot disappear when the next snapshot is already bilateral.
class GroundContactContinuityGuard {
 public:
  static constexpr int64_t kExpectedStepNs = 1'000'000;

  bool observe(uint64_t sequence, int64_t stamp_ns, uint64_t iteration,
               int64_t dt_ns, bool decoded_and_valid, bool bilateral,
               bool active) {
    if (failed_) return false;
    if (!decoded_and_valid || dt_ns != kExpectedStepNs || stamp_ns < 0) {
      if (active) return latch("contact_source_invalid");
      have_previous_ = false;
      return true;
    }
    if (!bilateral) {
      if (active) return latch("contact_not_bilateral");
      have_previous_ = false;
      return true;
    }
    if (have_previous_ &&
        (previous_sequence_ == std::numeric_limits<uint64_t>::max() ||
         previous_iteration_ == std::numeric_limits<uint64_t>::max() ||
         previous_stamp_ns_ > std::numeric_limits<int64_t>::max() - kExpectedStepNs ||
         sequence != previous_sequence_ + 1 ||
         iteration != previous_iteration_ + 1 ||
         stamp_ns != previous_stamp_ns_ + kExpectedStepNs)) {
      if (active) return latch("contact_clock_gap_or_rollback");
      have_previous_ = false;
    }
    previous_sequence_ = sequence;
    previous_iteration_ = iteration;
    previous_stamp_ns_ = stamp_ns;
    have_previous_ = true;
    return true;
  }

  bool failed() const { return failed_; }
  const std::string &reason() const { return reason_; }

 private:
  bool latch(const char *reason) {
    failed_ = true;
    reason_ = reason;
    return false;
  }

  bool have_previous_{false};
  bool failed_{false};
  uint64_t previous_sequence_{0};
  uint64_t previous_iteration_{0};
  int64_t previous_stamp_ns_{0};
  std::string reason_;
};

}  // namespace bbot_jump
