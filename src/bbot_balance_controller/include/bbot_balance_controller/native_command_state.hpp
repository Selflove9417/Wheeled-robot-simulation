#pragma once

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "bbot_balance_controller/thrust_support_dynamics.hpp"

namespace bbot_jump {

// NCS1 wire fields: magic, sim_ns, iteration, dt_ns, q/v/before_v/cmd masks,
// post q[9], post v[9], before-Physics v[9], and six simulation-input efforts.
// JointForceCmd values are pending simulation inputs, never measured torques.
struct NativeCommandState {
  bool valid{false};
  int64_t stamp_ns{0};
  int64_t dt_ns{0};
  uint64_t iteration{0};
  SupportQ9 q = SupportQ9::Zero();
  SupportQ9 v = SupportQ9::Zero();
  SupportQ9 before_v = SupportQ9::Zero();
  SupportActuatorVector simulation_input = SupportActuatorVector::Zero();
};

class NativeCommandStateWire {
 public:
  static bool parse(std::string_view wire, NativeCommandState &out) {
    out = {};
    std::vector<std::string_view> fields;
    for (size_t start = 0; start <= wire.size();) {
      const auto end = wire.find(',', start);
      const auto stop = end == std::string_view::npos ? wire.size() : end;
      fields.emplace_back(wire.substr(start, stop - start));
      if (end == std::string_view::npos) break;
      start = end + 1;
    }
    constexpr size_t kFields = 8 + 9 + 9 + 9 + 6;
    if (fields.size() != kFields || fields[0] != "NCS1") return false;
    int64_t stamp = 0, dt = 0;
    uint64_t iteration = 0;
    if (!Integer(fields[1], stamp) || !Integer(fields[2], iteration) ||
        !Integer(fields[3], dt) || stamp < 0 || iteration == 0 || dt <= 0)
      return false;
    constexpr uint64_t kQMask = (uint64_t{1} << 9) - 1;
    constexpr uint64_t kCommandMask = (uint64_t{1} << 6) - 1;
    uint64_t qmask = 0, vmask = 0, beforeMask = 0, commandMask = 0;
    if (!Integer(fields[4], qmask) || !Integer(fields[5], vmask) ||
        !Integer(fields[6], beforeMask) || !Integer(fields[7], commandMask) ||
        qmask != kQMask || vmask != kQMask || beforeMask != kQMask ||
        commandMask != kCommandMask)
      return false;
    size_t index = 8;
    for (int i = 0; i < 9; ++i) if (!Real(fields[index++], out.q[i])) return false;
    for (int i = 0; i < 9; ++i) if (!Real(fields[index++], out.v[i])) return false;
    for (int i = 0; i < 9; ++i) if (!Real(fields[index++], out.before_v[i])) return false;
    for (int i = 0; i < 6; ++i) if (!Real(fields[index++], out.simulation_input[i])) return false;
    out.stamp_ns = stamp;
    out.iteration = iteration;
    out.dt_ns = dt;
    out.valid = out.q.allFinite() && out.v.allFinite() && out.before_v.allFinite() &&
                out.simulation_input.allFinite();
    return out.valid;
  }

 private:
  template<class T> static bool Integer(std::string_view text, T &value) {
    if (text.empty()) return false;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    return result.ec == std::errc{} && result.ptr == text.data() + text.size();
  }
  static bool Real(std::string_view text, double &value) {
    if (text.empty()) return false;
    // from_chars rejects NaN/Inf on some standard libraries and accepts them
    // on others; the explicit finiteness test makes the wire contract stable.
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value,
                                        std::chars_format::general);
    return result.ec == std::errc{} && result.ptr == text.data() + text.size() &&
           std::isfinite(value);
  }
};

class NativeCommandStateHistory {
 public:
  static constexpr int64_t kMaxAgeNs = 10'000'000;
  static constexpr size_t kCapacity = 64;

  bool push(const NativeCommandState &state) {
    if (!state.valid || state.stamp_ns < 0 || state.dt_ns <= 0 || state.iteration == 0 ||
        !state.q.allFinite() || !state.v.allFinite() || !state.before_v.allFinite() ||
        !state.simulation_input.allFinite()) {
      clear();
      return false;
    }
    if (!states_.empty() && (state.stamp_ns <= states_.back().stamp_ns ||
        state.iteration <= states_.back().iteration)) {
      clear();
      return false;
    }
    states_.push_back(state);
    if (states_.size() > kCapacity) states_.pop_front();
    return true;
  }

  bool snapshot(int64_t control_ns, NativeCommandState &out) const {
    out = {};
    if (control_ns < 0) return false;
    for (auto it = states_.rbegin(); it != states_.rend(); ++it) {
      if (it->stamp_ns > control_ns) continue;
      if (control_ns - it->stamp_ns > kMaxAgeNs) return false;
      out = *it;
      return true;
    }
    return false;
  }

  void clear() { states_.clear(); }
  size_t size() const { return states_.size(); }

 private:
  std::deque<NativeCommandState> states_;
};

}  // namespace bbot_jump
