#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>

#include "bbot_balance_controller/complete_contact_takeoff.hpp"

namespace bbot_jump
{
struct AllocatorContactSnapshot
{
    bool valid{false};
    bool continuous{false};
    uint64_t sequence{0};
    uint64_t iteration{0};
    int64_t stamp_ns{0};
    uint8_t wheel_mask{0};
};

// Private time-aligned contact history for the opt-in thrust allocator.
// All timestamp comparisons stay in integer simulation nanoseconds.
class AllocatorContactHistory
{
  public:
    static constexpr std::size_t kCapacity = 64;
    static constexpr int64_t kFreshnessNs = 20'000'000;

    bool push(const CompleteGroundContactFrame &frame)
    {
        if (!frame.decoded || !frame.frame_valid || !frame.collision_pairs_allowed ||
            frame.sim_time_ns < 0 || frame.iteration == 0 ||
            frame.dt_ns != CompleteGroundContactWire::kExpectedDtNs)
        {
            clear();
            return false;
        }

        bool continuous = false;
        if (!frames_.empty())
        {
            const auto &previous = frames_.back();
            if (frame.sequence <= previous.sequence || frame.iteration <= previous.iteration ||
                frame.sim_time_ns <= previous.stamp_ns)
            {
                clear();
                return false;
            }
            continuous = previous.sequence != std::numeric_limits<uint64_t>::max() &&
                previous.iteration != std::numeric_limits<uint64_t>::max() &&
                frame.sequence == previous.sequence + 1 &&
                frame.iteration == previous.iteration + 1 &&
                frame.sim_time_ns - previous.stamp_ns ==
                    CompleteGroundContactWire::kExpectedDtNs;
        }
        frames_.push_back({frame.sequence, frame.iteration, frame.sim_time_ns,
                           frame.wheel_mask, continuous});
        if (frames_.size() > kCapacity)
            frames_.pop_front();
        return true;
    }

    bool snapshot(int64_t now_ns, AllocatorContactSnapshot &out) const
    {
        out = {};
        if (now_ns < 0)
            return false;
        for (auto it = frames_.rbegin(); it != frames_.rend(); ++it)
        {
            if (it->stamp_ns > now_ns)
                continue;  // A future callback frame cannot describe the current state.
            if (now_ns - it->stamp_ns > kFreshnessNs || !it->continuous)
                return false;
            out.valid = true;
            out.continuous = it->continuous;
            out.sequence = it->sequence;
            out.iteration = it->iteration;
            out.stamp_ns = it->stamp_ns;
            out.wheel_mask = it->wheel_mask;
            return true;
        }
        return false;
    }

    void clear() { frames_.clear(); }
    std::size_t size() const { return frames_.size(); }

  private:
    struct Entry
    {
        uint64_t sequence;
        uint64_t iteration;
        int64_t stamp_ns;
        uint8_t wheel_mask;
        bool continuous;
    };
    std::deque<Entry> frames_;
};
}  // namespace bbot_jump
