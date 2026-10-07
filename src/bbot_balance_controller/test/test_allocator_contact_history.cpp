#include <cassert>

#include "bbot_balance_controller/allocator_contact_history.hpp"

using bbot_jump::AllocatorContactHistory;
using bbot_jump::AllocatorContactSnapshot;
using bbot_jump::CompleteGroundContactFrame;

namespace
{
CompleteGroundContactFrame frame(uint64_t sequence, uint64_t iteration, int64_t stamp)
{
    CompleteGroundContactFrame value;
    value.sequence = sequence;
    value.iteration = iteration;
    value.sim_time_ns = stamp;
    value.dt_ns = bbot_jump::CompleteGroundContactWire::kExpectedDtNs;
    value.decoded = true;
    value.frame_valid = true;
    value.collision_pairs_allowed = true;
    value.wheel_mask = 0x3;
    return value;
}
}

int main()
{
    // A future callback frame must not hide the newest continuous frame at or
    // before the control clock, including when stamps are exactly equal.
    AllocatorContactHistory history;
    assert(history.push(frame(10, 20, 1'000'000)));
    assert(history.push(frame(11, 21, 2'000'000)));
    assert(history.push(frame(12, 22, 3'000'000)));
    AllocatorContactSnapshot snapshot;
    assert(history.snapshot(2'000'000, snapshot));
    assert(snapshot.valid && snapshot.continuous);
    assert(snapshot.sequence == 11 && snapshot.iteration == 21);
    assert(snapshot.stamp_ns == 2'000'000 && snapshot.wheel_mask == 0x3);

    // Integer freshness boundary is inclusive; one nanosecond beyond is stale.
    assert(history.snapshot(23'000'000, snapshot));
    assert(!history.snapshot(23'000'001, snapshot));

    // A sequence/iteration gap is retained as a discontinuity and cannot be
    // used as the current snapshot.
    assert(history.push(frame(14, 24, 4'000'000)));
    assert(!history.snapshot(4'000'000, snapshot));

    // Invalid input clears the private history rather than preserving stale contact.
    auto invalid = frame(15, 25, 5'000'000);
    invalid.frame_valid = false;
    assert(!history.push(invalid));
    assert(history.size() == 0);

    // A timestamp gap also breaks continuity, even with adjacent counters.
    assert(history.push(frame(30, 40, 10'000'000)));
    assert(history.push(frame(31, 41, 12'000'000)));
    assert(!history.snapshot(12'000'000, snapshot));
    return 0;
}
