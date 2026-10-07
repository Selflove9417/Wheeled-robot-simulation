#include <cassert>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "bbot_balance_controller/complete_contact_takeoff.hpp"

using namespace bbot_jump;

namespace
{
const std::string kGround = CompleteGroundContactWire::ground_name();
const std::string kLeft = CompleteGroundContactWire::left_wheel_name();
const std::string kRight = CompleteGroundContactWire::right_wheel_name();

CompleteGroundContactFrame frame(uint64_t seq, int64_t stamp,
                                 std::vector<std::pair<std::string, std::string>> pairs)
{
    CompleteGroundContactFrame out;
    out.sequence = seq;
    out.sim_time_ns = stamp;
    out.iteration = seq + 1;
    out.dt_ns = CompleteGroundContactWire::kExpectedDtNs;
    out.frame_valid = true;
    out.num_contacts = static_cast<int64_t>(pairs.size());
    out.pairs = std::move(pairs);
    out.decoded = true;
    out.collision_pairs_allowed = true;
    for (const auto &p : out.pairs)
    {
        const auto &robot = p.first == kGround ? p.second : p.first;
        if (robot == kLeft) out.wheel_mask |= 1;
        else if (robot == kRight) out.wheel_mask |= 2;
        else out.collision_pairs_allowed = false;
    }
    return out;
}

CompleteGroundContactFrame bilateral(uint64_t seq, int64_t stamp)
{
    return frame(seq, stamp, {{kGround, kLeft}, {kGround, kRight}});
}

CompleteGroundContactFrame unilateral(uint64_t seq, int64_t stamp)
{
    return frame(seq, stamp, {{kGround, kLeft}});
}

CompleteGroundContactFrame zero(uint64_t seq, int64_t stamp)
{
    return frame(seq, stamp, {});
}

CompleteTakeoffComEvidence com_at(int64_t stamp)
{
    return {true, stamp, 1.0};
}

bool candidate(const CompleteContactTakeoffObserver &observer, int64_t now,
               const CompleteTakeoffComEvidence &com)
{
    return observer.confirmed(now, observer.gate_stamp_ns(), true, true, false,
                              true, true, com);
}

void feed_ready_prefix(CompleteContactTakeoffObserver &observer, uint64_t &seq,
                       int64_t &stamp)
{
    observer.receive(bilateral(seq++, stamp)); stamp += 1'000'000;
    observer.receive(bilateral(seq++, stamp)); stamp += 1'000'000;
    observer.receive(unilateral(seq++, stamp)); stamp += 1'000'000;
}
}  // namespace

int main()
{
    // Hand-authored production wire fixture: pair names are separately byte-length-prefixed.
    const std::string exact =
        "BBOT_GCF1|7|1000000|8|1000000|1|2|2|46:flat_jump_world::ground_plane::link::collision"
        "61:flat_jump_world::bbot::link_004::link_004_collision_collision|"
        "46:flat_jump_world::ground_plane::link::collision"
        "61:flat_jump_world::bbot::link_007::link_007_collision_collision|0:";
    CompleteGroundContactFrame parsed;
    assert(CompleteGroundContactWire::parse(exact, parsed));
    assert(parsed.decoded && parsed.frame_valid && parsed.num_contacts == 2);
    assert(parsed.collision_pairs_allowed && parsed.wheel_mask == 3);
    assert(parsed.sequence == 7 && parsed.sim_time_ns == 1'000'000);

    const auto encoded = CompleteGroundContactWire::encode(7, 1'000'000, 8,
        1'000'000, true, 2, {{kGround, kLeft}, {kGround, kRight}});
    assert(encoded == exact);
    const std::string exact_zero = "BBOT_GCF1|8|2000000|9|1000000|1|0|0|0:";
    assert(CompleteGroundContactWire::encode(8, 2'000'000, 9, 1'000'000,
        true, 0, {}) == exact_zero);
    assert(CompleteGroundContactWire::parse(exact_zero, parsed));
    assert(parsed.wheel_mask == 0 && parsed.pairs.empty());

    // Strict parser rejects mismatched counts, unknown collisions, malformed lengths and tails.
    auto bad_count = exact;
    bad_count.replace(bad_count.find("|1|2|2|"), 7, "|1|1|2|");
    assert(!CompleteGroundContactWire::parse(bad_count, parsed));
    auto bad_tail = exact + "x";
    assert(!CompleteGroundContactWire::parse(bad_tail, parsed));
    auto unknown = CompleteGroundContactWire::encode(1, 1'000'000, 2, 1'000'000,
        true, 1, {{kGround, "flat_jump_world::bbot::base::collision"}});
    assert(CompleteGroundContactWire::parse(unknown, parsed));
    assert(!parsed.collision_pairs_allowed);
    auto invalid_wire = CompleteGroundContactWire::encode(1, 1'000'000, 2,
        1'000'000, false, -1, {}, "ground_component_missing");
    assert(CompleteGroundContactWire::parse(invalid_wire, parsed));
    assert(!parsed.frame_valid && !parsed.collision_pairs_allowed);

    // Two post-gate bilateral frames latch support; a final unilateral frame is allowed.
    CompleteContactTakeoffObserver observer;
    observer.reset_for_jump(1, 1'000'000);
    observer.open_gate(2'000'000);
    uint64_t seq = 0;
    int64_t stamp = 2'000'000;
    feed_ready_prefix(observer, seq, stamp);
    for (int i = 0; i < 10; ++i)
    {
        observer.receive(zero(seq++, stamp));
        assert(!candidate(observer, stamp, com_at(stamp)));
        stamp += 1'000'000;
    }
    observer.receive(zero(seq++, stamp));
    assert(candidate(observer, stamp, com_at(stamp)));
    assert(observer.zero_frame_count() == 11);
    assert(observer.zero_span_ns() == 10'000'000);

    // Fresh bilateral support cannot be fabricated by starting from zero frames.
    CompleteContactTakeoffObserver no_support;
    no_support.reset_for_jump(2, 1'000'000);
    no_support.open_gate(2'000'000);
    for (int i = 0; i < 20; ++i)
    {
        no_support.receive(zero(i, 2'000'000 + i * 1'000'000));
    }
    assert(!candidate(no_support, 21'000'000, com_at(21'000'000)));

    // A unilateral transition clears the zero run but retains already established dual support.
    CompleteContactTakeoffObserver unilateral_transition;
    unilateral_transition.reset_for_jump(3, 1'000'000);
    unilateral_transition.open_gate(2'000'000);
    seq = 0; stamp = 2'000'000;
    unilateral_transition.receive(bilateral(seq++, stamp)); stamp += 1'000'000;
    unilateral_transition.receive(bilateral(seq++, stamp)); stamp += 1'000'000;
    unilateral_transition.receive(unilateral(seq++, stamp)); stamp += 1'000'000;
    unilateral_transition.receive(zero(seq++, stamp));
    assert(unilateral_transition.zero_frame_count() == 1);
    stamp += 1'000'000;
    unilateral_transition.receive(zero(seq++, stamp));
    assert(unilateral_transition.zero_frame_count() == 2);

    // Duplicate, gap, rollback, invalid frame and unknown pair break accumulated evidence.
    auto exercise_break = [](auto corrupt) {
        CompleteContactTakeoffObserver o;
        o.reset_for_jump(4, 1'000'000); o.open_gate(2'000'000);
        uint64_t s = 0; int64_t t = 2'000'000;
        feed_ready_prefix(o, s, t);
        for (int i = 0; i < 5; ++i) { o.receive(zero(s++, t)); t += 1'000'000; }
        const auto injected = corrupt(s, t);
        o.receive(injected);
        assert(o.zero_frame_count() == 0);
    };
    exercise_break([](uint64_t s, int64_t t) { return zero(s - 1, t - 1'000'000); });
    exercise_break([](uint64_t s, int64_t t) { return zero(s + 1, t); });
    exercise_break([](uint64_t s, int64_t t) { return zero(s, t - 2'000'000); });
    exercise_break([](uint64_t s, int64_t t) {
        auto f = zero(s, t); f.frame_valid = false; f.error = "missing"; return f;
    });
    exercise_break([](uint64_t s, int64_t t) {
        auto f = frame(s, t, {{kGround, "flat_jump_world::bbot::base::collision"}});
        f.collision_pairs_allowed = false; return f;
    });

    // Freshness and controller state gates fail closed; jump/gate reset cannot reuse history.
    CompleteContactTakeoffObserver resetProbe;
    resetProbe.reset_for_jump(5, stamp);
    resetProbe.open_gate(stamp);
    seq = 0;
    for (int i = 0; i < 11; ++i) resetProbe.receive(zero(seq++, stamp + i * 1'000'000));
    assert(!resetProbe.confirmed(stamp + 10'000'000, resetProbe.gate_stamp_ns(), true,
        true, false, true, true, com_at(stamp + 10'000'000)));

    CompleteContactTakeoffObserver guarded;
    guarded.reset_for_jump(6, 1'000'000); guarded.open_gate(2'000'000);
    seq = 0; stamp = 2'000'000; feed_ready_prefix(guarded, seq, stamp);
    for (int i = 0; i < 11; ++i) { guarded.receive(zero(seq++, stamp)); stamp += 1'000'000; }
    const auto now = stamp - 1'000'000;
    const auto goodCom = com_at(now);
    assert(candidate(guarded, now, goodCom));
    auto staleSource = guarded;
    const int64_t latestContact = staleSource.latest_stamp_ns();
    assert(staleSource.source_fresh(latestContact));
    assert(!staleSource.refresh_source(latestContact + 21'000'000));
    assert(staleSource.zero_frame_count() == 0);
    assert(!staleSource.refresh_source(latestContact));  // clock catch-up cannot revive stale evidence
    auto futureSource = guarded;
    assert(!futureSource.refresh_source(futureSource.latest_stamp_ns() - 2'000'000));
    assert(futureSource.zero_frame_count() == 0);
    auto nextJump = guarded;
    nextJump.reset_for_jump(7, now + 1'000'000);
    nextJump.open_gate(now + 2'000'000);
    assert(nextJump.zero_frame_count() == 0);
    assert(!candidate(nextJump, now + 2'000'000, goodCom));
    assert(!guarded.confirmed(now, guarded.gate_stamp_ns(), true, false, false,
        true, true, goodCom));
    assert(!guarded.confirmed(now, guarded.gate_stamp_ns(), true, true, true,
        true, true, goodCom));
    assert(!guarded.confirmed(now, guarded.gate_stamp_ns(), true, true, false,
        false, true, goodCom));
    assert(!guarded.confirmed(now, guarded.gate_stamp_ns(), true, true, false,
        true, false, goodCom));
    auto staleCom = goodCom; staleCom.stamp_ns -= 50'000'000;
    assert(!candidate(observer, now, staleCom));
    auto futureCom = goodCom; futureCom.stamp_ns = now + 30'000'000;
    assert(!candidate(observer, now, futureCom));
    auto falling = goodCom; falling.vertical_velocity = -0.1;
    assert(!candidate(observer, now, falling));
    guarded.mark_taken(guarded.latest_stamp_ns(), goodCom.stamp_ns);
    assert(!candidate(guarded, now, goodCom));

    std::cout << "complete contact takeoff tests passed\n";
    return 0;
}
