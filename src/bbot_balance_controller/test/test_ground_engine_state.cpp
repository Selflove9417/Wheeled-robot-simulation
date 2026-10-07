#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iomanip>
#include <sstream>
#include "bbot_balance_controller/ground_engine_state.hpp"

std::string packet(const char *magic = "ECS1", long dt = 1000000,
                   unsigned mask = 511, const char *value = "0.125") {
  std::ostringstream out;
  out << magic << ",5000000,5," << dt << ',' << mask << ",511,511,63";
  for (int i = 0; i < 33; ++i) out << ',' << value;
  return out.str();
}

int main() {
  bbot_jump::NativeCommandState state;
  assert(bbot_jump::GroundEngineStateWire::parse(packet(), state));
  assert(state.valid && state.stamp_ns == 5000000 && state.q[3] == .125);
  for (const auto &bad : {packet("NCS1"), packet("ECS1", 2000000),
                         packet("ECS1", 1000000, 510),
                         packet("ECS1", 1000000, 511, "nan"),
                         packet("ECS1", 1000000, 511, "inf"),
                         packet()+",0"}) {
    assert(!bbot_jump::GroundEngineStateWire::parse(bad, state));
    assert(!state.valid);
  }
  assert(bbot_jump::GroundEngineStateWire::parse(packet(), state));
  bbot_jump::NativeCommandStateHistory history;
  assert(history.push(state));
  assert(!history.snapshot(4999999, state));
  assert(history.snapshot(15000000, state));
  assert(!history.snapshot(15000001, state));
  assert(bbot_jump::GroundEngineStateWire::parse(packet(), state));
  assert(!history.push(state));  // duplicate step clears the history
  assert(history.size() == 0);
  assert(bbot_jump::GroundEngineStateWire::parse(packet(), state));
  assert(history.push(state));
  state.stamp_ns -= 1000000; --state.iteration;
  assert(!history.push(state));
  assert(!history.snapshot(5000000, state));
  bbot_jump::GroundEngineStateHistory engine;
  assert(bbot_jump::GroundEngineStateWire::parse(packet(), state));
  assert(engine.push(state, 100000000));
  assert(engine.snapshot(5000000, 110000000, state));
  assert(!engine.snapshot(5000000, 110000001, state)); // sim clock stayed still
  assert(!state.valid);
  assert(!engine.snapshot(5000000, 99999999, state)); // steady clock rollback
  assert(bbot_jump::GroundEngineStateWire::parse(packet(), state));
  ++state.iteration; state.stamp_ns += 1000000;
  assert(!engine.push(state, 99999999));
  assert(engine.size() == 0);
  assert(bbot_jump::GroundEngineStateWire::parse(packet(), state));
  assert(engine.push(state, 100000000));
  ++state.iteration; state.stamp_ns += 1000000;
  assert(engine.push(state, 109000000));
  // A newer future packet's fresh receipt must not refresh the older step.
  assert(!engine.snapshot(5000000, 110000001, state));
  assert(engine.snapshot(6000000, 110000001, state));
  assert(state.iteration == 6);

  // First complete frame establishes an arbitrary origin; every later frame
  // must be the immediately adjacent physics step or the entire history clears.
  bbot_jump::GroundEngineStateHistory contiguous;
  assert(bbot_jump::GroundEngineStateWire::parse(packet(), state));
  assert(contiguous.push(state, 200000000));
  state.iteration = 6; state.stamp_ns = 6000000;
  assert(contiguous.push(state, 201000000));
  state.iteration = 8; state.stamp_ns = 8000000;
  assert(!contiguous.push(state, 202000000)); // a missing physics frame
  assert(contiguous.size() == 0);
  state.iteration = 8; state.stamp_ns = 8000000;
  assert(contiguous.push(state, 203000000)); // can establish a new origin
  state.iteration = 9; state.stamp_ns = 10000000;
  assert(!contiguous.push(state, 204000000)); // stamp gap despite adjacent iteration
  assert(contiguous.size() == 0);
  state.iteration = 9; state.stamp_ns = 9000000;
  assert(contiguous.push(state, 205000000));
  assert(!contiguous.push(state, 206000000)); // duplicate clears history
  assert(contiguous.size() == 0);

  // Contact is checked at callback/physics cadence. A one-frame loss followed
  // by recovery remains latched after experiment takeover.
  bbot_jump::GroundContactContinuityGuard contact;
  assert(contact.observe(100, 100000000, 400, 1000000, true, true, true));
  assert(!contact.observe(101, 101000000, 401, 1000000, true, false, true));
  assert(contact.failed());
  assert(contact.reason() == "contact_not_bilateral");
  assert(!contact.observe(102, 102000000, 402, 1000000, true, true, true));
  assert(contact.failed()); // recovery does not clear the fault latch

  bbot_jump::GroundContactContinuityGuard source_fault;
  assert(source_fault.observe(10, 1000000, 20, 1000000, true, true, true));
  assert(!source_fault.observe(11, 3000000, 22, 1000000, true, true, true));
  assert(source_fault.reason() == "contact_clock_gap_or_rollback");
  bbot_jump::GroundContactContinuityGuard invalid_contact;
  assert(invalid_contact.observe(10, 1000000, 20, 1000000, true, true, true));
  assert(!invalid_contact.observe(11, 2000000, 21, 1000000, false, true, true));
  assert(invalid_contact.reason() == "contact_source_invalid");

  // Before takeover, a non-bilateral frame only resets the baseline. It does
  // not pre-fail a trial that is still waiting for support.
  bbot_jump::GroundContactContinuityGuard waiting_contact;
  assert(waiting_contact.observe(1, 1000000, 1, 1000000, true, true, false));
  assert(waiting_contact.observe(2, 2000000, 2, 1000000, true, false, false));
  assert(!waiting_contact.failed());
  assert(waiting_contact.observe(3, 3000000, 3, 1000000, true, true, true));
  assert(!waiting_contact.failed());
}
