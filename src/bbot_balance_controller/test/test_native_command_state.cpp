#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include "bbot_balance_controller/native_command_state.hpp"

using namespace bbot_jump;

static std::string Wire(int64_t stamp=100000000, uint64_t iteration=10,
                        int64_t dt=1000000, unsigned qmask=511,
                        unsigned vmask=511, unsigned before=511, unsigned cmd=63) {
  std::ostringstream s;
  s<<"NCS1,"<<stamp<<','<<iteration<<','<<dt<<','<<qmask<<','<<vmask<<','<<before<<','<<cmd;
  for(int i=0;i<9;++i)s<<','<<(i+0.25);
  for(int i=0;i<9;++i)s<<','<<(i+10.25);
  for(int i=0;i<9;++i)s<<','<<(i+20.25);
  for(int i=0;i<6;++i)s<<','<<(i==0?0.0:i+30.25);
  return s.str();
}

int main() {
  NativeCommandState state;
  assert(NativeCommandStateWire::parse(Wire(),state));
  assert(state.valid && state.stamp_ns==100000000 && state.iteration==10 && state.dt_ns==1000000);
  assert(state.q[0]==.25 && state.q[8]==8.25 && state.v[0]==10.25 &&
         state.before_v[8]==28.25 && state.simulation_input[0]==0.0 &&
         state.simulation_input[5]==35.25);
  for(unsigned badMask:{0u,510u}) assert(!NativeCommandStateWire::parse(Wire(100000000,10,1000000,badMask),state));
  assert(!NativeCommandStateWire::parse(Wire(100000000,10,1000000,511,510),state));
  assert(!NativeCommandStateWire::parse(Wire(100000000,10,1000000,511,511,510),state));
  assert(!NativeCommandStateWire::parse(Wire(100000000,10,1000000,511,511,511,62),state));
  assert(!NativeCommandStateWire::parse(Wire(-1),state));
  assert(!NativeCommandStateWire::parse(Wire(100000000,0),state));
  assert(!NativeCommandStateWire::parse(Wire(100000000,10,0),state));
  auto nonfinite=Wire(); const auto comma=nonfinite.rfind(','); nonfinite.replace(comma+1,std::string::npos,"nan");
  assert(!NativeCommandStateWire::parse(nonfinite,state));
  assert(!NativeCommandStateWire::parse(Wire()+",extra",state));

  NativeCommandStateHistory history;
  assert(NativeCommandStateWire::parse(Wire(100000000,10),state)); assert(history.push(state));
  assert(NativeCommandStateWire::parse(Wire(105000000,11),state)); assert(history.push(state));
  NativeCommandState picked;
  assert(history.snapshot(104000000,picked) && picked.stamp_ns==100000000);
  assert(history.snapshot(115000000,picked) && picked.stamp_ns==105000000);
  assert(!history.snapshot(115000001,picked)); // 10 ms + 1 ns is stale.
  assert(!history.snapshot(104999999,picked) || picked.stamp_ns==100000000);
  assert(NativeCommandStateWire::parse(Wire(104000000,12),state));
  assert(!history.push(state) && history.size()==0); // out-of-order time clears history.
  assert(NativeCommandStateWire::parse(Wire(110000000,13),state)); assert(history.push(state));
  state.valid=false; assert(!history.push(state) && history.size()==0);
  std::cout<<"PASS: NCS1 ordering, flags, finite values, timestamp checks, and <=10ms history selection\n";
}
