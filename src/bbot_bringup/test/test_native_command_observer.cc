// ECS fixture: prove observation preserves command presence and values.
#include <cassert>
#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <sstream>
#include <vector>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/World.hh>
#include "../src/landing_repair_wrench_recorder.cc"
int main() {
  namespace sim=ignition::gazebo;
  namespace c=ignition::gazebo::components;
  const auto absent=bbot::ObserveSingleFiniteCommand(std::nullopt);
  assert(!absent.component_present && !absent.valid && absent.vector_size==-1 && std::isnan(absent.value));
  const auto empty=bbot::ObserveSingleFiniteCommand(std::vector<double>{});
  assert(empty.component_present && !empty.valid && empty.vector_size==0 && std::isnan(empty.value));
  const auto nonfinite=bbot::ObserveSingleFiniteCommand(std::vector<double>{std::numeric_limits<double>::quiet_NaN()});
  assert(nonfinite.component_present && !nonfinite.valid && nonfinite.vector_size==1 && std::isnan(nonfinite.value));
  const auto multidimensional=bbot::ObserveSingleFiniteCommand(std::vector<double>{1.,2.});
  assert(multidimensional.component_present && !multidimensional.valid && multidimensional.vector_size==2 && std::isnan(multidimensional.value));
  const auto finiteZero=bbot::ObserveSingleFiniteCommand(std::vector<double>{0.});
  assert(finiteZero.component_present && finiteZero.valid && finiteZero.vector_size==1 && finiteZero.value==0.);
  sim::EntityComponentManager ecm;
  auto world=ecm.CreateEntity(); ecm.CreateComponent(world,c::Name("flat_jump_world")); ecm.CreateComponent(world,c::World());
  auto model=ecm.CreateEntity(); ecm.CreateComponent(model,c::Name("bbot")); ecm.CreateComponent(model,c::Model()); ecm.CreateComponent(model,c::ParentEntity(world));
  auto base=ecm.CreateEntity(); ecm.CreateComponent(base,c::Name("base_link")); ecm.CreateComponent(base,c::Link()); ecm.CreateComponent(base,c::ParentEntity(model));
  const std::array<const char*,6> childNames{{"link_002","link_003","link_004","link_005","link_006","link_007"}};
  for(const auto *name:childNames) { auto link=ecm.CreateEntity(); ecm.CreateComponent(link,c::Name(name)); ecm.CreateComponent(link,c::Link()); ecm.CreateComponent(link,c::ParentEntity(model)); }
  std::array<sim::Entity,6> joints;
  std::array<std::vector<double>,5> fixture{{{}, {std::numeric_limits<double>::quiet_NaN()}, {1.,2.}, {0.}, {4.5}}};
  std::array<std::vector<double>,5> velocityFixture{{{}, {std::numeric_limits<double>::quiet_NaN()}, {1.,2.}, {0.}, {2.25}}};
  for(int i=0;i<6;++i) {
    auto e=ecm.CreateEntity();joints[i]=e;
    ecm.CreateComponent(e,c::Name("link_00"+std::to_string(i+2)+"_joint"));
    ecm.CreateComponent(e,c::Joint());ecm.CreateComponent(e,c::ParentEntity(model));
    ecm.CreateComponent(e,c::JointPosition({.1}));ecm.CreateComponent(e,c::JointVelocity({.2}));
    // Test setup owns these commands. The observer must only read them.
    if(i>0) ecm.CreateComponent(e,c::JointForceCmd(fixture[i-1]));
    if(i>0) ecm.CreateComponent(e,c::JointVelocityCmd(velocityFixture[i-1]));
  }
  const std::string path="/tmp/bbot_observer_ecs_"+std::to_string(::getpid())+".csv";
  setenv("BBOT_NATIVE_WRENCH_CSV",path.c_str(),1);
  {
    bbot::LandingRepairWrenchRecorder recorder;
    sim::EventManager events;
    recorder.Configure(world,{},ecm,events);
    sim::UpdateInfo info; info.paused=false;info.iterations=1;
    info.simTime=std::chrono::milliseconds(1);info.dt=std::chrono::milliseconds(1);
    recorder.PreUpdate(info,ecm);recorder.Update(info,ecm);recorder.PostUpdate(info,ecm);
    assert(!ecm.Component<c::JointForceCmd>(joints[0]));
    for(int i=1;i<6;++i) {
      auto value=ecm.ComponentData<c::JointForceCmd>(joints[i]);assert(value);
      const auto &expected=fixture[i-1];assert(value->size()==expected.size());
      for(size_t k=0;k<expected.size();++k) assert((std::isnan(expected[k])&&std::isnan((*value)[k])) || expected[k]==(*value)[k]);
    }
    assert(!ecm.Component<c::JointVelocityCmd>(joints[0]));
    for(int i=1;i<6;++i) {
      auto value=ecm.ComponentData<c::JointVelocityCmd>(joints[i]);assert(value);
      const auto &expected=velocityFixture[i-1];assert(value->size()==expected.size());
      for(size_t k=0;k<expected.size();++k)
        assert((std::isnan(expected[k])&&std::isnan((*value)[k])) || expected[k]==(*value)[k]);
    }
  }
  std::ifstream f(path);std::string line;std::getline(f,line);
  const auto split=[](const std::string &s){std::vector<std::string> v;std::stringstream ss(s);std::string x;while(std::getline(ss,x,','))v.push_back(x);return v;};
  const auto header=split(line);
  const auto col=[&](const std::string &name){auto it=std::find(header.begin(),header.end(),name);assert(it!=header.end());return it-header.begin();};
  assert(col("pre_joint_velocity_cmd_component_present")>col("post_base_world_wz"));
  assert(col("before_physics_joint_velocity_cmd_component_present")>
         col("pre_joint_velocity_cmd_value"));
  assert(col("post_joint_velocity_cmd_component_present")>
         col("before_physics_joint_velocity_cmd_value"));
  for(const auto &prefix:{std::string("pre_"),std::string("before_physics_"),std::string("post_")}) {
    assert(col(prefix+"joint_velocity_cmd_component_present")<col(prefix+"joint_velocity_cmd_valid"));
    assert(col(prefix+"joint_velocity_cmd_valid")<col(prefix+"joint_velocity_cmd_vector_size"));
    assert(col(prefix+"joint_velocity_cmd_vector_size")<col(prefix+"joint_velocity_cmd_value"));
  }
  int i=0;
  while(std::getline(f,line)) {
    const auto r=split(line);assert(r.size()==header.size());
    const bool valid=i>=4;
    for(const auto &prefix:{std::string(""),std::string("before_physics_"),std::string("post_")}) {
      const auto flag=col(prefix+"joint_force_cmd_valid");
      assert(r[flag]==(valid?"1":"0"));
      const double val=std::stod(r[col(prefix+"joint_force_cmd_sim_input")]);
      if(valid)assert(val==(i==4?0.:4.5));else assert(std::isnan(val));
    }
    const std::array<int64_t,6> expectedSize{{-1,0,1,2,1,1}};
    const bool velocityPresent=i!=0;
    const bool velocityValid=i>=4;
    const double expectedVelocity=(i==4)?0.:(i==5)?2.25:std::numeric_limits<double>::quiet_NaN();
    for(const auto &prefix:{std::string("pre_"),std::string("before_physics_"),std::string("post_")}) {
      assert(r[col(prefix+"joint_velocity_cmd_component_present")]==(velocityPresent?"1":"0"));
      assert(r[col(prefix+"joint_velocity_cmd_valid")] == (velocityValid?"1":"0"));
      assert(std::stoll(r[col(prefix+"joint_velocity_cmd_vector_size")])==expectedSize[i]);
      const double value=std::stod(r[col(prefix+"joint_velocity_cmd_value")]);
      if(velocityValid) assert(value==expectedVelocity); else assert(std::isnan(value));
    }
    ++i;
  }
  assert(i==6);std::remove(path.c_str());unsetenv("BBOT_NATIVE_WRENCH_CSV");
  std::cout<<"PASS: absent/empty/nonfinite/wrong-size remain invalid; finite zero valid; force/velocity commands unchanged across Pre/BeforePhysics/Post\n";
}
