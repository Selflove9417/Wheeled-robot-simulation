// Opt-in native physics observation for landing repair. No command publisher,
// force/velocity setter, controller parameter changes or contact modifications.
// PreUpdate requests observation components only; PostUpdate records physics.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>

namespace bbot {
struct CommandComponentObservation {
  bool component_present{false};
  bool valid{false};
  int64_t vector_size{-1};
  double value{std::numeric_limits<double>::quiet_NaN()};
};

inline CommandComponentObservation ObserveSingleFiniteCommand(
    const std::optional<std::vector<double>> &component) {
  CommandComponentObservation observation;
  observation.component_present = component.has_value();
  if (!component) return observation;
  observation.vector_size = static_cast<int64_t>(component->size());
  observation.valid = component->size() == 1 && std::isfinite(component->front());
  if (observation.valid) observation.value = component->front();
  return observation;
}

class LandingRepairWrenchRecorder final : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate,
    public ignition::gazebo::ISystemUpdate,
    public ignition::gazebo::ISystemPostUpdate {
 public:
  void Configure(const ignition::gazebo::Entity &,
      const std::shared_ptr<const sdf::Element> &,
      ignition::gazebo::EntityComponentManager &,
      ignition::gazebo::EventManager &) override {
    const char *topic=std::getenv("BBOT_NATIVE_COMMAND_STATE_TOPIC");
    if (topic && *topic) {
      stateTopic=topic;
      node=std::make_unique<ignition::transport::Node>();
      statePublisher=node->Advertise<ignition::msgs::StringMsg>(stateTopic);
      if (!statePublisher) { Fail("cannot advertise native command state topic"); return; }
    }
    const char *path=std::getenv("BBOT_NATIVE_WRENCH_CSV");
    if (path && *path) {
    csvEnabled=true;
    outputPath=path;
    output.open(outputPath,std::ios::out|std::ios::trunc);
    if (!output.is_open()) { Fail("cannot open native wrench CSV"); return; }
    output << "seq,sim_time_ns,physics_iteration,dt_ns,joint_index,joint_name,"
        "joint_valid,state_valid,joint_position,joint_velocity,axis_valid,axis_x,axis_y,axis_z,"
        "wrench_valid,joint_frame_fx,joint_frame_fy,joint_frame_fz,"
        "joint_frame_tx,joint_frame_ty,joint_frame_tz,transmitted_axis_torque,"
        "force_component_valid,joint_force_component,"
        "child_pose_valid,child_x,child_y,child_z,child_qx,child_qy,child_qz,child_qw,"
        "parent_pose_valid,parent_qx,parent_qy,parent_qz,parent_qw,"
        "child_velocity_valid,child_world_vx,child_world_vy,child_world_vz,"
        "child_world_wx,child_world_wy,child_world_wz,"
        "child_acceleration_valid,child_world_ax,child_world_ay,child_world_az,"
        "child_world_alpha_x,child_world_alpha_y,child_world_alpha_z,"
        "contact_frame_valid,num_contacts,contact_wrench_count,record_phase,"
        "pre_phase,pre_sim_time_ns,pre_physics_iteration,pre_dt_ns,"
        "pre_joint_state_valid,pre_joint_position,pre_joint_velocity,"
        "pre_base_velocity_valid,pre_base_world_vx,pre_base_world_vy,pre_base_world_vz,"
        "pre_base_world_wx,pre_base_world_wy,pre_base_world_wz,"
        "joint_force_cmd_component_present,joint_force_cmd_valid,joint_force_cmd_sim_input,";
    // Columns used for a snapshot in the Update phase, after every system's
    // PreUpdate and immediately before the physics system when ordered first.
    // Update ordering is controlled by the world plugin order and still must
    // be verified in the actual run.
    output << "before_physics_phase,before_physics_sim_time_ns,before_physics_iteration,"
        "before_physics_dt_ns,before_physics_joint_state_valid,before_physics_joint_position,"
        "before_physics_joint_velocity,before_physics_base_pose_valid,before_physics_base_x,"
        "before_physics_base_y,before_physics_base_z,before_physics_base_qx,"
        "before_physics_base_qy,before_physics_base_qz,before_physics_base_qw,"
        "before_physics_base_velocity_valid,before_physics_base_world_vx,"
        "before_physics_base_world_vy,before_physics_base_world_vz,"
        "before_physics_base_world_wx,before_physics_base_world_wy,"
        "before_physics_base_world_wz,before_physics_joint_force_cmd_component_present,"
        "before_physics_joint_force_cmd_valid,before_physics_joint_force_cmd_sim_input,"
        "post_joint_force_cmd_component_present,post_joint_force_cmd_valid,post_joint_force_cmd_sim_input,"
        "post_base_velocity_valid,post_base_world_vx,post_base_world_vy,post_base_world_vz,"
        "post_base_world_wx,post_base_world_wy,post_base_world_wz,"
        "pre_joint_velocity_cmd_component_present,pre_joint_velocity_cmd_valid,"
        "pre_joint_velocity_cmd_vector_size,pre_joint_velocity_cmd_value,"
        "before_physics_joint_velocity_cmd_component_present,before_physics_joint_velocity_cmd_valid,"
        "before_physics_joint_velocity_cmd_vector_size,before_physics_joint_velocity_cmd_value,"
        "post_joint_velocity_cmd_component_present,post_joint_velocity_cmd_valid,"
        "post_joint_velocity_cmd_vector_size,post_joint_velocity_cmd_value\n";
    output.flush(); enabled=output.good();
    if (!enabled) Fail("cannot write native wrench header");
    }
    observerEnabled=enabled || static_cast<bool>(statePublisher);
  }
  void Update(const ignition::gazebo::UpdateInfo &info,
      ignition::gazebo::EntityComponentManager &ecm) override {
    if (!observerEnabled || (fatal && !statePublisher)) return;
    updateNs=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    updateDt=std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count();
    updateIteration=info.iterations;
    for (size_t i=0;i<6;++i) {
      updateVelocityCommand[i] = CommandComponentObservation{};
      updateStateValid[i]=updateCommandPresent[i]=updateCommandValid[i]=false;
      updateQ[i]=updateV[i]=updateCommand[i]=NaN();
      if (joints[i]==ignition::gazebo::kNullEntity) continue;
      const auto q=ecm.ComponentData<ignition::gazebo::components::JointPosition>(joints[i]);
      const auto v=ecm.ComponentData<ignition::gazebo::components::JointVelocity>(joints[i]);
      updateStateValid[i]=q && v && q->size()==1 && v->size()==1 &&
          std::isfinite(q->front()) && std::isfinite(v->front());
      if(updateStateValid[i]) { updateQ[i]=q->front(); updateV[i]=v->front(); }
      const auto cmdData=ecm.ComponentData<ignition::gazebo::components::JointForceCmd>(joints[i]);
      updateCommandPresent[i]=cmdData.has_value();
      updateCommandValid[i]=cmdData && cmdData->size()==1 && std::isfinite(cmdData->front());
      if(updateCommandValid[i]) updateCommand[i]=cmdData->front();
      updateVelocityCommand[i] = ObserveSingleFiniteCommand(
          ecm.ComponentData<ignition::gazebo::components::JointVelocityCmd>(joints[i]));
    }
    updateBasePoseValid=base!=ignition::gazebo::kNullEntity &&
        ecm.Component<ignition::gazebo::components::Pose>(base);
    if(updateBasePoseValid) {
      updateBasePose=ignition::gazebo::worldPose(base,ecm);
      updateBasePoseValid=updateBasePose.Pos().IsFinite() && updateBasePose.Rot().IsFinite();
    }
    std::optional<ignition::math::Vector3d> linear,angular;
    if(base!=ignition::gazebo::kNullEntity) {
      const ignition::gazebo::Link baseLink(base);
      linear=baseLink.WorldLinearVelocity(ecm);
      angular=baseLink.WorldAngularVelocity(ecm);
    }
    updateBaseVelocityValid=linear && angular && linear->IsFinite() && angular->IsFinite();
    updateBaseLinear=updateBaseVelocityValid?*linear:Missing();
    updateBaseAngular=updateBaseVelocityValid?*angular:Missing();
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
      ignition::gazebo::EntityComponentManager &ecm) override {
    if (!observerEnabled || (fatal && !statePublisher)) return;
    for (size_t i=0;i<6;++i) {
      if (joints[i]==ignition::gazebo::kNullEntity)
        joints[i]=Find(ecm,"flat_jump_world::bbot::"+std::string(jointNames[i]),
                       ignition::gazebo::components::Joint());
      if (children[i]==ignition::gazebo::kNullEntity)
        children[i]=Find(ecm,"flat_jump_world::bbot::"+std::string(childNames[i]),
                         ignition::gazebo::components::Link());
      if (parents[i]==ignition::gazebo::kNullEntity)
        parents[i]=Find(ecm,"flat_jump_world::bbot::"+std::string(parentNames[i]),
                        ignition::gazebo::components::Link());
      if (joints[i]!=ignition::gazebo::kNullEntity && !requested[i]) {
        // Empty protobuf is deliberately invalid until physics fills both
        // vectors. Unsupported engine observations cannot become valid zeros.
        if (!ecm.Component<ignition::gazebo::components::JointTransmittedWrench>(joints[i]))
          ecm.CreateComponent(joints[i],ignition::gazebo::components::JointTransmittedWrench());
        requested[i]=true;
      }
      if (children[i]!=ignition::gazebo::kNullEntity && !linkRequested[i]) {
        const ignition::gazebo::Link child(children[i]);
        child.EnableVelocityChecks(ecm);
        child.EnableAccelerationChecks(ecm);
        linkRequested[i]=true;
      }
    }
    if (base==ignition::gazebo::kNullEntity)
      base=Find(ecm,"flat_jump_world::bbot::base_link",ignition::gazebo::components::Link());
    if (base!=ignition::gazebo::kNullEntity) {
      const ignition::gazebo::Link baseLink(base);
      baseLink.EnableVelocityChecks(ecm);
      const auto linear=baseLink.WorldLinearVelocity(ecm);
      const auto angular=baseLink.WorldAngularVelocity(ecm);
      preBaseValid=linear && angular && linear->IsFinite() && angular->IsFinite();
      preBaseLinear=preBaseValid?*linear:Missing();
      preBaseAngular=preBaseValid?*angular:Missing();
    } else {
      preBaseValid=false; preBaseLinear=Missing(); preBaseAngular=Missing();
    }
    preNs=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    preDt=std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count();
    preIteration=info.iterations;
    for (size_t i=0;i<6;++i) {
      preVelocityCommand[i] = CommandComponentObservation{};
      preQValid[i]=preVValid[i]=false;
      preQ[i]=preV[i]=NaN();
      commandPresent[i]=commandValid[i]=false;
      command[i]=NaN();
      if (joints[i]==ignition::gazebo::kNullEntity) continue;
      const auto q=ecm.ComponentData<ignition::gazebo::components::JointPosition>(joints[i]);
      const auto v=ecm.ComponentData<ignition::gazebo::components::JointVelocity>(joints[i]);
      preQValid[i]=q && q->size()==1 && std::isfinite(q->front());
      preVValid[i]=v && v->size()==1 && std::isfinite(v->front());
      preQ[i]=preQValid[i]?q->front():NaN();
      preV[i]=preVValid[i]?v->front():NaN();
      const auto cmdData=ecm.ComponentData<ignition::gazebo::components::JointForceCmd>(joints[i]);
      commandPresent[i]=cmdData.has_value();
      commandValid[i]=cmdData && cmdData->size()==1 && std::isfinite(cmdData->front());
      command[i]=commandValid[i]?cmdData->front():NaN();
      preVelocityCommand[i] = ObserveSingleFiniteCommand(
          ecm.ComponentData<ignition::gazebo::components::JointVelocityCmd>(joints[i]));
    }
    if (ground==ignition::gazebo::kNullEntity)
      ground=Find(ecm,"flat_jump_world::ground_plane::link::collision",
                  ignition::gazebo::components::Collision());
  }
  void PostUpdate(const ignition::gazebo::UpdateInfo &info,
      const ignition::gazebo::EntityComponentManager &ecm) override {
    if (info.paused || !observerEnabled || (fatal && !statePublisher)) return;
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    const auto dt=std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count();
    bool contactValid=false; int contacts=-1,wrenches=-1;
    if (ground!=ignition::gazebo::kNullEntity) {
      const auto data=ecm.ComponentData<ignition::gazebo::components::ContactSensorData>(ground);
      if (data) {
        contactValid=true;contacts=data->contact_size();wrenches=0;
        for (const auto &c:data->contact()) wrenches+=c.wrench_size();
      }
    }
    if (enabled && !fatal) for (size_t i=0;i<6;++i) {
      output << std::setprecision(17) << seq << ',' << ns << ',' << info.iterations << ',' << dt << ','
             << i << ',' << jointNames[i] << ',' << (joints[i]!=ignition::gazebo::kNullEntity);
      const auto q=ecm.ComponentData<ignition::gazebo::components::JointPosition>(joints[i]);
      const auto v=ecm.ComponentData<ignition::gazebo::components::JointVelocity>(joints[i]);
      const bool state=q && v && q->size()==1 && v->size()==1 && std::isfinite(q->front()) && std::isfinite(v->front());
      output << ',' << state << ',' << (state?q->front():NaN()) << ',' << (state?v->front():NaN());
      const auto axis=ecm.ComponentData<ignition::gazebo::components::JointAxis>(joints[i]);
      const bool axisValid=axis && axis->XyzExpressedIn().empty() && std::abs(axis->Xyz().Length()-1.)<1e-6;
      output << ',' << axisValid;
      WriteVector(axisValid?axis->Xyz():ignition::math::Vector3d(NaN(),NaN(),NaN()));
      const auto w=ecm.ComponentData<ignition::gazebo::components::JointTransmittedWrench>(joints[i]);
      const bool wrench=w && w->has_force() && w->has_torque() &&
          Finite(w->force()) && Finite(w->torque());
      output << ',' << wrench;
      const ignition::math::Vector3d force=wrench?Vector(w->force()):Missing();
      const ignition::math::Vector3d torque=wrench?Vector(w->torque()):Missing();
      WriteVector(force);WriteVector(torque);
      output << ',' << (wrench && axisValid?torque.Dot(axis->Xyz()):NaN());
      const auto jf=ecm.ComponentData<ignition::gazebo::components::JointForce>(joints[i]);
      const bool jfValid=jf && jf->size()==1 && std::isfinite(jf->front());
      output << ',' << jfValid << ',' << (jfValid?jf->front():NaN());
      const bool childPose=children[i]!=ignition::gazebo::kNullEntity &&
          ecm.Component<ignition::gazebo::components::Pose>(children[i]);
      output << ',' << childPose;
      if (childPose) {
        const auto p=ignition::gazebo::worldPose(children[i],ecm);
        WriteVector(p.Pos());WriteQuaternion(p.Rot());
      } else WriteMissing(7);
      const bool parentPose=parents[i]!=ignition::gazebo::kNullEntity &&
          ecm.Component<ignition::gazebo::components::Pose>(parents[i]);
      output << ',' << parentPose;
      if (parentPose) WriteQuaternion(ignition::gazebo::worldPose(parents[i],ecm).Rot());
      else WriteMissing(4);
      const ignition::gazebo::Link child(children[i]);
      const auto cv=child.WorldLinearVelocity(ecm),cw=child.WorldAngularVelocity(ecm);
      const bool velocity=cv && cw && cv->IsFinite() && cw->IsFinite();
      output << ',' << velocity;WriteVector(velocity?*cv:Missing());WriteVector(velocity?*cw:Missing());
      const auto ca=child.WorldLinearAcceleration(ecm),cal=child.WorldAngularAcceleration(ecm);
      const bool acceleration=ca && cal && ca->IsFinite() && cal->IsFinite();
      output << ',' << acceleration;WriteVector(acceleration?*ca:Missing());WriteVector(acceleration?*cal:Missing());
      output << ',' << contactValid << ',' << contacts << ',' << wrenches;
      // This is the command component observed in PreUpdate. System ordering
      // does not prove this observer ran after every possible command writer,
      // nor that Physics consumed the value. It is a simulation input, not a
      // measured actuator torque or proof of effective application.
      output << ",post_update,pre_update," << preNs << ',' << preIteration << ',' << preDt << ','
             << (preQValid[i]&&preVValid[i]) << ',' << preQ[i] << ',' << preV[i] << ','
             << preBaseValid;
      WriteVector(preBaseLinear); WriteVector(preBaseAngular);
      output << ',' << commandPresent[i] << ',' << commandValid[i] << ',' << command[i]
             << ",before_physics_update," << updateNs << ',' << updateIteration << ',' << updateDt << ','
             << updateStateValid[i] << ',' << updateQ[i] << ',' << updateV[i] << ',' << updateBasePoseValid;
      if(updateBasePoseValid) { WriteVector(updateBasePose.Pos()); WriteQuaternion(updateBasePose.Rot()); }
      else WriteMissing(7);
      output << ',' << updateBaseVelocityValid;
      WriteVector(updateBaseLinear); WriteVector(updateBaseAngular);
      output << ',' << updateCommandPresent[i] << ',' << updateCommandValid[i] << ',' << updateCommand[i];
      const auto postCmd=ecm.ComponentData<ignition::gazebo::components::JointForceCmd>(joints[i]);
      const bool postCmdValid=postCmd && postCmd->size()==1 && std::isfinite(postCmd->front());
      output << ',' << postCmd.has_value() << ',' << postCmdValid << ',' << (postCmdValid?postCmd->front():NaN());
      // Physics clears pending force components. A valid post zero is a
      // component observation after the step, never the motor input for it.
      const ignition::gazebo::Link body(base);
      const auto bv=body.WorldLinearVelocity(ecm),bw=body.WorldAngularVelocity(ecm);
      const bool bodyVelocity=bv && bw && bv->IsFinite() && bw->IsFinite();
      output << ',' << bodyVelocity;
      WriteVector(bodyVelocity?*bv:Missing()); WriteVector(bodyVelocity?*bw:Missing());
      WriteCommandObservation(preVelocityCommand[i]);
      WriteCommandObservation(updateVelocityCommand[i]);
      WriteCommandObservation(ObserveSingleFiniteCommand(
          ecm.ComponentData<ignition::gazebo::components::JointVelocityCmd>(joints[i])));
      output << '\n';
    }
    if (enabled && !fatal) { output.flush(); if (!output.good()) Fail("native wrench CSV write failed"); }
    if (statePublisher) PublishState(info,ecm);
    ++seq;
  }
 private:
  void WriteCommandObservation(const CommandComponentObservation &observation) {
    output << ',' << observation.component_present << ',' << observation.valid << ','
           << observation.vector_size << ',' << observation.value;
  }

  void PublishState(const ignition::gazebo::UpdateInfo &info,
      const ignition::gazebo::EntityComponentManager &ecm) {
    // Model order: [world-Y, world-Z, roll, hipL, kneeL, hipR, kneeR,
    // wheelL, wheelR]. Native joint indices map to model joints as below.
    constexpr std::array<size_t,6> nativeIndex{{0,1,3,4,2,5}};
    std::array<double,9> q,v,beforeV;
    std::array<double,6> input;
    q.fill(NaN()); v.fill(NaN()); beforeV.fill(NaN()); input.fill(NaN());
    uint16_t qMask=0,vMask=0,beforeMask=0;
    uint8_t inputMask=0;
    const auto stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    const auto dt=std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count();
    const bool sameStep=updateNs==stamp && updateDt==dt && updateIteration==info.iterations;
    bool postBasePoseValid=false,postBaseVelocityValid=false;
    ignition::math::Pose3d postPose;
    ignition::math::Vector3d postLinear=Missing(),postAngular=Missing();
    if(base!=ignition::gazebo::kNullEntity) {
      const ignition::gazebo::Link baseLink(base);
      postPose=ignition::gazebo::worldPose(base,ecm);
      postBasePoseValid=postPose.Pos().IsFinite() && postPose.Rot().IsFinite();
      const auto linear=baseLink.WorldLinearVelocity(ecm),angular=baseLink.WorldAngularVelocity(ecm);
      postBaseVelocityValid=linear && angular && linear->IsFinite() && angular->IsFinite();
      if(postBaseVelocityValid) { postLinear=*linear; postAngular=*angular; }
    }
    if(sameStep && updateBasePoseValid && postBasePoseValid) {
      q[0]=postPose.Pos().Y(); q[1]=postPose.Pos().Z(); q[2]=postPose.Rot().Roll();
      qMask|=0x7; // q carries PostUpdate state.
    }
    if(sameStep && updateBaseVelocityValid && postBaseVelocityValid) {
      v[0]=postLinear.Y(); v[1]=postLinear.Z(); v[2]=postAngular.X();
      beforeV[0]=updateBaseLinear.Y(); beforeV[1]=updateBaseLinear.Z();
      beforeV[2]=updateBaseAngular.X();
      vMask|=0x7; beforeMask|=0x7;
    }
    for(size_t i=0;i<6;++i) {
      const size_t modelIndex=3+i, native=nativeIndex[i];
      if(!sameStep || joints[native]==ignition::gazebo::kNullEntity) continue;
      const auto postQ=ecm.ComponentData<ignition::gazebo::components::JointPosition>(joints[native]);
      const auto postV=ecm.ComponentData<ignition::gazebo::components::JointVelocity>(joints[native]);
      if(postQ && postQ->size()==1 && std::isfinite(postQ->front())) {
        q[modelIndex]=postQ->front(); qMask|=uint16_t{1}<<modelIndex;
      }
      if(postV && postV->size()==1 && std::isfinite(postV->front())) {
        v[modelIndex]=postV->front(); vMask|=uint16_t{1}<<modelIndex;
      }
      if(updateStateValid[native]) {
        beforeV[modelIndex]=updateV[native]; beforeMask|=uint16_t{1}<<modelIndex;
      }
      const size_t cmdIndex=native;
      if(updateCommandValid[cmdIndex]) {
        input[i]=updateCommand[cmdIndex]; inputMask|=uint8_t{1}<<i;
      }
    }
    std::ostringstream wire; wire<<std::setprecision(17)<<"NCS1,"<<stamp<<','
      <<info.iterations<<','<<dt<<','<<qMask<<','<<vMask<<','<<beforeMask<<','
      <<static_cast<unsigned>(inputMask);
    for(double x:q) wire<<','<<x;
    for(double x:v) wire<<','<<x;
    for(double x:beforeV) wire<<','<<x;
    for(double x:input) wire<<','<<x;
    ignition::msgs::StringMsg msg; msg.set_data(wire.str());
    if(!statePublisher.Publish(msg)) {
      std::ofstream(outputPath+".topic.error",std::ios::app)<<"publish failed at sim_ns="<<stamp<<'\n';
    }
  }

  template<class T> ignition::gazebo::Entity Find(const ignition::gazebo::EntityComponentManager &ecm,
      const std::string &name,const T &component) const {
    ignition::gazebo::Entity result=ignition::gazebo::kNullEntity;
    for (auto e:ecm.EntitiesByComponents(component)) {
      if (ignition::gazebo::scopedName(e,ecm,"::",false)!=name) continue;
      if (result!=ignition::gazebo::kNullEntity) return ignition::gazebo::kNullEntity;
      result=e;
    }
    return result;
  }
  static double NaN() { return std::numeric_limits<double>::quiet_NaN(); }
  static ignition::math::Vector3d Missing() { return {NaN(),NaN(),NaN()}; }
  static ignition::math::Vector3d Vector(const ignition::msgs::Vector3d &v) { return {v.x(),v.y(),v.z()}; }
  static bool Finite(const ignition::msgs::Vector3d &v) { return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z()); }
  void WriteVector(const ignition::math::Vector3d &v) { output << ',' << v.X() << ',' << v.Y() << ',' << v.Z(); }
  void WriteQuaternion(const ignition::math::Quaterniond &q) { output << ',' << q.X() << ',' << q.Y() << ',' << q.Z() << ',' << q.W(); }
  void WriteMissing(int n) { for(int i=0;i<n;++i) output << ',' << NaN(); }
  void Fail(const std::string &message) { fatal=true; if(!outputPath.empty()) std::ofstream(outputPath+".error")<<message<<'\n'; }
  std::ofstream output;
  std::string outputPath;
  std::unique_ptr<ignition::transport::Node> node;
  ignition::transport::Node::Publisher statePublisher;
  std::string stateTopic;
  bool enabled=false,observerEnabled=false,fatal=false,csvEnabled=false;
  uint64_t seq=0;
  ignition::gazebo::Entity ground=ignition::gazebo::kNullEntity;
  ignition::gazebo::Entity base=ignition::gazebo::kNullEntity;
  std::array<ignition::gazebo::Entity,6> joints{},children{},parents{};
  std::array<bool,6> requested{},linkRequested{};
  std::array<bool,6> preQValid{},preVValid{},commandPresent{},commandValid{};
  std::array<double,6> preQ{},preV{},command{};
  std::array<bool,6> updateStateValid{},updateCommandPresent{},updateCommandValid{};
  std::array<double,6> updateQ{},updateV{},updateCommand{};
  std::array<CommandComponentObservation,6> preVelocityCommand{},updateVelocityCommand{};
  ignition::math::Vector3d preBaseLinear{0,0,0},preBaseAngular{0,0,0};
  ignition::math::Vector3d updateBaseLinear{0,0,0},updateBaseAngular{0,0,0};
  ignition::math::Pose3d updateBasePose;
  bool updateBasePoseValid=false,updateBaseVelocityValid=false;
  bool preBaseValid=false;
  int64_t preNs=0,preDt=0;
  uint64_t preIteration=0;
  int64_t updateNs=0,updateDt=0;
  uint64_t updateIteration=0;
  const std::array<const char*,6> jointNames{{"link_002_joint","link_003_joint","link_004_joint","link_005_joint","link_006_joint","link_007_joint"}};
  const std::array<const char*,6> childNames{{"link_002","link_003","link_004","link_005","link_006","link_007"}};
  const std::array<const char*,6> parentNames{{"base_link","link_002","link_003","base_link","link_005","link_006"}};
};
} // namespace bbot
IGNITION_ADD_PLUGIN(bbot::LandingRepairWrenchRecorder,ignition::gazebo::System,
    bbot::LandingRepairWrenchRecorder::ISystemConfigure,
    bbot::LandingRepairWrenchRecorder::ISystemPreUpdate,
    bbot::LandingRepairWrenchRecorder::ISystemUpdate,
    bbot::LandingRepairWrenchRecorder::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(bbot::LandingRepairWrenchRecorder,"bbot::LandingRepairWrenchRecorder")
