// Passive native geometry recorder for the independent reference experiment.
// Reads ECM state after physics; it has no command publisher, force API, or PreUpdate.
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <cmath>
#include <array>
#include <gz/plugin/Register.hh>
#include <ignition/common/Console.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Joint.hh>

namespace bbot {
class ReferenceJumpGeometryRecorder final : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate {
 public:
  void Configure(const ignition::gazebo::Entity &,
      const std::shared_ptr<const sdf::Element> &,
      ignition::gazebo::EntityComponentManager &,
      ignition::gazebo::EventManager &) override {
    const char *path = std::getenv("BBOT_REFERENCE_GEOMETRY_CSV");
    if (!path || !*path) return;
    outputPath = path;
    geometryOutput.open(outputPath, std::ios::out | std::ios::trunc);
    if (!geometryOutput.is_open()) {
      std::ofstream(outputPath + ".error") << "cannot open geometry CSV\n";
      return;
    }
    geometryOutput << "seq,sim_time_ns,physics_iteration,dt_ns,frame_valid,error,"
        "base_x,base_y,base_z,base_qx,base_qy,base_qz,base_qw,"
        "left_wheel_x,left_wheel_y,left_wheel_z,right_wheel_x,right_wheel_y,right_wheel_z,"
        "hip_left,knee_left,hip_right,knee_right\n";
    geometryOutput.flush();
    enabled = geometryOutput.good();
  }
  void PostUpdate(const ignition::gazebo::UpdateInfo &info,
      const ignition::gazebo::EntityComponentManager &ecm) override {
    if (info.paused || !enabled || geometryFatal) return;
    const auto simNs = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    const auto dtNs = std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count();
    WriteGeometryFrame(info, ecm, simNs, dtNs);
    ++seq;
  }
  private: template<typename ComponentT>
  ignition::gazebo::Entity FindNamedEntity(
      const ignition::gazebo::EntityComponentManager &_ecm,
      const std::string &_expected, const ComponentT &_component) const
  {
    ignition::gazebo::Entity found = ignition::gazebo::kNullEntity;
    for (const auto entity : _ecm.EntitiesByComponents(_component))
    {
      if (ignition::gazebo::scopedName(entity, _ecm, "::", false) == _expected)
      {
        if (found != ignition::gazebo::kNullEntity)
          return ignition::gazebo::kNullEntity;
        found = entity;
      }
    }
    return found;
  }

  private: void WriteGeometryFrame(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm,
      int64_t _simNs, int64_t _dtNs)
  {
    this->ResolveGeometryEntities(_ecm);
    bool valid = _simNs >= 0 && _dtNs == 1'000'000;
    std::string error;
    const auto pose = [&](ignition::gazebo::Entity entity,
                          ignition::math::Pose3d &out) {
      if (entity == ignition::gazebo::kNullEntity) return false;
      const auto data = _ecm.ComponentData<ignition::gazebo::components::Pose>(entity);
      if (!data.has_value()) return false;
      out = ignition::gazebo::worldPose(entity, _ecm);
      return std::isfinite(out.Pos().X()) && std::isfinite(out.Pos().Y()) &&
             std::isfinite(out.Pos().Z()) && std::isfinite(out.Rot().X()) &&
             std::isfinite(out.Rot().Y()) && std::isfinite(out.Rot().Z()) &&
             std::isfinite(out.Rot().W());
    };
    const auto joint = [&](ignition::gazebo::Entity entity, double &out) {
      if (entity == ignition::gazebo::kNullEntity) return false;
      const auto data = _ecm.ComponentData<ignition::gazebo::components::JointPosition>(entity);
      if (!data.has_value() || data->empty()) return false;
      out = data->front();
      return std::isfinite(out);
    };
    ignition::math::Pose3d base, left, right;
    std::array<double, 4> q{};
    valid = valid && pose(this->baseLink, base) && pose(this->leftWheelLink, left) &&
            pose(this->rightWheelLink, right) && joint(this->hipLeftJoint, q[0]) &&
            joint(this->kneeLeftJoint, q[1]) && joint(this->hipRightJoint, q[2]) &&
            joint(this->kneeRightJoint, q[3]);
    if (!valid) error = "geometry_component_missing_or_nonfinite";
    this->geometryOutput << this->seq << ',' << _simNs << ',' << _info.iterations << ','
                         << _dtNs << ',' << (valid ? 1 : 0) << ',' << error;
    if (valid)
    {
      this->geometryOutput << std::setprecision(17) << ','
          << base.Pos().X() << ',' << base.Pos().Y() << ',' << base.Pos().Z() << ','
          << base.Rot().X() << ',' << base.Rot().Y() << ',' << base.Rot().Z() << ',' << base.Rot().W() << ','
          << left.Pos().X() << ',' << left.Pos().Y() << ',' << left.Pos().Z() << ','
          << right.Pos().X() << ',' << right.Pos().Y() << ',' << right.Pos().Z() << ','
          << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3];
    }
    else
      for (int column = 0; column < 17; ++column)
        this->geometryOutput << ',';
    this->geometryOutput << '\n';
    this->geometryOutput.flush();
    if (!this->geometryOutput.good())
    {
      this->geometryFatal = true;
      std::ofstream(this->outputPath + ".error") << "geometry CSV write failed\n";
      ignition::common::Console::err << "[ReferenceJumpGeometryRecorder] geometry CSV write failed\n";
    }
  }

  private: void ResolveGeometryEntities(
      const ignition::gazebo::EntityComponentManager &_ecm)
  {
    if (this->baseLink == ignition::gazebo::kNullEntity)
      this->baseLink = this->FindNamedEntity(_ecm, "flat_jump_world::bbot::base_link",
                                            ignition::gazebo::components::Link());
    if (this->leftWheelLink == ignition::gazebo::kNullEntity)
      this->leftWheelLink = this->FindNamedEntity(_ecm, "flat_jump_world::bbot::link_004",
                                                  ignition::gazebo::components::Link());
    if (this->rightWheelLink == ignition::gazebo::kNullEntity)
      this->rightWheelLink = this->FindNamedEntity(_ecm, "flat_jump_world::bbot::link_007",
                                                   ignition::gazebo::components::Link());
    if (this->hipLeftJoint == ignition::gazebo::kNullEntity)
      this->hipLeftJoint = this->FindNamedEntity(_ecm, "flat_jump_world::bbot::link_002_joint",
                                                 ignition::gazebo::components::Joint());
    if (this->kneeLeftJoint == ignition::gazebo::kNullEntity)
      this->kneeLeftJoint = this->FindNamedEntity(_ecm, "flat_jump_world::bbot::link_003_joint",
                                                  ignition::gazebo::components::Joint());
    if (this->hipRightJoint == ignition::gazebo::kNullEntity)
      this->hipRightJoint = this->FindNamedEntity(_ecm, "flat_jump_world::bbot::link_005_joint",
                                                 ignition::gazebo::components::Joint());
    if (this->kneeRightJoint == ignition::gazebo::kNullEntity)
      this->kneeRightJoint = this->FindNamedEntity(_ecm, "flat_jump_world::bbot::link_006_joint",
                                                  ignition::gazebo::components::Joint());
  }

 private:
  std::string outputPath;
  bool enabled{false}, geometryFatal{false};
  std::ofstream geometryOutput;
  uint64_t seq{0};
  ignition::gazebo::Entity baseLink{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity leftWheelLink{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity rightWheelLink{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity hipLeftJoint{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity kneeLeftJoint{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity hipRightJoint{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity kneeRightJoint{ignition::gazebo::kNullEntity};
};
}
IGNITION_ADD_PLUGIN(bbot::ReferenceJumpGeometryRecorder,
    ignition::gazebo::System, bbot::ReferenceJumpGeometryRecorder::ISystemConfigure,
    bbot::ReferenceJumpGeometryRecorder::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(bbot::ReferenceJumpGeometryRecorder, "bbot::ReferenceJumpGeometryRecorder")
