#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/plugin/Register.hh>
#include <ignition/common/Console.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/transport/Node.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include "bbot_balance_controller/complete_contact_takeoff.hpp"

namespace bbot
{
/// Passive, read-only stream of the ground ContactSensorData after each
/// unpaused physics step. An empty ContactSensorData message is a real zero
/// contact frame; a missing component is explicitly invalid.
class GroundContactFrameRecorder final : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
{
  public: void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &) override
  {
    const auto worldName = ignition::gazebo::scopedName(
        _entity, _ecm, "::", false);
    if (worldName != "flat_jump_world")
    {
      ignition::common::Console::err << "[GroundContactFrameRecorder] unexpected world "
                                     << worldName << '\n';
      return;
    }
    this->topic = "/world/flat_jump_world/ground_contact_frames";
    this->publisher = this->node.Advertise<ignition::msgs::StringMsg>(this->topic);
    if (!this->publisher)
    {
      ignition::common::Console::err << "[GroundContactFrameRecorder] cannot advertise "
                                     << this->topic << '\n';
    }

    const char *path = std::getenv("BBOT_GROUND_CONTACT_FRAME_CSV");
    if (path != nullptr && *path != '\0')
    {
      this->csvEnabled = true;
      this->outputPath = path;
      this->output.open(this->outputPath, std::ios::out | std::ios::trunc);
      if (!this->output.is_open())
      {
        this->ReportFatal("cannot open output CSV");
        return;
      }
      this->output << "seq,sim_time_ns,physics_iteration,dt_ns,frame_valid,"
                      "num_contacts,collision_pairs_json,error\n";
      this->output.flush();
      if (!this->output.good())
        this->ReportFatal("cannot write CSV header");
    }
  }

  public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused || (!this->publisher && (!this->csvEnabled || this->fatalIo)))
      return;

    const auto simNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
        _info.simTime).count();
    const auto dtNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
        _info.dt).count();
    constexpr int64_t kExpectedStepNs = 1'000'000;
    bool valid = simNs >= 0 && dtNs == kExpectedStepNs;
    std::vector<std::string> errors;
    if (!valid)
      errors.emplace_back("invalid_step_time_or_unexpected_1ms_dt");
    if (this->hasPrevious)
    {
      if (simNs <= this->previousSimNs)
      {
        valid = false;
        errors.emplace_back("sim_time_not_increasing");
      }
      if (_info.iterations != this->previousIteration + 1)
      {
        valid = false;
        errors.emplace_back("physics_iteration_gap");
      }
      if (simNs - this->previousSimNs != dtNs)
      {
        valid = false;
        errors.emplace_back("sim_time_dt_mismatch");
      }
    }

    std::vector<std::pair<std::string, std::string>> actualPairs;
    std::vector<std::string> pairs;
    int64_t numContacts = -1;
    const ignition::gazebo::Entity groundCollision = this->FindGroundCollision(_ecm);
    if (groundCollision == ignition::gazebo::kNullEntity)
    {
      valid = false;
      errors.emplace_back("ground_collision_missing_or_ambiguous");
    }
    else
    {
      const auto data = _ecm.ComponentData<
          ignition::gazebo::components::ContactSensorData>(groundCollision);
      if (!data.has_value())
      {
        valid = false;
        errors.emplace_back("ground_collision_contact_data_missing");
      }
      else
      {
        const auto &contacts = *data;
        numContacts = contacts.contact_size();
        for (const auto &contact : contacts.contact())
        {
          const auto c1 = this->ResolveCollision(contact.collision1().id(), _ecm);
          const auto c2 = this->ResolveCollision(contact.collision2().id(), _ecm);
          if (c1.empty() || c2.empty())
          {
            valid = false;
            errors.emplace_back("collision_entity_unresolved");
          }
          actualPairs.emplace_back(c1, c2);
          pairs.emplace_back("[" + JsonString(c1) + "," + JsonString(c2) + "]");
        }
      }
    }

    std::ostringstream pairJson;
    pairJson << '[';
    for (size_t i = 0; i < pairs.size(); ++i)
    {
      if (i != 0)
        pairJson << ',';
      pairJson << pairs[i];
    }
    pairJson << ']';

    const std::string error = Join(errors, ";");
    ignition::msgs::StringMsg wire;
    wire.set_data(bbot_jump::CompleteGroundContactWire::encode(
        this->seq, simNs, _info.iterations, dtNs, valid, numContacts, actualPairs, error));
    if (this->publisher && !this->publisher.Publish(wire))
    {
      ignition::common::Console::err << "[GroundContactFrameRecorder] publish failed at seq "
                                     << this->seq << '\n';
    }

    if (this->csvEnabled && !this->fatalIo)
    {
      this->output << this->seq << ',' << simNs << ',' << _info.iterations << ','
                   << dtNs << ',' << (valid ? 1 : 0) << ',' << numContacts << ','
                   << CsvString(pairJson.str()) << ',' << CsvString(error) << '\n';
      this->output.flush();
      if (!this->output.good())
        this->ReportFatal("CSV frame write failed");
    }

    ++this->seq;
    this->previousSimNs = simNs;
    this->previousIteration = _info.iterations;
    this->hasPrevious = true;
  }

  private: ignition::gazebo::Entity FindGroundCollision(
      const ignition::gazebo::EntityComponentManager &_ecm) const
  {
    constexpr const char *kExpected =
        "flat_jump_world::ground_plane::link::collision";
    ignition::gazebo::Entity found = ignition::gazebo::kNullEntity;
    for (const auto entity : _ecm.EntitiesByComponents(
             ignition::gazebo::components::Collision()))
    {
      if (!_ecm.Component<ignition::gazebo::components::ContactSensorData>(entity))
        continue;
      const auto name = ignition::gazebo::scopedName(
          entity, _ecm, "::", false);
      if (name == kExpected)
      {
        if (found != ignition::gazebo::kNullEntity)
          return ignition::gazebo::kNullEntity;
        found = entity;
      }
    }
    return found;
  }

  private: std::string ResolveCollision(uint64_t _id,
      const ignition::gazebo::EntityComponentManager &_ecm) const
  {
    if (_id == 0)
      return {};
    const auto entity = static_cast<ignition::gazebo::Entity>(_id);
    if (_ecm.Component<ignition::gazebo::components::Collision>(entity) == nullptr)
      return {};
    const auto fullName = ignition::gazebo::scopedName(
        entity, _ecm, "::", false);
    constexpr const char *kWorldPrefix = "flat_jump_world::";
    if (fullName.rfind(kWorldPrefix, 0) != 0)
      return {};
    return fullName;
  }

  private: static std::string JsonString(const std::string &_value)
  {
    std::ostringstream out;
    out << '"';
    for (const unsigned char c : _value)
    {
      switch (c)
      {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
          if (c < 0x20)
          {
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<int>(c) << std::dec;
          }
          else
            out << static_cast<char>(c);
      }
    }
    out << '"';
    return out.str();
  }

  private: static std::string CsvString(const std::string &_value)
  {
    std::string escaped = "\"";
    for (const char c : _value)
    {
      if (c == '"')
        escaped += "\"\"";
      else
        escaped += c;
    }
    escaped += '"';
    return escaped;
  }

  private: static std::string Join(const std::vector<std::string> &_values,
      const std::string &_separator)
  {
    std::ostringstream out;
    for (size_t i = 0; i < _values.size(); ++i)
    {
      if (i != 0)
        out << _separator;
      out << _values[i];
    }
    return out.str();
  }

  private: void ReportFatal(const std::string &_reason)
  {
    this->fatalIo = true;
    ignition::common::Console::err << "[GroundContactFrameRecorder] "
        << _reason << " at " << this->outputPath << '\n';
    std::ofstream marker(this->outputPath + ".error", std::ios::out | std::ios::trunc);
    if (marker.is_open())
      marker << _reason << '\n';
  }

  private: ignition::transport::Node node;
  private: ignition::transport::Node::Publisher publisher;
  private: std::string topic;
  private: bool csvEnabled{false};
  private: bool fatalIo{false};
  private: bool hasPrevious{false};
  private: uint64_t seq{0};
  private: uint64_t previousIteration{0};
  private: int64_t previousSimNs{0};
  private: std::string outputPath;
  private: std::ofstream output;
};
}

IGNITION_ADD_PLUGIN(bbot::GroundContactFrameRecorder,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(bbot::GroundContactFrameRecorder,
    "bbot::GroundContactFrameRecorder")
