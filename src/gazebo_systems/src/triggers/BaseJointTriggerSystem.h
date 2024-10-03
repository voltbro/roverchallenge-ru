#pragma once

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Joint.hh>

#include "common.hpp"

namespace trigger_systems
{
constexpr double DEFAULT_PERCENTAGE_ACTIVATED = 80.0;
constexpr std::string_view DEFAULT_PLUGIN_NAME = "trigger_system";

class EndSignal{};
const static EndSignal* END_LOG = new EndSignal();

class GZ_SIM_VISIBLE BaseJointTriggerSystem:
    public gz::sim::System,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemConfigure {
public:
    virtual void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm
    ) override;
    virtual void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _event_mgr
    ) override;
private:
    bool is_configured = false;
    // will be setup in "Configure" and "post_configure" through const_cast
    const bool is_oneshot = false;
    const bool do_log = false;
    const std::string trigger_joint_name = "";
    const double trigger_upper = 0;
    const double trigger_lower = 0;
    const double percentage_activated = DEFAULT_PERCENTAGE_ACTIVATED;
    const gz::sim::Joint trigger_joint;
    // Logical variables
    double _current_percentage = 0;
    bool _is_activated = false;
    bool _state = false;
protected:
    virtual void post_configure(gz::sim::EntityComponentManager &_ecm);
    virtual void on_activation(gz::sim::EntityComponentManager &_ecm) = 0;
    virtual void on_deactivation(gz::sim::EntityComponentManager &_ecm) = 0;

    double current_percentage() {
        return _current_percentage;
    }
    bool is_activated() {
        return _is_activated;
    }
    bool state() {
        return _state;
    }

    gz::sim::Model world;
    const std::string plugin_name = {DEFAULT_PLUGIN_NAME.begin(), DEFAULT_PLUGIN_NAME.end()};
public:
    template <class T>
    BaseJointTriggerSystem& operator<<(const T &v) {
        if (do_log) {
            std::cout << v;
        }
        return *this;
    }

    BaseJointTriggerSystem& operator<<(const EndSignal* &v) {
        if (do_log) {
            std::cout << "\"" << std::endl;
        }
        return *this;
    }

    BaseJointTriggerSystem& log() {
        return operator<<("[") << plugin_name << "] \"";
    }
};
}
