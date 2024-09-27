#pragma once

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Joint.hh>

namespace gazebo_systems
{
constexpr double DEFAULT_PERCENTAGE_ACTIVATED = 80.0;

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
protected:
    virtual void post_configure(gz::sim::EntityComponentManager &_ecm);
    virtual void on_activation(gz::sim::EntityComponentManager &_ecm) = 0;
    virtual void on_deactivation(gz::sim::EntityComponentManager &_ecm) = 0;

    // Setup variables
    bool is_configured = false;
    gz::sim::Model world;

    // will be setup in "Configure" and "post_configure" through const_cast
    const std::string trigger_joint_name = "";
    const double trigger_upper = 0;
    const double trigger_lower = 0;
    const double percentage_activated = DEFAULT_PERCENTAGE_ACTIVATED;

    // Logical variables
    gz::sim::Joint trigger_joint;
    bool is_activated = false;
};
}
