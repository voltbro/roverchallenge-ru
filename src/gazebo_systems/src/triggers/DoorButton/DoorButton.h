#pragma once

#include <gz/sim/System.hh>

#include "../BaseJointTriggerSystem.h"

namespace trigger_systems
{
constexpr double DEFAULT_DOOR_FORCE = 10.0;

class GZ_SIM_VISIBLE DoorButton: public BaseJointTriggerSystem {
public:
    virtual void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _event_mgr
    ) override;

protected:
    virtual void post_configure(gz::sim::EntityComponentManager &_ecm) override;
    virtual void on_activation(gz::sim::EntityComponentManager &_ecm) override;
    virtual void on_deactivation(gz::sim::EntityComponentManager &_ecm) override;

    // will be setup in "Configure" and "post_configure" through const_cast
    const std::string hinge_joint_name = "";
    const double opening_force = DEFAULT_DOOR_FORCE;
    const double closing_force = -DEFAULT_DOOR_FORCE;
    const std::vector<double> opening_forces = {DEFAULT_DOOR_FORCE};
    const std::vector<double> closing_forces = {-DEFAULT_DOOR_FORCE};

    // Logical variables
    gz::sim::Joint hinge_joint;
};

}
