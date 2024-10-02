#include <cmath>
#include <iostream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>

#include "DoorButton.h"
#include <common.hpp>

// Recommended by gazebo docs
using namespace gz;
using namespace sim;
using namespace systems;

using namespace trigger_systems;

void DoorButton::post_configure(EntityComponentManager &_ecm) {
    BaseJointTriggerSystem::post_configure(_ecm);

    hinge_joint = Joint(_ecm.EntityByComponents(components::Name(hinge_joint_name)));
    NAMED_LOG << "Hinge: " << hinge_joint.Name(_ecm).value();
}

void DoorButton::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& _event_mgr
) {
    BaseJointTriggerSystem::Configure(_entity, _sdf, _ecm, _event_mgr);

    const_cast<std::string&>(hinge_joint_name) = _sdf->Get<std::string>("hinge_joint");
    NAMED_LOG << "hinge_joint_name: <" << hinge_joint_name << ">";

    std::pair<double, bool> opening_force_result_pair = _sdf->Get<double>("opening_force", DEFAULT_DOOR_FORCE);
    const_cast<double&>(opening_force) = std::get<double>(opening_force_result_pair);
    NAMED_LOG << "opening_force: <" << opening_force << ">";

    std::pair<double, bool> closing_force_result_pair = _sdf->Get<double>("closing_force", -DEFAULT_DOOR_FORCE);
    const_cast<double&>(closing_force) = std::get<double>(closing_force_result_pair);
    NAMED_LOG << "closing_force: <" << closing_force << ">";

    const_cast<std::vector<double>&>(opening_forces)[0] = opening_force;
    const_cast<std::vector<double>&>(closing_forces)[0] = closing_force;
}

void DoorButton::on_activation(EntityComponentManager &_ecm) {
    NAMED_LOG << "Activation";
    hinge_joint.SetForce(_ecm, opening_forces);
}

void DoorButton::on_deactivation(EntityComponentManager &_ecm) {
    NAMED_LOG << "Deactivation";
    hinge_joint.SetForce(_ecm, closing_forces);
}

GZ_ADD_PLUGIN(
    DoorButton,
    gz::sim::System,
    DoorButton::ISystemConfigure,
    DoorButton::ISystemPreUpdate
)
