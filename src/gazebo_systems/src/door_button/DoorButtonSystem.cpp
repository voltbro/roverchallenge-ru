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

#include "DoorButtonSystem.h"

// Recommended by gazebo docs
using namespace gz;
using namespace sim;
using namespace systems;

using namespace gazebo_systems;

void DoorButtonSystem::post_configure(EntityComponentManager &_ecm) {
    if (is_configured) {
        return;
    }
    is_configured = true;

    hinge_joint = Joint(_ecm.EntityByComponents(components::Name(hinge_joint_name)));
    button_joint = Joint(_ecm.EntityByComponents(components::Name(button_joint_name)));
    std::cout << "Hinge: " << hinge_joint.Name(_ecm).value() << std::endl;
    std::cout << "Button: " << button_joint.Name(_ecm).value() << std::endl;

    sdf::JointAxis button_axis = button_joint.Axis(_ecm).value()[0];
    const_cast<double&>(button_lower) = button_axis.Lower();
    const_cast<double&>(button_upper) = button_axis.Upper();
    button_joint.EnablePositionCheck(_ecm, true);
    std::cout << "Button limits: " << button_lower << " " << button_upper << std::endl;
}

void DoorButtonSystem::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm
) {
    if (!is_configured) {
        post_configure(_ecm);
    }
    if (_info.paused) {
        return;
    }

    auto button_pos = button_joint.Position(_ecm).value()[0];
    double button_move = button_pos - button_lower;
    double part_pressed = std::clamp(button_move / button_upper, 0.0, 1.0);
    is_pressed = (part_pressed * 100) >= percentage_pressed;
    if (is_pressed) {
        open_door(_ecm);
    }
    else {
        close_door(_ecm);
    }

    if (_info.iterations % 100 == 0) {
        std::cout << button_pos << " " << button_move << " " << part_pressed << " " << is_pressed << std::endl;
    }
}

void DoorButtonSystem::open_door(EntityComponentManager &_ecm) {
    hinge_joint.SetForce(_ecm, opening_forces);
}

void DoorButtonSystem::close_door(EntityComponentManager &_ecm) {
    hinge_joint.SetForce(_ecm, closing_forces);
}

void DoorButtonSystem::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager& _event_mgr
) {
    world = Model(_entity);

    const_cast<std::string&>(hinge_joint_name) = _sdf->Get<std::string>("hinge_joint");
    std::cout << "hinge_joint_name: <" << hinge_joint_name << ">" << std::endl;

    const_cast<std::string&>(button_joint_name) = _sdf->Get<std::string>("button_joint");
    std::cout << "button_joint_name: <" << button_joint_name << ">" << std::endl;

    std::pair<double, bool> force_result_pair = _sdf->Get<double>("opening_force", DEFAULT_DOOR_FORCE);
    const_cast<double&>(opening_force) = std::get<double>(force_result_pair);
    std::cout << "opening_force: <" << opening_force << ">" << std::endl;

    std::pair<double, bool> pressed_result_pair = _sdf->Get<double>("percentage_pressed", DEFAULT_PERCENTAGE_PRESSED);
    const_cast<double&>(percentage_pressed) = std::get<double>(pressed_result_pair);
    std::cout << "percentage_pressed: <" << percentage_pressed << ">" << std::endl;

    const_cast<std::vector<double>&>(opening_forces)[0] = opening_force;
    const_cast<std::vector<double>&>(closing_forces)[0] = -opening_force;
}

GZ_ADD_PLUGIN(
    DoorButtonSystem,
    gz::sim::System,
    DoorButtonSystem::ISystemConfigure,
    DoorButtonSystem::ISystemPreUpdate
)
