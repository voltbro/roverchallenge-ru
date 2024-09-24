#pragma once

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Joint.hh>

namespace gazebo_systems
{
constexpr double DEFAULT_DOOR_FORCE = 10.0;
constexpr double DEFAULT_PERCENTAGE_PRESSED = 80.0;

class GZ_SIM_VISIBLE DoorButtonSystem:
    public gz::sim::System,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemConfigure {
public:
    void PreUpdate(
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
    void post_configure(gz::sim::EntityComponentManager &_ecm);
    void open_door(gz::sim::EntityComponentManager &_ecm);
    void close_door(gz::sim::EntityComponentManager &_ecm);

    bool is_configured = false;
    gz::sim::Model world;

    // will be setup in "Configure" and "post_configure" through const_cast
    const std::string hinge_joint_name = "";
    const std::string button_joint_name = "";
    const double button_upper = 0;
    const double button_lower = 0;
    const double opening_force = DEFAULT_DOOR_FORCE;
    const double percentage_pressed = DEFAULT_PERCENTAGE_PRESSED;
    const std::vector<double> opening_forces = {DEFAULT_DOOR_FORCE};
    const std::vector<double> closing_forces = {-DEFAULT_DOOR_FORCE};

    gz::sim::Joint button_joint;
    gz::sim::Joint hinge_joint;
    bool is_pressed = false;
};
}
