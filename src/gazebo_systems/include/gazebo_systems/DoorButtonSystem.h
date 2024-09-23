#pragma once

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Joint.hh>

namespace gz
{
namespace sim
{
namespace systems
{
class GZ_SIM_VISIBLE DoorButtonSystem:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemConfigure {

public:
    void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm
    ) override;

    virtual void Configure(
        const gz::sim::Entity& _entity,
        const std::shared_ptr<const sdf::Element>& _sdf,
        gz::sim::EntityComponentManager& _ecm,
        gz::sim::EventManager& _event_mgr
    ) override;
    void open_door();
    void close_door();

private:
    void real_configure(const gz::sim::EntityComponentManager &_ecm);

    bool is_configured = false;
    gz::sim::Entity world;
    gz::sim::Entity door;
    gz::sim::Entity button_joint;
    gz::sim::Entity hinge_joint;

};
}
}
}
