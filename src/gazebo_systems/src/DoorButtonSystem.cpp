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

#include "gazebo_systems/DoorButtonSystem.h"

using namespace gz;
using namespace sim;
using namespace systems;

void DoorButtonSystem::real_configure(const gz::sim::EntityComponentManager &_ecm) {
    if (is_configured) {
        return;
    }
    is_configured = true;

    auto all = _ecm.Entities();
    auto vertices = all.Vertices();
    for (auto obj: vertices) {
        const Entity& ent = obj.second.get().Data();
        Model model = gz::sim::Model(ent);
        std::cout << model.Name(_ecm) << std::endl;
    }
}

void DoorButtonSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm
) {
    if (!is_configured) {
        real_configure(_ecm);
    }
    if (_info.paused || _info.iterations % 100 != 0) {
        return;
    }

}

void DoorButtonSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager& _event_mgr
) {
    world = _entity;
}

GZ_ADD_PLUGIN(
    DoorButtonSystem,
    gz::sim::System,
    DoorButtonSystem::ISystemConfigure,
    DoorButtonSystem::ISystemPostUpdate
)

/*
//gzlog << "ros_gz_example_gazebo::DoorButtonSystem::PostUpdate" << std::endl;
std::cout << "Hello from plugin" << std::endl;
for (auto& link: model.Links(_ecm)) {
    std::cout << worldPose(link, _ecm) << " ;;; ";
}
std::cout << std::endl;
*/

/*
auto first_element = _sdf->GetFirstElement();
std::cout << _sdf->OriginalVersion() << std::endl;
std::cout << (first_element ? 'y' : 'n') << std::endl;
for (sdf::ElementPtr elem_ptr = first_element; elem_ptr; elem_ptr = elem_ptr->GetNextElement()) {
    std::cout << "here" << std::endl;
    std::cout << elem_ptr->GetName() << std::endl;
}

auto linkName = _sdf->Get<std::string>("rover");
std::cout << linkName << std::endl;
door = _ecm.EntityByComponents(
    components::ParentEntity(_entity),
    components::Name(linkName),
    components::Link()
);
std::cout << +door << std::endl;
*/


//auto door_data =  _ecm.ComponentData<gz::sim::components::Pose>(door);
//std::cout << door_data.value() << std::endl;
/*door = model.LinkByName(_ecm, "rover::rover::base");
std::cout << +door << std::endl;
std::cout << model.Name(_ecm) << std::endl;
for (auto inner_model_id: model.Models(_ecm)) {
    auto inner_model = gz::sim::Model(inner_model_id);
    std::cout << inner_model.Name(_ecm) << std::endl;
}*/

//button_joint = model.JointByName(_ecm, "button_joint");
//hinge_joint = model.JointByName(_ecm, "hinge_joint");
