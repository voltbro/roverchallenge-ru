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

#include "BaseJointTriggerSystem.h"
#include "common.hpp"

// Recommended by gazebo docs
using namespace gz;
using namespace sim;
using namespace systems;

using namespace trigger_systems;

void BaseJointTriggerSystem::post_configure(EntityComponentManager &_ecm) {
    if (is_configured) {
        return;
    }
    is_configured = true;

    const_cast<Joint&>(trigger_joint) = Joint(_ecm.EntityByComponents(components::Name(trigger_joint_name)));
    NAMED_LOG << "Trigger: " << trigger_joint.Name(_ecm).value();

    sdf::JointAxis trigger_axis = trigger_joint.Axis(_ecm).value()[0];
    const_cast<double&>(trigger_lower) = trigger_axis.Lower();
    const_cast<double&>(trigger_upper) = trigger_axis.Upper();
    trigger_joint.EnablePositionCheck(_ecm, true);
    NAMED_LOG << "Trigger limits: " << trigger_lower << " " << trigger_upper;
}

void BaseJointTriggerSystem::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm
) {
    if (!is_configured) {
        post_configure(_ecm);
    }
    if (_info.paused) {
        return;
    }

    auto trigger_pos = trigger_joint.Position(_ecm).value()[0];
    double trigger_move = trigger_pos - trigger_lower;
    double part_pressed = std::clamp(trigger_move / trigger_upper, 0.0, 1.0);
    _current_percentage = part_pressed * 100;
    _is_activated = _current_percentage >= percentage_activated;
    if (_is_activated) {
        if (!is_oneshot || !_state) {
            on_activation(_ecm);
        }
        _state = true;
    }
    else {
        if (!is_oneshot || _state) {
            on_deactivation(_ecm);
        }
        _state = false;
    }

    if (_info.iterations % 100 == 0) {
        NAMED_LOG << trigger_pos << " " << trigger_move << " " << part_pressed << " " << _is_activated;
    }
}

void BaseJointTriggerSystem::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager& _event_mgr
) {
    world = Model(_entity);

    std::pair<std::string, bool> name_pair = _sdf->Get<std::string>("log_name", "");
    if (std::get<bool>(name_pair)) {
        const_cast<std::string&>(plugin_name) = std::get<std::string>(name_pair);
    }
    NAMED_LOG << "plugin_name: <" << plugin_name << ">";

    const_cast<std::string&>(trigger_joint_name) = _sdf->Get<std::string>("trigger_joint");
    NAMED_LOG << "trigger_joint_name: <" << trigger_joint_name << ">";

    std::pair<double, bool> pressed_result_pair = _sdf->Get<double>("percentage_activated", DEFAULT_PERCENTAGE_ACTIVATED);
    const_cast<double&>(percentage_activated) = std::get<double>(pressed_result_pair);
    NAMED_LOG << "percentage_activated: <" << percentage_activated << ">";

    std::pair<bool, bool> is_oneshot_pair = _sdf->Get<bool>("is_oneshot", false);
    const_cast<bool&>(is_oneshot) = std::get<0>(is_oneshot_pair);
    NAMED_LOG << "is_oneshot: <" << (is_oneshot ? "true" : "false") << ">";
}
