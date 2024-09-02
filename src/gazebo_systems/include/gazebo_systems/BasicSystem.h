#pragma once

#include <gz/sim/System.hh>

namespace gazebo_systems {

  class BasicSystem: public gz::sim::System, public gz::sim::ISystemPostUpdate {
    public: void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm
    ) override;
  };

}
