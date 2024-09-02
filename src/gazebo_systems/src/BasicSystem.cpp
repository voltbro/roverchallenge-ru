#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include "gazebo_systems/BasicSystem.h"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    gazebo_systems::BasicSystem,
    gz::sim::System,
    gazebo_systems::BasicSystem::ISystemPostUpdate
)

namespace gazebo_systems {

void BasicSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm
) {
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "ros_gz_example_gazebo::BasicSystem::PostUpdate" << std::endl;
  }
}

}  // namespace ros_gz_example_gazebo
