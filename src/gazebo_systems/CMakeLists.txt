cmake_minimum_required(VERSION 3.5)

project(gazebo_systems)

find_package(ament_cmake REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
message(STATUS "Compiling against Gazebo Harmonic")

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_library(TriggerSystems SHARED ${SOURCES})

target_include_directories(TriggerSystems PRIVATE src)

target_link_libraries(TriggerSystems PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(
  TARGETS TriggerSystems
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
