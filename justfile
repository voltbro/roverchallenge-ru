set shell := ["bash", "-uc"]

python := "/usr/bin/python3"

sw_render := "false"
sw_envs := if sw_render == "false" { "" } else { "LIBGL_ALWAYS_SOFTWARE=true MESA_GL_VERSION_OVERRIDE=3.3" }

ros_packages_no_sim := "gazebo_systems controller ruka_gz radiolink"
build_command := if `if [ -f "${PROJECT_ROOT}/install/setup.sh" ]; then echo "has_ros"; fi` == "has_ros" { "colcon build" } else { "colcon build --packages-select " + ros_packages_no_sim + "; colcon build --packages-select simulation"}

build_models_cmd := "${MODELS_DIR}/build_model.py"
verbose_build_cmd := "VERBOSE=1 MAKE_JOBS=2 colcon build --symlink-install --parallel-workers 2 --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_VERBOSE_MAKEFILE=ON"
verbose_clean_build_cmd := verbose_build_cmd + "--cmake-clean-first"

# RVIZ

rviz:
    ros2 launch simulation full_rviz.launch.py

# BUILD

manifest_launch:
    jsonnet --ext-str SIM_DIR --ext-str MODELS_DIR --ext-str PYTHONPATH --ext-str AMENT_PREFIX_PATH .vscode/launch.jsonnet > .vscode/launch.json

build_models:
    {{build_models_cmd}}

clear_build:
    rm -rf "${EXTERNAL_DIR}/build" "${EXTERNAL_DIR}/log" "${EXTERNAL_DIR}/install"
    rm -rf "${ROS_DIR}/build" "${ROS_DIR}/log" "${ROS_DIR}/install"

rosdep_install:
    rosdep install --from-paths . -y --ignore-src

build: manifest_launch
    {{build_command}}
    {{build_models_cmd}}

# GAZEBO

sim ARGS="world:=worlds/field/field.sdf": build
    @echo "Additional env: {{sw_envs}}"
    {{sw_envs}} ros2 launch simulation sim.launch.py {{ARGS}}
