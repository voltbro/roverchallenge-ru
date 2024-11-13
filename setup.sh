toplevel_dir=$(realpath $(dirname "$BASH_SOURCE"))

echo "Root is - ${toplevel_dir}"
if [ ! -f "${toplevel_dir}/setup.sh" ]; then
  echo "Not in main workspace"
  return 0
fi

export PROJECT_ROOT=${toplevel_dir}

# Setup utils
export PROJECT_UTILS_ROOT=${PROJECT_ROOT}/_utils
if [ -z "$CONTROL_ROOT" ]; then
    # Not in systemd
    source ${PROJECT_UTILS_ROOT}/base.sh
    information "Sourced base script"
else
    # In systemd - do not have term and tput
    function cecho() {
        echo "${3}"
    }
    function information() {
        cecho "${1}"
    }
    function success() {
        cecho "${1}"
    }
fi

env_file="${PROJECT_ROOT}/.env"
if [ -f "${env_file}" ]; then
    export $(cat ${env_file} | xargs)
    information "Sourced .env"
fi

# Dirs variabless
export PATH=${PROJECT_UTILS_ROOT}:$PATH
export ROS_DIR=${PROJECT_ROOT}/src
export SIM_DIR=${ROS_DIR}/simulation
export MODELS_DIR=${SIM_DIR}/models

# ROS2
ros_setup_file="${PROJECT_ROOT}/install/setup.sh"
if [ -f "${ros_setup_file}" ]; then
    information "Source ROS setup"
    source ${ros_setup_file}
fi

# Python config
export PYTHONPATH=${PROJECT_UTILS_ROOT}:$PYTHONPATH

# Setup Gazebo
export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}:${SIM_DIR}/simulation
export SDF_PATH=${SDF_PATH}:${MODELS_DIR}

success "Setup complete"
return 0
