if [ -z "$CONTROL_ROOT" ]; then
    toplevel_dir=$(git rev-parse --show-toplevel)
else
    toplevel_dir=${CONTROL_ROOT}
fi

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
export EXTERNAL_DIR=${PROJECT_ROOT}/external
export ROS_DIR=${PROJECT_ROOT}/src
export SIM_DIR=${ROS_DIR}/simulation
export MODELS_DIR=${SIM_DIR}/models

# ROS2
drivers_setup_file="${EXTERNAL_DIR}/install/setup.sh"
if [ -f "${drivers_setup_file}" ]; then
    information "Source drivers setup"
    source ${drivers_setup_file}
fi
ros_setup_file="${PROJECT_ROOT}/install/setup.sh"
if [ -f "${ros_setup_file}" ]; then
    information "Source ROS setup"
    source ${ros_setup_file}
fi

function run_string() {
    echo "${1}"
    eval "${1}"
}

# Python config
export PYTHONPATH=${PROJECT_UTILS_ROOT}:$PYTHONPATH

# Setup Gazebo
export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}:${SIM_DIR}/simulation
export SDF_PATH=${SDF_PATH}:${MODELS_DIR}
#export DISPLAY=:0
#export QT_QPA_PLATFORM=offscreen

pushd . > /dev/null
cd $PROJECT_ROOT
# Any commands relative to top dir go here
popd > /dev/null

success "Setup complete"
return 0
