from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from simulation.common import (
    SIM_DIR,
    MODELS_DIR,
    RVIZ_CONFIG_PARAM_NAME,
    WORLD_PARAM_NAME,
)


pkg_simulation = Path(get_package_share_directory("simulation"))
pkg_ros_gz_sim = Path(get_package_share_directory("ros_gz_sim"))


def generate_gz_sim(context: LaunchContext, world_path: LaunchConfiguration):
    world_path_str = context.perform_substitution(world_path)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(pkg_ros_gz_sim / "launch" / "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"--gui-config {str(SIM_DIR / 'config' / 'gz.config')} \
                         --render-engine ogre \
                         {str(MODELS_DIR / world_path_str)}"
            }.items(),
        )
    ]


def generate_launch_description():
    ld = LaunchDescription()

    # Setup to launch the simulator and Gazebo world
    ld.add_action(
        DeclareLaunchArgument(
            WORLD_PARAM_NAME,
            default_value="worlds/plane.sdf",
            description="Gazebo world SDF",
        )
    )
    ld.add_action(
        OpaqueFunction(
            function=generate_gz_sim,
            args=[LaunchConfiguration(WORLD_PARAM_NAME)],
        )
    )

    # Rviz
    ld.add_action(
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz")
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(pkg_simulation / "launch" / "simple_rviz.launch.py")
            ),
            launch_arguments={
                RVIZ_CONFIG_PARAM_NAME: str(SIM_DIR / "config" / "gazebo.rviz")
            }.items(),
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )

    # Robot state publisher
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(pkg_simulation / "launch" / "rsp_model.launch.py")
            )
        )
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    ld.add_action(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[
                {
                    "config_file": str(SIM_DIR / "config" / "ros_bridge.yaml"),
                    "qos_overrides./tf_static.publisher.durability": "transient_local",
                }
            ],
            output="screen",
        )
    )

    return ld
