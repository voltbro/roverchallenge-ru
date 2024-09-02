from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from simulation.common import SIM_DIR, RVIZ_CONFIG_PARAM_NAME


def generate_launch_description():
    ld = LaunchDescription()

    default_rviz_config_path = SIM_DIR / "config" / "gazebo.rviz"

    GUI_PARAM_NAME = "jsp_gui"

    ld.add_action(
        DeclareLaunchArgument(
            name=GUI_PARAM_NAME,
            default_value="false",
            choices=["true", "false"],
            description="Flag to enable joint_state_publisher_gui",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name=RVIZ_CONFIG_PARAM_NAME,
            default_value=str(default_rviz_config_path),
            description="Absolute path to rviz config file",
        )
    )

    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("jsp_gui")),
        )
    )
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration(RVIZ_CONFIG_PARAM_NAME)],
        )
    )

    return ld
