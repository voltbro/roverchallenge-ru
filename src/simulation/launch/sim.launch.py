from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from simulation.common import (
    SIM_DIR,
    MODELS_DIR,
    RVIZ_CONFIG_PARAM_NAME,
    WORLD_PARAM_NAME,
    ROBOT_MODEL_PATH,
)

pkg_ros_gz_sim = Path(get_package_share_directory("ros_gz_sim")).resolve()
pkg_ruka_gz = Path(get_package_share_directory("ruka_gz")).resolve()

MOVE_GROUP_PATH = str(pkg_ruka_gz / "config" / "move_group.yaml")
MOVEIT_CONFIG = (
    MoveItConfigsBuilder("explorer", package_name="ruka_gz")
    .robot_description_semantic(file_path=str(MODELS_DIR / "explorer" / "explorer.srdf"))
    .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=False)
    .trajectory_execution(file_path=str(pkg_ruka_gz / "config" / "moveit_controllers.yaml"))
    .planning_pipelines(pipelines=["chomp", "pilz_industrial_motion_planner"])
    # .moveit_config_dict.update({'use_sim_time' : True})
    .to_moveit_configs()
)


def generate_gz_sim(context: LaunchContext, world_path: LaunchConfiguration):
    world_path_str = context.perform_substitution(world_path)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_ros_gz_sim / "launch" / "gz_sim.launch.py")),
            launch_arguments={
                "gz_args": f"--gui-config {str(SIM_DIR / 'config' / 'gz.config')} \
                             {str(MODELS_DIR / world_path_str)}"
            }.items(),
        )
    ]


def add_gazebo(ld: LaunchDescription):
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


def add_rs_publisher(ld: LaunchDescription):
    with open(ROBOT_MODEL_PATH, "r") as robot_file:
        robot_description_content = robot_file.read()
    rs_parameters = [
        {"robot_description": robot_description_content},
        MOVE_GROUP_PATH,
    ]
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=rs_parameters,
        )
    )


def add_moveit(ld: LaunchDescription):
    moveit = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            MOVEIT_CONFIG.to_dict(),
            # moveit_config.update,
            MOVE_GROUP_PATH,
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )
    ld.add_action(moveit)

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=moveit,
                on_start=[joint_state_broadcaster],
            )
        )
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ruka_arm_controller",
            "--param-file",
            str(pkg_ruka_gz / "config" / "shok_controllers.yaml"),
        ],
    )
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_trajectory_controller],
            )
        ),
    )


def add_rviz(ld: LaunchDescription):
    ld.add_action(DeclareLaunchArgument("rviz", default_value="true", description="Open RViz"))
    ld.add_action(
        DeclareLaunchArgument(
            name=RVIZ_CONFIG_PARAM_NAME,
            default_value=str(SIM_DIR / "config" / "gazebo.rviz"),
            description="Absolute path to rviz config file",
        )
    )
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration(RVIZ_CONFIG_PARAM_NAME)],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[
                MOVEIT_CONFIG.robot_description_semantic,
                MOVEIT_CONFIG.planning_pipelines,
                MOVEIT_CONFIG.robot_description_kinematics,
                MOVEIT_CONFIG.joint_limits,
                MOVE_GROUP_PATH,
            ],
        )
    )


def add_ros_bridge(ld: LaunchDescription):
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


def add_radiolink(ld: LaunchDescription):
    ld.add_action(DeclareLaunchArgument("joy", default_value="true", description="Add joystick control"))
    ld.add_action(
        Node(
            package="joy",
            executable="joy_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("joy")),
        )
    )
    ld.add_action(
        Node(
            package="radiolink",
            executable="radiolink",
            output="screen",
            condition=IfCondition(LaunchConfiguration("joy")),
        )
    )


def generate_launch_description():
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=use_sim_time,
            description="RViz configuration file",
        )
    )

    # Setup to launch the simulator and Gazebo world
    add_gazebo(ld)
    # Bridge ROS topics and Gazebo messages for establishing communication
    add_ros_bridge(ld)
    # Create node for publishing topic /robot_state
    add_rs_publisher(ld)
    # Add moveit
    add_moveit(ld)
    # Rviz
    add_rviz(ld)
    # Radiolink
    add_radiolink(ld)

    return ld
