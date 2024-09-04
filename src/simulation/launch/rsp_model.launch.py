from launch import LaunchDescription
from launch.actions import OpaqueFunction

from simulation.common import ROBOT_MODEL_PATH, get_robot_state_publisher_generator


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        OpaqueFunction(
            function=get_robot_state_publisher_generator(),
            args=[ROBOT_MODEL_PATH],
        )
    )

    return ld
