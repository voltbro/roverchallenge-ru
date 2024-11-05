from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

LOG_FORMAT = """{
    "_": "{{message}}",
    "level": "{{severity}}",
    "timestamp": {{time}},
    "loc": "{{name}}"
}"""


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="radiolink",
                executable="radiolink",
                name="radiolink",
            )
        ]
    )
