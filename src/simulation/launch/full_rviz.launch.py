from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    pkg_simulation = Path(get_package_share_directory('simulation'))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_simulation / 'launch' / 'simple_rviz.launch.py'))
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_simulation / 'launch' / 'rsp_model.launch.py'))
    ))

    return ld
