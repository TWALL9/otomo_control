import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name = 'otomo_control'
    pkg_share_dir = get_package_share_directory(pkg_name)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share_dir, 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share_dir, 'launch', 'controllers.launch.py'
        )])
    )

    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share_dir, 'launch', 'twist_mux.launch.py'
        )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([
        rsp,
        controllers,
        twist_mux,
    ])
