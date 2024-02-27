import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    this_package = 'otomo_control'
    twist_mux_params = os.path.join(
        get_package_share_directory(this_package),
        'config',
        'twist_mux.yaml')

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        # parameters=[twist_mux_params],
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_controller/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        twist_mux
    ])
