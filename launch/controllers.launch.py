import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import TimerAction

from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'otomo_control'
    pkg_share_dir = get_package_share_directory(pkg_name)

    robot_description = Command(
        ['ros2 param get --hide-type /robot_state_publisher robot_description']
    )

    controller_params_file = os.path.join(
        pkg_share_dir,
        'config',
        'controllers.yaml'
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['diff_controller'],
    )

    pid_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['pid_controller']
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_broad'],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    delayed_pid_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[pid_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Launch them all!
    return LaunchDescription([
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_pid_spawner,
        delayed_joint_broad_spawner,
    ])
