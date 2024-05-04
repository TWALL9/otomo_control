import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def generate_launch_description():
    package_share_directory = get_package_share_directory('otomo_control')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    mode = LaunchConfiguration('mode')
    map_file = LaunchConfiguration('map_file')

    default_params_file = os.path.join(package_share_directory,
                                       'config', 'mapper_params_online_async.yaml')
    default_map_file = os.path.join(package_share_directory, 'maps', 'apartment_sim_serial')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        choices=['mapping', 'localization'],
        description='Whether to launch slam_toolbox in localization or mapping mode'
    )
    map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Path to the map serial location'
    )

    # If the provided param file doesn't have slam_toolbox params, we must pass the
    # default_params_file instead. This could happen due to automatic propagation of
    # LaunchArguments. See:
    # https://github.com/ros-planning/navigation2/pull/2243#issuecomment-800479866
    has_node_params = HasNodeParams(source_file=params_file,
                                    node_name='slam_toolbox')

    actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', default_params_file, '"'])

    # localization_params_file = PythonExpression(['"', localization_config, '" if ', localization_mode,
    #                                        ' == "true" else "', mapping_config, '"'])

    log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    default_params_file],
                               condition=UnlessCondition(has_node_params))

    start_async_slam_toolbox_node = Node(
        parameters=[
          actual_params_file,
          {'use_sim_time': use_sim_time,
           'mode': mode}
        #    'map_file_name': map_file}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(mode_cmd)
    ld.add_action(map_file_cmd)
    ld.add_action(log_param_change)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
