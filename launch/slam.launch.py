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

    # use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    localization_mode = LaunchConfiguration('localization_mode')
    localization_config = os.path.join(package_share_directory, 'config', 'online_async_localization.yaml')
    mapping_config = os.path.join(package_share_directory, 'config', 'online_async_mapping.yaml')

    base_params_file = os.path.join(
        package_share_directory,
        'config', 'mapper_params_online_async.yaml')

    # If the provided param file doesn't have slam_toolbox params, we must pass the
    # base_params_file instead. This could happen due to automatic propagation of
    # LaunchArguments. See:
    # https://github.com/ros-planning/navigation2/pull/2243#issuecomment-800479866
    has_node_params = HasNodeParams(source_file=params_file,
                                    node_name='slam_toolbox')

    base_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', base_params_file, '"'])
    
    params_overlay = PythonExpression(['"', localization_config, '" if ', localization_mode,
                                       ' else "', mapping_config, '"'])

    log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    base_params_file],
                               condition=UnlessCondition(has_node_params))

    start_async_slam_toolbox_node = Node(
        parameters=[
          base_params_file,
          params_overlay,
        #   {'use_sim_time': use_sim_time}
          {'use_sim_time': 'true'}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    return LaunchDescription(
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='true',
        #     description='Use simulation/Gazebo clock'),
        DeclareLaunchArgument(
            'params_file',
            default_value=base_params_file,
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
        # DeclareLaunchArgument(
        #     'localization_mode',
        #     default_value='false',
        #     description='Whether to launch slam_toolbox in localization or mapping mode'
        # ),
        log_param_change,
        start_async_slam_toolbox_node,
    )
