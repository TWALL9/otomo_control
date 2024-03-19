import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
# import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'otomo_control'
    file_subpath = 'description/robot.urdf.xacro'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Use xacro to process the file
    xacro_file = os.path.join(
        get_package_share_directory(pkg_name), file_subpath
    )

    # robot_description_raw = xacro.process_file(xacro_file).toxml()
    robot_description_raw = Command(
        [
            'xacro ', xacro_file,
            ' use_ros2_control:=', use_ros2_control,
            ' use_sim_mode:=', use_sim_time
        ]
    )
    robot_description_params = {
        'robot_description': robot_description_raw,
        'use_sim_time': use_sim_time
    }
    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_params]
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use the sim time'
        ),

        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2 control'
        )
    ])
