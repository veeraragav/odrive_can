import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    odrive_params_file = LaunchConfiguration('odrive_params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'odrive_params_file', default_value=os.path.join(get_package_share_directory('odrive_can'), 'params',  'odrive.yaml'),
        description='Full path to the odrive config file to use')

    odrive_node_cmd = Node(
        package='odrive_can',
        executable='odrive_can_node',
        output='screen',
        # namespace='fl_motor',
        parameters = [odrive_params_file]
        )

    odometry_node_cmd = Node(
        package='odrive_can',
        executable='odometry_node',
        output='screen'
        )

    cmdvel_to_wheelvel_cmd = Node(
        package='odrive_can',
        executable='cmdvel_to_wheelvel_node'
        )


# Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_params_file_cmd)
    ld.add_action(odrive_node_cmd)
    ld.add_action(odometry_node_cmd)
    ld.add_action(cmdvel_to_wheelvel_cmd)

    return ld