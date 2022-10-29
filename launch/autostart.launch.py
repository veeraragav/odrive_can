import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    odrive_dir = get_package_share_directory('odrive_can')
    cmd_vel_mux_dir = get_package_share_directory('cmd_vel_mux')
    ublox_dir = get_package_share_directory('ublox_gps')
    thingstream_dir = get_package_share_directory('thingstream_client')

    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')


    joy_vel_cmd = DeclareLaunchArgument('joy_vel', default_value='/joy/cmd_vel')
    joy_config_cmd = DeclareLaunchArgument('joy_config', default_value='mtg_xbox_wired')
    joy_dev_cmd = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')
    config_filepath_cmd = DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('joystick_pkg'), 'config', '')),
            joy_config, TextSubstitution(text='.config.yaml')])

    joy_node_cmd = Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }])

    joy_teleop_cmd = Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_twist_joy_node', 
            parameters=[config_filepath],
            remappings={('/cmd_vel', LaunchConfiguration('joy_vel'))},
            )

    odrive_node_cmd= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odrive_dir, 'launch/odrive_can.launch.py')))

    cmd_vel_mux_cmd= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cmd_vel_mux_dir, 'launch/cmd_vel_mux-launch.py')))

    ublox_gps_cmd= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ublox_dir, 'launch/ublox_gps_node_multi-launch.py')))

    thingstream_client_cmd= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(thingstream_dir, 'launch/thingstream_client.launch.py')))


# Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(joy_vel_cmd)
    ld.add_action(joy_config_cmd)
    ld.add_action(joy_dev_cmd)
    ld.add_action(config_filepath_cmd)
    ld.add_action(joy_node_cmd)
    ld.add_action(joy_teleop_cmd)

    ld.add_action(odrive_node_cmd)
    
    ld.add_action(cmd_vel_mux_cmd)

    ld.add_action(ublox_gps_cmd)
    ld.add_action(thingstream_client_cmd)


    return ld
