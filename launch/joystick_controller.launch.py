from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name='explorerbot_v2_control' 

    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')

    motor_controller_node = Node(
            package = package_name,
            executable='motor_controller',
         )

    joy_node = Node(
            package='joy',
            executable='joy_node',
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            remappings=[('/cmd_vel','/cmd_vel_joy')],
            parameters=[joy_params]
         )
    
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )

    return LaunchDescription([
        motor_controller_node,
        joy_node,
        teleop_node,
        twist_mux
    ])
