from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('log', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value='/home/moonshot/capstone_final_ws/src/third_impact/debug_log'),
        Node(package='third_impact', executable='debug_recorder', output='screen',
             condition=IfCondition(LaunchConfiguration('log')),
             parameters=[{'log_dir': LaunchConfiguration('log_dir')}]),
    ])
