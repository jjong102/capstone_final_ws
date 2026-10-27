from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def start(context):
    teaching = LaunchConfiguration('teach').perform(context).lower() == 'true'
    config = Path(get_package_share_directory('third_impact'))/'rviz'/('teach.rviz' if teaching else 'debug.rviz')
    return [Node(package='rviz2', executable='rviz2', arguments=['-d', str(config)], output='screen')]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('teach', default_value='false'), OpaqueFunction(function=start)])
