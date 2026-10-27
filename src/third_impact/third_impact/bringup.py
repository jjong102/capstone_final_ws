"""Jetson bringup. Nav2 output is isolated from the motor command topic."""
from pathlib import Path
import fcntl
import os
import math
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_leases = []


def default_map():
    return str(Path(share('third_impact'))/'maps'/'third_impact_map.yaml')


def generate(mode):
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map()),
        DeclareLaunchArgument('state_file', default_value=str(Path.home()/'.ros/third_impact/goal.json')),
        DeclareLaunchArgument('params_file', default_value=str(Path(share('third_impact'))/'config/nav2.yaml')),
        DeclareLaunchArgument('impact_params', default_value=str(Path(share('third_impact'))/'config/impact.yaml')),
        OpaqueFunction(function=lambda context: start(context, mode))])


def start(context, mode):
    values = {k: LaunchConfiguration(k).perform(context) for k in ('map', 'state_file', 'params_file', 'impact_params')}
    map_path = Path(values['map']).expanduser().resolve()
    if not map_path.is_file():
        raise RuntimeError(f'Create third_impact_map first; map YAML missing: {map_path}. Supply map:=/absolute/path/third_impact_map.yaml')
    config = yaml.safe_load(map_path.read_text())
    if not (map_path.parent/config['image']).is_file():
        raise RuntimeError('Map image referenced by map YAML does not exist')
    values['map'] = str(map_path)
    impact_config = yaml.safe_load(Path(values['impact_params']).expanduser().read_text())
    patrol_tolerance = float(impact_config.get('third_impact', {}).get('ros__parameters', {}).get('patrol_tolerance', .05))
    if not math.isfinite(patrol_tolerance) or patrol_tolerance <= 0.:
        raise ValueError('patrol_tolerance must be a positive finite distance')
    # Reject duplicate bringup before opening any hardware devices.
    ros_home = Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))
    lock_path = ros_home/('third_impact_launch_'+os.environ.get('ROS_DOMAIN_ID', '0')+'.lock')
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lease = lock_path.open('a')
    try:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError('Stop the other impact/make_goal launch before starting this one') from exc
    _leases.append(lease)
    def include(pkg, path, arguments=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share(pkg))/path)),
                                        launch_arguments=(arguments or {}).items())
    # Match existing serial launch hardware and URDF without launching a second driver.
    urdf = (Path(share('amr'))/'description/amr_description.urdf').read_text()
    supervisor = Node(package='third_impact', executable='impact_node', output='screen',
                      parameters=[values['impact_params'], {'mode': mode, 'map_file': values['map'], 'state_file': values['state_file']}])
    actions = [
        RegisterEventHandler(OnProcessExit(target_action=supervisor,
            on_exit=[EmitEvent(event=Shutdown(reason='third_impact supervisor exited'))])),
        Node(package='serial_test', executable='test_node', name='test_node', output='screen',
             parameters=[{'initial_manual': mode == 'teach', 'command_timeout': .3, 'reset_on_stop': True}]),
        Node(package='joy', executable='joy_node', name='joy_node', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': urdf}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher'),
        include('ebimu_pkg', 'launch/ebimu_ekf.launch.py'),
        include('sllidar_ros2', 'launch/sllidar_c1_2_launch.py'),
        include('amr_navigator', 'launch/nav2_bringup/localization_launch.py',
                {'map': values['map'], 'params_file': values['params_file'], 'use_sim_time': 'false', 'autostart': 'true'}),
        Node(package='third_impact', executable='localization_bootstrap', output='screen',
             parameters=[{'pose_file': str(Path(values['state_file']).expanduser().with_suffix('.localization.json')),
                          'front_scan': impact_config.get('third_impact', {}).get('ros__parameters', {}).get('front_scan', '/rplidar1/scan_filtered'),
                          'rear_scan': impact_config.get('third_impact', {}).get('ros__parameters', {}).get('rear_scan', '/rplidar2/scan_filtered')}]),
        supervisor,
    ]
    if mode == 'mission':
        specifications = [
            ('nav2_controller', 'controller_server', [('cmd_vel', '/third_impact/nav_raw_cmd_vel')]),
            ('nav2_smoother', 'smoother_server', []),
            ('nav2_planner', 'planner_server', []),
            ('nav2_behaviors', 'behavior_server', [('cmd_vel', '/third_impact/nav_raw_cmd_vel')]),
            ('nav2_bt_navigator', 'bt_navigator', []),
            ('nav2_waypoint_follower', 'waypoint_follower', []),
            ('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', '/third_impact/nav_raw_cmd_vel'),
                                                         ('cmd_vel_smoothed', '/third_impact/nav_cmd_vel')]),
        ]
        nav = [Node(package=pkg, executable=name, name=name, output='screen',
                    parameters=[values['params_file'], {'use_sim_time': False}]
                    + ([{'patrol_goal_checker.xy_goal_tolerance': patrol_tolerance,
                         'PatrolFollowPath.xy_goal_tolerance': patrol_tolerance}]
                       if name == 'controller_server' else []), remappings=remap)
               for pkg, name, remap in specifications]
        nav.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
                        parameters=[{'autostart': True, 'node_names': [name for _, name, _ in specifications]}]))
        actions.append(TimerAction(period=2., actions=nav))
    return actions
