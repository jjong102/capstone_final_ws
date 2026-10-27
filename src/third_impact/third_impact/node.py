"""Single-owner navigation supervisor. Runs without RViz or debug processes."""
import json
import fcntl
import math
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import random
import time

import numpy as np
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

from .core import (wrap, relative, transform, voxel, register_scan, alignment_command,
                   swept_clear, inside_rectangle, map_fingerprint, atomic_save,
                   accepts_return, accepts_patrol)
from .editor import RectangleEditor, orientation, yaw


class ImpactNode(Node):
    def __init__(self):
        super().__init__('third_impact')
        defaults = dict(mode='mission', map_file='', state_file='~/.ros/third_impact/goal.json',
                        xy_tolerance=.008, yaw_tolerance_deg=5., settle_seconds=1.,
                        capture_seconds=2., align_radius=.4, scan_timeout=.45,
                        patrol_tolerance=.05,
                        require_localization_ready=True,
                        patrol_bt_file=str(Path(get_package_share_directory('third_impact'))/'config/patrol.xml'),
                        return_bt_file=str(Path(get_package_share_directory('third_impact'))/'config/return.xml'),
                        pose_timeout=1., odom_timeout=.3, stopped_linear=.005,
                        stopped_angular=.01, max_align_v=.035, max_align_w=.12,
                        match_rmse=.025, match_overlap=.65, match_observability=.015,
                        match_max_condition=500., match_max_correction=.20,
                        match_max_angle_deg=15., retry_seconds=2., patrol_clearance=.45,
                        amcl_max_xy_variance=.08, amcl_max_yaw_variance=.08,
                        footprint_half_length=.31, footprint_half_width=.24,
                        obstacle_margin=.04, scan_max_range=6.,
                        front_scan='/rplidar1/scan_filtered', rear_scan='/rplidar2/scan_filtered')
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.mode = self.p['mode']
        if self.mode not in ('mission', 'teach'):
            raise ValueError('mode must be mission or teach')
        self.path = Path(self.p['state_file']).expanduser()
        ros_home = Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))
        lock_path = ros_home/('third_impact_domain_'+os.environ.get('ROS_DOMAIN_ID', '0')+'.lock')
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        self.owner_lock = lock_path.open('a')
        try:
            fcntl.flock(self.owner_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Another third_impact teaching/mission node is already running') from exc
        self.fingerprint = map_fingerprint(self.p['map_file'])
        self.data = None
        self.detail = 'Waiting for map, localization, sensors and saved goal/rectangle'
        self.state = 'WAIT' if self.mode == 'mission' else 'TEACH'
        self.intent = 'patrol'
        self.manual = None; self.manual_stamp = 0.
        self.odom = None; self.odom_stamp = 0.
        self.amcl = None; self.amcl_stamp = 0.
        self.localization_info = {}; self.localization_stamp = 0.
        self.scans = {}; self.grid = None; self.candidates = None
        self.nav_handle = None; self.nav_pending = False; self.cancel_requested = False
        self.nav_purpose = None; self.nav_target = None
        self.patrol_reached = False; self.patrol_visits = 0
        self.last_navigation_result = None
        self.nav_velocity = Twist(); self.nav_velocity_stamp = 0.
        self.next_attempt = 0.; self.settled_since = None
        self.capture_start = 0.; self.capture_pose = None; self.capture_clouds = []
        self.capture_signature = None; self.match = None
        self.align_last_key = None; self.align_velocity = (0., 0.); self.align_valid_stamp = 0.
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arrival_pub = self.create_publisher(Empty, '/arrival_at_goal', 10)
        self.status_pub = self.create_publisher(String, '/third_impact/status', 10)
        self.event_pub = self.create_publisher(String, '/third_impact/events', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/third_impact/markers', durable)
        self.goal_pub = self.create_publisher(PoseStamped, '/third_impact/goal', durable)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, durable)
        self.create_subscription(String, '/third_impact/localization', self.on_localization, durable)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Bool, '/third_impact/manual_control', self.on_manual, durable)
        for key in ('front', 'rear'):
            self.create_subscription(LaserScan, self.p[key+'_scan'],
                                     lambda msg, k=key: self.on_scan(k, msg), qos_profile_sensor_data)
        self.create_subscription(Twist, '/third_impact/nav_cmd_vel', self.on_nav_velocity, 10)
        self.create_subscription(Empty, '/go_to_goal', self.on_return, 10)
        self.create_subscription(Empty, '/go_around', self.on_patrol, 10)
        self.create_subscription(Empty, '/save_goal', self.on_save, 10)
        self.create_subscription(Empty, '/third_impact/save_region', self.on_region, 10)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.editor = RectangleEditor(self) if self.mode == 'teach' else None
        self.load()
        self.create_timer(.05, self.tick)  # serial driver integrates commands at 20 Hz
        self.create_timer(.2, self.publish_status)
        self.create_timer(.5, self.publish_markers)

    def event(self, text):
        self.detail = text
        self.get_logger().info(text)
        self.event_pub.publish(String(data=json.dumps({'time': time.time(), 'state': self.state, 'message': text})))

    def load(self):
        if not self.path.exists():
            return
        try:
            data = json.loads(self.path.read_text())
            pose = np.asarray(data['goal'], dtype=float)
            reference = np.asarray(data['reference'], dtype=float)
            rect = data.get('rectangle')
            if data['map_fingerprint'] != self.fingerprint:
                self.event('Saved map differs: teach a new goal and rectangle')
                return
            if (pose.shape != (3,) or not np.isfinite(pose).all() or reference.ndim != 2
                    or reference.shape[1] != 2 or len(reference) < 50 or not np.isfinite(reference).all()):
                raise ValueError('Invalid goal/reference')
            if rect is not None and (len(rect) != 5 or not np.isfinite(rect).all() or min(rect[3:]) <= 0):
                raise ValueError('Invalid rectangle')
            self.data = data
            if self.editor:
                self.editor.reset(data['goal'], rect)
            self.event('Loaded saved goal' + (' and rectangle' if rect else '; rectangle required'))
        except (ValueError, KeyError, OSError, TypeError) as exc:
            self.event(f'Cannot load goal: {exc}; teach again')

    def on_map(self, msg):
        self.grid = msg
        values = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        free = values == 0
        # Unknown space and map boundaries are obstacles for candidate selection.
        distance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1]*msg.info.resolution
        self.candidates = np.argwhere(distance >= self.p['patrol_clearance'])

    def on_amcl(self, msg):
        self.amcl, self.amcl_stamp = msg, time.monotonic()

    def on_localization(self, msg):
        try:
            info = json.loads(msg.data)
            if isinstance(info, dict):
                self.localization_info, self.localization_stamp = info, time.monotonic()
        except (ValueError, TypeError):
            pass

    def on_odom(self, msg):
        self.odom, self.odom_stamp = msg, time.monotonic()

    def on_manual(self, msg):
        previous = self.manual
        self.manual, self.manual_stamp = msg.data, time.monotonic()
        if previous != self.manual:
            self.settled_since = None
            self.align_last_key = None
            if self.manual:
                self.cancel_nav()
                self.stop()
                if self.state == 'CAPTURE':
                    self.abort_capture('Manual mode changed during capture')
            self.event('Manual control: mission paused' if self.manual else 'Automatic control: resume mission')

    def on_nav_velocity(self, msg):
        self.nav_velocity, self.nav_velocity_stamp = msg, time.monotonic()

    def message_age(self, stamp):
        return (self.get_clock().now().nanoseconds-Time.from_msg(stamp).nanoseconds)*1e-9

    def on_scan(self, key, msg):
        try:
            if not 0. <= self.message_age(msg.header.stamp) <= self.p['scan_timeout']:
                return
            tf = self.tf.lookup_transform('base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            q = tf.transform.rotation
            # Full quaternion rotation: both lidar mounts include a pi pitch.
            x, y, z, w = q.x, q.y, q.z, q.w
            rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w)],
                                 [2*(x*y+z*w), 1-2*(x*x+z*z)]])
            ranges = np.asarray(msg.ranges)
            angles = msg.angle_min+np.arange(len(ranges))*msg.angle_increment
            valid = np.isfinite(ranges) & (ranges > max(msg.range_min, .10)) & (ranges < min(msg.range_max, self.p['scan_max_range']))
            points = np.column_stack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])))
            t = tf.transform.translation
            points = points @ rotation.T+[t.x, t.y]
            # Remove own chassis returns, but retain all external obstacles.
            points = points[(abs(points[:, 0]) > self.p['footprint_half_length']) |
                            (abs(points[:, 1]) > self.p['footprint_half_width'])]
            self.scans[key] = (time.monotonic(), msg.header.stamp, points)
        except TransformException:
            pass

    def pose(self):
        try:
            tf = self.tf.lookup_transform('map', 'base_link', Time())
            if not 0. <= self.message_age(tf.header.stamp) <= self.p['pose_timeout']:
                return None
            t = tf.transform.translation
            return np.array([t.x, t.y, yaw(tf.transform.rotation)])
        except TransformException:
            return None

    def cloud(self):
        now = time.monotonic()
        if any(k not in self.scans or now-self.scans[k][0] > self.p['scan_timeout'] for k in ('front', 'rear')):
            return None
        stamps = [Time.from_msg(self.scans[k][1]).nanoseconds for k in ('front', 'rear')]
        if abs(stamps[0]-stamps[1])*1e-9 > .15:
            return None
        return voxel(np.vstack([self.scans[k][2] for k in ('front', 'rear')]))

    def scan_key(self):
        return tuple(Time.from_msg(self.scans[k][1]).nanoseconds for k in ('front', 'rear'))

    def healthy(self):
        now = time.monotonic()
        if self.p['require_localization_ready'] and (
                now-self.localization_stamp > 2. or not self.localization_info.get('ready', False)):
            return False
        if self.manual is None or now-self.manual_stamp > 1. or self.odom is None or now-self.odom_stamp > self.p['odom_timeout']:
            return False
        if self.amcl is None:
            return False
        cov = self.amcl.pose.covariance
        if not np.isfinite(cov).all() or max(cov[0], cov[7]) > self.p['amcl_max_xy_variance'] or cov[35] > self.p['amcl_max_yaw_variance']:
            return False
        return self.pose() is not None and self.cloud() is not None and self.grid is not None

    def stopped(self):
        if self.odom is None or time.monotonic()-self.odom_stamp > self.p['odom_timeout']:
            return False
        t = self.odom.twist.twist
        return math.hypot(t.linear.x, t.linear.y) < self.p['stopped_linear'] and abs(t.angular.z) < self.p['stopped_angular']

    def odom_pose(self):
        pose = self.odom.pose.pose
        return np.array([pose.position.x, pose.position.y, yaw(pose.orientation)])

    def on_save(self, _):
        if self.mode != 'teach' or self.state == 'CAPTURE':
            return
        if not self.healthy() or not self.stopped():
            self.event('Save rejected: localization and both scans must be valid; stop the robot first')
            return
        self.state = 'CAPTURE'; self.capture_start = time.monotonic()
        self.capture_pose = self.pose(); self.capture_clouds = []; self.capture_signature = None
        self.capture_odom = self.odom_pose()
        self.event('Capturing stationary lidar reference; do not move')

    def abort_capture(self, reason):
        self.state = 'TEACH'; self.capture_clouds = []
        self.event('Capture rejected: '+reason+'; previous saved goal unchanged')

    def capture(self):
        pose, cloud = self.pose(), self.cloud()
        if not self.healthy() or not self.stopped() or pose is None or cloud is None:
            self.abort_capture('robot moved or sensor/localization data missing')
            return
        delta = relative(self.capture_pose, pose)
        odom_delta = relative(self.capture_odom, self.odom_pose())
        if np.linalg.norm(odom_delta[:2]) > .002 or abs(odom_delta[2]) > math.radians(.3):
            self.abort_capture('odometry detected motion during reference capture')
            return
        if np.linalg.norm(delta[:2]) > .005 or abs(delta[2]) > math.radians(1):
            self.abort_capture('pose changed during capture')
            return
        key = self.scan_key()
        if key != self.capture_signature:
            self.capture_clouds.append(cloud); self.capture_signature = key
        if time.monotonic()-self.capture_start < self.p['capture_seconds']:
            return
        if len(self.capture_clouds) < 8:
            self.abort_capture('not enough fresh scan pairs')
            return
        reference = voxel(np.vstack(self.capture_clouds))
        test = register_scan(cloud, reference, [0., 0., 0.])
        if not self.match_ok(test):
            self.abort_capture('surrounding geometry is insufficient for reliable alignment')
            return
        data = {'version': 1, 'map_fingerprint': self.fingerprint, 'map_file': self.p['map_file'],
                'saved_at': time.time(), 'goal': self.capture_pose.tolist(),
                'reference': reference.tolist(), 'rectangle': None}
        try:
            atomic_save(self.path, data)
        except OSError as exc:
            self.abort_capture(str(exc)); return
        self.data = data; self.state = 'TEACH'
        self.editor.reset(data['goal'])
        self.event('Goal saved; old rectangle deleted. Adjust rectangle and press Save Rectangle')

    def on_region(self, _):
        if self.mode != 'teach' or not self.data or self.state == 'CAPTURE':
            return
        data = dict(self.data, rectangle=list(self.editor.rect))
        try:
            atomic_save(self.path, data)
        except OSError as exc:
            self.event('Rectangle save failed: '+str(exc)); return
        self.data = data
        self.event('Goal and rectangle ready. Stop make_goal_launch before starting impact_launch')

    def on_return(self, _):
        if self.mode != 'mission' or not accepts_return(self.intent) or self.intent == 'return':
            return
        self.intent = 'return'; self.state = 'RETURN'; self.next_attempt = 0.
        self.cancel_nav(); self.stop(); self.settled_since = None
        self.event('Return requested; patrol cancelled')

    def on_patrol(self, _):
        if self.mode != 'mission' or not accepts_patrol(self.intent):
            return
        self.intent = 'patrol'; self.state = 'PATROL'; self.next_attempt = 0.
        self.event('Patrol requested')

    def stop(self):
        self.velocity_pub.publish(Twist())

    def cancel_nav(self):
        self.cancel_requested = True
        self.nav_velocity_stamp = 0.
        if self.nav_handle is not None:
            # Result callback is the handoff barrier, not cancellation acceptance.
            if not getattr(self, 'cancel_sent', False):
                self.cancel_sent = True
                self.nav_handle.cancel_goal_async()

    def send_nav(self, pose, purpose):
        if self.nav_pending or self.nav_handle is not None:
            return
        if not self.nav.server_is_ready():
            self.detail = 'Waiting for Nav2 action server'
            return
        request = NavigateToPose.Goal(); request.pose = self.pose_message(pose)
        request.behavior_tree = self.p['patrol_bt_file' if purpose == 'patrol' else 'return_bt_file']
        self.nav_pending = True; self.cancel_requested = False; self.cancel_sent = False
        self.patrol_reached = False
        self.nav_purpose = purpose; self.nav_target = list(pose)
        self.nav_velocity_stamp = 0.
        self.detail = 'Navigation request pending'
        self.nav.send_goal_async(request).add_done_callback(self.nav_accepted)

    def nav_accepted(self, future):
        self.nav_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.event(f'Navigation submission failed; retrying: {exc}')
            self.next_attempt = time.monotonic()+self.p['retry_seconds']; return
        if not handle.accepted:
            self.next_attempt = time.monotonic()+self.p['retry_seconds']
            self.event('Navigation rejected; retrying'); return
        self.nav_handle = handle
        handle.get_result_async().add_done_callback(self.nav_finished)
        if self.cancel_requested or self.manual or self.nav_purpose != self.intent:
            self.cancel_nav()
        else:
            self.event('Navigation accepted: ' + self.nav_purpose)

    def nav_finished(self, future):
        purpose, cancelled = self.nav_purpose, self.cancel_requested
        self.nav_handle = None; self.nav_velocity_stamp = 0.
        try:
            status = future.result().status
        except Exception as exc:
            self.event(f'Navigation result unavailable; retrying: {exc}')
            status = GoalStatus.STATUS_UNKNOWN
        patrol_arrived = purpose == 'patrol' and (self.patrol_reached or status == GoalStatus.STATUS_SUCCEEDED)
        self.last_navigation_result = {'purpose': purpose, 'status': int(status),
                                       'cancel_requested': bool(cancelled),
                                       'patrol_arrived': patrol_arrived}
        names = {0: '결과 확인 불가', 1: '수락됨', 2: '실행 중', 3: '취소 처리 중',
                 4: '이동 성공', 5: '이동 취소', 6: '이동 실패'}
        if patrol_arrived:
            self.patrol_visits += 1
            self.event('정찰 목적지 도착: 다음 목적지를 선택합니다.')
        else:
            self.event(('정찰' if purpose == 'patrol' else '복귀') + ': ' + names.get(status, '결과 확인 불가'))
        if purpose == 'return' and self.intent == 'return' and not cancelled and status == GoalStatus.STATUS_SUCCEEDED:
            self.state = 'ALIGN'; self.align_last_key = None
        self.next_attempt = time.monotonic()+(0. if cancelled or patrol_arrived else self.p['retry_seconds'])

    def patrol_target(self):
        if self.candidates is None or not len(self.candidates):
            return None
        origin = self.grid.info.origin
        base = [origin.position.x, origin.position.y, yaw(origin.orientation)]
        current = self.pose()
        if current is None:
            return None
        for _ in range(100):
            row, col = self.candidates[random.randrange(len(self.candidates))]
            p = transform(np.array([[(col+.5)*self.grid.info.resolution, (row+.5)*self.grid.info.resolution]]), base)[0]
            if not inside_rectangle(p, self.data['rectangle']) and np.linalg.norm(p-current[:2]) > .6:
                return [*p, random.uniform(-math.pi, math.pi)]
        return None

    def match_ok(self, result):
        return (result is not None and result['rmse'] <= self.p['match_rmse']
                and result['overlap'] >= self.p['match_overlap']
                and result['observability'] >= self.p['match_observability']
                and result['condition'] <= self.p['match_max_condition'])

    def align(self):
        cloud = self.cloud(); pose = self.pose()
        if cloud is None or pose is None:
            self.stop(); self.settled_since = None; return
        key = self.scan_key()
        now = time.monotonic()
        if key != self.align_last_key:
            self.align_last_key = key
            initial = relative(self.data['goal'], pose)
            result = register_scan(cloud, self.data['reference'], initial)
            self.match = result
            valid = self.match_ok(result)
            if valid:
                correction = relative(initial, result['pose'])
                valid = (np.linalg.norm(correction[:2]) <= self.p['match_max_correction']
                         and abs(correction[2]) <= math.radians(self.p['match_max_angle_deg']))
            if not valid:
                self.align_velocity = (0., 0.); self.settled_since = None
                self.detail = 'Alignment paused: ambiguous lidar match; reobserving'
                self.stop(); return
            self.align_valid_stamp = now
            error = result['pose']
            xy = np.linalg.norm(error[:2]); heading = abs(error[2])
            satisfied = xy < self.p['xy_tolerance'] and heading <= math.radians(self.p['yaw_tolerance_deg'])
            if satisfied:
                self.align_velocity = (0., 0.)
                if self.stopped():
                    self.settled_since = self.settled_since or now
                    if now-self.settled_since >= self.p['settle_seconds']:
                        self.intent = 'hold'; self.state = 'HOLD'; self.stop()
                        self.arrival_pub.publish(Empty())
                        self.event('Arrival emitted once; holding until a new go_around command')
                        return
                else:
                    self.settled_since = None
            else:
                self.settled_since = None
                self.align_velocity = alignment_command(error, self.p['xy_tolerance'],
                    math.radians(self.p['yaw_tolerance_deg']), self.p['max_align_v'], self.p['max_align_w'])
            self.detail = f'Alignment estimate: {xy*1000:.1f} mm, {math.degrees(error[2]):.2f} deg'
        if now-self.align_valid_stamp > .2:
            self.stop(); self.settled_since = None; return
        v, w = self.align_velocity
        if (v or w) and not swept_clear(cloud, v, w, self.p['footprint_half_length'],
                self.p['footprint_half_width'], self.p['obstacle_margin']):
            self.stop(); self.settled_since = None
            self.detail = 'Alignment blocked: waiting for clearance; retry remains active'
            return
        command = Twist(); command.linear.x = float(v); command.angular.z = float(w)
        self.velocity_pub.publish(command)

    def tick(self):
        if self.mode == 'teach':
            self.stop()
            if self.state == 'CAPTURE':
                self.capture()
            return
        if self.intent == 'hold':
            self.state = 'HOLD'; self.stop(); return
        if self.manual or not self.healthy():
            self.stop(); self.cancel_nav(); self.settled_since = None
            self.detail = 'Paused: manual control or localization/sensor readiness'
            return
        if self.data is None or self.data.get('rectangle') is None:
            self.state = 'WAIT'; self.stop()
            self.detail = 'Waiting for a saved goal and rectangle; use make_goal_launch'
            return
        current_pose = self.pose()
        if current_pose is None:
            self.stop(); self.cancel_nav(); self.settled_since = None
            self.detail = 'Paused: manual control or localization/sensor readiness'
            return
        busy = self.nav_pending or self.nav_handle is not None
        if self.intent == 'return':
            distance = np.linalg.norm(relative(self.data['goal'], current_pose)[:2])
            if distance <= self.p['align_radius']:
                self.cancel_nav()
                if busy:
                    self.stop()
                else:
                    self.state = 'ALIGN'; self.align()
                return
            if self.state == 'ALIGN':
                self.align_last_key = None; self.settled_since = None
            self.state = 'RETURN'
            if not busy and time.monotonic() >= self.next_attempt:
                self.send_nav(self.data['goal'], 'return')
        else:
            self.state = 'PATROL'
            if (busy and self.nav_purpose == 'patrol' and self.nav_target is not None
                    and not self.cancel_requested
                    and np.linalg.norm(current_pose[:2]-np.asarray(self.nav_target[:2])) <= self.p['patrol_tolerance']):
                # Proximity alone completes patrol. Do not wait for yaw or settle.
                # Cancel/result handoff still prevents simultaneous controllers.
                self.patrol_reached = True
                self.cancel_nav(); self.stop()
                self.event(f"정찰 목적지 {self.p['patrol_tolerance']*100:g}cm 이내 도착: 이전 이동을 마무리하고 다음 목적지로 전환합니다.")
                return
            if not busy and time.monotonic() >= self.next_attempt:
                target = self.patrol_target()
                if target is not None:
                    self.send_nav(target, 'patrol')
                else:
                    self.next_attempt = time.monotonic()+self.p['retry_seconds']
                    self.detail = 'No valid patrol candidate; retrying'
        if (self.nav_handle is not None and not self.cancel_requested and self.nav_purpose == self.intent
                and time.monotonic()-self.nav_velocity_stamp < .2):
            self.velocity_pub.publish(self.nav_velocity)
        else:
            self.stop()

    def pose_message(self, pose):
        msg = PoseStamped(); msg.header.frame_id = 'map'; msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(pose[0]), float(pose[1])
        orientation(msg.pose, float(pose[2]))
        return msg

    def publish_status(self):
        result = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in (self.match or {}).items()}
        busy = self.nav_pending or self.nav_handle is not None
        phase = ('cancel_requested' if busy and self.cancel_requested else
                 'requesting' if self.nav_pending else 'executing' if self.nav_handle is not None else 'idle')
        pose = self.pose()
        distance = (float(np.linalg.norm(pose[:2]-np.asarray(self.nav_target[:2])))
                    if busy and pose is not None and self.nav_target is not None else None)
        msg = {'mode': self.mode, 'state': self.state, 'intent': self.intent, 'manual': self.manual,
               'ready': self.healthy(),
               'has_goal': self.data is not None, 'has_rectangle': bool(self.data and self.data.get('rectangle')),
               'detail': self.detail, 'match': result, 'time': time.time(),
               'localization': dict(self.localization_info,
                                    fresh=time.monotonic()-self.localization_stamp <= 2.),
               'navigation': {'phase': phase, 'purpose': self.nav_purpose, 'distance_m': distance,
                              'last_result': self.last_navigation_result, 'patrol_visits': self.patrol_visits,
                              'patrol_tolerance_m': self.p['patrol_tolerance']}}
        self.status_pub.publish(String(data=json.dumps(msg)))

    def publish_markers(self):
        markers = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; markers.markers.append(clear)
        def add(pose, kind, ident, color, size):
            m = Marker(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'robot' if ident < 2 else 'saved_goal' if ident < 4 else 'patrol_region'
            m.id = ident; m.type = kind; m.action = Marker.ADD
            m.pose = self.pose_message(pose).pose; m.pose.position.z = .15
            m.scale.x, m.scale.y, m.scale.z = size
            m.color.r, m.color.g, m.color.b, m.color.a = color
            markers.markers.append(m)
        pose = self.pose()
        if pose is not None:
            add(pose, Marker.CUBE, 0, (.2, .7, 1., .8), (.62, .48, .3))
            add(pose, Marker.ARROW, 1, (0., 0., 1., 1.), (.6, .08, .08))
        if self.data:
            goal = self.data['goal']; self.goal_pub.publish(self.pose_message(goal))
            add(goal, Marker.CUBE, 2, (.1, 1., .2, .4), (.62, .48, .3))
            add(goal, Marker.ARROW, 3, (0., 1., 0., 1.), (.6, .08, .08))
            rect = self.data.get('rectangle')
            if rect:
                add(rect[:3], Marker.CUBE, 4, (1., .4, 0., .25), (rect[3], rect[4], .02))
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ImpactNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop(); node.cancel_nav()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
