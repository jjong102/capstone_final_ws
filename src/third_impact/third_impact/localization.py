"""Autonomous stationary localization bootstrap. Never publishes velocity commands."""
import json
import math
from pathlib import Path
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from std_srvs.srv import Empty
from lifecycle_msgs.srv import GetState
from .core import atomic_save, relative, voxel
from .editor import orientation, yaw
from .localization_search import GridModel, search_map


class LocalizationBootstrap(Node):
    def __init__(self):
        super().__init__('third_impact_localization')
        self.file = Path(self.declare_parameter('pose_file', '~/.ros/third_impact/last_pose.json').value).expanduser()
        self.front = self.declare_parameter('front_scan', '/rplidar1/scan_filtered').value
        self.rear = self.declare_parameter('rear_scan', '/rplidar2/scan_filtered').value
        self.search_timeout = float(self.declare_parameter('search_timeout', 20.).value)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.model = None; self.scans = {}; self.odom = None; self.odom_time = 0.
        self.amcl = None; self.amcl_time = 0.; self.amcl_active = False
        self.state = 'WAIT_MAP'; self.reason = '지도를 기다립니다.'; self.ready = False
        self.epoch = 0; self.future = None; self.job_epoch = -1; self.job_odom = None
        self.stop_search = threading.Event(); self.pool = ThreadPoolExecutor(max_workers=1)
        self.next_search = 0.; self.last_save = 0.; self.last_nomotion = 0.
        self.last_seed = None; self.seed_time = 0.; self.verify_since = None; self.verify_count = 0
        self.last_verified_key = None; self.last_quality = {}; self.last_search = {}; self.prior = None
        self.bad_checks = 0; self.get_state_pending = False; self.nomotion_pending = False
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/third_impact/localization', durable)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, durable)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        for key, topic in [('front', self.front), ('rear', self.rear)]:
            self.create_subscription(LaserScan, topic, lambda msg,k=key:self.on_scan(k,msg), qos_profile_sensor_data)
        self.lifecycle = self.create_client(GetState, '/amcl/get_state')
        self.nomotion = self.create_client(Empty, '/request_nomotion_update')
        self.create_timer(1., self.check_amcl)
        self.create_timer(.2, self.tick)
        self.create_timer(.2, self.publish_status)

    def set_state(self, state, reason, ready=False):
        if state != self.state:
            self.get_logger().info(reason)
        self.state, self.reason, self.ready = state, reason, ready

    def cancel_search(self):
        self.epoch += 1; self.stop_search.set()

    def on_map(self, msg):
        cells = np.asarray(msg.data).reshape(msg.info.height,msg.info.width)
        origin = [msg.info.origin.position.x,msg.info.origin.position.y,yaw(msg.info.origin.orientation)]
        model = GridModel(cells,msg.info.resolution,origin)
        if self.model is not None and self.model.fingerprint == model.fingerprint:
            return
        self.cancel_search(); self.model = model; self.verify_since = None; self.verify_count = 0
        self.last_verified_key = None; self.prior = None; self.next_search = 0.
        self.set_state('WAIT_SENSORS','현재 지도에서 위치를 찾기 위해 LiDAR와 odometry를 기다립니다.')
        try:
            data = json.loads(self.file.read_text())
            pose = np.asarray(data['pose'],dtype=float)
            if data['map_fingerprint']==model.fingerprint and pose.shape==(3,) and np.isfinite(pose).all():
                self.prior = pose
        except (OSError, ValueError, KeyError, TypeError):
            pass

    def on_odom(self,msg):
        self.odom,self.odom_time=msg,time.monotonic()

    def on_amcl(self,msg):
        self.amcl,self.amcl_time=msg,time.monotonic()

    def on_initialpose(self,msg):
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        if stamp==self.last_seed:
            return
        self.cancel_search(); self.verify_since=None; self.verify_count=0; self.seed_time=time.monotonic()
        self.set_state('VERIFYING','사용자가 지정한 초기 위치를 LiDAR와 대조하고 있습니다.')

    def age(self,stamp):
        return (self.get_clock().now().nanoseconds-Time.from_msg(stamp).nanoseconds)*1e-9

    def on_scan(self,key,msg):
        if not 0. <= self.age(msg.header.stamp) <= .5:
            return
        try:
            tf=self.tf.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        q=tf.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
        rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w)],[2*(x*y+z*w),1-2*(x*x+z*z)]])
        ranges=np.asarray(msg.ranges); angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
        valid=np.isfinite(ranges)&(ranges>max(.1,msg.range_min))&(ranges<min(6.,msg.range_max))
        points=np.column_stack((ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])))
        t=tf.transform.translation; points=points@rotation.T+[t.x,t.y]
        points=points[(abs(points[:,0])>.31)|(abs(points[:,1])>.24)]
        self.scans[key]=(time.monotonic(),Time.from_msg(msg.header.stamp).nanoseconds,points)

    def cloud(self):
        now=time.monotonic()
        if any(k not in self.scans or now-self.scans[k][0]>.5 for k in ('front','rear')):
            return None
        if abs(self.scans['front'][1]-self.scans['rear'][1])*1e-9>.15:
            return None
        cloud=voxel(np.vstack([self.scans[k][2] for k in ('front','rear')]),.025)
        return cloud if len(cloud)>=80 else None

    def odom_pose(self):
        p=self.odom.pose.pose
        return np.array([p.position.x,p.position.y,yaw(p.orientation)])

    def stopped(self):
        if self.odom is None or time.monotonic()-self.odom_time>.4:
            return False
        t=self.odom.twist.twist
        return math.hypot(t.linear.x,t.linear.y)<.01 and abs(t.angular.z)<.02

    def pose(self):
        if self.amcl is None or self.amcl_time<self.seed_time:
            return None
        cov=self.amcl.pose.covariance
        if not np.isfinite(cov).all() or max(cov[0],cov[7])>.08 or cov[35]>.08:
            return None
        try:
            tf=self.tf.lookup_transform('map','base_link',Time())
            if not 0. <= self.age(tf.header.stamp) <= 1.:
                return None
            t=tf.transform.translation
            return np.array([t.x,t.y,yaw(tf.transform.rotation)])
        except TransformException:
            return None

    def check_amcl(self):
        if not self.lifecycle.service_is_ready():
            self.amcl_active=False
        if not self.get_state_pending and self.lifecycle.service_is_ready():
            self.get_state_pending=True
            self.lifecycle.call_async(GetState.Request()).add_done_callback(self.amcl_state)
        if (self.state in ('INITIALIZING','VERIFYING') and not self.nomotion_pending
                and self.nomotion.service_is_ready() and time.monotonic()-self.last_nomotion>1.):
            self.last_nomotion=time.monotonic(); self.nomotion_pending=True
            self.nomotion.call_async(Empty.Request()).add_done_callback(self.nomotion_done)

    def amcl_state(self,future):
        self.get_state_pending=False
        try:
            self.amcl_active=future.result().current_state.id==3
        except Exception:
            self.amcl_active=False

    def nomotion_done(self,future):
        self.nomotion_pending=False
        try: future.result()
        except Exception: pass

    def seed(self,pose):
        msg=PoseWithCovarianceStamped(); msg.header.frame_id='map'
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.pose.position.x=float(pose[0]); msg.pose.pose.position.y=float(pose[1])
        orientation(msg.pose.pose,float(pose[2]))
        msg.pose.covariance[0]=msg.pose.covariance[7]=.15**2; msg.pose.covariance[35]=.1**2
        self.last_seed=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        self.seed_time=time.monotonic(); self.verify_since=None; self.verify_count=0
        self.initial_pub.publish(msg)
        self.set_state('INITIALIZING','지도 전체에서 찾은 위치로 AMCL을 초기화하고 검증합니다.')

    def tick(self):
        now=time.monotonic()
        if self.model is None:
            self.set_state('WAIT_MAP','지도를 기다립니다.'); return
        cloud=self.cloud()
        if cloud is None or self.odom is None or now-self.odom_time>.4:
            self.verify_since=None; self.verify_count=0
            self.set_state('WAIT_SENSORS','전후방 LiDAR, 센서 TF 또는 odometry가 아직 준비되지 않았습니다.'); return
        if not self.amcl_active:
            self.set_state('WAIT_AMCL','AMCL 노드가 활성화되기를 기다립니다.'); return
        pose=self.pose()
        key=(self.scans['front'][1],self.scans['rear'][1])
        if pose is not None:
            if self.ready and not self.stopped():
                self.set_state('READY','초기 위치 확인 완료 · AMCL이 현재 위치를 추적하고 있습니다.',True)
                return
            if key != self.last_verified_key:
                self.last_verified_key=key
                quality=self.model.verify(cloud,pose); self.last_quality=quality
                if quality['valid']:
                    self.bad_checks=0
                    self.verify_since=self.verify_since or now
                    self.verify_count+=1
                    if self.verify_count>=3 and now-self.verify_since>=1.:
                        self.cancel_search() if self.future is not None and not self.future.done() else None
                        self.set_state('READY','현재 LiDAR와 지도 일치 확인 완료 · 주행 준비됨',True)
                        if now-self.last_save>5.:
                            try:
                                atomic_save(self.file,{'map_fingerprint':self.model.fingerprint,
                                                       'pose':pose.tolist(),'saved_at':time.time()})
                                self.prior=pose; self.last_save=now
                            except OSError as exc:
                                self.get_logger().warning('Last-pose hint save failed: '+str(exc))
                        return
                    self.set_state('VERIFYING','현재 위치가 지도와 일치하는지 연속 관측으로 확인합니다.')
                    return
                self.verify_since=None; self.verify_count=0; self.bad_checks+=1
                if self.ready and self.bad_checks<3:
                    return
            elif self.ready:
                return
        self.ready=False
        if not self.stopped():
            self.set_state('WAIT_STOP','자동 위치 탐색을 위해 로봇이 정지하기를 기다립니다.'); return
        if self.future is not None:
            if not self.future.done():
                self.set_state('SEARCHING','현재 LiDAR를 지도 전체와 비교해 위치를 찾는 중입니다.'); return
            future=self.future; self.future=None
            if self.job_epoch!=self.epoch:
                return
            delta=relative(self.job_odom,self.odom_pose())
            if np.linalg.norm(delta[:2])>.02 or abs(delta[2])>.05:
                self.next_search=now+1.; self.set_state('WAIT_STOP','탐색 중 로봇이 움직여 현재 관측으로 다시 찾습니다.'); return
            try:
                result=future.result()
            except Exception as exc:
                self.get_logger().error('Global search failed: '+str(exc))
                self.next_search=now+3.; self.set_state('SEARCH_FAILED','위치 탐색 처리 실패 · 다시 시도합니다.'); return
            self.last_search={k:v for k,v in result.items() if k!='pose'}
            if result['confident'] and self.model.verify(cloud,result['pose'])['valid']:
                self.seed(result['pose']); return
            reasons={'ambiguous':'비슷한 위치 후보가 여러 개입니다. 정지 관측으로 재시도하며, 필요하면 초기 위치를 지정해 주세요.',
                     'no_consistent_pose':'LiDAR와 충분히 일치하는 지도 위치를 찾지 못했습니다. 주변 환경과 지도를 확인하며 재시도합니다.',
                     'search_interrupted':'자동 탐색 제한 시간에 도달했습니다. 다시 시도합니다.',
                     'map_too_large':'현재 전체 지도 탐색 범위를 초과했습니다. 초기 위치 지정이 필요합니다.',
                     'insufficient_geometry':'위치를 구분할 LiDAR 관측 또는 지도 형상이 부족합니다.'}
            self.set_state('AMBIGUOUS' if result['reason']=='ambiguous' else 'SEARCH_FAILED',
                           reasons.get(result['reason'],'관측이 바뀌어 위치 후보를 다시 확인합니다.'))
            self.next_search=now+4.; return
        if self.seed_time and now-self.seed_time<12.:
            self.set_state('VERIFYING','AMCL 초기 위치 적용과 LiDAR 일치 검증을 기다립니다.'); return
        if now<self.next_search:
            return
        self.stop_search=threading.Event(); self.job_epoch=self.epoch; self.job_odom=self.odom_pose()
        self.future=self.pool.submit(search_map,self.model,cloud,self.prior,self.search_timeout,self.stop_search)
        self.set_state('SEARCHING','현재 LiDAR를 지도 전체와 비교해 위치를 찾는 중입니다.')

    def publish_status(self):
        data={'state':self.state,'ready':self.ready,'reason':self.reason,
              'quality':self.last_quality,'search':self.last_search}
        self.status_pub.publish(String(data=json.dumps(data)))

    def close(self):
        self.stop_search.set(); self.pool.shutdown(wait=True,cancel_futures=True)


def main(args=None):
    rclpy.init(args=args); node=LocalizationBootstrap()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
