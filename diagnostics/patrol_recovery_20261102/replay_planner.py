import os,sys,time,subprocess,sqlite3,json,copy,signal
from pathlib import Path
import numpy as np,yaml
sys.path.insert(0,str(Path.cwd()/'src/third_impact'))
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped,PoseStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from third_impact.core import region_map,relative
os.environ.update(ROS_DOMAIN_ID='195',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR='/tmp/patrol_planner_logs')
os.environ.pop('CYCLONEDDS_URI',None)
p=Path('src/third_impact/debug_log/20261102_210423_905315')
c=sqlite3.connect(str(p/'bag/bag_0.db3'));i=c.execute("select id from topics where name='/map'").fetchone()[0]
m=deserialize_message(c.execute('select data from messages where topic_id=? limit 1',(i,)).fetchone()[0],OccupancyGrid)
z=json.loads((p/'goal.json').read_text());cfg=yaml.safe_load(Path('src/third_impact/config/nav2.yaml').read_text())
cfg['global_costmap']['global_costmap']['ros__parameters']['plugins']=['static_layer','inflation_layer']
Path('/tmp/patrol_planner_params.yaml').write_text(yaml.safe_dump(cfg))
log=open('/tmp/patrol_planner_process.log','w')
proc=subprocess.Popen(['/opt/ros/humble/lib/nav2_planner/planner_server','--ros-args','--params-file','/tmp/patrol_planner_params.yaml'],stdout=log,stderr=log)
rclpy.init();n=rclpy.create_node('patrol_planner_replay');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
pub=n.create_publisher(OccupancyGrid,'/third_impact/planning_map',q)
tf=StaticTransformBroadcaster(n);tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='base_link';tr.header.stamp=n.get_clock().now().to_msg();tr.transform.translation.x=1.23758065;tr.transform.translation.y=.55028018;tr.transform.rotation.w=1.;tf.sendTransform(tr)
def wait(f,seconds=20):
 rclpy.spin_until_future_complete(n,f,timeout_sec=seconds)
 if not f.done():raise RuntimeError('timeout')
 return f.result()
def transition(id):
 cli=n.create_client(ChangeState,'/planner_server/change_state')
 if not cli.wait_for_service(timeout_sec=15):raise RuntimeError('lifecycle missing')
 req=ChangeState.Request();req.transition.id=id
 assert wait(cli.call_async(req)).success
try:
 cells=np.asarray(m.data).reshape(m.info.height,m.info.width);origin=[m.info.origin.position.x,m.info.origin.position.y,0]
 m.data=region_map(cells,m.info.resolution,origin,z['boundary'],z['rectangle'],.3434005322,[1.23758065,.55028018]).astype(np.int8).ravel().tolist()
 m.header.stamp=n.get_clock().now().to_msg();pub.publish(m)
 transition(1);transition(3)
 for _ in range(20):rclpy.spin_once(n,timeout_sec=.05)
 client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');assert client.wait_for_server(timeout_sec=10)
 g=ComputePathToPose.Goal();g.use_start=True;g.planner_id='GridBased'
 for msg,x,y in [(g.start,1.23758065,.55028018),(g.goal,1.4949998823,3.2949997817)]:
  msg.header.frame_id='map';msg.header.stamp=n.get_clock().now().to_msg();msg.pose.position.x=x;msg.pose.position.y=y;msg.pose.orientation.w=1.
 h=wait(client.send_goal_async(g));assert h.accepted
 result=wait(h.get_result_async());print('action_status',result.status)
 points=np.array([[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]);assert len(points)>2
 rect=z['rectangle'];dist=[]
 for pt in points:
  loc=relative(rect[:3],[*pt,0]);dist.append(np.linalg.norm(np.maximum(np.abs(loc[:2])-np.array(rect[3:])/2,0)))
 print('path_points',len(points),'min_center_clearance',min(dist),'clearance_after_first_30cm',min(d for pt,d in zip(points,dist) if np.linalg.norm(pt-points[0])>.3))
 np.savez('/tmp/patrol_replanned_path.npz',points=points)
finally:
 n.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=8)
 except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
 log.close()
