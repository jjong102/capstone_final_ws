"""Sensor-only comparison on domain 218, localhost. Never publishes motor topics."""
import os,sqlite3,subprocess,time,signal,json
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message,serialize_message
from rosidl_runtime_py.utilities import get_message
R=Path('/home/moonshot/capstone_final_ws');src=R/'src/amr_navigator/debug_log/20261103_215252_874737';out=R/'diagnostics/mapping_accuracy_20261103_215252/replay';out.mkdir(exist_ok=True)
c=sqlite3.connect(f'file:{src}/bag/bag_0.db3?mode=ro',uri=True)
topics={i:(n,t) for i,n,t in c.execute('select id,name,type from topics') if n in ['/tf_static','/tf','/odom','/rplidar1/scan_filtered','/rplidar2/scan_filtered']}
collector='''
import rclpy,time,json,sys
from pathlib import Path
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from rclpy.serialization import serialize_message
rclpy.init();n=rclpy.create_node('offline_capture');d=Path(sys.argv[1]);rows=[]
def mt(m): (d/'map.cdr').write_bytes(serialize_message(m))
def tt(m):
 for t in m.transforms:
  if t.header.frame_id=='map' and t.child_frame_id=='odom':
   v=t.transform;rows.append([t.header.stamp.sec+t.header.stamp.nanosec/1e9,v.translation.x,v.translation.y,v.rotation.z,v.rotation.w])
n.create_subscription(OccupancyGrid,'/map',mt,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(TFMessage,'/tf',tt,100)
try:
 while rclpy.ok():rclpy.spin_once(n,timeout_sec=.1)
except KeyboardInterrupt:pass
finally:(d/'tf.json').write_text(json.dumps(rows))
'''
for variant in ['baseline','instantaneous']:
 dest=out/variant;dest.mkdir(exist_ok=True);bag=dest/'input'
 if not bag.exists():
  w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
  for n,t in topics.values():w.create_topic(rosbag2_py.TopicMetadata(name=n,type=t,serialization_format='cdr'))
  for tid,stamp,b in c.execute('select topic_id,timestamp,data from messages where topic_id in (%s) order by timestamp'%','.join(map(str,topics))):
   n,t=topics[tid];m=deserialize_message(b,get_message(t))
   if n=='/tf':
    m.transforms=[x for x in m.transforms if x.header.frame_id=='odom' and x.child_frame_id=='base_link']
    if not m.transforms:continue
   if variant=='instantaneous' and n.endswith('scan_filtered'):m.time_increment=0.
   w.write(n,serialize_message(m),stamp)
  del w
 (dest/'amr.lua').write_text((src/'config/amr_cartographer/config/amr.lua').read_text())
 env=dict(os.environ,ROS_DOMAIN_ID='218',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>',ROS_LOG_DIR=str(dest/'ros_logs'))
 children=[];handles=[]
 def start(cmd,name):
  h=(dest/(name+'.log')).open('w');handles.append(h);p=subprocess.Popen(cmd,env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
 try:
  start(['/opt/ros/humble/lib/cartographer_ros/cartographer_node','-configuration_directory',str(dest),'-configuration_basename','amr.lua','--ros-args','-p','use_sim_time:=true','-r','scan_1:=/rplidar1/scan_filtered','-r','scan_2:=/rplidar2/scan_filtered'],'cartographer')
  start(['/opt/ros/humble/lib/cartographer_ros/cartographer_occupancy_grid_node','-resolution','0.03','-publish_period_sec','0.5','--ros-args','-p','use_sim_time:=true'],'grid')
  start(['python3','-c',collector,str(dest)],'capture');time.sleep(2)
  play=start(['ros2','bag','play',str(bag),'--clock','100','--rate','1.0','--delay','2','--qos-profile-overrides-path',str(src/'bag_qos.yaml')],'play')
  print('START',variant,flush=True);play.wait(timeout=100);time.sleep(2)
 finally:
  for p in reversed(children):
   if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  for p in children:
   try:p.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for h in handles:h.close()
 print('DONE',variant,'map',(dest/'map.cdr').exists(),flush=True)
