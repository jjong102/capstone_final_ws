"""Isolated, sensor-only Cartographer replay. Never starts motor/sensor hardware."""
import os,sqlite3,subprocess,time,signal,json
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message,serialize_message
from rosidl_runtime_py.utilities import get_message
ROOT=Path('/home/moonshot/capstone_final_ws')
SRC=ROOT/'src/amr_navigator/debug_log/20261102_185900_195315'
OUT=ROOT/'diagnostics/mapping_20261102_185900/replay'
OUT.mkdir(exist_ok=True)
db=sqlite3.connect(f'file:{SRC}/bag/bag_0.db3?mode=ro',uri=True)
topics={r[0]:(r[1],r[2]) for r in db.execute('select id,name,type from topics')}
for variant in ['baseline','geometry','geometry_scanmatch']:
 dest=OUT/variant;dest.mkdir(exist_ok=True)
 bag=dest/'input'
 scale=1. if variant=='baseline' else .13/.17
 if not bag.exists():
  writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
  keep={i:(n,t) for i,(n,t) in topics.items() if n in ['/tf_static','/tf','/odom','/rplidar1/scan_filtered']}
  for n,t in keep.values():writer.create_topic(rosbag2_py.TopicMetadata(name=n,type=t,serialization_format='cdr'))
  classes={i:get_message(t) for i,(n,t) in keep.items()}
  for tid,stamp,data in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
   if tid not in keep:continue
   n,_=keep[tid];m=deserialize_message(data,classes[tid])
   if n=='/tf':
    m.transforms=[x for x in m.transforms if x.header.frame_id=='odom' and x.child_frame_id=='base_link']
    if not m.transforms:continue
    for x in m.transforms:x.transform.translation.x*=scale;x.transform.translation.y*=scale
   if n=='/odom':
    m.pose.pose.position.x*=scale;m.pose.pose.position.y*=scale;m.twist.twist.linear.x*=scale;m.twist.twist.linear.y*=scale
   writer.write(n,serialize_message(m),stamp)
  del writer
 config=(SRC/'config/amr_cartographer/config/amr.lua').read_text()
 if variant=='geometry_scanmatch':
  config=config.replace('use_online_correlative_scan_matching = false','use_online_correlative_scan_matching = true').replace('rotation_weight = 250.','rotation_weight = 40.')
 (dest/'amr.lua').write_text(config)
 # Snapshot resulting maps at shutdown with map_saver_cli; all processes on private domain.
 env=os.environ.copy();env.update(ROS_DOMAIN_ID='174',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(dest/'ros_logs'))
 children=[];handles=[]
 def start(args,name):
  out=(dest/(name+'.log')).open('w');handles.append(out)
  p=subprocess.Popen(args,env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
 try:
  cart=start(['/opt/ros/humble/lib/cartographer_ros/cartographer_node','-configuration_directory',str(dest),'-configuration_basename','amr.lua','--ros-args','-p','use_sim_time:=true','-r','scan:=/rplidar1/scan_filtered'],'cartographer')
  grid=start(['/opt/ros/humble/lib/cartographer_ros/cartographer_occupancy_grid_node','-resolution','0.03','--ros-args','-p','use_sim_time:=true'],'grid')
  time.sleep(2)
  play=start(['ros2','bag','play',str(bag),'--clock','100','--rate','1.0','--delay','2','--qos-profile-overrides-path',str(SRC/'bag_qos.yaml')],'play')
  play.wait(timeout=70);time.sleep(3)
  cmd=['ros2','run','nav2_map_server','map_saver_cli','-f',str(dest/'map'),'--ros-args','-p','save_map_timeout:=10.0']
  with (dest/'save.log').open('w') as out:subprocess.run(cmd,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=18)
  print(variant,'map saved:',(dest/'map.pgm').exists(),flush=True)
 finally:
  for p in children:
   if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  for p in children:
   try:p.wait(timeout=15)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for h in handles:h.close()
