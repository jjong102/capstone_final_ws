"""Orin-local, real-time, sensor-only AMCL tuning. Domain isolation required."""
import os,time,json,sqlite3,math,subprocess,signal
from pathlib import Path
import numpy as np,yaml
from PIL import Image
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from lifecycle_msgs.srv import ChangeState
from third_impact.editor import yaw
from third_impact.core import transform
R=Path('/home/moonshot/capstone_final_ws');OUT=R/'diagnostics/amcl_tuning_20261103_221345';SRC=R/'src/amr_navigator/debug_log/20261103_221345_526770'
assert os.environ.get('ROS_DOMAIN_ID')=='219' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
c=sqlite3.connect(f'file:{SRC}/bag/bag_0.db3?mode=ro',uri=True)
want=['/tf_static','/tf','/rplidar1/scan_filtered','/rplidar2/scan_filtered']
topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in want}
rows=[];static=[];seedpose=None;mt=None;ot=None
for ts,i,b in c.execute('select timestamp,topic_id,data from messages where topic_id in (%s) order by timestamp'%','.join(map(str,topics))):
 n,cl=topics[i];m=deserialize_message(b,cl)
 if n=='/tf_static':static+=m.transforms;continue
 if n=='/tf':
  for t in m.transforms:
   p=t.transform;v=np.array([p.translation.x,p.translation.y,yaw(p.rotation)])
   if t.header.frame_id=='map' and t.child_frame_id=='odom':mt=v
   if t.header.frame_id=='odom' and t.child_frame_id=='base_link':ot=v
  if seedpose is None and mt is not None and ot is not None:seedpose=np.r_[transform([ot[:2]],mt)[0],mt[2]+ot[2]]
  m.transforms=[t for t in m.transforms if t.header.frame_id=='odom' and t.child_frame_id=='base_link']
  if not m.transforms:continue
 rows.append((ts/1e9,n,m))
start=rows[0][0];end=rows[-1][0]
cfg=yaml.safe_load((R/'src/amr_navigator/map/debug_map_corrected.yaml').read_text());im=np.flipud(np.array(Image.open(R/'src/amr_navigator/map'/cfg['image'])));cells=np.full(im.shape,-1,dtype=np.int8);cells[im<89]=100;cells[im>205]=0
mapmsg=OccupancyGrid();mapmsg.header.frame_id='map';mapmsg.info.width=im.shape[1];mapmsg.info.height=im.shape[0];mapmsg.info.resolution=float(cfg['resolution']);mapmsg.info.origin.position.x=float(cfg['origin'][0]);mapmsg.info.origin.position.y=float(cfg['origin'][1]);mapmsg.info.origin.orientation.w=1.;mapmsg.data=cells.ravel().tolist()
params=yaml.safe_load((R/'src/third_impact/config/nav2.yaml').read_text())['amcl']['ros__parameters'];params.update(use_sim_time=True,tf_broadcast=False,set_initial_pose=False)
variants=[('baseline',{}),('beams120',{'max_beams':120}),('beams180',{'max_beams':180}),('model120',{'max_beams':120,'sigma_hit':.15,'z_hit':.7,'z_rand':.3}),('updates120',{'max_beams':120,'update_min_d':.02,'update_min_a':.035})]
if os.environ.get('AMCL_REPEAT') == '1':
 variants=[('baseline_repeat',{}),('beams120_repeat',{'max_beams':120}),('beams180_repeat',{'max_beams':180})]
rclpy.init();node=rclpy.create_node('isolated_tuning_driver');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
mp=node.create_publisher(OccupancyGrid,'/map',q);sp=node.create_publisher(TFMessage,'/tf_static',q);tp=node.create_publisher(TFMessage,'/tf',100);cp=node.create_publisher(Clock,'/clock',10);ip=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
scanpub={s:node.create_publisher(LaserScan,s,qos_profile_sensor_data) for s in want if s.endswith('scan_filtered')}
poses=[];fusions=[];scans=[];simnow=start
node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:poses.append(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,received_sim=simnow,pose=[m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)],cov=[m.pose.covariance[i] for i in [0,7,35]])),100)
node.create_subscription(String,'/third_impact/amcl_scan_status',lambda m:fusions.append(json.loads(m.data)),100)
node.create_subscription(LaserScan,'/third_impact/amcl_scan',lambda m:scans.append(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,received_sim=simnow,valid=int(np.isfinite(m.ranges).sum()))),qos_profile_sensor_data)
def spin(d):
 until=time.monotonic()+d
 while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
def clock(t):
 global simnow
 simnow=t;m=Clock();ns=int(t*1e9);m.clock.sec=ns//10**9;m.clock.nanosec=ns%10**9;cp.publish(m)
def procstats(pid):
 try:
  s=Path(f'/proc/{pid}/stat').read_text().split();return dict(cpu=(int(s[13])+int(s[14]))/os.sysconf('SC_CLK_TCK'),rss_bytes=int(s[23])*os.sysconf('SC_PAGE_SIZE'))
 except OSError:return None
hardware={'model':Path('/proc/device-tree/model').read_bytes().decode().strip('\0'),'cpus':Path('/sys/devices/system/cpu/online').read_text().strip(),'power':subprocess.run(['nvpmodel','-q'],capture_output=True,text=True).stdout,'seed':seedpose.tolist(),'duration':end-start,'note':'Real-time sequential AMCL+fusion only. Full navigation load is not included. CPU100%=one core.'}
(OUT/'hardware.json').write_text(json.dumps(hardware,indent=2))
for name,changes in variants:
 folder=OUT/name;folder.mkdir(exist_ok=True);pp=dict(params,**changes);pf=folder/'amcl.yaml';pf.write_text(yaml.safe_dump({'amcl':{'ros__parameters':pp}}))
 poses.clear();fusions.clear();scans.clear();children=[];handles=[];cpu=[];clock(start)
 def launch(cmd,label):
  h=(folder/(label+'.log')).open('w');handles.append(h);p=subprocess.Popen(cmd,stdout=h,stderr=h,start_new_session=True);children.append(p);return p
 try:
  amcl=launch(['/opt/ros/humble/lib/nav2_amcl/amcl','--ros-args','--params-file',str(pf)],'amcl')
  fusion=launch([str(R/'install/third_impact/lib/third_impact/amcl_scan'),'--ros-args','-p','use_sim_time:=true'],'fusion')
  cl=node.create_client(ChangeState,'/amcl/change_state')
  if not cl.wait_for_service(timeout_sec=15):raise RuntimeError('AMCL unavailable')
  for st in [1,3]:
   req=ChangeState.Request();req.transition.id=st;f=cl.call_async(req);deadline=time.monotonic()+15
   while not f.done() and time.monotonic()<deadline:spin(.02)
   if not f.done() or not f.result().success:raise RuntimeError('AMCL lifecycle failed')
  mp.publish(mapmsg);sp.publish(TFMessage(transforms=static));spin(.3)
  # Warm up TF and seed at the initial mapping pose, identical for every candidate.
  for t,n,m in rows:
   if t>start+1:break
   if n=='/tf':tp.publish(m)
  spin(.2)
  seed=PoseWithCovarianceStamped();seed.header.frame_id='map';seed.pose.pose.position.x=float(seedpose[0]);seed.pose.pose.position.y=float(seedpose[1]);seed.pose.pose.orientation.z=math.sin(seedpose[2]/2);seed.pose.pose.orientation.w=math.cos(seedpose[2]/2);seed.pose.covariance[0]=.05**2;seed.pose.covariance[7]=.05**2;seed.pose.covariance[35]=math.radians(3)**2;ip.publish(seed);spin(.2)
  wall=time.monotonic();lastcpu=wall-1;maxlag=0.;print('START',name,flush=True)
  for t,n,m in rows:
   while time.monotonic()-wall<t-start:spin(.002)
   maxlag=max(maxlag,time.monotonic()-wall-(t-start));clock(t)
   if n=='/tf':tp.publish(m)
   else:scanpub[n].publish(m)
   spin(.0001)
   if time.monotonic()-lastcpu>=.5:
    now=time.monotonic();cpu.append({'wall':now,'amcl':procstats(amcl.pid),'fusion':procstats(fusion.pid),'load':os.getloadavg()});lastcpu=now
  spin(.5)
  (folder/'result.json').write_text(json.dumps(dict(poses=poses,fusion_status=fusions,scans=scans,cpu=cpu,max_replay_lag=maxlag),indent=2));print('DONE',name,len(poses),'poses',len(scans),'scans','lag',maxlag,flush=True)
 finally:
  for p in children:
   if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  for p in children:
   try:p.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for h in handles:h.close()
  if 'cl' in locals():node.destroy_client(cl)
  spin(.2)
node.destroy_node();rclpy.shutdown()
