"""Isolated offline AMCL comparison; never publishes robot command topics."""
import os, time, json, sqlite3, math, subprocess, copy
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from lifecycle_msgs.srv import ChangeState
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from scipy.spatial import cKDTree
from third_impact.core import transform,voxel
from third_impact.editor import yaw
from third_impact.localization_search import GridModel
ROOT=Path('src/third_impact/debug_log/20261103_174827_039173');OUT=Path(os.environ.get('AMCL_COMPARE_OUT','diagnostics/amcl_compare_20261103'));OUT.mkdir(parents=True,exist_ok=True)
c=sqlite3.connect(f'file:{ROOT}/bag/bag_0.db3?mode=ro',uri=True)
topics={i:(n,t) for i,n,t in c.execute('select id,name,type from topics')}
wanted={'/map','/tf','/tf_static','/initialpose','/rplidar1/scan','/rplidar1/scan_filtered','/rplidar2/scan_filtered'}
rows=[];classes={};static={};mapmsg=None
for ts,tid,b in c.execute('select timestamp,topic_id,data from messages order by timestamp'):
 n,typ=topics[tid]
 if n not in wanted:continue
 if typ not in classes:classes[typ]=get_message(typ)
 m=deserialize_message(b,classes[typ]);rows.append((ts/1e9,n,m))
 if n=='/tf_static':
  for tf in m.transforms:static[tf.child_frame_id]=tf
 if n=='/map' and mapmsg is None:mapmsg=m
seed=next((t,m) for t,n,m in rows if n=='/initialpose');start=seed[0];end=rows[-1][0]
print('SEED',start,seed[1].pose.pose,'duration',end-start,flush=True)
params=yaml.safe_load((ROOT/'amcl.yaml').read_text())['/amcl']['ros__parameters']
params.update(use_sim_time=True,tf_broadcast=False,scan_topic='scan',map_topic='/map',save_pose_rate=.5)
paramfile=OUT/'params.yaml';paramfile.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
rclpy.init();node=Node('offline_amcl_driver')
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
mp=node.create_publisher(OccupancyGrid,'/map',q);sp=node.create_publisher(TFMessage,'/tf_static',q);tp=node.create_publisher(TFMessage,'/tf',100);cp=node.create_publisher(Clock,'/clock',10)
names=['front','rear','combined','original_front'];processes=[];outputs={k:[] for k in names};scanpub={};seedpub={};subs=[]
for name in names:
 log=(OUT/(name+'.log')).open('w')
 processes.append(subprocess.Popen(['/opt/ros/humble/lib/nav2_amcl/amcl','--ros-args','-r',f'__ns:=/eval/{name}','--params-file',str(paramfile.resolve())],stdout=log,stderr=log))
 scanpub[name]=node.create_publisher(LaserScan,f'/eval/{name}/scan',qos_profile_sensor_data)
 seedpub[name]=node.create_publisher(PoseWithCovarianceStamped,f'/eval/{name}/initialpose',10)
 subs.append(node.create_subscription(PoseWithCovarianceStamped,f'/eval/{name}/amcl_pose',lambda m,k=name:outputs[k].append(m),10))
fusion_log=(OUT/'fusion.log').open('w')
processes.append(subprocess.Popen(['python3','-c','from third_impact.amcl_scan import main; main()', '--ros-args','-p','use_sim_time:=true'], stdout=fusion_log, stderr=fusion_log))
inputpub={s:node.create_publisher(LaserScan,f'/rplidar{i}/scan_filtered',qos_profile_sensor_data) for s,i in [('front',1),('rear',2)]}
fusion_counts={}
def fusion_status(m):
 d=json.loads(m.data);reason=d['reason'];fusion_counts[reason]=fusion_counts.get(reason,0)+1
from std_msgs.msg import String
subs.append(node.create_subscription(String,'/third_impact/amcl_scan_status',fusion_status,10))
subs.append(node.create_subscription(LaserScan,'/third_impact/amcl_scan',scanpub['combined'].publish,qos_profile_sensor_data))
def spin(duration):
 until=time.monotonic()+duration
 while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
def clock(t):
 m=Clock();m.clock.sec=int(t);m.clock.nanosec=int((t-int(t))*1e9);cp.publish(m)
def points(m):
 tf=static[m.header.frame_id].transform;q=tf.rotation;x,y,z,w=q.x,q.y,q.z,q.w
 R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w)],[2*(x*y+z*w),1-2*(x*x+z*z)]])
 r=np.array(m.ranges);a=m.angle_min+np.arange(len(r))*m.angle_increment
 ok=np.isfinite(r)&(r>max(.1,m.range_min))&(r<min(6.,m.range_max))
 xy=np.c_[r[ok]*np.cos(a[ok]),r[ok]*np.sin(a[ok])]@R.T+[tf.translation.x,tf.translation.y]
 return xy[(abs(xy[:,0])>.35)|(abs(xy[:,1])>.22)]
def virtual(cloud,stamp):
 m=LaserScan();m.header.frame_id='base_link';m.header.stamp=stamp;m.angle_min=-math.pi;m.angle_increment=2*math.pi/720;m.angle_max=m.angle_min+719*m.angle_increment;m.range_min=.1;m.range_max=6.;m.scan_time=.1
 r=np.linalg.norm(cloud,axis=1);a=np.arctan2(cloud[:,1],cloud[:,0]);idx=np.floor((a+math.pi)/m.angle_increment).astype(int)%720
 bins=np.full(720,np.inf);np.minimum.at(bins,idx,r);m.ranges=bins.astype(np.float32).tolist();return m
try:
 clock(start);mp.publish(mapmsg);sp.publish(TFMessage(transforms=list(static.values())))
 for name in names:
  client=node.create_client(ChangeState,f'/eval/{name}/amcl/change_state')
  if not client.wait_for_service(timeout_sec=10):raise RuntimeError('no lifecycle '+name)
  for state in [1,3]:
   req=ChangeState.Request();req.transition.id=state;f=client.call_async(req)
   deadline=time.monotonic()+10
   while not f.done() and time.monotonic()<deadline:spin(.02)
   if not f.done() or not f.result().success:raise RuntimeError('lifecycle '+name)
 spin(.5);mp.publish(mapmsg);sp.publish(TFMessage(transforms=list(static.values())))
 # Preload odom TF history before common seed. Never replay recorded map->odom.
 for t,n,m in rows:
  if t>=start:break
  if n=='/tf':tp.publish(TFMessage(transforms=[x for x in m.transforms if x.header.frame_id=='odom']))
 spin(.3);clock(start);spin(.1)
 for p in seedpub.values():p.publish(seed[1])
 spin(.3);wall=time.monotonic();clouds={};evaluation=[];lastprint=-1
 for t,n,m in rows:
  if t<start:continue
  while time.monotonic()-wall<t-start:spin(.002)
  clock(t)
  if n=='/tf':tp.publish(TFMessage(transforms=[x for x in m.transforms if x.header.frame_id=='odom']))
  elif n=='/rplidar1/scan':scanpub['original_front'].publish(m)
  elif n.endswith('scan_filtered'):
   side='front' if 'rplidar1' in n else 'rear';clouds[side]=(m.header.stamp,points(m))
   scanpub[side].publish(virtual(clouds[side][1],m.header.stamp));inputpub[side].publish(m)
   if side=='front' and 'rear' in clouds:
    rs=clouds['rear'][0];delta=abs(m.header.stamp.sec+ m.header.stamp.nanosec/1e9-rs.sec-rs.nanosec/1e9)
    if delta<=.15:
     pass # Actual production fusion output is forwarded by subscription
     evaluation.append((m.header.stamp.sec+m.header.stamp.nanosec/1e9,clouds['front'][1],clouds['rear'][1]))
  spin(.001)
  if int(t-start)//20>lastprint:lastprint=int(t-start)//20;print('progress',int(t-start),{k:len(v) for k,v in outputs.items()},flush=True)
 spin(1.)
 origin=mapmsg.info.origin;model=GridModel(np.array(mapmsg.data).reshape(mapmsg.info.height,mapmsg.info.width),mapmsg.info.resolution,[origin.position.x,origin.position.y,yaw(origin.orientation)])
 tree=cKDTree(model.reference);stamps=np.array([x[0] for x in evaluation]);summary={};details={}
 for name,msgs in outputs.items():
  results=[]
  for m in msgs:
   t=m.header.stamp.sec+m.header.stamp.nanosec/1e9;i=int(np.argmin(abs(stamps-t)))
   if abs(stamps[i]-t)>.2:continue
   p=m.pose.pose;wp=[p.position.x,p.position.y,yaw(p.orientation)];lp=model.local_pose(wp);r={'time':t,'pose':wp}
   for j,side in [(1,'front'),(2,'rear')]:
    dist=tree.query(transform(evaluation[i][j],lp))[0];r[side+'_overlap']=float(np.mean(dist<.12));r[side+'_median_m']=float(np.median(dist))
   results.append(r)
  details[name]=results
  summary[name]={'updates':len(msgs),'evaluated':len(results),'metrics':{k:float(np.mean([r[k] for r in results])) for k in ['front_overlap','rear_overlap','front_median_m','rear_median_m']} if results else {}}
 (OUT/'results.json').write_text(json.dumps({'seed_time':start,'summary':summary,'trajectories':details},indent=2));print(json.dumps(summary,indent=2),flush=True)
finally:
 (OUT/'fusion_status.json').write_text(json.dumps(fusion_counts,indent=2))
 for proc in processes:proc.terminate()
 for proc in processes:
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
 node.destroy_node();rclpy.shutdown()
