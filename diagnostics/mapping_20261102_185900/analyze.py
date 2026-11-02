from pathlib import Path
import sqlite3,json,math
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
B=Path('src/amr_navigator/debug_log/20261102_185900_195315')
OUT=Path('diagnostics/mapping_20261102_185900')
db=sqlite3.connect(f'file:{B}/bag/bag_0.db3?mode=ro',uri=True)
topics={r[0]:(r[1],r[2]) for r in db.execute('select id,name,type from topics')}
data={}
selected=['/imu/data','/ebimu_data','/wheel/odometry','/odom','/tf','/tf_static','/robot_description','/rplidar1/scan','/rplidar1/scan_filtered','/rplidar2/scan','/diagnostics','/map']
for tid,(name,typ) in topics.items():
 if name in selected:
  cls=get_message(typ)
  data[name]=[(t/1e9,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
def stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec/1e9
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stats(x): return np.percentile(x,[0,5,50,95,100]).round(5).tolist()
report={}
arrays={}
for name,rows in data.items():
 if not rows: continue
 r={'count':len(rows),'duration':rows[-1][0]-rows[0][0]}
 if hasattr(rows[0][1],'header'):
  ts=np.array([stamp(m) for t,m in rows]);r['header_dt']=stats(np.diff(ts));r['receipt_minus_stamp']=stats([t-stamp(m) for t,m in rows])
 if name in ['/odom','/wheel/odometry']:
  a=np.array([[stamp(m),m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation),m.twist.twist.linear.x,m.twist.twist.angular.z] for t,m in rows]);a[:,3]=np.unwrap(a[:,3]);arrays[name]=a
  r.update({'yaw_change_deg':float(np.rad2deg(a[-1,3]-a[0,3])),'v':stats(a[:,4]),'w':stats(a[:,5]),'integral_v':float(np.trapz(a[:,4],a[:,0])),'integral_w_deg':float(np.rad2deg(np.trapz(a[:,5],a[:,0])))})
 if name=='/imu/data':
  a=np.array([[stamp(m),yaw(m.orientation),m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z] for t,m in rows]);a[:,1]=np.unwrap(a[:,1]);arrays[name]=a
  r.update({'yaw_change_deg':float(np.rad2deg(a[-1,1]-a[0,1])),'gyro_xyz':[stats(a[:,i]) for i in (2,3,4)],'gyro_integral_deg':np.rad2deg(np.trapz(a[:,2:5],a[:,0],axis=0)).tolist(),'accel_xyz':[stats(a[:,i]) for i in (5,6,7)]})
 if 'scan' in name:
  r.update({'scan_time':stats([m.scan_time for t,m in rows]),'time_increment':stats([m.time_increment for t,m in rows]),'N':stats([len(m.ranges) for t,m in rows]),'frame':rows[0][1].header.frame_id,'angle':[rows[0][1].angle_min,rows[0][1].angle_max]})
 if name=='/ebimu_data': r['samples']=[rows[i][1].data for i in [0,len(rows)//3,2*len(rows)//3,-1]]
 if name=='/tf_static':
  r['transforms']=[{'parent':x.header.frame_id,'child':x.child_frame_id,'t':[x.transform.translation.x,x.transform.translation.y,x.transform.translation.z],'q':[x.transform.rotation.x,x.transform.rotation.y,x.transform.rotation.z,x.transform.rotation.w]} for t,m in rows for x in m.transforms]
 if name=='/tf':
  pairs={}
  for t,m in rows:
   for x in m.transforms: pairs.setdefault(x.header.frame_id+'->'+x.child_frame_id,[]).append([stamp(x),x.transform.translation.x,x.transform.translation.y,yaw(x.transform.rotation)])
  r['pairs']={k:len(v) for k,v in pairs.items()}
  for k,v in pairs.items():
   a=np.array(v);a[:,3]=np.unwrap(a[:,3]);arrays[k]=a
 if name=='/robot_description':
  text=rows[0][1].data;(OUT/'recorded_robot.urdf').write_text(text);r['matches_current']=text==Path('src/amr/description/amr_description.urdf').read_text()
 if name=='/diagnostics':r['statuses']=list(set((s.name,s.level,s.message) for t,m in rows for s in m.status))
 report[name]=r
np.savez(OUT/'series.npz',**arrays)
(OUT/'summary.json').write_text(json.dumps(report,indent=2,default=str))
print(json.dumps(report,indent=2,default=str))
