import sqlite3,json,math
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from third_impact.localization_search import GridModel
from third_impact.core import voxel
P=Path('diagnostics/heading_20261102');bag=Path((P/'latest_bag.txt').read_text());db=sqlite3.connect(f'file:{next(bag.glob("*.db3"))}?mode=ro',uri=True)
D={}
for i,n,typ in db.execute('select id,name,type from topics'):
 cls=get_message(typ);D[n]=[(t/1e9,deserialize_message(b,cls)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9
print('counts',{n:len(v) for n,v in D.items()})
for n in ['/third_impact/localization']:
 if D[n]:print(n,D[n][-1][1].data)
if D.get('/robot_description'):print('URDF exact match',D['/robot_description'][0][1].data==Path('src/amr/description/amr_description.urdf').read_text())
static={tr.child_frame_id:tr for t,m in D['/tf_static'] for tr in m.transforms}
for child in ['imu_link','lidar1_link','lidar2_link']: print('static',child,str(static[child].transform))
for name in ['/odom','/imu/data']:
 rows=D[name]; rate=[m.twist.twist.angular.z if name=='/odom' else -m.angular_velocity.z for t,m in rows];print(name,'angular range',min(rate),max(rate))
tfs={}
for t,m in D['/tf']:
 for tr in m.transforms:
  if tr.child_frame_id in ['odom','base_link']: tfs.setdefault(tr.child_frame_id,[]).append([stamp(tr),tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation)])
def pose_at(t):
 vals=[]
 for child in ['odom','base_link']:
  ar=np.array(tfs[child]);ar[:,3]=np.unwrap(ar[:,3]);vals.append(np.array([np.interp(t,ar[:,0],ar[:,i]) for i in [1,2,3]]))
 a,b=vals;c=math.cos(a[2]);s=math.sin(a[2]);return np.array([a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],a[2]+b[2]])
def cloud(m):
 tr=static[m.header.frame_id];q=tr.transform.rotation;x,y,z,w=q.x,q.y,q.z,q.w;rot=np.array([[1-2*(y*y+z*z),2*(x*y-z*w)],[2*(x*y+z*w),1-2*(x*x+z*z)]])
 r=np.array(m.ranges);a=m.angle_min+np.arange(len(r))*m.angle_increment;ok=np.isfinite(r)&(r>max(.15,m.range_min))&(r<6);pts=np.c_[r[ok]*np.cos(a[ok]),r[ok]*np.sin(a[ok])]@rot.T+[tr.transform.translation.x,tr.transform.translation.y]
 return pts
m=D['/map'][-1][1];cells=np.array(m.data).reshape(m.info.height,m.info.width);model=GridModel(cells,m.info.resolution,[m.info.origin.position.x,m.info.origin.position.y,yaw(m.info.origin.orientation)])
for topic in ['/rplidar1/scan','/rplidar1/scan_filtered','/rplidar2/scan_filtered']:
 rows=D[topic];m=rows[-1][1];pts=cloud(m);inside=(abs(pts[:,0])<.35)&(abs(pts[:,1])<.22);print(topic,'points',len(pts),'inside footprint',inside.sum())
 pose=pose_at(stamp(m)+m.scan_time/2);angles=np.deg2rad(np.arange(-20,20.01,.25));poses=np.tile(pose,(len(angles),1));poses[:,2]+=angles
 scores=model.scores(pts,np.array([model.local_pose(p) for p in poses]));j=np.argmin(scores);print('pose',pose,'best fixedXY yaw correction deg',np.rad2deg(angles[j]),'score before/best',scores[len(angles)//2],scores[j])
print('amcl covariance',D['/amcl_pose'][-1][1].pose.covariance[35] if D.get('/amcl_pose') else None)
# Quantify ambiguity: distance to all occupied cells versus free/occupied boundary.
from scipy.ndimage import distance_transform_edt,binary_dilation
from third_impact.core import transform
boundary=model.occupied & binary_dilation(model.free)
boundary_distance=distance_transform_edt(~boundary)*model.resolution
m=D['/rplidar1/scan'][-1][1];pts=cloud(m);p=pose_at(stamp(m)+m.scan_time/2)
for deg in [-15,-10,-5,0,5,10,15]:
 pp=p.copy();pp[2]+=math.radians(deg);xy=transform(pts,model.local_pose(pp));r,c,ok=model.indices(xy)
 inside=model.occupied[r,c]&ok
 print('yaw offset',deg,'occupied endpoints fraction',round(inside.mean(),3),'mean capped occupied distance',round(np.minimum(model.distance[r,c],.4).mean(),4),'mean capped boundary distance',round(np.minimum(boundary_distance[r,c],.4).mean(),4))
