import sqlite3,json,datetime
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from third_impact.localization_search import GridModel
from third_impact.core import transform,voxel
from third_impact.editor import yaw
from third_impact.amcl_scan import apply_transform,scan_points
from scipy.spatial import cKDTree
root='src/third_impact/debug_log/20261103_182114_228934'
c=sqlite3.connect(f'file:{root}/bag/bag_0.db3?mode=ro',uri=True)
topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in ['/map','/tf','/tf_static','/rplidar1/scan_filtered','/rplidar2/scan_filtered','/third_impact/status','/amcl_pose']}
static={};tf={};scans={};model=None;last=0;out=[];amcl=None
for ts,tid,data in c.execute('select timestamp,topic_id,data from messages order by timestamp'):
 if tid not in topics:continue
 n,cls=topics[tid];m=deserialize_message(data,cls)
 if n=='/map' and model is None:
  o=m.info.origin;model=GridModel(np.array(m.data).reshape(m.info.height,m.info.width),m.info.resolution,[o.position.x,o.position.y,yaw(o.orientation)]);tree=cKDTree(model.reference)
 elif n=='/tf_static':
  for t in m.transforms:static[t.child_frame_id]=t.transform
 elif n=='/tf':
  for t in m.transforms:tf[t.child_frame_id]=t
 elif n=='/amcl_pose':amcl=m
 elif n.endswith('scan_filtered'):
  if m.header.frame_id in static:
   p=apply_transform(scan_points(m,6.),static[m.header.frame_id])[:,:2];scans[n]=p[(abs(p[:,0])>.35)|(abs(p[:,1])>.22)]
 elif n=='/third_impact/status' and model is not None and len(scans)==2 and 'odom' in tf and 'base_link' in tf and ts-last>1e9:
  last=ts;d=json.loads(m.data)
  def pose(t):return np.array([t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)])
  a=pose(tf['odom']);b=pose(tf['base_link']);wp=np.r_[transform([b[:2]],a)[0],a[2]+b[2]];lp=model.local_pose(wp)
  cloud=voxel(np.vstack(list(scans.values())),.025);r=model.verify(cloud,wp)
  samples=np.vstack([cloud*f for f in (.25,.5,.75)]);row,col,g=model.indices(transform(samples,lp));free=float(np.mean(g&model.free[row,col]))
  moved=transform(cloud,lp);dist=tree.query(moved)[0]
  row,col,g=model.indices(moved)
  r.update(time=datetime.datetime.fromtimestamp(ts/1e9).strftime('%H:%M:%S'),pose=wp.tolist(),footprint=model.footprint_clear(lp),rays_fraction=free,boundary_overlap=float(np.mean(dist<.12)),occupied_fraction=float(np.mean(g&model.occupied[row,col])),state=d['state'],ready=d['ready'],detail=d['detail'],cov=[amcl.pose.covariance[i] for i in [0,7,35]] if amcl else [])
  out.append(r)
json.dump(out,open('diagnostics/patrol_20261103_182114/geometry.json','w'),indent=2)
for r in out:
 if '18:21:45'<r['time']<'18:22:12' or r is out[-1]:print(r)
