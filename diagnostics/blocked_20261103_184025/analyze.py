import sqlite3,json,datetime,math,time
from types import SimpleNamespace as NS
from third_impact.patrol_control import PatrolControl
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from third_impact.core import transform,relative,travel_allowed,inside_patrol_region,outside_exclusion,swept_clear,voxel,region_map
from third_impact.amcl_scan import apply_transform,scan_points
from third_impact.editor import yaw
from third_impact.return_route import FootprintMap
root='src/third_impact/debug_log/20261103_184025_014330'
data=json.load(open(root+'/goal.json'));c=sqlite3.connect(f'file:{root}/bag/bag_0.db3?mode=ro',uri=True)
names=['/tf','/tf_static','/map','/rplidar1/scan_filtered','/rplidar2/scan_filtered','/third_impact/status']
topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in names}
tfs={};static={};scans={};grid=None;last=0;out=[]
for ts,tid,b in c.execute('select timestamp,topic_id,data from messages order by timestamp'):
 if tid not in topics:continue
 n,cl=topics[tid];m=deserialize_message(b,cl)
 if n=='/tf':
  for t in m.transforms:tfs[t.child_frame_id]=t.transform
 elif n=='/tf_static':
  for t in m.transforms:static[t.child_frame_id]=t.transform
 elif n=='/map':
  o=m.info.origin;origin=[o.position.x,o.position.y,yaw(o.orientation)];cells=np.array(m.data).reshape(m.info.height,m.info.width)
  grid=FootprintMap(region_map(cells,m.info.resolution,origin,data['boundary']),m.info.resolution,origin)
 elif n.endswith('scan_filtered') and m.header.frame_id in static:
  p=apply_transform(scan_points(m,6.),static[m.header.frame_id])[:,:2]
  scans[n]=p[(abs(p[:,0])>.35)|(abs(p[:,1])>.22)]
 elif n=='/third_impact/status' and ts-last>1e9 and len(scans)==2 and grid is not None and 'odom' in tfs and 'base_link' in tfs:
  last=ts;d=json.loads(m.data)
  def pose(t):return [t.translation.x,t.translation.y,yaw(t.rotation)]
  a=pose(tfs['odom']);b=pose(tfs['base_link']);p=np.r_[transform([b[:2]],a)[0],a[2]+b[2]]
  radius=math.hypot(.35,.22)+.05;rel=relative(data['rectangle'][:3],p);dist=float(np.linalg.norm(np.maximum(abs(rel[:2])-np.array(data['rectangle'][3:])/2,0)))
  cloud=voxel(np.vstack(list(scans.values())))
  r=dict(time=datetime.datetime.fromtimestamp(ts/1e9).strftime('%H:%M:%S'),state=d['state'],pose=p.tolist(),exclusion_distance=dist,required_clearance=radius,boundary_ok=inside_patrol_region(p,data['boundary'],radius),exclusion_ok=outside_exclusion(p,data['rectangle'],radius),map_clear=grid.clear(p),scan_rotation_ccw=swept_clear(cloud,0.,.25,.35,.22,.04,horizon=math.pi/2/.25),scan_rotation_cw=swept_clear(cloud,0.,-.25,.35,.22,.04,horizon=math.pi/2/.25))
  for side,sign in [('ccw',1),('cw',-1)]:
   bad=[float(a) for a in np.linspace(0,sign*math.pi/2,80) if not grid.clear([p[0],p[1],p[2]+a])]
   r['map_rotation_'+side]=not bool(bad)
   r['first_bad_angle_'+side]=math.degrees(bad[0]) if bad else None
  if r['time']=='18:40:46':
   for aging in [False,True]:
    start=time.monotonic(); calls=[]
    def getcloud():
     age=time.monotonic()-start+.03; calls.append(age)
     return None if aging and age>.45 else cloud
    obj=NS(patrol_path_stamp=0.,patrol_path=None,patrol_retry_target=[p[0]+1,p[1],0],
      data=data,p={'footprint_half_length':.35,'footprint_half_width':.22,'obstacle_margin':.04},
      cloud=getcloud,return_grid=lambda:grid,
      motion_allowed=lambda pos,v,angular:travel_allowed(pos,data['boundary'],data['rectangle'],v,0.,.35,.22,angular=angular))
    obj.safe_patrol_command=lambda pos,v,w:PatrolControl.safe_patrol_command(obj,pos,v,w)
    result=PatrolControl.recovery_heading(obj,p)
    print('REPRO',aging,'heading',result,'duration',time.monotonic()-start,'max scan age',max(calls),'cloud calls',len(calls))
  out.append(r)
json.dump(out,open('diagnostics/blocked_20261103_184025/geometry.json','w'),indent=2,default=lambda x:bool(x))
for r in out:
 if '18:40:43'<=r['time']<='18:40:49':print(r)
