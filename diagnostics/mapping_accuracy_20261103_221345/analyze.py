import sqlite3,json,collections,sys
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from third_impact.editor import yaw
from third_impact.amcl_scan import apply_transform
from third_impact.localization_search import GridModel
root=Path('src/amr_navigator/debug_log/20261103_221345_526770');out=Path('diagnostics/mapping_accuracy_20261103_221345')
c=sqlite3.connect(f'file:{root}/bag/bag_0.db3?mode=ro',uri=True)
names=['/tf','/tf_static','/odom','/map','/rplidar1/scan_filtered','/rplidar2/scan_filtered']
topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in names}
tf=collections.defaultdict(dict);scans=[];static={};odom=[];maps=[]
stamp=lambda m:m.header.stamp.sec+m.header.stamp.nanosec/1e9
for ts,tid,b in c.execute('select timestamp,topic_id,data from messages where topic_id in (%s) order by timestamp'%','.join(map(str,topics))):
 n,cl=topics[tid];m=deserialize_message(b,cl)
 if n=='/tf':
  for t in m.transforms:
   if t.child_frame_id in ['odom','base_link']:tf[t.child_frame_id][stamp(t)]=[t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)]
 elif n=='/tf_static':
  for t in m.transforms:static[t.child_frame_id]=t.transform
 elif n=='/odom':odom.append([stamp(m),m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation),m.twist.twist.linear.x,m.twist.twist.angular.z])
 elif n=='/map':maps.append((ts/1e9,m))
 else:scans.append((n,m))
for k in tf:
 ss=sorted(tf[k]);v=np.array([tf[k][t] for t in ss]);v[:,2]=np.unwrap(v[:,2]);tf[k]=(np.array(ss),v)
def at(k,t):
 ts,v=tf[k];return np.array([np.interp(t,ts,v[:,j]) for j in range(3)]).T
p=Path('src/amr_navigator/map/debug_map_corrected.yaml');cfg=yaml.safe_load(p.read_text());im=np.flipud(np.array(Image.open(p.parent/cfg['image'])));cells=np.full(im.shape,-1);cells[im<89]=100;cells[im>205]=0
model=GridModel(cells,cfg['resolution'],cfg['origin']);tree=cKDTree(model.reference)
if len(sys.argv)>1:
 variant=sys.argv[1];d=out/'replay'/variant
 from nav_msgs.msg import OccupancyGrid
 rm=deserialize_message((d/'map.cdr').read_bytes(),OccupancyGrid)
 ro=rm.info.origin;model=GridModel(np.array(rm.data).reshape(rm.info.height,rm.info.width),rm.info.resolution,[ro.position.x,ro.position.y,yaw(ro.orientation)]);tree=cKDTree(model.reference)
 a=np.array(json.loads((d/'tf.json').read_text()));ix=np.argsort(a[:,0]);a=a[ix];tf['odom']=(a[:,0],np.c_[a[:,1:3],np.unwrap(2*np.arctan2(a[:,3],a[:,4]))]);out=d
print('savedmap',im.shape,cfg['origin'],'lastbagmap',maps[-1][1].info.width,maps[-1][1].info.height,maps[-1][1].info.origin,flush=True)
o=np.array(odom);print('odom extent',np.ptp(o[:,1:3],axis=0),'yawspan',np.ptp(np.unwrap(o[:,3])),'max v,w',np.max(abs(o[:,4:]),axis=0),flush=True)
rows=[];clouds={};variants={'start':(0,0),'mid':(.5,0),'end':(1,0),'forward':(0,1),'reverse':(1,-1),'minus':(0,-1)}
for side,m in scans[::3]:
 t=stamp(m);r=np.array(m.ranges);idx=np.arange(len(r));valid=np.isfinite(r)&(r>.15)&(r<6);idx=idx[valid];r=r[valid];ang=m.angle_min+idx*m.angle_increment
 pts=apply_transform(np.c_[r*np.cos(ang),r*np.sin(ang),np.zeros(len(r))],static[m.header.frame_id])[:,:2];valid=(abs(pts[:,0])>.35)|(abs(pts[:,1])>.22);pts=pts[valid];idx=idx[valid]
 if not len(pts):continue
 mo=at('odom',t);w=np.interp(t,o[:,0],o[:,5]);v=np.interp(t,o[:,0],o[:,4]);row={'stamp':t,'side':side,'w':float(w),'v':float(v)}
 for name,(off,scale) in variants.items():
  times=t+off*m.scan_time+scale*idx*m.time_increment;ob=at('base_link',times)
  if ob.ndim==1:ob=np.repeat(ob[None,:],len(pts),axis=0)
  co,si=np.cos(ob[:,2]),np.sin(ob[:,2]);xy=np.c_[co*pts[:,0]-si*pts[:,1]+ob[:,0],si*pts[:,0]+co*pts[:,1]+ob[:,1]]
  co,si=np.cos(mo[2]),np.sin(mo[2]);world=xy@np.array([[co,si],[-si,co]])+mo[:2]-model.origin[:2]
  ds=tree.query(world)[0];row[name]={'median':float(np.median(ds)),'overlap':float(np.mean(ds<.12)),'score':float(np.mean(np.sort(np.minimum(ds,.4))[:max(1,int(.85*len(ds)))]))}
  if name=='start':clouds[side]=(world,t)
 rows.append(row)
summary={}
for side in sorted(set(r['side'] for r in rows)):
 summary[side]={}
 for state,fn in [('stopped',lambda r:abs(r['w'])<.02 and abs(r['v'])<.01),('rotating',lambda r:abs(r['w'])>.15),('last10',lambda r:r['stamp']>rows[-1]['stamp']-10)]:
  rr=[r for r in rows if r['side']==side and fn(r)];summary[side][state]={'n':len(rr),**{name:{key:float(np.median([r[name][key] for r in rr])) if rr else None for key in ['median','overlap','score']} for name in variants}}
(out/'analysis.json').write_text(json.dumps({'summary':summary,'rows':rows},indent=2));print(json.dumps(summary,indent=2),flush=True)
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,9));ax.scatter(model.reference[:,0],model.reference[:,1],s=1,c='black',label='saved map boundary')
for (side,(p,t)),col in zip(sorted(clouds.items()),['blue','red']):ax.scatter(p[:,0],p[:,1],s=5,c=col,label=side)
ax.legend();ax.axis('equal');fig.savefig(out/'end_overlay.png',dpi=130)
