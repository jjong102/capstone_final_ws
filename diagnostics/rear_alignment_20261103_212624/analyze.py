import json,sqlite3,collections,datetime
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from third_impact.localization_search import GridModel
from third_impact.core import transform
from third_impact.editor import yaw
from third_impact.amcl_scan import apply_transform,scan_points
root=Path('src/third_impact/debug_log/20261103_212624_609744');out=Path('diagnostics/rear_alignment_20261103_212624')
rows=[];fusion=[];events=[]
for line in (root/'diagnostics.jsonl').open():
 try:r=json.loads(line);d=json.loads(r['data'])
 except:continue
 t=r['received_at']
 if r['topic']=='/third_impact/localization' and d.get('quality',{}).get('sensors'):rows.append((t,d))
 if r['topic']=='/third_impact/amcl_scan_status' and d.get('ready'):fusion.append(d)
 if r['topic']=='/third_impact/events':events.append(d)
print('quality last',rows[-1]);print('fusion last',fusion[-1]);print('events',events[-12:])
c=sqlite3.connect(f'file:{root}/bag/bag_0.db3?mode=ro',uri=True)
wanted=['/map','/tf','/tf_static','/odom','/rplidar1/scan_filtered','/rplidar2/scan_filtered']
topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in wanted}
static={};tfs=collections.defaultdict(list);scans=collections.defaultdict(list);odom=[];model=None
for ts,tid,data in c.execute('select timestamp,topic_id,data from messages where topic_id in (%s) order by timestamp'%(','.join(map(str,topics)))):
 n,cls=topics[tid];m=deserialize_message(data,cls)
 if n=='/map' and model is None:
  o=m.info.origin;model=GridModel(np.array(m.data).reshape(m.info.height,m.info.width),m.info.resolution,[o.position.x,o.position.y,yaw(o.orientation)])
 elif n=='/tf_static':
  for t in m.transforms:static[t.child_frame_id]=t.transform
 elif n=='/tf':
  for t in m.transforms:
   if t.child_frame_id in ['odom','base_link']:
    tfs[t.child_frame_id].append((t.header.stamp.sec+t.header.stamp.nanosec/1e9,[t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)]))
 elif n=='/odom':odom.append((m.header.stamp.sec+m.header.stamp.nanosec/1e9,m.twist.twist.linear.x,m.twist.twist.angular.z))
 elif n.endswith('scan_filtered'):scans[n].append(m)
for k,v in static.items():
 if 'lidar' in k:print('TF',k,v)
for k in tfs:
 a=sorted(tfs[k]);tfs[k]=(np.array([x[0] for x in a]),np.array([x[1] for x in a]));tfs[k][1][:,2]=np.unwrap(tfs[k][1][:,2])
def pose_at(t):
 ps=[]
 for k in ['odom','base_link']:
  ts,vs=tfs[k];ps.append(np.array([np.interp(t,ts,vs[:,j]) for j in range(3)]))
 a,b=ps;return np.r_[transform([b[:2]],a)[0],a[2]+b[2]]
print('odom last30',np.max(np.abs(np.array(odom)[np.array(odom)[:,0]>odom[-1][0]-30,1:]),axis=0))
summary={};clouds={};tree=cKDTree(model.reference)
for name,ss in scans.items():
 m=ss[-10];t=m.header.stamp.sec+m.header.stamp.nanosec/1e9
 p=apply_transform(scan_points(m,6.),static[m.header.frame_id])[:,:2];p=p[(abs(p[:,0])>.35)|(abs(p[:,1])>.22)]
 lp=model.local_pose(pose_at(t));world=transform(p,lp);clouds[name]=world
 def residual(delta):return tree.query(transform(p,lp+delta))[0]
 fits=[]
 for a in [-.15,0,.15]:
  fit=least_squares(residual,[0,0,a],bounds=([-.8,-.8,-.35],[.8,.8,.35]),loss='soft_l1',f_scale=.08,diff_step=1e-3,max_nfev=100)
  fits.append(fit)
 fit=min(fits,key=lambda f:np.mean(np.minimum(residual(f.x),.4)))
 d=tree.query(world)[0];df=residual(fit.x)
 summary[name]={'stamp':t,'frame':m.header.frame_id,'scan_time':m.scan_time,'time_increment':m.time_increment,'pose':lp.tolist(),'n':len(p),'distance_median':float(np.median(d)),'distance_p90':float(np.quantile(d,.9)),'overlap12cm':float(np.mean(d<.12)),'fit_delta_xy_yaw':fit.x.tolist(),'fit_overlap12cm':float(np.mean(df<.12)),'fit_distance_median':float(np.median(df))}
 # Temporal samples at fixed intervals, using observation-time TF.
 sample=[]
 for m in ss[::max(1,len(ss)//30)]:
  t=m.header.stamp.sec+m.header.stamp.nanosec/1e9;p=apply_transform(scan_points(m,6.),static[m.header.frame_id])[:,:2];p=p[(abs(p[:,0])>.35)|(abs(p[:,1])>.22)]
  d=tree.query(transform(p,model.local_pose(pose_at(t))))[0];sample.append([t,float(np.mean(d<.12)),float(np.median(d))])
 summary[name]['timeline']=sample
print(json.dumps(summary,indent=2));(out/'summary.json').write_text(json.dumps(summary,indent=2))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,9));ax.scatter(model.reference[:,0],model.reference[:,1],s=1,c='black',label='Map boundary')
for (name,p),color in zip(clouds.items(),['blue','red']):ax.scatter(p[:,0],p[:,1],s=5,c=color,label=name)
ax.axis('equal');ax.legend();ax.grid();fig.savefig(out/'last_scans.png',dpi=150)
