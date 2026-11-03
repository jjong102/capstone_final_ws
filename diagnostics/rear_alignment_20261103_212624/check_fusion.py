"""Offline bag audit. No ROS node or robot command is created."""
import json,sqlite3,collections,bisect
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import minimize
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from third_impact.amcl_scan import scan_points,apply_transform,virtual_ranges
from third_impact.core import transform
from third_impact.editor import yaw
from third_impact.localization_search import GridModel
root=Path('src/third_impact/debug_log/20261103_212624_609744');out=Path('diagnostics/rear_alignment_20261103_212624')
c=sqlite3.connect(f'file:{root}/bag/bag_0.db3?mode=ro',uri=True)
names=['/map','/tf','/tf_static','/rplidar1/scan_filtered','/rplidar2/scan_filtered','/third_impact/amcl_scan']
topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in names}
scans=[{},{}];merged=[];static={};tf=collections.defaultdict(dict);model=None
stamp=lambda m:m.header.stamp.sec*10**9+m.header.stamp.nanosec
for ts,tid,b in c.execute('select timestamp,topic_id,data from messages where topic_id in (%s) order by timestamp'%','.join(map(str,topics))):
 n,cls=topics[tid];m=deserialize_message(b,cls)
 if n=='/map' and model is None:
  o=m.info.origin;model=GridModel(np.array(m.data).reshape(m.info.height,m.info.width),m.info.resolution,[o.position.x,o.position.y,yaw(o.orientation)])
 elif n=='/tf_static':
  for t in m.transforms:static[t.child_frame_id]=t.transform
 elif n=='/tf':
  for t in m.transforms:
   if t.child_frame_id in ['odom','base_link']:tf[t.child_frame_id][stamp(t)]=[t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)]
 elif n=='/third_impact/amcl_scan':merged.append(m)
 elif n.endswith('scan_filtered'):scans[0 if 'rplidar1' in n else 1][stamp(m)]=m
for k in tf:
 ss=sorted(tf[k]);v=np.array([tf[k][t] for t in ss]);v[:,2]=np.unwrap(v[:,2]);tf[k]=(np.array(ss,dtype=np.int64),v)
def at(k,t):
 ts,v=tf[k];return np.array([np.interp(t,ts,v[:,j]) for j in range(3)])
def motion(p,s,t):
 a=at('base_link',s);b=at('base_link',t);world=transform(p,a);d=world-b[:2];co,si=np.cos(b[2]),np.sin(b[2]);return d@np.array([[co,-si],[si,co]])
def map_pose(t):
 a=at('odom',t);b=at('base_link',t);return model.local_pose(np.r_[transform([b[:2]],a)[0],a[2]+b[2]])
cache={}
def cloud(side,t):
 key=(side,t)
 if key not in cache:
  m=scans[side][t];p=apply_transform(scan_points(m,6.),static[m.header.frame_id])[:,:2];cache[key]=p[(abs(p[:,0])>.35)|(abs(p[:,1])>.22)]
 return cache[key]
def compare(a,b):
 fa,fb=np.isfinite(a),np.isfinite(b);same=fa&fb;return int(np.count_nonzero(fa!=fb)),float(np.mean(abs(a[same]-b[same]))) if same.any() else 1e9
keys=[sorted(x) for x in scans];results=[];lastcloud=None
for m in merged[::max(1,len(merged)//120)]+merged[-1:]:
 target=stamp(m);actual=np.array(m.ranges);opts=[]
 for side in [0,1]:
  ks=keys[side];i=bisect.bisect_right(ks,target);opts.append([x for x in ks[max(0,i-3):i] if target-x<=150000000])
 best=None
 for a in opts[0]:
  for b in opts[1]:
   if max(a,b)!=target:continue
   p=[cloud(0,a),cloud(1,b)];pm=[motion(p[0],a,target),motion(p[1],b,target)]
   rec=virtual_ranges(np.vstack(pm));metric=compare(actual,rec)
   if best is None or metric<best[0]:best=(metric,a,b,p,pm,rec)
 if best is None:continue
 metric,a,b,p,pm,rec=best;unc=virtual_ranges(np.vstack(p));shift=np.linalg.norm(np.vstack(pm)-np.vstack(p),axis=1)
 results.append(dict(stamp=target/1e9,pair_skew_ms=abs(a-b)/1e6,finite_bin_mismatches=metric[0],mean_range_error_m=metric[1],uncompensated_compare=compare(actual,unc),motion_shift_max_m=float(max(shift)),motion_shift_mean_m=float(np.mean(shift))))
 lastcloud=(target,pm)
print('audit matched',len(results),'of sampled outputs',flush=True)
stat={}
for key in ['finite_bin_mismatches','mean_range_error_m','motion_shift_max_m','pair_skew_ms']:
 v=np.array([x[key] for x in results]);stat[key]={'median':float(np.median(v)),'p95':float(np.quantile(v,.95)),'max':float(max(v))}
stat['last_30_seconds']=[x for x in results if x['stamp']>results[-1]['stamp']-30]
# Local hypothesis search, NOT sensor calibration. All candidates evaluated on both sensors.
t,clouds=lastcloud;pose=map_pose(t);tree=cKDTree(model.reference)
def cost(p,d):
 dist=tree.query(transform(p,pose+d))[0];v=np.minimum(dist,.4);return float(np.mean(np.partition(v,int(.85*len(v))-1)[:int(.85*len(v))]))
def metrics(p,d):
 ds=tree.query(transform(p,pose+d))[0];return dict(overlap12cm=float(np.mean(ds<.12)),median_m=float(np.median(ds)),trimmed_capped_score_m=cost(p,d))
fits={}
for label,p in [('front',clouds[0]),('rear',clouds[1]),('both',np.vstack(clouds))]:
 candidates=[SimpleNamespace(x=np.zeros(3),fun=cost(p,np.zeros(3)))]
 seeds=[]
 for dx in np.linspace(-.3,.3,9):
  for dy in np.linspace(-.3,.3,9):
   for da in np.deg2rad(np.linspace(-10,10,9)):
    d=np.array([dx,dy,da]);seeds.append((cost(p,d),d))
 seeds.sort(key=lambda x:x[0])
 for _,seed in seeds[:5]:
  candidates.append(SimpleNamespace(x=seed,fun=cost(p,seed)))
  f=minimize(lambda d:cost(p,d),seed,method='Powell',bounds=[(-.3,.3),(-.3,.3),(-np.deg2rad(10),np.deg2rad(10))],options={'maxiter':70,'xtol':.0005,'ftol':.0001});candidates.append(f)
 f=min(candidates,key=lambda x:x.fun);fits[label]=dict(delta=f.x.tolist(),front=metrics(clouds[0],f.x),rear=metrics(clouds[1],f.x))
fits['unchanged']=dict(front=metrics(clouds[0],np.zeros(3)),rear=metrics(clouds[1],np.zeros(3)))
report=dict(samples=len(results),summary=stat,pose_hypotheses=fits,samples_detail=results)
(out/'fusion_audit.json').write_text(json.dumps(report,indent=2));print(json.dumps({'summary':{k:v for k,v in stat.items() if k!='last_30_seconds'},'last':results[-1],'fits':fits},indent=2))
