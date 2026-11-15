import sqlite3,json,collections,time
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer
from rosidl_runtime_py.utilities import get_message
from third_impact.amcl_scan import scan_points,apply_transform
from third_impact.localization_search import GridModel,verify_tracking,verify_sensors
from third_impact.core import voxel
root=Path('src/third_impact/debug_log/20261115_183611_825157')
config=yaml.safe_load((root/'third_impact_map.yaml').read_text());pixels=np.flipud(np.asarray(Image.open(root/'third_impact_map.pgm')))
occ=(255-pixels)/255.;cells=np.full(pixels.shape,-1);cells[occ<config['free_thresh']]=0;cells[occ>config['occupied_thresh']]=100
model=GridModel(cells,config['resolution'],config['origin']);tf=Buffer(cache_time=Duration(seconds=300));scans={};maps={}
for path in sorted((root/'bag').glob('*.db3')):
 db=sqlite3.connect(f'file:{path}?mode=ro',uri=True)
 topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics') if n in ('/tf','/tf_static','/rplidar1/scan_filtered','/rplidar2/scan_filtered')}
 ids=','.join(map(str,topics))
 for tid,data in db.execute(f'select topic_id,data from messages where topic_id in ({ids}) order by timestamp'):
  name,cls=topics[tid];msg=deserialize_message(data,cls)
  if name.startswith('/tf'):
   for tr in msg.transforms:
    if name=='/tf_static':tf.set_transform_static(tr,'bag')
    else:tf.set_transform(tr,'bag')
  else:scans[(name,Time.from_msg(msg.header.stamp).nanoseconds)]=msg
counts=collections.Counter();results=[];seen=set();times=[]
for line in (root/'diagnostics.jsonl').open():
 row=json.loads(line)
 if row['topic']!='/third_impact/localization':continue
 data=json.loads(row['data']);q=data.get('quality',{});stamp=q.get('observation_stamp_ns')
 if stamp is None or stamp in seen:continue
 seen.add(stamp)
 clouds={};origins={}
 try:
  for side,topic in [('front','/rplidar1/scan_filtered'),('rear','/rplidar2/scan_filtered')]:
   ns=q['sensor_stamp_ns'][side];msg=scans[(topic,ns)];sensor=tf.lookup_transform('base_link',msg.header.frame_id,Time(nanoseconds=ns)).transform
   points=apply_transform(scan_points(msg,6.),sensor);points=points[(abs(points[:,0])>.35)|(abs(points[:,1])>.22)]
   motion=tf.lookup_transform_full('base_link',Time(nanoseconds=stamp),'base_link',Time(nanoseconds=ns),'odom').transform
   clouds[side]=apply_transform(points,motion)[:,:2]
   origins[side]=apply_transform([[sensor.translation.x,sensor.translation.y,0.]],motion)[0,:2]
 except Exception as e:counts['missing_input']+=1;continue
 start=time.perf_counter();new=verify_sensors(model,voxel(np.vstack(list(clouds.values())),.025),clouds,origins,q['evaluated_pose'])
 if not new['valid']:new=verify_tracking(model,clouds,origins,q['evaluated_pose'])
 times.append(time.perf_counter()-start)
 old=q['valid'];label='valid' if new['valid'] else 'hold' if new['insufficient'] else 'contradiction'
 counts[f'old_{old}_new_{label}']+=1
 results.append(dict(time=row['received_at'],old_valid=old,**new))
print(dict(counts));print('latency_ms median/p95/max',np.percentile(times,[50,95,100])*1000)
Path('/tmp/occlusion_audit.json').write_text(json.dumps(dict(counts=dict(counts),latency_ms=(np.percentile(times,[50,95,100])*1000).tolist(),samples=results),ensure_ascii=False))
for r in results:
 if not r['old_valid'] and not r['valid']:print(json.dumps(r,ensure_ascii=False))
