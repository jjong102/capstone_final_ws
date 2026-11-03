import json,sqlite3,math
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from third_impact.core import relative,transform,wrap
from third_impact.editor import yaw
root=Path('diagnostics/amcl_compare_20261103')
c=sqlite3.connect('file:src/third_impact/debug_log/20261103_174827_039173/bag/bag_0.db3?mode=ro',uri=True)
i=c.execute("select id from topics where name='/tf'").fetchone()[0];od=[]
for b, in c.execute('select data from messages where topic_id=? order by timestamp',(i,)):
 for x in deserialize_message(b,TFMessage).transforms:
  if x.header.frame_id=='odom' and x.child_frame_id=='base_link':
   p=x.transform;od.append([x.header.stamp.sec+x.header.stamp.nanosec/1e9,p.translation.x,p.translation.y,yaw(p.rotation)])
od=np.array(sorted(od,key=lambda x:x[0]));out={}
for path in [root/'results.json',root/'repeat/results.json']:
 if not path.exists():continue
 data=json.loads(path.read_text());metrics={}
 for name,rows in data['trajectories'].items():
  corrections=[]
  for a,b in zip(rows,rows[1:]):
   x=od[np.argmin(abs(od[:,0]-a['time'])),1:];y=od[np.argmin(abs(od[:,0]-b['time'])),1:]
   delta=relative(x,y);pred=np.r_[transform(np.array([delta[:2]]),a['pose'])[0],a['pose'][2]+delta[2]]
   corrections.append([np.linalg.norm(pred[:2]-b['pose'][:2]),abs(wrap(pred[2]-b['pose'][2]))*180/math.pi])
  stats=dict(data['summary'][name]);stats['correction_step_p95_m_deg']=np.percentile(corrections,95,axis=0).tolist();stats['correction_step_max_m_deg']=np.max(corrections,axis=0).tolist()
  metrics[name]=stats
 out[str(path)]=metrics
(root/'summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
