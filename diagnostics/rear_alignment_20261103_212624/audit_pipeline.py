import sqlite3,json,collections,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from third_impact.amcl_scan import scan_points,apply_transform,virtual_ranges
root=Path('src/third_impact/debug_log/20261103_212624_609744')
c=sqlite3.connect(f'file:{root}/bag/bag_0.db3?mode=ro',uri=True)
wanted=['/tf_static','/rplidar1/scan','/rplidar2/scan','/rplidar1/scan_filtered','/rplidar2/scan_filtered']
topics={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in wanted}
raw={};last={};static={};counts=collections.Counter();mismatch=collections.Counter();meta={}
for tid,data in c.execute('select topic_id,data from messages where topic_id in (%s) order by timestamp'%','.join(map(str,topics))):
 n,cls=topics[tid];m=deserialize_message(data,cls)
 if n=='/tf_static':
  for t in m.transforms:static[t.child_frame_id]=t.transform
  continue
 stamp=(m.header.stamp.sec,m.header.stamp.nanosec);side=n.split('/')[1]
 if not n.endswith('_filtered'):
  raw[(side,stamp)]=m
  continue
 r=raw.pop((side,stamp),None)
 if r is None:counts[side+'_unpaired']+=1;continue
 counts[side+'_pairs']+=1
 a=np.array(r.ranges);b=np.array(m.ranges)
 if len(a)!=len(b):mismatch[side+'_length']+=1;continue
 expected=a.copy();angle=r.angle_min
 for i in range(len(a)):
  if -1.5708<angle<1.5708:expected[i]=np.float32(r.range_max+1)
  angle+=r.angle_increment
 if not np.array_equal(expected,b,equal_nan=True):mismatch[side+'_ranges']+=1
 fields=['angle_min','angle_max','angle_increment','time_increment','scan_time','range_min','range_max']
 if any(getattr(r,x)!=getattr(m,x) for x in fields) or r.header.frame_id!=m.header.frame_id:mismatch[side+'_metadata']+=1
 last[side]=m;meta[side]={x:getattr(m,x) for x in fields};meta[side].update(frame=m.header.frame_id,n=len(b))
robot=ET.parse('src/amr/description/amr_description.urdf').getroot();tfcheck={};halfplanes={};bins={}
for side,link,joint in [('rplidar1','lidar1_link','lidar1_joint'),('rplidar2','lidar2_link','lidar2_joint')]:
 o=robot.find(f"joint[@name='{joint}']/origin");xyz=np.fromstring(o.get('xyz'),sep=' ');rpy=np.fromstring(o.get('rpy'),sep=' ')
 tf=static[link];q=tf.rotation;rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();ref=Rotation.from_euler('xyz',rpy).as_matrix()
 tfcheck[side]={'translation_error_m':float(np.linalg.norm(xyz-[tf.translation.x,tf.translation.y,tf.translation.z])),'rotation_matrix_max_error':float(np.max(abs(rot-ref))),'urdf_xyz':xyz.tolist(),'urdf_rpy':rpy.tolist()}
 m=last[side];p=apply_transform(scan_points(m,6.),tf);own=(abs(p[:,0])<=.35)&(abs(p[:,1])<=.22);p=p[~own]
 halfplanes[side]={'positive_base_x_fraction':float(np.mean(p[:,0]>0)),'negative_base_x_fraction':float(np.mean(p[:,0]<0))}
 ranges=virtual_ranges(p);valid=np.isfinite(ranges);a=-np.pi+np.arange(720)*2*np.pi/720
 # Pure quantization error upper bound (not motion compensation or bin occlusion).
 bins[side]={'maximum_angular_bin_error_deg':.5,'endpoint_error_bound_at_6m_m':float(2*6*np.sin(np.pi/720))}
result=dict(counts=dict(counts),filter_mismatches=dict(mismatch),metadata=meta,urdf_tf=tfcheck,filtered_halfplanes=halfplanes,binning=bins)
Path('diagnostics/rear_alignment_20261103_212624/pipeline_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
