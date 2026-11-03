"""Replay recorded observations through production verification and readiness logic.
No ROS graph, no robot commands; recorded trajectory cannot respond to our stops.
"""
import json,sqlite3,datetime,runpy
from types import SimpleNamespace as NS, MethodType
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rclpy.time import Time
from tf2_ros import Buffer
from third_impact.localization import LocalizationBootstrap as L
from third_impact.localization_search import GridModel
from third_impact.editor import yaw

n=runpy.run_path('src/third_impact/test/test_localization_tracking.py')['harness']()
n.tf=Buffer();n.scans={};n.ready=False;n.model=None;n.amcl=None;n.amcl_time=0.;n.seed_time=0.
n.next_search=0.;n.epoch=0;n.prior=None;n.search_timeout=20.;n.last_save=1e20
n.pool=Mock();n.pool.submit.return_value=NS(done=lambda:True, result=lambda:{'confident':False,'reason':'no_consistent_pose'})
n.file=Path('/tmp/localization_replay_pose.json');n.get_logger=lambda:Mock();n.cancel_search=lambda:setattr(n,'future',None)
for name in ['on_scan','cloud','pose','age','stopped','odom_pose']:
 setattr(n,name,MethodType(getattr(L,name),n))
root='src/third_impact/debug_log/20261103_182114_228934'
c=sqlite3.connect(f'file:{root}/bag/bag_0.db3?mode=ro',uri=True)
wanted=['/map','/tf','/tf_static','/amcl_pose','/odom','/rplidar1/scan_filtered','/rplidar2/scan_filtered','/third_impact/status']
topics={i:(name,get_message(t)) for i,name,t in c.execute('select id,name,type from topics') if name in wanted}
out=[];lastkey=None
for ts,tid,b in c.execute('select timestamp,topic_id,data from messages order by timestamp'):
 if tid not in topics:continue
 name,cl=topics[tid];m=deserialize_message(b,cl);now=ts/1e9
 n.get_clock=lambda ts=ts:NS(now=lambda:Time(nanoseconds=ts))
 with patch('third_impact.localization.time.monotonic',return_value=now):
  if name=='/map' and n.model is None:
   p=m.info.origin;n.model=GridModel(np.array(m.data).reshape(m.info.height,m.info.width),m.info.resolution,[p.position.x,p.position.y,yaw(p.orientation)])
  elif name in ['/tf','/tf_static']:
   for tf in m.transforms:
    (n.tf.set_transform_static if name=='/tf_static' else n.tf.set_transform)(tf,'bag')
  elif name=='/odom':n.odom=m;n.odom_time=now
  elif name=='/amcl_pose':n.amcl=m;n.amcl_time=now
  elif name.endswith('scan_filtered'):n.on_scan('front' if 'rplidar1' in name else 'rear',m)
  elif name=='/third_impact/status' and n.model is not None and n.amcl is not None:
   L.tick(n)
   d={'time':datetime.datetime.fromtimestamp(now).strftime('%H:%M:%S.%f')[:12], 'state':n.state,'ready':n.ready,'quality':n.last_quality}
   out.append(d)
   key=(n.state,n.ready)
   if key!=lastkey:print(d['time'],key,n.last_quality.get('reason'));lastkey=key
Path('diagnostics/patrol_20261103_182114/replay_validation.json').write_text(json.dumps(out,ensure_ascii=False,indent=2))
