"""Read-only recorded-pose safety audit; no ROS node, publication or hardware."""
import json, sqlite3, math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from third_impact.core import transform, voxel, region_map, travel_allowed
from third_impact.editor import yaw
from third_impact.return_route import FootprintMap, tracking_command
from third_impact.return_control import ReturnControl
root=Path('src/third_impact/debug_log/20261102_234102_001923')
c=sqlite3.connect(f'file:{root}/bag/bag_0.db3?mode=ro',uri=True)
topics={n:(i,t) for i,n,t in c.execute('select id,name,type from topics')}
cache={}
def latest(name,t,count=1):
    if name not in cache:
        i,typ=topics[name]
        rows=c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,)).fetchall()
        cache[name]=(np.array([a/1e9 for a,b in rows]),rows,get_message(typ))
    stamps,rows,cls=cache[name];end=int(np.searchsorted(stamps,t,side='right'))
    return [deserialize_message(b,cls) for a,b in reversed(rows[max(0,end-count):end])]
def pose(x):return np.array([x.translation.x,x.translation.y,yaw(x.rotation)])
def compose(a,b):return np.r_[transform(np.array([b[:2]]),a)[0],a[2]+b[2]]
g=json.loads((root/'goal.json').read_text())
records=[]
for line in (root/'diagnostics.jsonl').open():
    r=json.loads(line)
    if r['topic'].endswith('/status'):
        d=json.loads(r['data'])
        if d['state']=='RETURN_FORWARD':records.append((r['received_at'],d))
static={x.child_frame_id:x.transform for m in latest('/tf_static',records[-1][0],100) for x in m.transforms}
m=latest('/map',records[-1][0])[0];o=m.info.origin
origin=[o.position.x,o.position.y,yaw(o.orientation)]
grid=FootprintMap(region_map(np.array(m.data).reshape(m.info.height,m.info.width),m.info.resolution,origin,g['boundary']),m.info.resolution,origin,.35,.22)
results=[];index=0;previous_path=None
for t,d in records:
    tf={}
    for msg in reversed(latest('/tf',t,150)):
        for x in msg.transforms:tf[x.child_frame_id]=x.transform
    current=compose(pose(tf['odom']),pose(tf['base_link']))
    pathmsg=latest('/third_impact/return_path',t)[0]
    path=np.array([[x.pose.position.x,x.pose.position.y] for x in pathmsg.poses])
    if previous_path is None or not np.array_equal(path,previous_path):index=0
    previous_path=path
    # Recorded status is 5 Hz; advance through the same path's monotone index.
    index=max(index,int(np.argmin(np.linalg.norm(path-current[:2],axis=1))))
    v,w,index=tracking_command(current,path,index,False,.4,.35)
    remaining=float(np.linalg.norm(path[-1]-current[:2]));v=min(v,.7*max(0.,remaining-.03))
    points=[]
    for name in ['/rplidar1/scan_filtered','/rplidar2/scan_filtered']:
        scan=latest(name,t)[0];mount=static[scan.header.frame_id];q=mount.rotation
        x,y,z,ww=q.x,q.y,q.z,q.w
        rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*ww)],[2*(x*y+z*ww),1-2*(x*x+z*z)]])
        r=np.array(scan.ranges);a=scan.angle_min+np.arange(len(r))*scan.angle_increment
        valid=np.isfinite(r)&(r>max(scan.range_min,.1))&(r<min(scan.range_max,6.))
        xy=np.c_[r[valid]*np.cos(a[valid]),r[valid]*np.sin(a[valid])]@rotation.T+[mount.translation.x,mount.translation.y]
        points.extend(xy[(abs(xy[:,0])>.35)|(abs(xy[:,1])>.22)])
    cloud=voxel(np.array(points));old=d['return']['collision_check']
    n=NS(data=g,odom=latest('/odom',t)[0],return_route=path,return_last_tick=t-.05,
         return_last_v=min(.4,max(0.,old['selected'][0])),return_last_w=old['selected'][1],
         return_blocked=None,return_epoch=0,cloud=lambda:cloud,return_grid=lambda:grid,
         stop=Mock(),event=Mock(),velocity_pub=Mock(),show_return_path=Mock(),return_local_pub=Mock(),
         p={'footprint_half_length':.35,'footprint_half_width':.22,'return_obstacle_margin':.01},
         motion_allowed=lambda p:travel_allowed(p,g['boundary'],None,0.,0.,.35,.22))
    with patch('third_impact.return_control.time.monotonic',return_value=t):
        ReturnControl.return_command(n,current,v,w,False)
    results.append(dict(time=t,old_request=old['requested'][0],new_request=v,
                        diagnostic=n.return_collision,published=n.velocity_pub.publish.called))
summary=dict(samples=len(results),max_new_request=max(r['new_request'] for r in results),
             old_zero=sum(r['old_request']<.001 for r in results),new_zero=sum(r['new_request']<.001 for r in results),
             blocked=sum(not r['published'] for r in results),
             limitation='Recorded-pose audit, not a changed closed-loop physical trajectory replay; path index reconstructed from recorded progress.')
print(json.dumps(summary,indent=2,default=lambda x:x.item()))
Path('diagnostics/speed_20261103/replay.json').write_text(json.dumps(dict(summary=summary,samples=results),indent=2,default=lambda x:x.item()))
