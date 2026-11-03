import json,sqlite3,math
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from third_impact.amcl_scan import apply_transform,scan_points
root=Path('diagnostics/lidar_wall_check')

def fit(points,side):
    # Broad forward/rear sector; fit a wall without forcing its measured distance.
    p=points[(side*points[:,0]>.4)&(side*points[:,0]<1.7)&(abs(points[:,1])<.65)]
    if len(p)<30:return None
    rng=np.random.default_rng(42);best=None
    for _ in range(100):
        a,b=p[rng.choice(len(p),2,replace=False)];delta=b-a
        if np.linalg.norm(delta)<.2:continue
        n=np.array([delta[1],-delta[0]]);n/=np.linalg.norm(n)
        if side*n[0]<0:n=-n
        if side*n[0]<.8:continue
        d=a@n;mask=abs(p@n-d)<.015
        if best is None or mask.sum()>best.sum():best=mask
    if best is None or best.sum()<30:return None
    pp=p[best];center=pp.mean(axis=0);_,_,vh=np.linalg.svd(pp-center,full_matrices=False);n=vh[-1]
    if side*n[0]<0:n=-n
    d=float(center@n)
    return n,d,float(np.sqrt(np.mean((pp@n-d)**2))),len(pp)

results={}
for label,filename,topic,side in [('front','current_session.txt','/rplidar1/scan',1),('rear','rear_session.txt','/rplidar2/scan',-1)]:
    path=Path((root/filename).read_text().strip())
    (path/'corrected_measurement.json').write_text(json.dumps({'front_and_rear_outer_edge_to_wall_m':.82,'approximate':True,'source':'user correction'},indent=2))
    c=sqlite3.connect('file:'+str(path/'bag/bag_0.db3')+'?mode=ro',uri=True)
    types={i:(n,get_message(t)) for i,n,t in c.execute('select id,name,type from topics') if n in ['/tf_static','/odom',topic,topic+'_filtered']}
    end=c.execute('select max(timestamp) from messages').fetchone()[0]/1e9
    if label=='front':
        marks=[json.loads(x) for x in (path/'markers.jsonl').read_text().splitlines()]
        end=min(end,next(x['time'] for x in marks if x['phase']=='front_stationary_complete'))
    start=end-15;static={};rows=[];odom=[];fits=[]
    for t,i,b in c.execute('select timestamp,topic_id,data from messages order by timestamp'):
        if i not in types:continue
        name,cl=types[i];m=deserialize_message(b,cl)
        if name=='/tf_static':
            for tf in m.transforms:static[tf.child_frame_id]=tf.transform
        if start<=t/1e9<=end:
            if name=='/odom':odom.append([m.twist.twist.linear.x,m.twist.twist.angular.z,m.pose.pose.position.x,m.pose.pose.position.y])
            elif name in [topic,topic+'_filtered']:rows.append((name,m))
    for name,m in rows:
        tf=static[m.header.frame_id];points=apply_transform(scan_points(m,6.),tf)[:,:2];result=fit(points,side)
        if result is None:continue
        n,d,rms,count=result;origin=np.array([tf.translation.x,tf.translation.y])
        fits.append({'topic':name,'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'base_distance_m':d,
                     'sensor_distance_m':float(d-n@origin),'outer_distance_m':float(d-n@np.array([side*.3495,0.])),
                     'normal_angle_deg':math.degrees(math.atan2(side*n[1],side*n[0])), 'rms_m':rms,'inliers':count})
    summary={}
    for name in [topic,topic+'_filtered']:
        f=[x for x in fits if x['topic']==name]
        summary[name]={'count':len(f)}
        for k in ['base_distance_m','sensor_distance_m','outer_distance_m','normal_angle_deg','rms_m']:
            a=np.array([x[k] for x in f]);summary[name][k]={'median':float(np.median(a)),'p05':float(np.quantile(a,.05)),'p95':float(np.quantile(a,.95))}
    o=np.array(odom)
    results[label]={'start':start,'end':end,'summary':summary,'odom_abs_velocity_p95':np.quantile(abs(o[:,:2]),.95,axis=0).tolist(),'odom_position_span_m':np.ptp(o[:,2:],axis=0).tolist(),'fits':fits,
                    'sensor_tf':str(static['lidar1_link' if side==1 else 'lidar2_link'])}
    c.close()
(root/'comparison_82cm.json').write_text(json.dumps(results,indent=2))
for label,r in results.items():
 print(label,json.dumps({k:v for k,v in r.items() if k not in ['fits','sensor_tf']},indent=2))
