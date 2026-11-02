import runpy
s=runpy.run_path('diagnostics/mapping_20261102_185900/analyze.py')
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=s['data']; A=s['arrays']; O=s['OUT']; st=s['stamp']
imu=A['/imu/data']; wheel=A['/wheel/odometry']; odom=A['/odom']
scans=D['/rplidar1/scan_filtered']
def cloud(m):
 r=np.array(m.ranges); a=m.angle_min+np.arange(len(r))*m.angle_increment
 keep=np.isfinite(r)&(r>.25)&(r<6)
 return np.c_[-r[keep]*np.cos(a[keep])+.311,r[keep]*np.sin(a[keep])]
def R(a):return np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
result=[]
for i in range(1,len(scans)):
 t=st(scans[i][1]); t0=st(scans[i-1][1]); dt=t-t0
 p=cloud(scans[i][1]); q=cloud(scans[i-1][1]); tree=cKDTree(q)
 angle=np.interp(t,odom[:,0],odom[:,3])-np.interp(t0,odom[:,0],odom[:,3]); rot=R(angle); trans=np.zeros(2)
 for j in range(30):
  moved=p@rot.T+trans; dist,ids=tree.query(moved)
  mask=dist<min(.3,np.percentile(dist,85))
  if mask.sum()<30:break
  u=moved[mask]; v=q[ids[mask]]; uc=u.mean(0);vc=v.mean(0)
  U,S,V=np.linalg.svd((u-uc).T@(v-vc)); rr=V.T@U.T
  if np.linalg.det(rr)<0:V[-1]*=-1;rr=V.T@U.T
  tt=vc-rr@uc;rot=rr@rot;trans=rr@trans+tt
  if np.linalg.norm(tt)<1e-5 and abs(np.arctan2(rr[1,0],rr[0,0]))<1e-5:break
 result.append([t,dt,np.arctan2(rot[1,0],rot[0,0]),*trans,np.median(dist)])
a=np.array(result);np.save(O/'scan_motion.npy',a)
x=a[:,0];tstart=odom[0,0]
print('ICP yaw total deg',np.rad2deg(a[:,2].sum()),'median residual',np.median(a[:,5]))
print('corrected wheel yaw',np.rad2deg(wheel[-1,3]-wheel[0,3])*.13/.17*.272/.392)
print('ICP total translation norm',np.linalg.norm(a[:,3:5],axis=1).sum())
# Pose errors relative to scan-based motion over 1 s windows (ICP is not ground truth).
for start in np.arange(x[0],x[-1]-1,5):
 mask=(x>=start)&(x<start+5)
 iw=(wheel[:,0]>=start)&(wheel[:,0]<start+5)
 io=(odom[:,0]>=start)&(odom[:,0]<start+5)
 print('window',round(start-tstart,1),'scan yaw',round(np.rad2deg(a[mask,2].sum()),1),'wheel yaw',round(np.rad2deg(wheel[iw][-1,3]-wheel[iw][0,3]),1),'ekf yaw',round(np.rad2deg(odom[io][-1,3]-odom[io][0,3]),1),'wheel v mean',round(wheel[iw,4].mean(),3))
fig,ax=plt.subplots(3,1,figsize=(11,9),sharex=True)
for name,arr,factor in [('Wheel (old geometry)',wheel,1),('Wheel (URDF geometry)',wheel,.13/.17*.272/.392),('EKF',odom,1)]:
 yaw0=np.interp(x[0]-a[0,1],arr[:,0],arr[:,3]);ax[0].plot(arr[:,0]-tstart,np.rad2deg((arr[:,3]-yaw0)*factor),label=name)
ax[0].plot(x-tstart,np.rad2deg(np.cumsum(a[:,2])),label='Scan pair ICP (estimate)');ax[0].set_ylabel('Yaw change [deg]');ax[0].legend()
ax[1].plot(odom[:,0]-tstart,odom[:,4],label='EKF forward speed');ax[1].plot(x-tstart,a[:,3]/a[:,1],label='Scan ICP forward speed',alpha=.6);ax[1].set_ylabel('m/s');ax[1].legend()
ax[2].plot(x-tstart,np.rad2deg(a[:,2]/a[:,1]),label='Scan ICP');ax[2].plot(imu[:,0]-tstart,np.rad2deg(-imu[:,4]),label='IMU transformed to base');ax[2].set_ylabel('Yaw rate [deg/s]');ax[2].set_xlabel('Seconds from first EKF odometry');ax[2].legend()
for p in ax:p.grid(alpha=.3)
fig.tight_layout();fig.savefig(O/'motion_comparison.png',dpi=150)
