from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
from third_impact.localization import LocalizationBootstrap


def harness():
    n=NS(model=Mock(), odom=object(), odom_time=100., amcl_active=True,
         scans={'front':(100.,1), 'rear':(100.,2)}, last_verified_key=None,
         sensor_clouds={'front':np.ones((100,2)), 'rear':np.ones((100,2))},
         sensor_origins={'front':np.zeros(2),'rear':np.zeros(2)},
         observation_stamp=NS(to_msg=lambda:None,nanoseconds=2),age=lambda stamp:.1,
         cloud=lambda:np.ones((100,2)),pose=lambda:np.zeros(3),stopped=lambda:False,
         ready=True,bad_checks=0,bad_since=None,recheck_since=None,verify_since=None,
         verify_count=0,future=None,last_quality={},state='READY')
    def state(s,r,ready=False):n.state=s;n.reason=r;n.ready=ready
    n.set_state=state
    n.model.verify.return_value={'valid':False,'reason':'壁','overlap':.2}
    return n


def tick(n,t):
    n.odom_time=t;n.scans['front']=(t,int(t*1e9));n.scans['rear']=(t,int(t*1e9)+10000000)
    n.observation_stamp.nanoseconds=n.scans['rear'][1]
    with patch('third_impact.localization.time.monotonic',return_value=t):
        LocalizationBootstrap.tick(n)


def test_moving_bad_geometry_stops_after_sustained_checks():
    n=harness()
    tick(n,100.);assert n.ready and n.state=='DEGRADED'
    tick(n,100.2);assert n.ready
    tick(n,100.5);assert not n.ready and n.state=='WAIT_STOP'
    n.stopped=lambda:True
    tick(n,100.7);assert n.state=='LOCAL_RECHECK'
    tick(n,106.);assert n.state=='LOCAL_RECHECK'
    n.seed_time=0.;n.next_search=0.;n.epoch=0;n.odom_pose=lambda:np.zeros(3)
    n.pool=Mock();n.prior=None;n.search_timeout=20.
    tick(n,106.8)
    assert n.state=='SEARCHING' and n.pool.submit.call_count==1


def test_transient_bad_scan_recovers_without_stop():
    n=harness();tick(n,100.)
    n.model.verify.return_value={'valid':True,'reason':'ok','overlap':1.}
    tick(n,100.2)
    assert n.ready and n.state=='READY' and n.bad_checks==0
    assert n.last_quality['sensor_stamp_ns']=={'front':100200000000,'rear':100210000000}
    assert n.last_quality['observation_stamp_ns']==100210000000
    assert n.last_quality['pair_skew']==.01
    assert n.last_quality['evaluated_pose']==[0.,0.,0.]


def test_stationary_recheck_requires_repeated_good_observations():
    n=harness();n.ready=False;n.stopped=lambda:True
    n.model.verify.return_value={'valid':True,'reason':'ok','overlap':1.}
    n.last_save=100.;n.future=None
    tick(n,100.);assert not n.ready
    tick(n,100.5);assert not n.ready
    tick(n,101.1);assert n.ready
