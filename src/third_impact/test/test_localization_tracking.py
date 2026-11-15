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


def test_occlusion_hold_never_starts_global_search_and_recovers():
    n=harness(); n.tracking_active=True; n.last_save=100.
    n.pool=Mock();n.cancel_search=Mock()
    n.tracking_quality=Mock(return_value={'valid':False,'insufficient':True,'reason':'occluded'})
    for t in (100.,101.,110.,130.):
        tick(n,t)
        assert n.state=='OCCLUDED_HOLD' and not n.ready and n.tracking_active
    n.pool.submit.assert_not_called()
    # A repeated observation cannot initiate recovery either.
    with patch('third_impact.localization.time.monotonic',return_value=130.1):
        LocalizationBootstrap.tick(n)
    assert n.state=='OCCLUDED_HOLD'
    n.tracking_quality.return_value={'valid':True,'reason':'ok','occluded':80}
    n.last_save=131.
    tick(n,131.);assert not n.ready
    tick(n,131.5);assert not n.ready
    tick(n,132.1);assert n.ready
    tick(n,132.3);assert n.state=='OCCLUDED_TRACKING' and n.ready
    n.pool.submit.assert_not_called()


def test_pose_unavailable_during_established_tracking_holds_without_seed():
    n=harness();n.tracking_active=True;n.pose=lambda:None;n.pool=Mock()
    tick(n,100.)
    assert n.state=='TRACKING_HOLD' and not n.ready and n.tracking_active
    n.pool.submit.assert_not_called()


def test_tracking_rejects_amcl_jump_even_when_scan_verification_passes():
    n=harness();n.tracking_anchor=(np.zeros(3),np.zeros(3))
    n.model.verify.return_value={'valid':True,'reason':'ok','overlap':1.}
    n.tf=Mock()
    n.tf.lookup_transform.return_value=NS(transform=NS(
        translation=NS(x=0.,y=0.), rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    quality=LocalizationBootstrap.tracking_quality(n,np.array([.6,0.,0.]))
    assert not quality['valid'] and not quality['insufficient']
    np.testing.assert_array_equal(n.tracking_anchor[0],np.zeros(3))
