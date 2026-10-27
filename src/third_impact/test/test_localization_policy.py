from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time
import numpy as np
from third_impact.localization import LocalizationBootstrap
from third_impact.node import ImpactNode


def completed_search(confident=True, moved=False, epoch=0):
    future=Future()
    future.set_result({'confident':confident,'reason':'unique_match' if confident else 'ambiguous',
                       'pose':np.array([1.,2.,.5])})
    n=NS(model=Mock(), cloud=lambda:np.zeros((80,2)), odom=object(), odom_time=time.monotonic(),
         amcl_active=True, pose=lambda:None, scans={'front':(0,1,None),'rear':(0,1,None)},
         ready=False, stopped=lambda:True, future=future, job_epoch=0, epoch=epoch,
         job_odom=np.zeros(3), odom_pose=lambda:np.array([.2 if moved else 0.,0.,0.]),
         seed=Mock(), set_state=Mock(), next_search=0., last_search={})
    n.model.verify.return_value={'valid':True}
    return n


def test_unique_current_observation_is_required_before_seed():
    n=completed_search()
    LocalizationBootstrap.tick(n)
    n.seed.assert_called_once()
    n.model.verify.assert_called_once()  # Recheck current scan, not just the old snapshot.


def test_motion_during_global_search_rejects_stale_solution():
    n=completed_search(moved=True)
    LocalizationBootstrap.tick(n)
    n.seed.assert_not_called()
    assert n.set_state.call_args.args[0]=='WAIT_STOP'


def test_manual_initialpose_or_map_change_invalidates_inflight_result():
    n=completed_search(epoch=1)
    LocalizationBootstrap.tick(n)
    n.seed.assert_not_called()


def test_ambiguous_scene_never_publishes_initialpose():
    n=completed_search(confident=False)
    LocalizationBootstrap.tick(n)
    n.seed.assert_not_called()
    assert n.set_state.call_args.args[0]=='AMBIGUOUS'


def test_changed_scene_rejects_even_previously_confident_result():
    n=completed_search()
    n.model.verify.return_value={'valid':False}
    LocalizationBootstrap.tick(n)
    n.seed.assert_not_called()


def test_mission_gate_rejects_missing_stale_or_unverified_localization():
    n=NS(p={'require_localization_ready':True}, localization_stamp=0.,localization_info={'ready':True})
    assert not ImpactNode.healthy(n)
    n.localization_stamp=time.monotonic(); n.localization_info={'ready':False}
    assert not ImpactNode.healthy(n)
