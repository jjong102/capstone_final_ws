import math
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
from third_impact.return_control import ReturnControl
from third_impact.node import ImpactNode


def coarse_robot():
    n=NS(coarse_alignment=None,data={'goal':[0.,0.,0.]},alignment_diagnostics={},
         stopped=lambda:True,stop=Mock(),cloud=lambda:np.array([[10.,10.]]),
         return_grid=lambda:NS(rotation=lambda *a:True),return_command=Mock(),
         p={'footprint_half_length':.35,'footprint_half_width':.22,'obstacle_margin':.04})
    return n


def test_large_recorded_heading_rotates_in_place_before_icp():
    n=coarse_robot();pose=np.array([0.,0.,math.radians(156)])
    for i in range(500):
        n.return_command.reset_mock()
        with patch('third_impact.return_control.time.monotonic',return_value=100.+i*.05):
            busy=ReturnControl.coarse_align(n,pose)
        if not busy:break
        if n.return_command.called:
            _,v,w,reverse=n.return_command.call_args.args
            assert v==0 and w<0
            pose[2]+=w*.05
    assert n.coarse_alignment['phase']=='DONE'
    assert abs(pose[2])<.10


def test_coarse_waits_for_stop_and_rejects_wall_rotation():
    n=coarse_robot();n.stopped=lambda:False
    ReturnControl.coarse_align(n,np.array([0.,0.,2.7]))
    n.return_command.assert_not_called()
    n.stopped=lambda:True;n.return_grid=lambda:NS(rotation=lambda *a:False)
    ReturnControl.coarse_align(n,np.array([0.,0.,2.7]))
    assert n.coarse_alignment['phase']=='WAIT'
    n.return_command.assert_not_called()


def fine_robot():
    return NS(cloud=lambda:np.zeros((60,2)),pose=lambda:np.zeros(3),coarse_align=lambda p:False,
         scan_key=lambda:(1,1),align_last_key=None,alignment_recheck_until=0.,
         alignment_reject_since=None,alignment_valid_count=0,alignment_last_pose=None,
         alignment_diagnostics={},data={'goal':np.zeros(3),'reference':np.zeros((60,2))},
         stop=Mock(),p={'match_rmse':.025,'match_overlap':.65,'match_observability':.015,
             'match_max_condition':500.,'match_max_correction':.2,'match_max_angle_deg':15.})


def test_22cm_correction_stays_rejected_and_enters_reverify():
    n=fine_robot();fit=dict(pose=np.array([.22,0.,0.]),rmse=.019,overlap=.75,observability=.2,condition=9.)
    with patch('third_impact.node.register_scan',return_value=fit):
        with patch('third_impact.node.time.monotonic',return_value=100.):
            ImpactNode.align(n)
        assert '22.0cm > 허용 20.0cm' in n.detail
        n.scan_key=lambda:(2,2)
        with patch('third_impact.node.time.monotonic',return_value=106.):
            ImpactNode.align(n)
        assert n.alignment_diagnostics['phase']=='REVERIFY'
        assert n.alignment_recheck_until==108.
        assert n.align_velocity==(0.,0.)


def test_valid_fine_match_requires_three_fresh_observations():
    n=fine_robot();fit=dict(pose=np.array([.01,0.,0.]),rmse=.019,overlap=.75,observability=.2,condition=9.)
    with patch('third_impact.node.register_scan',return_value=fit):
        for i in range(3):
            n.scan_key=lambda i=i:(i,i)
            ImpactNode.align(n)
            assert n.alignment_valid_count==i+1
            assert n.align_velocity==(0.,0.)
