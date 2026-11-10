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
    return NS(tick_micro_adjustment=lambda p:False,return_command=Mock(),position_return=lambda p:False, fine_alignment_started=None,return_reapproaches=0,cloud=lambda:np.zeros((60,2)),pose=lambda:np.zeros(3),coarse_align=lambda p:False,
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


def test_recorded_lateral_error_reaches_position_before_final_yaw():
    from types import MethodType
    from third_impact.core import wrap
    pose=np.array([.013505,.083087,-.047672])
    command=np.zeros(2)
    n=coarse_robot()
    n.position_alignment=None;n.return_reapproaches=0
    n.p.update(xy_tolerance=.02,yaw_tolerance_deg=5.,max_align_v=.035,max_align_w=.12)
    n.stop=lambda:command.fill(0)
    n.reapproach_return=Mock(side_effect=AssertionError('Open space should converge'))
    n.return_command=lambda p,v,w,reverse:command.__setitem__(slice(None),[v,w])
    moved=False;phase='position'
    for i in range(2200):
        with patch('third_impact.return_control.time.monotonic',return_value=100.+i*.05):
            if phase=='position':
                if not ReturnControl.position_return(n,pose):phase='yaw'
            elif not ReturnControl.coarse_align(n,pose):break
        moved |= abs(command[0])>0
        pose[:2]+=command[0]*.05*np.array([math.cos(pose[2]),math.sin(pose[2])])
        pose[2]=wrap(pose[2]+command[1]*.05)
    assert moved and np.linalg.norm(pose[:2])<.02 and abs(pose[2])<.10
    assert n.coarse_alignment['phase']=='DONE'


def test_position_obstacle_no_progress_triggers_bounded_reapproach():
    n=coarse_robot();n.position_alignment=None;n.return_reapproaches=0
    n.p.update(xy_tolerance=.02,yaw_tolerance_deg=5.,max_align_v=.035,max_align_w=.12)
    n.reapproach_return=Mock()
    for now in (100.,107.):
        with patch('third_impact.return_control.time.monotonic',return_value=now):
            ReturnControl.position_return(n,np.array([.01,.083,0.]))
    n.reapproach_return.assert_called_once()


def test_fine_overlap_flapping_cannot_wait_forever():
    n=fine_robot();n.fine_alignment_started=100.;n.reapproach_return=Mock()
    with patch('third_impact.node.time.monotonic',return_value=116.):
        ImpactNode.align(n)
    n.reapproach_return.assert_called_once()


def test_reapproach_budget_never_reports_arrival():
    n=NS(stop=Mock(),return_epoch=0,return_reapproaches=0,event=Mock(),arrival_pub=Mock(),
         return_alignment_wait=False)
    for i in range(4):
        ReturnControl.reapproach_return(n,'scan mismatch')
        if i<3:
            ReturnControl.install_return_plan(n,dict(path=np.array([[0,0],[1,0]]),entry=np.zeros(3),reverse=False))
    assert n.return_alignment_wait and n.state=='RETURN_WAIT'
    n.arrival_pub.publish.assert_not_called()


def test_coarse_done_rechecks_large_heading_without_overwriting_fine_diagnostics():
    n=coarse_robot();n.coarse_alignment={'phase':'DONE'}
    n.alignment_diagnostics={'phase':'FINE_VERIFY','updated_at':10.}
    assert not ReturnControl.coarse_align(n,np.array([0.,0.,.03]))
    assert n.alignment_diagnostics['phase']=='FINE_VERIFY'
    assert ReturnControl.coarse_align(n,np.array([0.,0.,1.07]))
    assert n.coarse_alignment is None


def test_arrival_still_requires_tight_pose_and_one_second_stopped():
    n=fine_robot();n.velocity_pub=Mock();n.arrival_pub=Mock();n.event=Mock()
    n.stopped=lambda:True;n.motion_allowed=lambda *a,**kw:True
    n.settled_since=None;n.intent='return';n.state='ALIGN'
    n.p.update(xy_tolerance=.02,yaw_tolerance_deg=5.,settle_seconds=1.,
               max_align_v=.035,max_align_w=.12,footprint_half_length=.35,
               footprint_half_width=.22,obstacle_margin=.04)
    fit=dict(pose=np.array([.01,0.,math.radians(4)]),rmse=.019,overlap=.9,
             observability=.2,condition=9.)
    with patch('third_impact.node.register_scan',return_value=fit):
        for i in range(20):
            n.scan_key=lambda i=i:(i,i)
            with patch('third_impact.node.time.monotonic',return_value=100.+i*.1):
                ImpactNode.align(n)
            if i<13:n.arrival_pub.publish.assert_not_called()
            if n.intent=='hold':break
    assert n.intent=='hold' and n.state=='HOLD'
    n.arrival_pub.publish.assert_called_once()
