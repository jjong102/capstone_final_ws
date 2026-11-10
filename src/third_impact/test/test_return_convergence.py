"""Whole-state regressions for the 20261110_164236 return failure."""
import math
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock, patch
import numpy as np
from third_impact.core import relative, wrap
from third_impact.node import ImpactNode
from third_impact.return_control import ReturnControl
from third_impact.return_route import FootprintMap


def robot():
    p=np.array([.01317,.00837,.02982]);command=np.zeros(2);actual=np.zeros(2)
    n=NS(return_epoch=0, p=dict(xy_tolerance=.02,yaw_tolerance_deg=5.,settle_seconds=1.,
        max_align_v=.035,max_align_w=.12,match_rmse=.025,match_overlap=.65,
        match_observability=.015,match_max_condition=500.,match_max_correction=.2,
        match_max_angle_deg=15.,footprint_half_length=.35,footprint_half_width=.22,
        obstacle_margin=.04,return_obstacle_margin=.01),
        data={'goal':np.zeros(3),'reference':np.zeros((60,2))},intent='return',
        nav_pending=False,nav_handle=None,manual=False,healthy=lambda:True,
        stop=lambda:command.fill(0),event=Mock(),arrival_pub=Mock(),pose=lambda:p.copy(),
        cloud=lambda:np.array([[20.,20.]]),stopped=lambda:abs(actual[0])<.005 and abs(actual[1])<.01,
        return_grid=lambda:FootprintMap(np.zeros((160,160)),.05,[-4.,-4.,0.]),
        velocity_pub=NS(publish=lambda msg:command.__setitem__(slice(None),[msg.linear.x,msg.angular.z])),
        odom=NS(twist=NS(twist=NS(linear=NS(x=0.),angular=NS(z=0.)))),
        show_return_path=Mock(),return_local_pub=Mock(),return_path_pub=Mock(),motion_allowed=lambda *a,**k:True)
    for method in ('reset_return','reapproach_return','position_return','coarse_align',
                   'start_micro_adjustment','tick_micro_adjustment','return_command','install_return_plan'):
        setattr(n,method,MethodType(getattr(ReturnControl,method),n))
    n.reset_return();n.return_phase='ALIGN';n.position_alignment={'done':True}
    n.coarse_alignment={'phase':'DONE'};n.align_last_key=None;n.settled_since=None
    n.align=MethodType(ImpactNode.align,n)
    return n,p,command,actual


def test_recorded_map_icp_disagreement_converges_and_publishes_arrival_once():
    n,p,cmd,actual=robot()
    # Map thinks (1.3cm,0.8cm); saved scan sees (0.3cm,2.8cm).
    reference_goal=np.array([.01,-.02,0.]);saved=n.data['goal'].copy()
    states=set()
    def fit(*args):
        return dict(pose=relative(reference_goal,p),rmse=.015,overlap=.99,observability=.36,condition=5.5)
    for i in range(1800):
        now=100.+i*.05;n.scan_key=lambda i=i:(i,i)
        n.odom.twist.twist.linear.x=float(actual[0]);n.odom.twist.twist.angular.z=float(actual[1])
        with patch('third_impact.return_control.time.monotonic',return_value=now), patch('third_impact.node.register_scan',side_effect=fit):
            ReturnControl.tick_return(n,p)
        states.add(n.alignment_diagnostics.get('phase'))
        actual+=.3*(cmd-actual)
        p[:2]+=actual[0]*.05*np.array([math.cos(p[2]),math.sin(p[2])]);p[2]=wrap(p[2]+actual[1]*.05)
        if n.intent=='hold':break
        assert not n.return_alignment_wait
    assert n.intent=='hold', (p,n.detail,states)
    assert np.linalg.norm(relative(reference_goal,p)[:2])<.02
    assert abs(relative(reference_goal,p)[2])<=math.radians(5)
    assert 'MICRO_OUT' in states and 'MICRO_IN' in states
    n.mode='run';n.publish_navigation_map=Mock()
    for _ in range(5):
        ImpactNode.tick(n)
    n.arrival_pub.publish.assert_called_once()
    assert np.array_equal(saved,n.data['goal']) and n.return_reapproaches==0


def test_close_goal_cannot_skip_reapproach_and_duplicate_requests_do_not_count():
    n,p,cmd,actual=robot();n.request_return_path=Mock();n.align=Mock()
    n.reapproach_return('test')
    for _ in range(5):
        n.reapproach_return('duplicate')
        ReturnControl.tick_return(n,p)
    assert n.return_phase=='REAPPROACH_PLAN' and n.return_reapproaches==0
    n.align.assert_not_called()
    assert n.request_return_path.call_count==5
    n.install_return_plan(dict(path=np.array([[0,0],[-.5,0]]),entry=np.array([-.5,0,0]),reverse=False,docking=True))
    assert n.return_reapproaches==1 and n.return_phase=='FORWARD'


def test_micro_collision_stops_and_bounded_progress_requests_reapproach():
    n,p,cmd,actual=robot()
    with patch('third_impact.return_control.time.monotonic',return_value=100.):
        n.start_micro_adjustment(p,np.array([.003,.028,.03]))
    n.cloud=lambda:np.array([[0.,0.]])
    for now in (100.1,107.):
        with patch('third_impact.return_control.time.monotonic',return_value=now):
            n.tick_micro_adjustment(p)
        assert np.all(cmd==0)
    assert n.return_phase=='REAPPROACH_PLAN'
    n.arrival_pub.publish.assert_not_called()


def test_three_actual_reapproaches_stop_without_fourth_launch():
    n,p,cmd,actual=robot()
    for _ in range(3):
        n.reapproach_return('test')
        n.install_return_plan(dict(path=np.array([[0,0],[-.5,0]]),entry=np.array([-.5,0,0]),reverse=False,docking=True))
    n.reapproach_return('test')
    assert n.return_reapproaches==3 and n.return_alignment_wait
    n.arrival_pub.publish.assert_not_called()


def test_delayed_drive_tracks_without_sustained_steering_oscillation():
    from collections import deque
    from third_impact.return_route import dense_path, tracking_command
    # 700ms transport/drive delay plus first-order wheel response, 15cm offset.
    path=dense_path([[0.,0.],[10.,0.]])
    pose=np.array([0.,.15,0.]);actual=np.zeros(2);last=np.zeros(2);index=0
    pending=deque([np.zeros(2) for _ in range(14)]);history=[]
    for _ in range(500):
        v,w,index=tracking_command(pose,path,index,False,measured_speed=actual[0])
        command=np.array([v,w])
        last+=np.clip(command-last,-np.array([.35,.4])*.05,np.array([.35,.4])*.05)
        pending.append(last.copy());actual+=.25*(pending.popleft()-actual)
        pose[:2]+=actual[0]*.05*np.array([math.cos(pose[2]),math.sin(pose[2])])
        pose[2]=wrap(pose[2]+actual[1]*.05)
        history.append([*pose,*last])
    history=np.asarray(history)
    turns=history[abs(history[:,4])>.08,4]
    assert np.count_nonzero(turns[1:]*turns[:-1]<0)<=3
    assert np.sqrt(np.mean(history[100:,1]**2))<.05
    assert pose[0]>8. and abs(pose[1])<.01
    assert np.max(abs(np.diff(history[:,4])))<=.4*.05+1e-9


def test_pause_discards_frozen_micro_target_and_requires_fresh_verification():
    n,p,cmd,actual=robot()
    n.start_micro_adjustment(p,np.array([.003,.028,.03]))
    n.alignment_valid_count=3
    ReturnControl.pause_return(n)
    assert n.micro_adjustment is None and n.alignment_valid_count==0
    assert n.position_alignment is None and n.coarse_alignment is None
    n.arrival_pub.publish.assert_not_called()


def test_bad_scan_after_micro_cannot_emit_arrival():
    n,p,cmd,actual=robot();n.scan_key=lambda:(1,1)
    n.alignment_valid_count=3;n.settled_since=1.
    with patch('third_impact.node.register_scan',return_value=None):
        n.align()
    assert n.intent=='return' and n.alignment_valid_count==0
    assert np.all(cmd==0)
    n.arrival_pub.publish.assert_not_called()


def test_small_readiness_pause_preserves_route_but_manual_pause_replans():
    for manual in (False,True):
        n,p,cmd,actual=robot();p[:]=[1.,0.,0.]
        n.return_phase='FORWARD';n.return_paused_pose=p.copy()
        n.return_pause_manual=manual
        route=np.array([[.99,0.],[1.02,0.]])
        n.return_route=route;n.request_return_path=Mock()
        ReturnControl.tick_return(n,p)
        if manual:
            assert n.return_route is None
            n.request_return_path.assert_called_once()
        else:
            assert n.return_route is route
            n.request_return_path.assert_not_called()
