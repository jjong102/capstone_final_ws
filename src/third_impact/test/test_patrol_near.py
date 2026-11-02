import math
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock, patch
import numpy as np
from third_impact.patrol_control import PatrolControl
from third_impact.core import travel_allowed, wrap


def robot(goal, pose):
    n=NS(nav_target=list(goal),nav_pending=False,nav_handle=object(),nav_purpose='patrol',
         cancel_requested=False,patrol_visits=0,event=Mock(),next_attempt=0.,
         data={'boundary':[0,0,0,10,10],'rectangle':[4,4,0,1,1]},
         p={'footprint_half_length':.35,'footprint_half_width':.22,'obstacle_margin':.04,'patrol_tolerance':.05})
    n.init_patrol=MethodType(PatrolControl.init_patrol,n);n.init_patrol()
    n.command=np.zeros(2);n.actual=np.zeros(2)
    n.stop=lambda:n.command.fill(0)
    n.cancel_nav=lambda:setattr(n,'cancel_requested',True)
    n.velocity_pub=NS(publish=lambda m:n.command.__setitem__(slice(None),[m.linear.x,m.angular.z]))
    n.stopped=lambda:abs(n.actual[0])<.005 and abs(n.actual[1])<.01
    n.cloud=lambda:np.array([[20.,20.]])
    n.return_grid=lambda:NS(clear=lambda p:True)
    n.motion_allowed=lambda p,vx=0,vy=0,angular=0:travel_allowed(p,n.data['boundary'],n.data['rectangle'],vx,vy,.35,.22,angular=angular)
    n.recovery_heading=MethodType(PatrolControl.recovery_heading,n)
    n.safe_patrol_command=MethodType(PatrolControl.safe_patrol_command,n)
    return n


def simulate(angle, goal_yaw, jitter=False, overshoot=False):
    pose=np.array([0.,0.,0.]);goal=[.3*math.cos(angle),.3*math.sin(angle),goal_yaw]
    n=robot(goal,pose);history=[];crossed=False
    for i in range(1800):
        now=100.+i*.05
        measured=pose.copy()
        if jitter:measured[:2]+=.002*np.array([math.sin(i),math.cos(i)])
        with patch('third_impact.patrol_control.time.monotonic',return_value=now):
            PatrolControl.tick_patrol_near(n,measured)
        history.append(n.command.copy())
        assert n.command[0]==0 or n.command[1]==0  # No simultaneous drive/turn.
        if i<4:assert np.all(n.command==0)  # Cancel/result handoff barrier.
        if i==3:n.nav_handle=None
        n.actual+=np.clip(n.command-n.actual,[-.3*.05,-.8*.05],[.3*.05,.8*.05])
        pose[:2]+=n.actual[0]*.05*np.array([math.cos(pose[2]),math.sin(pose[2])])
        pose[2]=wrap(pose[2]+n.actual[1]*.05)
        if overshoot and not crossed and n.command[0]>.02:
            pose[:2]=np.asarray(goal[:2])+.08*np.array([math.cos(pose[2]),math.sin(pose[2])]);crossed=True
        if n.patrol_visits:
            assert n.patrol_visits==1
            assert np.linalg.norm(pose[:2]-goal[:2])<.055
            return n,pose,np.asarray(history),now-100.
        assert n.patrol_recovery!='WAIT',n.detail
    raise AssertionError('Failed to reach goal')


def test_left_86_degrees_reaches_position_without_goal_yaw():
    a=simulate(math.radians(86),0.)
    b=simulate(math.radians(86),math.pi)
    assert np.array_equal(a[2],b[2])  # Goal yaw has no influence whatsoever.


def test_behind_170_degrees_is_not_cut_off_after_12_seconds():
    n,pose,history,elapsed=simulate(math.radians(170),-1.)
    assert elapsed>12. and n.patrol_visits==1


def test_goal_overshoot_stops_before_reversing_turn_and_recovers():
    simulate(0.,2.,overshoot=True)


def test_small_localization_jitter_does_not_prevent_arrival():
    simulate(math.radians(86),2.,jitter=True)


def test_near_control_does_not_cross_exclusion():
    n=robot([.3,0,0],np.zeros(3))
    n.recovery_heading=MethodType(PatrolControl.recovery_heading,n)
    n.safe_patrol_command=Mock(return_value=False)
    n.nav_handle=None;n.patrol_near=dict(goal=np.array([.3,0]),phase='DRIVE',started=100.,progress=100.,best_distance=.3,turn_sign=0,turn_deadline=None,best_angle=math.inf,turn_progress=100.)
    with patch('third_impact.patrol_control.time.monotonic',return_value=101.):
        PatrolControl.tick_patrol_near(n,np.zeros(3))
    assert np.all(n.command==0)
    with patch('third_impact.patrol_control.time.monotonic',return_value=107.):
        PatrolControl.tick_patrol_near(n,np.zeros(3))
    assert n.patrol_recovery=='WAIT' and n.patrol_visits==0


def test_no_motion_wait_reason_is_not_reported_as_zone_collision():
    n=robot([0,.3,0],np.zeros(3));n.nav_handle=None
    n.patrol_near=dict(goal=np.array([0,.3]),phase='STOP',started=100.,progress=100.,best_distance=.3,turn_sign=0,turn_deadline=None,best_angle=math.inf,turn_progress=100.)
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):PatrolControl.tick_patrol_near(n,np.zeros(3))
    with patch('third_impact.patrol_control.time.monotonic',return_value=105.):PatrolControl.tick_patrol_near(n,np.zeros(3))
    assert n.patrol_recovery=='WAIT' and '진행 정체' in n.patrol_wait_reason


def test_measured_rotation_must_stop_before_opposite_turn():
    n=robot([0,.3,0],np.zeros(3));n.nav_handle=None
    n.actual[1]=-.2
    n.patrol_near=dict(goal=np.array([0,.3]),phase='STOP',started=100.,progress=100.,best_distance=.3,turn_sign=0,turn_deadline=None,best_angle=math.inf,turn_progress=100.)
    with patch('third_impact.patrol_control.time.monotonic',return_value=101.):
        PatrolControl.tick_patrol_near(n,np.zeros(3))
    assert np.all(n.command==0) and n.patrol_near['phase']=='STOP'
    n.actual[:]=0
    with patch('third_impact.patrol_control.time.monotonic',return_value=102.):
        PatrolControl.tick_patrol_near(n,np.zeros(3))
    assert n.command[0]==0 and n.command[1]>0


def test_recovery_170_degree_turn_can_settle_after_more_than_15_seconds():
    pose=np.zeros(3);angle=math.radians(170)
    target=np.array([math.cos(angle),math.sin(angle)])
    n=robot([*target,0],pose);n.nav_handle=None
    n.patrol_path=np.array([[0.,0.],target*.2,target*.4,target]);n.patrol_path_stamp=100.
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        PatrolControl.begin_patrol_recovery(n)
    for i in range(600):
        now=100.+i*.05
        with patch('third_impact.patrol_control.time.monotonic',return_value=now):
            PatrolControl.tick_patrol_recovery(n,pose)
        n.actual+=np.clip(n.command-n.actual,[-.3*.05,-.3*.05],[.3*.05,.3*.05])
        pose[2]=wrap(pose[2]+n.actual[1]*.05)
        if n.patrol_recovery is None:
            assert abs(wrap(angle-pose[2]))<.10
            return
        assert n.patrol_recovery!='WAIT',n.detail
    raise AssertionError('Recovery did not finish')
