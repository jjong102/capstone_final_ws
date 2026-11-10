import math
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock, patch
import numpy as np
from third_impact.patrol_control import PatrolControl
from third_impact.core import travel_allowed, wrap
from third_impact.node import ImpactNode
from geometry_msgs.msg import Twist


def robot(goal, pose):
    n=NS(nav_target=list(goal),nav_pending=False,nav_handle=object(),nav_purpose='patrol',
         cancel_requested=False,patrol_visits=0,event=Mock(),next_attempt=0.,
         data={'boundary':[0,0,0,10,10],'rectangle':[4,4,0,1,1]},
         p={'footprint_half_length':.35,'footprint_half_width':.22,'obstacle_margin':.04,'patrol_tolerance':.05})
    n.init_patrol=MethodType(PatrolControl.init_patrol,n);n.init_patrol()
    n.command=np.zeros(2);n.actual=np.zeros(2)
    n.stop=MethodType(ImpactNode.stop,n)
    n.cancel_nav=lambda:setattr(n,'cancel_requested',True)
    n.velocity_pub=NS(publish=lambda m:n.command.__setitem__(slice(None),[m.linear.x,m.angular.z]))
    n.stopped=lambda:abs(n.actual[0])<.005 and abs(n.actual[1])<.01
    n.cloud=lambda:np.array([[20.,20.]])
    n.return_grid=lambda:NS(clear=lambda p:True)
    n.motion_allowed=lambda p,vx=0,vy=0,angular=0:travel_allowed(p,n.data['boundary'],n.data['rectangle'],vx,vy,.35,.22,angular=angular)
    n.recovery_heading=MethodType(PatrolControl.recovery_heading,n)
    n.safe_patrol_command=MethodType(PatrolControl.safe_patrol_command,n)
    return n


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


def output(n, pose, v, w, now):
    command=Twist();command.linear.x=v;command.angular.z=w
    with patch('third_impact.patrol_control.time.monotonic',return_value=now):
        return PatrolControl.publish_patrol_command(n,pose,command)


def test_small_steering_reversal_keeps_forward_motion_with_bounded_acceleration():
    n=robot([3.,0.,0.],np.zeros(3))
    n.patrol_output=(.3,.16);n.patrol_output_stamp=100.
    history=[]
    for i in range(1,21):
        assert output(n,np.zeros(3),.3,-.16,100.+i*.05)
        history.append(n.command.copy())
    assert all(v>0 for v,w in history)
    assert history[0][1]>0 and history[-1][1]<0
    assert np.max(np.abs(np.diff(np.array([[.3,.16],*history]),axis=0)[:,1]))<=.040001


def test_speed_limit_stays_point_four_and_long_gap_cannot_jump_to_full_speed():
    n=robot([3.,0.,0.],np.zeros(3))
    for i in range(30): assert output(n,np.zeros(3),.4,0.,100.+i*.05)
    assert abs(n.command[0]-.4)<1e-8
    n.stop()
    assert output(n,np.zeros(3),.4,0.,200.)
    assert n.command[0]<=.065001


def test_both_requested_and_smoothed_arcs_are_collision_checked():
    for checks in ([False], [True,False]):
        n=robot([3.,0.,0.],np.zeros(3))
        n.patrol_output=(.3,.2);n.patrol_output_stamp=100.
        n.safe_patrol_command=Mock(side_effect=checks)
        assert not output(n,np.zeros(3),.3,-.2,100.05)
        assert np.all(n.command==0) and n.patrol_output==(0.,0.)


def test_real_lidar_obstacle_causes_immediate_stop():
    n=robot([3.,0.,0.],np.zeros(3))
    n.patrol_output=(.3,0.);n.patrol_output_stamp=100.
    n.cloud=lambda:np.array([[.4,0.]])
    assert not output(n,np.zeros(3),.3,0.,100.05)
    assert np.all(n.command==0)


def test_near_goal_decelerates_without_cancel_or_forcing_turn_only():
    n=robot([.3,0.,math.pi],np.zeros(3))
    n.patrol_output=(.2,.1);n.patrol_output_stamp=100.
    assert output(n,np.zeros(3),.2,.1,100.05)
    assert n.command[0]>0 and n.command[1]>0
    assert not n.cancel_requested and n.patrol_near is None


def test_position_approach_reaches_five_cm_without_intermediate_stop():
    n=robot([.6,0.,math.pi],np.zeros(3));pose=np.zeros(3)
    for i in range(500):
        if np.linalg.norm(pose[:2]-n.nav_target[:2])<=.05: break
        assert output(n,pose,.4,0.,100.+i*.05)
        assert n.command[0]>0
        pose[0]+=n.command[0]*.05
    assert np.linalg.norm(pose[:2]-n.nav_target[:2])<=.05
    assert not n.cancel_requested


def test_stale_sensor_stops_without_ramp_and_resume_starts_from_zero():
    n=robot([3.,0.,0.],np.zeros(3));n.patrol_output=(.4,0.);n.patrol_output_stamp=100.
    n.cloud=lambda:None
    assert not output(n,np.zeros(3),.4,0.,100.05)
    assert np.all(n.command==0)
    n.cloud=lambda:np.array([[20.,20.]])
    assert output(n,np.zeros(3),.4,0.,100.10)
    assert 0<n.command[0]<=.032501
