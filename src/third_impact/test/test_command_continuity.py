from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock, patch
import json
import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.time import Time
from third_impact.node import ImpactNode
from third_impact.progress_watch import ProgressWatch


def test_cloud_fixed_for_cycle_and_expiry_still_stops():
    stamp=Time(seconds=100.).to_msg()
    tf=TransformStamped();tf.transform.rotation.w=1.
    age=[.05]
    n=NS(_pose_snapshot_active=True,_cycle_cloud_done=False,
        _pose_stamp_ns=100000000000,pose_diagnostics={'tf_stamp':100.},p={'scan_timeout':.45,'odom_timeout':.3},
        selected_scans=lambda:{k:(0.,stamp,np.array([[1.,0.]])) for k in ('front','rear')},
        message_age=lambda s:age[0], tf=Mock())
    n.tf.lookup_transform_full.return_value=tf
    first=ImpactNode.cloud(n)
    n.tf.lookup_transform_full.side_effect=AssertionError('must not repeat conversion within cycle')
    assert ImpactNode.cloud(n) is first
    assert n.tf.lookup_transform_full.call_count==2
    age[0]=.5
    assert ImpactNode.cloud(n) is None


def test_fresh_command_received_while_control_blocked_is_consumed_next_tick():
    n=NS(_nav_accept_after=5.,nav_velocity_stamp=0.,pose=lambda:np.zeros(3),
         selected_scans=lambda:None,tick=Mock())
    msg=Twist();msg.linear.x=.4
    with patch('third_impact.node.time.monotonic',return_value=10.):
        ImpactNode.on_nav_velocity(n,msg)
    with patch('third_impact.node.time.monotonic',return_value=10.05):
        ImpactNode.timed_tick(n)
    assert n.nav_velocity is msg and n.nav_velocity_stamp==10.
    n.nav_velocity_stamp=0.;n._nav_accept_after=11.
    ImpactNode.timed_tick(n)
    assert n.nav_velocity_stamp==0.  # Old mission command cannot revive after cancel.


def test_stop_publishes_diagnostic_and_immediate_zero():
    n=NS(state='PATROL',nav_velocity_stamp=0.,velocity_pub=Mock(),event_pub=Mock())
    ImpactNode.stop(n,'nav_command_unavailable')
    cmd=n.velocity_pub.publish.call_args.args[0]
    assert cmd.linear.x==0. and cmd.angular.z==0.
    event=json.loads(n.event_pub.publish.call_args.args[0].data)
    assert event['stop']['reason']=='nav_command_unavailable'


def test_command_gaps_excluded_but_stationary_commanded_motion_not_excused():
    w=ProgressWatch();w.update(0.,[0,0,0],(.4,0),True)
    excluded,progress=w.update(5.,[0,0,0],(.4,0),False)
    assert excluded==5. and not progress
    assert w.update(6.,[0,0,0],(.4,0),True)==(0.,False)


def test_rotation_credit_requires_correct_direction_and_is_bounded():
    w=ProgressWatch();w.update(0.,[0,0,0],(0,.2),True)
    assert not w.update(1.,[0,0,-.2],(0,.2),True)[1]
    assert w.update(2.,[0,0,.2],(0,.2),True)[1]
    assert not w.update(33.,[0,0,.4],(0,.2),True)[1]


def test_recorded_epoch_stamp_remains_exact_at_latest_tf_boundary():
    from tf2_ros import Buffer, ExtrapolationException
    from third_impact.pose_tracking import PoseBridge
    import pytest
    exact = 1794300321080120754  # 20261110_174510 bag: +334ns float roundtrip failed.
    buffer = Buffer()
    for ns in (exact-200000000, exact):
        for parent,child in (('map','odom'),('odom','base_link')):
            tf=TransformStamped();tf.header.frame_id=parent;tf.child_frame_id=child
            tf.header.stamp=Time(nanoseconds=ns).to_msg();tf.transform.rotation.w=1.
            buffer.set_transform(tf,'recorded_boundary')
    rounded=Time(seconds=exact*1e-9)
    assert rounded.nanoseconds-exact==334
    with pytest.raises(ExtrapolationException):
        buffer.lookup_transform_full('base_link',rounded,'base_link',Time(nanoseconds=exact-100000000),'odom')
    now=[exact+20000000]
    stamp=Time(nanoseconds=exact-100000000).to_msg()
    n=NS(tf=buffer,pose_bridge=PoseBridge(),p={'pose_timeout':1.,'scan_timeout':.45,'odom_timeout':.3},
         get_clock=lambda:NS(now=lambda:Time(nanoseconds=now[0])),
         selected_scans=lambda:{k:(0.,stamp,np.array([[1.,0.]])) for k in ('front','rear')},
         message_age=lambda s:(now[0]-Time.from_msg(s).nanoseconds)*1e-9)
    np.testing.assert_allclose(ImpactNode.pose(n),[0,0,0])
    assert n._pose_stamp_ns==exact
    n._pose_snapshot_active=True;n._cycle_cloud_done=False
    assert ImpactNode.cloud(n) is not None
    assert n._cloud_key[-1]==exact
    n._cycle_cloud_done=False;n._pose_stamp_ns=exact+1
    assert ImpactNode.cloud(n) is None  # Genuine future lookup is still rejected.
    n._pose_stamp_ns=exact;n._cycle_cloud_done=False;now[0]=exact+600000000
    assert ImpactNode.cloud(n) is None  # A stale observation is never accepted.


def test_control_status_preserves_failed_tick_not_later_live_assessment():
    n=NS(pose=lambda:np.zeros(3),selected_scans=lambda:None,
         pose_diagnostics={'source':'tf'},health_diagnostics={})
    def tick():
        n._cycle_ready=False
        n.health_diagnostics={'reason':'scan_motion_tf_unavailable'}
    n.tick=tick
    ImpactNode.timed_tick(n)
    n.health_diagnostics['reason']='ready'
    assert n.control_status['ready'] is False
    assert n.control_status['health']['reason']=='scan_motion_tf_unavailable'
