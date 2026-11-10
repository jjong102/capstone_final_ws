"""Regression for 20261110_170916 transient TF/readiness stops."""
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from tf2_ros import LookupException
from third_impact.pose_tracking import PoseBridge
from third_impact.node import ImpactNode


def test_bridge_uses_odometry_and_never_extends_its_own_lifetime():
    bridge=PoseBridge();bridge.update([1.,2.,np.pi/2],[0.,0.,0.],100.)
    for dt in (.05,.1,.19):
        pose,why=bridge.estimate([.4*dt,0.,0.],100.+dt,100.+dt)
        np.testing.assert_allclose(pose,[1.,2.+.4*dt,np.pi/2])
        assert why=='odom_bridge'
    assert bridge.estimate([.08,0.,0.],100.21,100.21)[0] is None


def test_bridge_rejects_stale_odom_jump_and_clock_reset():
    b=PoseBridge();b.update([0.,0.,0.],[0.,0.,0.],100.)
    for odom,stamp,now in (([0,0,0],100.,100.19),([.2,0,0],100.1,100.1),
                           ([0,0,.2],100.1,100.1),([0,0,0],99.,99.)):
        assert b.estimate(odom,stamp,now)[0] is None


def node():
    od=Odometry();od.header.frame_id='odom';od.child_frame_id='base_link'
    od.header.stamp=Time(seconds=100.1).to_msg();od.pose.pose.orientation.w=1.
    od.pose.pose.position.x=.04
    n=NS(tf=Mock(),pose_bridge=PoseBridge(),odom=od,intent='return',return_phase='FORWARD',
         manual=False,p={'pose_timeout':1.}, get_clock=lambda:NS(now=lambda:Time(seconds=100.1)))
    n.pose_bridge.update([1,2,0],[0,0,0],100.)
    n.tf.lookup_transform.side_effect=LookupException('injected short lookup gap')
    n.pose=MethodType(ImpactNode.pose,n)
    n.selected_scans=lambda:None
    return n


def test_transient_tf_failure_can_bridge_but_final_alignment_cannot():
    n=node()
    np.testing.assert_allclose(n.pose(),[1.04,2.,0.])
    assert n.pose_diagnostics['source']=='odom_bridge'
    assert n.pose_diagnostics['exception']=='injected short lookup gap'
    for phase in ('FINAL','ALIGN','REVERSE','ROTATE'):
        n.return_phase=phase
        assert n.pose() is None
    n.return_phase='FORWARD';n.manual=True
    assert n.pose() is None


def test_one_tick_uses_one_pose_even_with_alternating_lookup_failures():
    n=node();seen=[]
    n.tick=lambda:seen.extend([n.pose(),n.pose(),n.pose()])
    ImpactNode.timed_tick(n)
    assert n.tf.lookup_transform.call_count==1
    assert all(np.array_equal(p,[1.04,2.,0.]) for p in seen)
    assert not n._pose_snapshot_active
    n.get_clock=lambda:NS(now=lambda:Time(seconds=100.3))
    assert n.pose() is None


def test_direct_tf_recovery_reanchors_and_records_age():
    n=node();tf=TransformStamped();tf.header.stamp=Time(seconds=100.09).to_msg()
    tf.transform.rotation.w=1.;tf.transform.translation.x=3.
    n.tf.lookup_transform.side_effect=None;n.tf.lookup_transform.return_value=tf
    np.testing.assert_allclose(n.pose(),[3.,0.,0.])
    assert n.pose_diagnostics['source']=='tf'
    assert abs(n.pose_diagnostics['tf_age']-.01)<1e-6


def test_bridge_does_not_override_sensor_or_localization_loss():
    import time
    n=node();now=time.monotonic()
    n.p.update(require_localization_ready=True,odom_timeout=.3,
               amcl_max_xy_variance=.5,amcl_max_yaw_variance=.5)
    n.odom_stamp=n.manual_stamp=n.localization_stamp=now
    n.localization_info={'ready':True};n.amcl=NS(pose=NS(covariance=np.zeros(36)))
    n.scans={}
    assert not ImpactNode.healthy(n)
    assert n.health_diagnostics['reason']=='scan_pair_unavailable'
    n.localization_info={'ready':False}
    assert not ImpactNode.healthy(n)
    assert n.health_diagnostics['reason']=='localization_not_ready'
