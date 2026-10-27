"""Command and action-race regression tests, no hardware or ROS graph needed."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time
import numpy as np
from third_impact.node import ImpactNode


def stub(**values):
    defaults = dict(mode='mission', intent='patrol', state='PATROL', next_attempt=0.,
                    cancel_nav=Mock(), stop=Mock(), event=Mock(), settled_since=None)
    defaults.update(values)
    return NS(**defaults)


def test_hold_rejects_return_without_motion_or_duplicate_event():
    n = stub(intent='hold', state='HOLD')
    ImpactNode.on_return(n, None)
    assert n.intent == 'hold' and n.state == 'HOLD'
    n.cancel_nav.assert_not_called(); n.event.assert_not_called()
    ImpactNode.on_patrol(n, None)
    assert n.intent == 'patrol'


def test_return_preempts_patrol_and_ignores_go_around():
    n = stub()
    ImpactNode.on_return(n, None)
    assert n.intent == 'return' and n.state == 'RETURN'
    n.cancel_nav.assert_called_once(); n.stop.assert_called_once()
    ImpactNode.on_patrol(n, None)
    assert n.intent == 'return'


def test_pending_action_is_cancelled_if_acceptance_arrives_after_manual_takeover():
    handle = Mock(accepted=True)
    future = Mock(); future.result.return_value = handle
    n = stub(nav_pending=True, nav_handle=None, manual=True, cancel_requested=True,
             nav_purpose='patrol', nav_finished=Mock())
    ImpactNode.nav_accepted(n, future)
    assert n.nav_handle is handle and not n.nav_pending
    n.cancel_nav.assert_called_once()


def test_alignment_waits_for_action_result_before_handoff():
    n = stub(intent='return', state='RETURN', manual=False,
             healthy=lambda: True, data={'goal':[0.,0.,0.], 'rectangle':[0,0,0,1,1]},
             nav_pending=False, nav_handle=object(), pose=lambda: np.array([.1,0.,0.]),
             p={'align_radius':.4}, align=Mock())
    ImpactNode.tick(n)
    n.cancel_nav.assert_called_once(); n.stop.assert_called_once(); n.align.assert_not_called()
    n.nav_handle = None; n.stop.reset_mock()
    ImpactNode.tick(n)
    n.align.assert_called_once(); n.stop.assert_not_called()


def test_readiness_loss_cancels_action_and_resets_settle():
    n=stub(manual=False, healthy=lambda:False, settled_since=time.monotonic())
    ImpactNode.tick(n)
    n.cancel_nav.assert_called_once(); n.stop.assert_called_once()
    assert n.settled_since is None


def test_hold_stops_even_if_sensor_readiness_is_lost():
    n=stub(intent='hold', healthy=Mock(side_effect=AssertionError('must not be called')))
    ImpactNode.tick(n)
    n.stop.assert_called_once(); assert n.state=='HOLD'


def test_navigation_acceptance_replaces_old_rejection_message():
    handle = Mock(accepted=True)
    future = Mock(); future.result.return_value = handle
    n = stub(nav_pending=True, nav_handle=None, manual=False, cancel_requested=False,
             nav_purpose='patrol', nav_finished=Mock(), detail='Navigation rejected; retrying')
    n.event = lambda text: setattr(n, 'detail', text)
    ImpactNode.nav_accepted(n, future)
    assert n.detail == 'Navigation accepted: patrol'
    n.cancel_nav.assert_not_called()


def patrol_stub(distance):
    n = stub(manual=False, healthy=lambda:True,
             data={'goal':[0.,0.,0.], 'rectangle':[0,0,0,1,1]},
             nav_pending=False, nav_handle=object(), nav_purpose='patrol', nav_target=[0.,0.,0.],
             pose=lambda:np.array([distance,0.,np.pi]), cancel_requested=False,
             p={'patrol_tolerance':.05}, patrol_reached=False,
             send_nav=Mock(), nav_velocity_stamp=time.monotonic(), nav_velocity=Mock(),
             velocity_pub=Mock(), arrival_pub=Mock(), stopped=Mock(side_effect=AssertionError('no settling for patrol')))
    def cancel():
        n.cancel_requested=True
    n.cancel_nav=Mock(side_effect=cancel)
    return n


def test_patrol_within_5cm_ignores_heading_and_settle_waits_for_cancel_result():
    n=patrol_stub(.049)
    ImpactNode.tick(n)
    assert n.patrol_reached
    n.cancel_nav.assert_called_once(); n.stop.assert_called_once()
    n.send_nav.assert_not_called(); n.arrival_pub.publish.assert_not_called()
    ImpactNode.tick(n)
    n.send_nav.assert_not_called()  # Cannot start a competing action before result.


def test_patrol_outside_5cm_keeps_existing_navigation():
    n=patrol_stub(.051)
    ImpactNode.tick(n)
    assert not n.patrol_reached
    n.cancel_nav.assert_not_called(); n.velocity_pub.publish.assert_called_once()


def test_patrol_proximity_cancellation_counts_arrival_without_retry_delay():
    from unittest.mock import patch
    n=stub(nav_purpose='patrol', cancel_requested=True, patrol_reached=True,
           patrol_visits=0, p={'retry_seconds':2.}, arrival_pub=Mock())
    future=Mock(); future.result.return_value=NS(status=5)
    with patch('third_impact.node.time.monotonic',return_value=100.):
        ImpactNode.nav_finished(n,future)
    assert n.next_attempt==100.
    assert n.patrol_visits==1 and n.last_navigation_result['patrol_arrived']
    n.arrival_pub.publish.assert_not_called()


def test_patrol_native_success_also_has_no_retry_delay():
    from unittest.mock import patch
    n=stub(nav_purpose='patrol', cancel_requested=False, patrol_reached=False,
           patrol_visits=0, p={'retry_seconds':2.}, arrival_pub=Mock())
    future=Mock(); future.result.return_value=NS(status=4)
    with patch('third_impact.node.time.monotonic',return_value=100.):
        ImpactNode.nav_finished(n,future)
    assert n.next_attempt==100. and n.patrol_visits==1
    n.arrival_pub.publish.assert_not_called()


def test_patrol_and_return_select_distinct_behavior_trees():
    from geometry_msgs.msg import PoseStamped
    n=stub(nav_pending=False, nav_handle=None, nav=Mock(), pose_message=lambda p:PoseStamped(),
           nav_accepted=Mock(),p={'patrol_bt_file':'/patrol.xml','return_bt_file':'/return.xml'})
    for purpose, path in [('patrol','/patrol.xml'),('return','/return.xml')]:
        n.nav_pending=False
        ImpactNode.send_nav(n,[0,0,0],purpose)
        assert n.nav.send_goal_async.call_args.args[0].behavior_tree==path


def test_behavior_tree_controller_and_goal_checker_ids_exist():
    from pathlib import Path
    import xml.etree.ElementTree as ET
    import yaml
    config=Path(__file__).resolve().parents[1]/'config'
    params=yaml.safe_load((config/'nav2.yaml').read_text())['controller_server']['ros__parameters']
    for filename in ('patrol.xml','return.xml'):
        follow=ET.parse(config/filename).find('.//FollowPath')
        assert follow.attrib['controller_id'] in params['controller_plugins']
        assert follow.attrib['goal_checker_id'] in params['goal_checker_plugins']
    assert params['patrol_goal_checker']['xy_goal_tolerance']==.05
    assert params['patrol_goal_checker']['yaw_goal_tolerance']>np.pi
    assert not params['patrol_goal_checker']['stateful']
    assert 'RotateToGoal' not in params['PatrolFollowPath']['critics']
    assert 'RotateToGoal' in params['FollowPath']['critics']


def test_tf_expiry_between_readiness_and_patrol_distance_check_stops():
    n=patrol_stub(.049)
    n.pose=lambda:None
    ImpactNode.tick(n)
    n.stop.assert_called_once(); n.cancel_nav.assert_called_once()
    assert not n.patrol_reached
