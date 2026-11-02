"""Command and action-race regression tests, no hardware or ROS graph needed."""
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock
import time
import numpy as np
from third_impact.node import ImpactNode


def stub(**values):
    defaults = dict(patrol_direction_stop=Mock(return_value=False), mode='mission', intent='patrol', state='PATROL', next_attempt=0.,
                    safe_patrol_command=Mock(return_value=True), init_patrol=Mock(), tick_patrol_near=Mock(return_value=False), tick_patrol_recovery=Mock(return_value=False), begin_patrol_recovery=Mock(), patrol_progress_pose=None, patrol_progress_time=time.monotonic(), publish_navigation_map=Mock(), motion_allowed=Mock(return_value=True), cancel_nav=Mock(), stop=Mock(), event=Mock(), settled_since=None, reset_return=Mock(), pause_return=Mock())
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


def test_return_uses_route_owner_instead_of_radius_only_alignment():
    n = stub(intent='return', state='RETURN', manual=False,
             healthy=lambda: True, data={'goal':[0.,0.,0.], 'boundary':[0,0,0,4,4], 'rectangle':[10,10,0,1,1]},
             nav_pending=False, nav_handle=None, pose=lambda: np.array([.1,0.,3.14]),
             tick_return=Mock(), align=Mock(),
             tick_patrol_near=Mock(side_effect=AssertionError('return must not enter patrol near control')))
    ImpactNode.tick(n)
    n.tick_return.assert_called_once()
    n.align.assert_not_called()


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
             data={'goal':[0.,0.,0.], 'boundary':[0,0,0,4,4], 'rectangle':[10,10,0,1,1]},
             nav_pending=False, nav_handle=object(), nav_purpose='patrol', nav_target=[0.,0.,0.],
             pose=lambda:np.array([distance,0.,np.pi]), cancel_requested=False,
             p={'patrol_tolerance':.05, 'footprint_half_length':.35, 'footprint_half_width':.22, 'retry_seconds':2.}, patrol_reached=False,
             send_nav=Mock(), nav_velocity_stamp=time.monotonic(), nav_velocity=NS(linear=NS(x=.1,y=0.),angular=NS(z=0.)),
             velocity_pub=Mock(), arrival_pub=Mock(), stopped=Mock(side_effect=AssertionError('no settling for patrol')))
    def cancel():
        n.cancel_requested=True
    n.cancel_nav=Mock(side_effect=cancel)
    n.motion_allowed=MethodType(ImpactNode.motion_allowed,n)
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


def test_patrol_boundary_slows_down_without_cancelling_goal():
    n=patrol_stub(1.4)
    n.pose=lambda:np.array([1.4,0.,0.])
    n.nav_velocity.linear.x=.3
    ImpactNode.tick(n)
    n.cancel_nav.assert_not_called()
    command=n.velocity_pub.publish.call_args.args[0]
    assert 0. < command.linear.x < .3


def test_patrol_outside_region_waits_instead_of_driving_in():
    n=patrol_stub(3.)
    ImpactNode.tick(n)
    n.stop.assert_called_once(); n.cancel_nav.assert_called_once()
    n.velocity_pub.publish.assert_not_called(); n.send_nav.assert_not_called()


def test_patrol_target_is_inside_allowed_region_with_body_clearance():
    from geometry_msgs.msg import Pose
    origin=Pose(); origin.orientation.w=1.
    n=NS(candidates=np.array([[0,0],[1,1],[2,2],[3,3]]),
         grid=NS(info=NS(origin=origin,resolution=1.)),
         pose=lambda:np.array([0.,0.,0.]),
         data={'boundary':[2.,2.,0.,2.,2.], 'rectangle':[10,10,0,1,1]},
         p={'footprint_half_length':.35,'footprint_half_width':.22})
    for _ in range(20):
        p=ImpactNode.patrol_target(n)
        assert p is not None and 1.47 < p[0] < 2.53 and 1.47 < p[1] < 2.53
    n.data['boundary']=[10.,10.,0.,2.,2.]
    assert ImpactNode.patrol_target(n) is None


def test_old_exclusion_region_requires_explicit_resave(tmp_path):
    import json
    from unittest.mock import patch
    path=tmp_path/'goal.json'
    saved=dict(goal=[0,0,0],reference=[[0,0]]*50,rectangle=[0,0,0,4,4],map_fingerprint='map')
    path.write_text(json.dumps(saved))
    n=NS(path=path,fingerprint='map',editor=Mock(),boundary_editor=Mock(),event=Mock())
    ImpactNode.load(n)
    assert n.data['rectangle'] is None
    assert json.loads(path.read_text())['rectangle']==[0,0,0,4,4]
    saved['region_policy']='boundary_and_exclusion'; saved['boundary']=[0,0,0,10,10];path.write_text(json.dumps(saved))
    ImpactNode.load(n)
    assert n.data['rectangle']==[0,0,0,4,4]


def test_save_boundary_requires_goal_and_body_inside(tmp_path):
    n=NS(mode='teach',state='TEACH',data={'goal':[0,0,0]},
         editor=NS(rect=[0,0,0,.5,.5]),path=tmp_path/'goal.json',event=Mock(),
         p={'footprint_half_length':.35,'footprint_half_width':.22})
    ImpactNode.save_zone(n,'boundary',n.editor)
    assert not n.path.exists()
    n.editor.rect=[0,0,0,4,4]
    ImpactNode.save_zone(n,'boundary',n.editor)
    assert n.data['region_policy']=='boundary_and_exclusion' and n.path.exists()


def test_patrol_exclusion_blocks_motion_but_return_ignores_it():
    n=patrol_stub(.8)
    n.data['rectangle']=[.8,0,0,1,1]
    ImpactNode.tick(n)
    n.stop.assert_called_once(); n.velocity_pub.publish.assert_not_called()
    n.intent='return'
    assert ImpactNode.motion_allowed(n,n.pose())


def test_manual_override_is_processed_before_region_guard():
    n=patrol_stub(3.);n.manual=True
    n.motion_allowed=Mock(side_effect=AssertionError('manual must bypass automatic zone control'))
    ImpactNode.tick(n)
    n.cancel_nav.assert_called_once()


def test_navigation_map_switches_exclusion_on_return_and_back():
    from nav_msgs.msg import OccupancyGrid
    from rclpy.time import Time
    n=NS(grid=OccupancyGrid(), data={'boundary':[2,2,0,4,4],'rectangle':[2,2,0,1,1]},
         pose=lambda:None,intent='patrol',navigation_map_key=None,navigation_map_pub=Mock(),planning_map_pub=Mock(),
         p={'footprint_half_length':.35,'footprint_half_width':.22},
         get_clock=lambda:NS(now=lambda:Time()))
    n.grid.info.width=n.grid.info.height=40;n.grid.info.resolution=.1
    n.grid.info.origin.orientation.w=1.;n.grid.data=[0]*1600
    ImpactNode.publish_navigation_map(n)
    assert n.navigation_map_pub.publish.call_args.args[0].data[20*40+20]==100
    n.intent='return';ImpactNode.publish_navigation_map(n)
    assert n.navigation_map_pub.publish.call_args.args[0].data[20*40+20]==0
    n.intent='patrol';ImpactNode.publish_navigation_map(n)
    assert n.navigation_map_pub.publish.call_args.args[0].data[20*40+20]==100
    assert all(v==0 for v in n.grid.data)


def test_blocked_command_enters_recovery_instead_of_picking_new_goal():
    n=patrol_stub(1.)
    n.motion_allowed=Mock(side_effect=[True,False,False,False])
    ImpactNode.tick(n)
    n.begin_patrol_recovery.assert_called_once()
    n.velocity_pub.publish.assert_not_called()


def test_cancelled_patrol_retains_retry_delay():
    from unittest.mock import patch
    n=stub(nav_purpose='patrol',cancel_requested=True,patrol_reached=False,
           p={'retry_seconds':2.})
    future=Mock();future.result.return_value=NS(status=5)
    with patch('third_impact.node.time.monotonic',return_value=100.):
        ImpactNode.nav_finished(n,future)
    assert n.next_attempt==102.


def test_first_planning_failure_enters_supervised_recovery():
    n=stub(nav_purpose='patrol', cancel_requested=False, patrol_reached=False,
           patrol_retry_target=None,nav_target=[1.,2.,0.],nav_handle=object(),
           p={'retry_seconds':2.})
    future=Mock();future.result.return_value=NS(status=6)
    ImpactNode.nav_finished(n,future)
    n.begin_patrol_recovery.assert_called_once()
    assert n.nav_target==[1.,2.,0.]


def test_health_distinguishes_pair_skew_and_stale_sensor():
    from builtin_interfaces.msg import Time
    from unittest.mock import patch
    n=NS(p={'require_localization_ready':True,'odom_timeout':.3,'scan_timeout':.45,
            'amcl_max_xy_variance':.08,'amcl_max_yaw_variance':.08},
         localization_stamp=100.,localization_info={'ready':True},manual=False,
         manual_stamp=100.,odom=object(),odom_stamp=100.,grid=object(),
         amcl=NS(pose=NS(covariance=[0.]*36)),pose=lambda:np.zeros(3),
         scans={'front':(100.,Time(sec=10),np.zeros((2,2))),
                'rear':(100.,Time(sec=10,nanosec=200000000),np.zeros((2,2)))})
    with patch('third_impact.node.time.monotonic',return_value=100.):
        assert not ImpactNode.healthy(n)
        assert n.health_diagnostics['reason']=='scan_pair_skew'
        n.scans['rear']=(100.,Time(sec=10,nanosec=50000000),np.zeros((2,2)))
        assert ImpactNode.healthy(n)
        n.scans['front']=(99.,Time(sec=10),np.zeros((2,2)))
        assert not ImpactNode.healthy(n)
        assert n.health_diagnostics['reason']=='front_scan_stale'
