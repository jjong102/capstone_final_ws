import math
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock, patch
import numpy as np
from third_impact.patrol_control import PatrolControl, escape_heading
from third_impact.core import travel_allowed, region_map, relative

BOUNDARY=[1.8842333555,1.6548368931,-1.2551995626,10.3990636358,6.7299690704]
EXCLUSION=[-.5257819891,1.7143392563,-1.2551995626,2.3341915980,2.6109676022]
POSE=np.array([1.23758065,.55028018,1.28954895])
PATH=np.array([[1.24,.55],[1.244,.532],[1.255,.521],[1.265,.511],
               [1.28,.514],[1.31,.519],[1.345,.540],[1.375,.548],
               [1.42,.56],[1.48,.6],[1.5,.8],[1.495,3.295]])


def robot():
    n=NS(nav_target=[1.495,3.295,0], nav_pending=False, nav_handle=None,
         data={'boundary':BOUNDARY,'rectangle':EXCLUSION},
         stop=Mock(),cancel_nav=Mock(),velocity_pub=Mock(),stopped=lambda:True,
         cloud=lambda:np.array([[10.,10.]]),
         p={'footprint_half_length':.35,'footprint_half_width':.22,'obstacle_margin':.04})
    PatrolControl.init_patrol(n)
    n.recovery_heading=MethodType(PatrolControl.recovery_heading,n)
    n.return_grid=lambda:NS(clear=lambda pose:True)
    n.safe_patrol_command=MethodType(PatrolControl.safe_patrol_command,n)
    n.motion_allowed=lambda pose,vx=0,vy=0,angular=0:travel_allowed(pose,BOUNDARY,EXCLUSION,vx,vy,.35,.22,angular=angular)
    return n


def test_recorded_corner_can_rotate_then_leave_without_entering_zone():
    n=robot();pose=POSE.copy();clock=100.
    n.patrol_path=PATH;n.patrol_path_stamp=100.
    with patch('third_impact.patrol_control.time.monotonic',return_value=clock):
        PatrolControl.begin_patrol_recovery(n)
    assert n.patrol_retry_target==n.nav_target
    for _ in range(240):
        clock+=.05;n.velocity_pub.publish.reset_mock()
        with patch('third_impact.patrol_control.time.monotonic',return_value=clock):
            PatrolControl.tick_patrol_recovery(n,pose)
        if n.velocity_pub.publish.called:
            cmd=n.velocity_pub.publish.call_args.args[0]
            assert cmd.linear.x==0
            pose[2]+=cmd.angular.z*.05
            assert n.motion_allowed(pose)
        if n.patrol_recovery is None:
            break
    assert n.patrol_recovery is None
    assert n.motion_allowed(pose,.2)
    assert n.patrol_retry_target==n.nav_target


def test_recovery_waits_for_action_result_and_measured_stop():
    n=robot();n.patrol_path=PATH;n.patrol_path_stamp=100.
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        PatrolControl.begin_patrol_recovery(n)
        n.nav_handle=object()
        PatrolControl.tick_patrol_recovery(n,POSE)
        n.velocity_pub.publish.assert_not_called()
        n.nav_handle=None;n.stopped=lambda:False
        PatrolControl.tick_patrol_recovery(n,POSE)
        n.velocity_pub.publish.assert_not_called()


def test_recovery_never_rotates_into_lidar_obstacle_and_times_out():
    n=robot();n.patrol_path=PATH;n.patrol_path_stamp=100.
    n.cloud=lambda:np.array([[.3,.25]])
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        PatrolControl.begin_patrol_recovery(n)
        PatrolControl.tick_patrol_recovery(n,POSE)
    n.velocity_pub.publish.assert_not_called()
    with patch('third_impact.patrol_control.time.monotonic',return_value=113.):
        PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='WAIT'


def test_repeated_blocks_widen_once_then_stop_retrying():
    n=robot()
    for i in range(1,5):
        PatrolControl.begin_patrol_recovery(n)
        assert n.patrol_recovery_count==i
        assert n.patrol_retry_target==n.nav_target
        if i in (2,3):
            assert n.patrol_recovery=='STOP' and n.patrol_extra_clearance==.15
    assert n.patrol_recovery=='WAIT'
    PatrolControl.tick_patrol_recovery(n,POSE)
    n.velocity_pub.publish.assert_not_called()


def test_stale_path_uses_collision_checked_quarter_turn():
    n=robot();n.patrol_path=PATH;n.patrol_path_stamp=0.
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        PatrolControl.begin_patrol_recovery(n)
        PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='ROTATE'
    assert n.velocity_pub.publish.call_args.args[0].linear.x == 0


def test_planner_zone_margin_does_not_change_local_collision_map():
    cells=np.zeros((100,100),dtype=np.int8);boundary=[5,5,0,10,10];exclusion=[5,5,0,2,2]
    local=region_map(cells,.1,[0,0,0],boundary,exclusion)
    planning=region_map(cells,.1,[0,0,0],boundary,exclusion,.3434)
    assert local[50,62]==0 and planning[50,62]==100
    assert np.count_nonzero(planning==100)>np.count_nonzero(local==100)


def test_start_relief_never_erases_original_zone_or_real_obstacles():
    cells=np.zeros((100,100),dtype=np.int8);cells[50,65]=100
    boundary=[5,5,0,10,10];exclusion=[5,5,0,2,2]
    original=region_map(cells,.1,[0,0,0],boundary,exclusion)
    relaxed=region_map(cells,.1,[0,0,0],boundary,exclusion,.3434,[6.4,5.])
    assert np.all(relaxed[original==100]==100)
    assert relaxed[50,65]==100


def test_configuration_uses_measured_velocity_and_separate_planning_map():
    import yaml
    from pathlib import Path
    cfg=yaml.safe_load((Path(__file__).parents[1]/'config/nav2.yaml').read_text())
    assert cfg['velocity_smoother']['ros__parameters']['feedback']=='OPEN_LOOP'
    for costmap,topic in [('global_costmap','/third_impact/planning_map'),('local_costmap','/third_impact/navigation_map')]:
        assert cfg[costmap][costmap]['ros__parameters']['static_layer']['map_topic']==topic


def test_opposite_turn_waits_for_measured_stop():
    from geometry_msgs.msg import Twist
    n=robot(); n.event=Mock()
    n.odom=NS(twist=NS(twist=NS(angular=NS(z=.16))))
    n.stopped=lambda:False
    command=Twist();command.angular.z=-.35;command.linear.x=.6
    assert PatrolControl.patrol_direction_stop(n,command)
    n.stop.assert_called()
    n.odom.twist.twist.angular.z=0.;n.stopped=lambda:True
    assert PatrolControl.patrol_direction_stop(n,command)
    assert not PatrolControl.patrol_direction_stop(n,command)


def test_patrol_bt_delegates_recovery_without_changing_return():
    from pathlib import Path
    import xml.etree.ElementTree as ET
    tree=ET.parse(Path(__file__).parents[1]/'config/patrol.xml')
    assert not tree.findall('.//Spin') and not tree.findall('.//BackUp')
    assert tree.find('.//FollowPath').attrib['controller_id']=='PatrolFollowPath'


def test_stale_observation_waits_for_both_new_scans_then_recovers():
    n=robot();key=[1,1];n.scan_key=lambda:tuple(key)
    n.cloud=lambda:None
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        PatrolControl.begin_patrol_recovery(n)
        PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='SENSOR_WAIT'
    assert n.patrol_recovery_check['reason']=='sensor_unavailable'
    n.cloud=lambda:np.array([[10.,10.]])
    with patch('third_impact.patrol_control.time.monotonic',return_value=101.):
        key[0]=2
        PatrolControl.tick_patrol_recovery(n,POSE)
        assert n.patrol_recovery=='SENSOR_WAIT'
        key[1]=2
        PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='ROTATE'
    assert n.patrol_sensor_retries==1


def test_sensor_wait_has_deadline_and_does_not_publish_motion():
    n=robot();n.scan_key=lambda:(1,1);n.cloud=lambda:None
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        PatrolControl.begin_patrol_recovery(n)
        PatrolControl.tick_patrol_recovery(n,POSE)
    with patch('third_impact.patrol_control.time.monotonic',return_value=105.1):
        PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='WAIT'
    assert '충돌 판정이 아닙니다' in n.patrol_wait_reason
    n.velocity_pub.publish.assert_not_called()


def test_sensor_expiring_during_check_is_not_collision():
    n=robot();n.cloud=Mock(side_effect=[np.array([[10.,10.]]),None])
    n.patrol_retry_target=n.nav_target
    assert n.recovery_heading(POSE) is None
    assert n.patrol_recovery_check['reason']=='sensor_unavailable'
    assert n.patrol_recovery_check['candidates'][-1]['reason']=='clear'


def test_readiness_pause_preserves_only_active_progress_time():
    n=robot();n.patrol_progress_time=90.
    for t,paused in [(100.,True),(104.,True),(110.,False)]:
        with patch('third_impact.patrol_control.time.monotonic',return_value=t):
            PatrolControl.patrol_readiness_pause(n,paused)
    assert n.patrol_progress_time==100.  # 10 active seconds, not 20.
    assert n.patrol_pause_started is None


def test_sensor_retry_budget_is_three_even_with_many_new_scans():
    n=robot();key=[0,0];n.scan_key=lambda:tuple(key)
    def unavailable(pose):
        n.patrol_recovery_check={'reason':'sensor_unavailable'}
        return None
    n.recovery_heading=unavailable
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        PatrolControl.begin_patrol_recovery(n)
        PatrolControl.tick_patrol_recovery(n,POSE)
    for i in range(1,5):
        key[:]=[i,i]
        with patch('third_impact.patrol_control.time.monotonic',return_value=100.+i*.2):
            PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='WAIT' and n.patrol_sensor_retries==3
    n.velocity_pub.publish.assert_not_called()
