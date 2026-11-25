import math
import time
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
         scan_key=lambda:(int(time.monotonic()*1e9),int(time.monotonic()*1e9)),
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


def obstacle_wait_robot():
    n=robot();n.cloud=lambda:np.array([[.3,.25]])
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        PatrolControl.begin_patrol_recovery(n)
        PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='WAIT' and n.patrol_obstacle_wait
    return n


def test_obstacle_disappearing_releases_wait_to_same_goal_replan():
    n=obstacle_wait_robot();target=list(n.patrol_retry_target)
    for t in (101.,110.,150.):
        with patch('third_impact.patrol_control.time.monotonic',return_value=t):
            PatrolControl.tick_patrol_recovery(n,POSE)
        assert n.patrol_recovery=='WAIT'
    n.cloud=lambda:np.array([[10.,10.]])
    for t in (151.,151.5,152.):
        with patch('third_impact.patrol_control.time.monotonic',return_value=t):
            PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery is None and n.state=='PATROL_SEARCH'
    assert n.patrol_retry_target==target and n.next_attempt==152.
    assert n.patrol_path is None and n.patrol_wait_reason==''
    n.velocity_pub.publish.assert_not_called()


def test_clear_wait_requires_both_fresh_scans_and_resets_on_obstacle():
    n=obstacle_wait_robot();n.cloud=lambda:np.array([[10.,10.]])
    key=[1,1];n.scan_key=lambda:tuple(key)
    for t in (101.,101.5,102.):
        key[0]+=1
        with patch('third_impact.patrol_control.time.monotonic',return_value=t):
            PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='WAIT' and n.patrol_obstacle_clear_count==0
    key[:]=[10,10]
    with patch('third_impact.patrol_control.time.monotonic',return_value=103.):
        PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_obstacle_clear_count==1
    n.cloud=lambda:np.array([[.3,.25]]);key[:]=[11,11]
    with patch('third_impact.patrol_control.time.monotonic',return_value=103.5):
        PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_obstacle_clear_count==0 and n.patrol_recovery=='WAIT'
    n.velocity_pub.publish.assert_not_called()


def test_obstacle_recheck_obeys_cancel_stop_map_and_region_barriers():
    n=obstacle_wait_robot();n.cloud=lambda:np.array([[10.,10.]])
    n.recovery_heading=Mock(return_value=0.)
    n.nav_handle=object()
    with patch('third_impact.patrol_control.time.monotonic',return_value=101.):
        PatrolControl.tick_patrol_recovery(n,POSE)
    n.recovery_heading.assert_not_called()
    n.nav_handle=None;n.stopped=lambda:False
    with patch('third_impact.patrol_control.time.monotonic',return_value=102.):
        PatrolControl.tick_patrol_recovery(n,POSE)
    n.recovery_heading.assert_not_called()
    n.stopped=lambda:True
    n.recovery_heading=MethodType(PatrolControl.recovery_heading,n)
    n.return_grid=lambda:NS(clear=lambda pose:False)
    for t in (103.,103.5,104.):
        with patch('third_impact.patrol_control.time.monotonic',return_value=t):
            PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='WAIT' and n.patrol_obstacle_clear_count==0
    n.velocity_pub.publish.assert_not_called()


def test_obstacle_during_rotation_enters_recheckable_wait():
    n=robot()
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        PatrolControl.begin_patrol_recovery(n)
        PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='ROTATE'
    n.velocity_pub.publish.reset_mock()
    n.cloud=lambda:np.array([[.3,.25]])
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.1):
        PatrolControl.tick_patrol_recovery(n,POSE)
    assert n.patrol_recovery=='WAIT' and n.patrol_obstacle_wait
    n.velocity_pub.publish.assert_not_called()


def rear_turn_robot(angle):
    n=robot()
    pose=np.array([1.0,1.0,0.])
    heading=np.deg2rad(angle)
    delta=np.array([math.cos(heading),math.sin(heading)])
    n.nav_target=[*(pose[:2]+delta),0.]
    n.nav_handle=object()
    n.cancel_requested=False
    n.nav_purpose='patrol'
    n.patrol_path=np.array([pose[:2],n.nav_target[:2]])
    n.patrol_path_stamp=100.
    n.rear_turn_arc_clear=Mock(return_value=True)
    n.safe_patrol_command=Mock(return_value=True)
    return n,pose


def rear_tick(n,pose,t):
    with patch('third_impact.patrol_control.time.monotonic',return_value=t):
        return PatrolControl.tick_patrol_rear_turn(n,pose)


def test_rear_path_keeps_one_direction_across_wrap_and_plan_updates():
    n,pose=rear_turn_robot(179)
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        assert PatrolControl.start_patrol_rear_turn(n,pose)
    rear_tick(n,pose,100.1)
    n.velocity_pub.publish.assert_not_called()
    n.nav_handle=None
    target=n.nav_target.copy()
    signs=[]
    for i in range(1,300):
        n.velocity_pub.publish.reset_mock()
        # Replanned path changes its first bearing after the turn was chosen.
        n.patrol_path=np.array([pose[:2],pose[:2]+[.5,.5],target[:2]])
        rear_tick(n,pose,100.1+i*.05)
        if n.velocity_pub.publish.called:
            cmd=n.velocity_pub.publish.call_args.args[0]
            assert cmd.linear.x==0.
            signs.append(math.copysign(1,cmd.angular.z))
            pose[2]=math.remainder(pose[2]+cmd.angular.z*.05,math.tau)
        if n.patrol_rear_turn is None:
            break
    assert signs and set(signs)=={-1}
    assert n.patrol_rear_turn is None and n.patrol_retry_target==target
    assert abs(math.remainder(np.deg2rad(179)-pose[2],math.tau))<=np.deg2rad(20)


def test_blocked_preferred_turn_uses_opposite_only_if_clear():
    n,pose=rear_turn_robot(-150)
    with patch('third_impact.patrol_control.time.monotonic',return_value=100.):
        assert PatrolControl.start_patrol_rear_turn(n,pose)
    n.nav_handle=None
    n.rear_turn_arc_clear.return_value=False
    rear_tick(n,pose,100.1)
    assert n.patrol_rear_turn['phase']=='BLOCKED'
    n.velocity_pub.publish.assert_not_called()
    n.rear_turn_arc_clear.side_effect=lambda pose,angle,cloud,grid:angle>0
    rear_tick(n,pose,100.7)
    assert n.patrol_rear_turn['sign']==1
    assert n.patrol_rear_turn['remaining']>math.pi
    assert n.velocity_pub.publish.call_args.args[0].angular.z>0
    n.velocity_pub.publish.reset_mock()
    n.safe_patrol_command.return_value=False
    rear_tick(n,pose,100.75)
    assert n.patrol_rear_turn['phase']=='BLOCKED'
    n.velocity_pub.publish.assert_not_called()
    stale=n.patrol_rear_turn['scan_key']
    n.scan_key=lambda:stale
    rear_tick(n,pose,101.3)
    n.velocity_pub.publish.assert_not_called()


def test_rear_turn_only_uses_recent_nearby_path():
    n,pose=rear_turn_robot(-150)
    with patch('third_impact.patrol_control.time.monotonic',return_value=102.):
        assert not PatrolControl.start_patrol_rear_turn(n,pose)
    n.patrol_path_stamp=102.
    with patch('third_impact.patrol_control.time.monotonic',return_value=102.):
        assert not PatrolControl.start_patrol_rear_turn(n,pose+np.array([0.,1.,0.]))


def test_rear_turn_checks_intermediate_footprint_even_when_endpoints_fit():
    n,pose=rear_turn_robot(-150)
    n.data={'boundary':[1.,1.,0.,4.,4.]}
    n._patrol_exclusion=lambda _:None
    grid=NS(clear_many=lambda poses:np.ones(len(poses),dtype=bool))
    # Clear at 0 and -150 degrees, yet inside the footprint at -90 degrees.
    assert not PatrolControl.rear_turn_arc_clear(n,pose,np.deg2rad(-150),np.array([[0.,.37]]),grid)
    assert PatrolControl.rear_turn_arc_clear(n,pose,np.deg2rad(-150),np.array([[0.,1.]]),grid)


def test_initial_exit_finds_the_open_side_of_three_walled_room():
    from third_impact.patrol_exit import straight_exit_candidates, exit_line_clear
    from third_impact.return_route import FootprintMap
    cells=np.zeros((160,160),dtype=np.int8)
    cells[60:103,60]=100
    cells[60:103,100]=100
    cells[102,60:101]=100
    cells[90:93,90:93]=-1  # Keep unknown space away from the initial turn.
    g=FootprintMap(cells,.05,[-4.,-4.,0.])
    pose=np.array([-.1,0.,math.pi/2])
    boundary=[0.,0.,0.,7.,7.]
    exclusion=[0.,0.,0.,2.,2.]
    candidates=straight_exit_candidates(pose,boundary,exclusion,g,.35,.23)
    assert candidates
    assert all(p['end'][1]<-1. for p in candidates)
    planned=candidates[0]
    heading=planned['heading']
    midpoint=np.array([math.cos(heading),math.sin(heading)])*.8
    cloud=transform_test_point(midpoint,pose[2])
    assert not exit_line_clear(pose,planned,cloud,.35,.23)
    assert exit_line_clear(pose,planned,np.array([[3.,3.]]),.35,.23)
    cells[59,60:101]=100
    assert not straight_exit_candidates(pose,boundary,exclusion,g,.35,.23)


def transform_test_point(point, yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c*point[0]+s*point[1],-s*point[0]+c*point[1]]])


def test_initial_exit_only_hands_off_after_full_clearance_and_stop():
    from third_impact.patrol_exit import PatrolExit
    n=robot()
    grid=object()
    n.return_grid=lambda:grid
    n.data={'rectangle':[0.,0.,0.,2.,2.],'boundary':[0.,0.,0.,8.,8.]}
    n._initial_exclusion_exit=True
    n.patrol_exit_plan=dict(start=[0.,0.],heading=0.,length=1.7,phase='DRIVE',
                            best=0.,progress_at=100.,check_grid=grid)
    n.safe_patrol_command=Mock(return_value=True)
    n.cloud=lambda:np.array([[.8,0.]])
    n.event=Mock()
    with patch('third_impact.patrol_exit.time.monotonic',return_value=100.):
        PatrolExit.tick_patrol_exit(n,np.zeros(3))
        n.velocity_pub.publish.assert_not_called()
        n.cloud=lambda:np.array([[4.,4.]])
        PatrolExit.tick_patrol_exit(n,np.array([1.35,0.,0.]))
        assert n._initial_exclusion_exit
        assert n.velocity_pub.publish.call_args.args[0].linear.x>0.
        n.stopped=lambda:False
        PatrolExit.tick_patrol_exit(n,np.array([1.6,0.,0.]))
        assert n._initial_exclusion_exit
        n.stopped=lambda:True
        PatrolExit.tick_patrol_exit(n,np.array([1.6,0.,0.]))
    assert n._initial_exclusion_exit is False and n.patrol_exit_plan is None
    assert n.next_attempt==100.
