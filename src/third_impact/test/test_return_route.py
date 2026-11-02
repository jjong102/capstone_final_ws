import math
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
from third_impact.return_route import FootprintMap, select_entry, tracking_command, obstacle_action
from third_impact.return_control import ReturnControl


def room(narrow=False):
    cells=np.zeros((160,160), dtype=int)
    if narrow:
        # Goal in left alcove, opening to the right. Width .60 fits .48 body
        # but not the diagonal during a 180-degree turn.
        cells[:74, :90]=100; cells[86:, :90]=100
    return FootprintMap(cells,.05,[-4,-4,0])


def test_open_room_keeps_forward():
    plan=select_entry([[2,0],[0,0]], [0,0,0], room(), math.pi)
    assert plan is not None and not plan['reverse']


def test_narrow_goal_rotates_before_entry_on_existing_path():
    g=room(True)
    assert g.clear([0,0,0]) and not g.rotation([0,0], math.pi, 0)
    plan=select_entry([[2,0],[0,0]], np.array([0,0,0]), g, math.pi)
    assert plan is not None and plan['reverse']
    assert plan['entry'][0] > .5
    assert abs(plan['entry'][1]) < 1e-9
    assert g.rotation(plan['entry'][:2], math.pi, 0)
    assert g.corridor(plan['entry'], [0,0,0])


def test_no_entry_does_not_invent_detour():
    assert select_entry([[.2,0],[0,0]], [0,0,0], room(True), math.pi) is None


def test_map_checks_interior_unknown_outside_and_rotated_origin():
    g=room(); assert not g.clear([4,4,0])
    g.cells[80,80]=-1; assert not g.clear([0,0,0])
    assert FootprintMap(np.zeros((40,40)),.05,[1,2,math.pi/2]).clear([0,3,0])


def test_forward_reverse_share_speed_and_reverse_does_not_turn_180():
    a=tracking_command(np.array([0,0,0]),np.array([[0,0],[2,0]]),0,False)
    b=tracking_command(np.array([0,0,0]),np.array([[0,0],[-2,0]]),0,True)
    assert a[0] == -b[0] == .6
    assert abs(b[1]) < 1e-8


def test_obstacle_direction_policy():
    assert obstacle_action(False,4.999)=='wait'
    assert obstacle_action(False,5.)=='replan'
    assert obstacle_action(True,10000.)=='wait'


def command_node(reverse=False):
    return NS(odom=NS(twist=NS(twist=NS(linear=NS(x=0.),angular=NS(z=0.)))), motion_allowed=Mock(return_value=True), return_last_tick=99.95,return_last_v=0.,return_last_w=0.,
        return_blocked=None,return_route=np.array([[0,0],[1,0]]),return_phase='REVERSE' if reverse else 'FORWARD',
        return_epoch=0,stop=Mock(),event=Mock(),velocity_pub=Mock(),show_return_path=Mock(),return_local_pub=Mock(),
        cloud=lambda:np.array([[-.5,0]]) if reverse else np.array([[.325,0]]),
        return_grid=lambda:room(),p={'footprint_half_length':.31,'footprint_half_width':.24,'return_obstacle_margin':.01})


def test_reverse_blockage_never_replans_or_creeps():
    n=command_node(True)
    for now in (100.,104.,106.,1000.):
        with patch('third_impact.return_control.time.monotonic',return_value=now):
            ReturnControl.return_command(n,np.zeros(3),-.6,0.,True)
    assert n.return_phase=='REVERSE' and n.return_route is not None
    assert n.stop.call_count==4
    n.velocity_pub.publish.assert_not_called()


def test_forward_blockage_waits_five_seconds_then_replans():
    n=command_node()
    for now in (100.,104.9):
        with patch('third_impact.return_control.time.monotonic',return_value=now):
            ReturnControl.return_command(n,np.zeros(3),.6,0.,False)
        assert n.return_phase=='FORWARD'
    with patch('third_impact.return_control.time.monotonic',return_value=105.):
        ReturnControl.return_command(n,np.zeros(3),.6,0.,False)
    assert n.return_phase=='PLAN' and n.return_route is None
    n.velocity_pub.publish.assert_not_called()


def test_clear_reverse_resumes_same_route():
    n=command_node(True); n.return_blocked=90.
    n.cloud=lambda:np.array([[0,2.]])
    with patch('third_impact.return_control.time.monotonic',return_value=100.):
        ReturnControl.return_command(n,np.zeros(3),-.6,0.,True)
    assert n.return_blocked is None and n.return_phase=='REVERSE'
    assert n.velocity_pub.publish.call_args.args[0].linear.x < 0


def test_return_waits_for_previous_nav_cancel_result():
    n=NS(nav_pending=True,nav_handle=None,cancel_nav=Mock(),stop=Mock())
    ReturnControl.tick_return(n,np.zeros(3))
    n.cancel_nav.assert_called_once();n.stop.assert_called_once()


def test_late_plan_after_manual_takeover_is_discarded():
    n=NS(return_epoch=2,intent='return',return_future=object())
    result=Mock()
    ReturnControl.return_path_done(n,result,1)
    result.result.assert_not_called()
    assert n.return_future is None


def test_tracking_simulation_converges_at_corner_and_in_reverse():
    from third_impact.return_route import dense_path
    for reverse, points, pose in [(False, [[0,0],[1,0],[1,1]], np.zeros(3)),
                                  (True, [[2,0],[0,0]], np.array([2.,0.,0.]))]:
        path=dense_path(points); index=0
        for _ in range(600):
            if np.linalg.norm(pose[:2]-path[-1]) < .035:
                break
            v,w,index=tracking_command(pose,path,index,reverse)
            pose+=np.array([v*np.cos(pose[2]),v*np.sin(pose[2]),w])*.05
        assert np.linalg.norm(pose[:2]-path[-1]) < .035


def test_no_entry_rechecks_same_path_without_requesting_detour():
    n=NS(nav_pending=False,nav_handle=None,return_paused_pose=None,
         data={'goal':np.zeros(3)},return_phase='NO_ENTRY',return_retry=0.,
         return_candidate=np.array([[.2,0],[0,0]]),return_grid=lambda:room(True),
         stop=Mock(),install_return_plan=Mock(),request_return_path=Mock())
    ReturnControl.tick_return(n,np.array([.2,0,math.pi]))
    n.stop.assert_called_once(); n.request_return_path.assert_not_called()
    n.install_return_plan.assert_not_called()


def test_reverse_manual_relocation_holds_without_new_route():
    n=NS(nav_pending=False,nav_handle=None,return_paused_pose=np.zeros(3),
         return_phase='REVERSE',stop=Mock(),request_return_path=Mock())
    ReturnControl.tick_return(n,np.array([1.,0,0]))
    n.stop.assert_called_once(); n.request_return_path.assert_not_called()


def test_full_narrow_return_reaches_alignment_without_turning_in_alcove():
    import types
    g=room(True); pose=np.array([2.,0.,math.pi]); clock=[100.]
    n=command_node()
    n.nav_pending=False; n.nav_handle=None; n.return_paused_pose=None
    n.data={'goal':np.zeros(3)}; n.return_grid=lambda:g
    n.p.update(return_max_speed=.6,return_max_angular_speed=.35)
    n.return_path_pub=Mock(); n.event=Mock(); n.align=Mock()
    n.cloud=lambda:np.array([[20.,20.]])
    command=[0.,0.]
    n.stop=lambda:command.__setitem__(slice(None),[0.,0.])
    n.stopped=lambda:abs(command[0])<.005 and abs(command[1])<.01
    n.velocity_pub.publish=lambda msg:command.__setitem__(slice(None),[msg.linear.x,msg.angular.z])
    n.return_command=types.MethodType(ReturnControl.return_command,n)
    ReturnControl.install_return_plan(n,select_entry([[2,0],[0,0]],np.zeros(3),g,math.pi))
    phases=set()
    for _ in range(1600):
        clock[0]+=.05
        with patch('third_impact.return_control.time.monotonic',return_value=clock[0]):
            ReturnControl.tick_return(n,pose)
        phases.add(n.return_phase)
        if n.return_phase=='ALIGN':break
        v,w=command
        pose+=np.array([v*np.cos(pose[2]),v*np.sin(pose[2]),w])*.05
        assert g.clear(pose), 'body crossed a wall'
        if pose[0]<.45:
            assert abs(((pose[2]+math.pi)%(2*math.pi))-math.pi)<.15
    assert {'FORWARD','ROTATE','REVERSE','ALIGN'} <= phases
    assert np.linalg.norm(pose[:2])<.06


def test_return_boundary_rejects_command_even_with_clear_scans():
    n=command_node(); n.motion_allowed=Mock(return_value=False)
    n.cloud=lambda:np.array([[20.,20.]])
    ReturnControl.return_command(n,np.zeros(3),.6,0.,False)
    n.stop.assert_called_once(); n.velocity_pub.publish.assert_not_called()


def test_braking_path_preserves_real_stopping_distance():
    from third_impact.return_route import braking_path
    p=braking_path(.6,0.)
    assert .48 <= p[-1,1] <= .50  # .6*.2 + .6**2/(2*.5), plus discretization
    assert np.allclose(p[:,2:],0.)
    assert np.allclose(braking_path(-.6,0.)[:,1],-p[:,1])


def test_stationary_return_can_choose_safe_slower_command():
    n=command_node();n.cloud=lambda:np.array([[.5,0.]])
    ReturnControl.return_command(n,np.zeros(3),.6,0.,False)
    assert n.velocity_pub.publish.call_args.args[0].linear.x > 0
    assert n.return_collision['scale'] < 1
    assert n.return_collision['rejected'][0]['reason']=='lidar'


def test_fast_actual_motion_is_not_hidden_by_low_request():
    n=command_node();n.cloud=lambda:np.array([[.5,0.]])
    n.odom.twist.twist.linear.x=.6
    ReturnControl.return_command(n,np.zeros(3),.1,0.,False)
    n.stop.assert_called_once();n.velocity_pub.publish.assert_not_called()
    assert n.return_collision['source']=='actual_braking'


def test_map_wall_is_distinguished_from_clear_lidar():
    n=command_node();n.cloud=lambda:np.array([[20.,20.]])
    n.return_grid=lambda:NS(clear=lambda pose:False)
    ReturnControl.return_command(n,np.zeros(3),.1,0.,False)
    assert n.return_collision['reason']=='map'
    assert '지도 벽' in n.return_wait_reason


def test_missing_scan_remains_stopped():
    n=command_node();n.cloud=lambda:None
    ReturnControl.return_command(n,np.zeros(3),.1,0.,False)
    assert n.return_collision['reason']=='sensor_missing'
    n.velocity_pub.publish.assert_not_called()


def test_wall_beyond_near_goal_does_not_block_braking_approach():
    # Recorded failure shape: goal 48 cm away, constant-speed extrapolation
    # crosses a wall past the goal; the full braking footprint stays clear.
    n=command_node();n.cloud=lambda:np.array([[.95,0.]])
    n.p.update(footprint_half_length=.35,footprint_half_width=.22)
    n.return_route=np.array([[0.,0.],[.48,0.]])
    n.return_grid=lambda:NS(clear=lambda pose:pose[0]+.35 < .95)
    n.odom.twist.twist.linear.x=.29
    ReturnControl.return_command(n,np.zeros(3),.7*(.48-.03),0.,False)
    assert n.return_collision['reason']=='clear'
    assert n.velocity_pub.publish.call_args.args[0].linear.x > 0
    assert n.return_blocked is None


def test_batch_footprints_match_scalar_including_rotated_map_and_outside():
    rng=np.random.default_rng(91)
    cells=np.zeros((100,100),dtype=np.int8);cells[45:50,30:80]=100;cells[80:,:]=-1
    grid=FootprintMap(cells,.05,[-2.,-2.,.37],.35,.22)
    poses=rng.uniform([-3,-3,-math.pi],[4,4,math.pi],size=(300,3))
    assert np.array_equal(grid.clear_many(poses),[grid.clear(p) for p in poses])


def test_batch_region_checks_match_scalar_body_envelope():
    from third_impact.core import stationary_allowed_many, travel_allowed
    rng=np.random.default_rng(92);poses=rng.uniform([-4,-4,-math.pi],[4,4,math.pi],size=(400,3))
    boundary=[0.,0.,.3,6.,5.];exclusion=[1.,1.,-.6,1.,2.]
    for excluded in (None,exclusion):
        assert np.array_equal(stationary_allowed_many(poses,boundary,excluded,.35,.22),
            [travel_allowed(p,boundary,excluded,0.,0.,.35,.22) for p in poses])
