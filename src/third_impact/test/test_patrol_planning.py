from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock, patch
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from nav2_msgs.msg import Costmap
from third_impact.patrol_planning import PatrolPlanning, costmap_component
from third_impact.node import ImpactNode


def grid(cells, resolution=1., origin=(0.,0.)):
    m=Costmap();m.header.frame_id='map'
    m.metadata.size_y,m.metadata.size_x=cells.shape
    m.metadata.resolution=resolution;m.metadata.origin.orientation.w=1.
    m.metadata.origin.position.x,m.metadata.origin.position.y=origin
    m.data=cells.astype(np.uint8).ravel().tolist()
    return m


def robot():
    n=NS(intent='patrol',manual=False,healthy=lambda:True,nav_pending=False,nav_handle=None,
         navigation_map_key=('map',),p={'patrol_tolerance':.05,'retry_seconds':2.},
         pose=lambda:np.array([1.5,1.5,0.]),stop=Mock(),event=Mock(),send_nav=Mock(),
         pose_message=lambda p:PoseStamped(),patrol_planner=Mock())
    for name in ('init_patrol_planning','on_patrol_costmap','reachable_patrol_target',
                 'reject_patrol_target','cancel_patrol_plan','patrol_plan_waiting','request_patrol_plan'):
        setattr(n,name,MethodType(getattr(PatrolPlanning,name),n))
    n.init_patrol_planning();n.on_patrol_costmap(grid(np.zeros((10,10))))
    accept=Future();result=Future();handle=Mock(accepted=True)
    handle.get_result_async.return_value=result
    n.patrol_planner.send_goal_async.return_value=accept
    n.patrol_planner.server_is_ready.return_value=True
    return n,accept,result,handle


def answer(status=4,end=(3.5,1.5)):
    path=NavPath();path.header.frame_id='map'
    for xy in ((1.5,1.5),end):
        p=PoseStamped();p.pose.position.x,p.pose.position.y=xy;path.poses.append(p)
    return NS(status=status,result=NS(path=path))


def test_recorded_failed_goal_is_rejected_and_room_goal_remains_reachable():
    f=np.load(Path(__file__).parent/'data/patrol_costmap_160953.npz')
    m=grid(f['cells'],float(f['resolution']),tuple(f['origin']))
    component=costmap_component(m,f['start'])
    def index(point):
        x,y=np.floor((np.array(point[:2])-f['origin'])/f['resolution']).astype(int)
        return y,x
    assert not component[index(f['goal'])]
    assert component[index([0.,2.])]


def test_unknown_inscribed_lethal_and_disconnected_cells_are_excluded():
    a=np.zeros((7,7));a[:,3]=254;a[0,0]=255;a[0,1]=253
    mask=costmap_component(grid(a),[1.5,1.5,0.])
    assert mask[2,2] and not mask[2,4] and not mask[0,0] and not mask[0,1]
    assert not costmap_component(grid(a),[3.5,1.5,0.]).any()


def test_preflight_does_not_move_until_successful_result():
    n,a,r,h=robot();n.request_patrol_plan([3.5,1.5,0.])
    n.send_nav.assert_not_called();a.set_result(h);n.send_nav.assert_not_called()
    r.set_result(answer());n.send_nav.assert_called_once_with([3.5,1.5,0.],'patrol')


def test_planner_failure_and_tolerance_shortfall_blacklist_without_rotation():
    for response in (answer(status=6),answer(end=(3.1,1.5))):
        n,a,r,h=robot();n.request_patrol_plan([3.5,1.5,0.]);a.set_result(h);r.set_result(response)
        n.send_nav.assert_not_called();n.stop.assert_called_once()
        assert n.state=='PATROL_SEARCH' and n.patrol_rejected and n.patrol_recovery is None


def test_manual_return_and_zone_change_discard_late_success():
    for key,value in [('manual',True),('intent','return'),('navigation_map_key',('changed',))]:
        n,a,r,h=robot();n.request_patrol_plan([3.5,1.5,0.]);a.set_result(h)
        setattr(n,key,value);r.set_result(answer());n.send_nav.assert_not_called()


def test_cancel_before_acceptance_cancels_late_handle():
    n,a,r,h=robot();n.request_patrol_plan([3.5,1.5,0.]);n.cancel_patrol_plan();a.set_result(h)
    h.cancel_goal_async.assert_called_once();n.send_nav.assert_not_called()


def test_timeout_cancels_and_stale_result_never_drives():
    n,a,r,h=robot();n.request_patrol_plan([3.5,1.5,0.]);a.set_result(h)
    n.patrol_plan_job['started']-=6.
    assert n.patrol_plan_waiting();h.cancel_goal_async.assert_called_once()
    r.set_result(answer());n.send_nav.assert_not_called()


def test_changed_or_stale_costmap_prevents_dispatch():
    for stale in (False,True):
        n,a,r,h=robot();n.request_patrol_plan([3.5,1.5,0.]);a.set_result(h)
        if stale:n.patrol_costmap_stamp-=3.
        else:
            cells=np.zeros((10,10));cells[:,2]=254;n.on_patrol_costmap(grid(cells))
        r.set_result(answer());n.send_nav.assert_not_called()


def test_no_duplicate_requests_while_preflight_pending():
    n,a,r,h=robot();n.request_patrol_plan([3.5,1.5,0.]);n.request_patrol_plan([4.5,1.5,0.])
    n.patrol_planner.send_goal_async.assert_called_once()


def test_failure_after_path_exists_keeps_motion_recovery():
    n,a,r,h=robot();n.nav_purpose='patrol';n.cancel_requested=False;n.patrol_reached=False
    n.nav_target=[3.5,1.5,0.];n.patrol_path=np.array([[1.5,1.5],[3.5,1.5]])
    n.begin_patrol_recovery=Mock();f=Future();f.set_result(NS(status=6))
    ImpactNode.nav_finished(n,f);n.begin_patrol_recovery.assert_called_once()


def test_rejected_neighborhood_is_skipped_until_cooldown_expires():
    n,a,r,h=robot();n.grid=NS(info=NS(origin=n.patrol_costmap.metadata.origin,resolution=1.))
    n.candidates=np.array([[1,3],[1,4]])
    n.data={'boundary':[5,5,0,10,10],'rectangle':[20,20,0,1,1]}
    n.p.update(footprint_half_length=.35,footprint_half_width=.22)
    n.reject_patrol_target([3.5,1.5,0.],'test')
    for _ in range(10):assert ImpactNode.patrol_target(n)[0]==4.5
    n.patrol_rejected=[([3.5,1.5],time.monotonic()-1)]
    n.candidates=np.array([[1,3]])
    assert ImpactNode.patrol_target(n)[0]==3.5
