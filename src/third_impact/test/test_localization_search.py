import math
import numpy as np
from third_impact.localization_search import GridModel, search_map
from third_impact.core import relative


def room(asymmetric=True):
    cells=np.zeros((100,120),dtype=np.int16)
    cells[:2,:]=cells[-2:,:]=100; cells[:,:2]=cells[:,-2:]=100
    if asymmetric:
        cells[25:60,68:71]=100
        cells[25:28,68:96]=100
        cells[70:76,25:32]=100
        cells[10:17,30:36]=100
    return GridModel(cells,.05,[-1.4,2.1,.35])


def scan(model,local_pose):
    angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
    distances=np.arange(.1,6.,.0125)
    a=angles+local_pose[2]
    x=local_pose[0]+np.cos(a[:,None])*distances
    y=local_pose[1]+np.sin(a[:,None])*distances
    row,col,good=model.indices(np.stack((x,y),axis=-1))
    hits=good & model.occupied[row,col]
    valid=np.any(hits,axis=1)
    lengths=distances[np.argmax(hits,axis=1)][valid]
    return np.column_stack((np.cos(angles[valid])*lengths,np.sin(angles[valid])*lengths))


def test_moved_robot_is_found_even_with_wrong_saved_pose():
    model=room()
    true=np.array([1.4,2.1,1.1])
    cloud=scan(model,true)
    wrong_hint=model.world_pose([4.7,3.6,-1.])
    result=search_map(model,cloud,wrong_hint,timeout=30.)
    assert result['confident'], result
    error=relative(model.world_pose(true),result['pose'])
    assert np.linalg.norm(error[:2])<.1, error
    assert abs(error[2])<math.radians(4), error


def test_symmetric_room_does_not_authorize_arbitrary_pose():
    model=room(False)
    result=search_map(model,scan(model,[1.4,2.1,.7]),timeout=30.)
    assert not result['confident'], result
    assert result['reason']=='ambiguous', result


def test_incomplete_search_and_missing_geometry_do_not_seed():
    model=room()
    assert not search_map(model,np.zeros((10,2)))['confident']
    result=search_map(model,scan(model,[1.4,2.1,.7]),timeout=0.)
    assert not result['confident'] and result['reason']=='search_interrupted'


def test_wrong_hint_must_still_match_current_scan_and_map():
    model=room(); cloud=scan(model,[1.4,2.1,1.1])
    assert model.verify(cloud,model.world_pose([1.4,2.1,1.1]))['valid']
    assert not model.verify(cloud,model.world_pose([4.7,3.6,-1.]))['valid']


def test_confident_matches_at_other_positions_are_not_wrong_aliases():
    model=room()
    for pose in ([2.2,3.2,-2.], [4.8,3.8,.3], [5.2,1.1,2.4]):
        result=search_map(model,scan(model,pose),model.world_pose([1.4,2.1,1.1]),timeout=30.)
        if result['confident']:
            error=relative(model.world_pose(pose),result['pose'])
            assert np.linalg.norm(error[:2])<.15, result
            assert abs(error[2])<math.radians(5), result
