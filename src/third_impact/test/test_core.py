import math
import json
from pathlib import Path
import numpy as np
import pytest
from third_impact.core import (relative, transform, wrap, inside_rectangle, register_scan,
                               alignment_command, swept_clear, atomic_save, map_fingerprint)


def scene():
    t = np.linspace(-2., 2., 300)
    wall = np.column_stack((t, np.full_like(t, 2.)))
    side = np.column_stack((np.full_like(t, -2.), t))
    angle = np.linspace(0., 2*math.pi, 200)
    column = np.column_stack((.8+.2*np.cos(angle), .6+.2*np.sin(angle)))
    return np.vstack((wall, side, column))


def test_registration_recovers_pose_with_noise_and_outliers():
    reference = scene()
    truth = np.array([.065, -.04, math.radians(3)])
    inverse = relative(truth, [0., 0., 0.])
    current = transform(reference, inverse)
    rng = np.random.default_rng(9)
    current += rng.normal(0., .002, current.shape)
    current = np.vstack((current, rng.uniform(-3, 3, (40, 2))))
    result = register_scan(current, reference, [.05, -.03, .03])
    assert result is not None
    assert np.linalg.norm(result['pose'][:2]-truth[:2]) < .005
    assert abs(wrap(result['pose'][2]-truth[2])) < .005
    assert result['overlap'] > .85
    assert result['observability'] > .015


def test_parallel_corridor_is_not_observable():
    x = np.linspace(-3., 3., 400)
    ref = np.vstack((np.column_stack((x, x*0+1)), np.column_stack((x, x*0-1))))
    result = register_scan(ref, ref, [0., 0., 0.])
    assert result is not None
    assert result['observability'] < .015


def test_rotated_rectangle():
    rect = [2., 3., math.pi/4, 2., .4]
    assert inside_rectangle(transform(np.array([[.9, .1]]), rect[:3])[0], rect)
    assert not inside_rectangle(transform(np.array([[0., .3]]), rect[:3])[0], rect)


def test_reverse_and_turn_control():
    v, w = alignment_command([.2, 0., 0.], .008, .087)
    assert v < 0 and w == 0
    v, w = alignment_command([0., 0., .2], .008, .087)
    assert v == 0 and w < 0
    assert alignment_command([.001, 0., .02], .008, .087) == (0., 0.)


def test_swept_collision_forward_reverse_rotation():
    assert not swept_clear(np.array([[.39, 0.]]), .1, 0.)
    assert not swept_clear(np.array([[-.39, 0.]]), -.1, 0.)
    assert not swept_clear(np.array([[.30, .30]]), 0., .6)
    assert swept_clear(np.array([[2., 2.]]), .03, .1)
    assert not swept_clear(np.empty((0, 2)), .03, 0.)


def test_atomic_save_and_map_identity(tmp_path):
    path = tmp_path/'goal.json'
    atomic_save(path, {'rectangle': [1, 2, 0, 3, 4]})
    atomic_save(path, {'rectangle': None})
    assert json.loads(path.read_text())['rectangle'] is None
    (tmp_path/'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    mapfile = tmp_path/'map.yaml'
    mapfile.write_text('image: map.pgm\nresolution: 0.03\norigin: [0, 0, 0]\n')
    before = map_fingerprint(mapfile)
    mapfile.write_text('image: map.pgm\nresolution: 0.04\norigin: [0, 0, 0]\n')
    assert map_fingerprint(mapfile) != before
    with pytest.raises(ValueError):
        atomic_save(path, {'bad': float('nan')})
    assert json.loads(path.read_text()) == {'rectangle': None}


def test_allowed_region_handles_rotation_and_motion_margin():
    from third_impact.core import inside_patrol_region, patrol_motion_allowed
    rect=[3.,-2.,math.pi/3,4.,2.]
    point=transform(np.array([[0.,.8]]),rect[:3])[0]
    assert inside_patrol_region(point,rect,.1)
    assert not inside_patrol_region(point,rect,.3)
    assert patrol_motion_allowed(rect[:3],rect,.3,0.,.35,.22)
    assert not patrol_motion_allowed(rect[:3],rect,.6,0.,.35,.22)
    assert not patrol_motion_allowed(rect[:3],rect,0.,.6,.35,.22)
    assert not patrol_motion_allowed(rect[:3],rect,float('nan'),0.,.35,.22)


def test_two_zone_policy_patrol_excludes_goal_but_return_can_enter():
    from third_impact.core import travel_allowed
    boundary=[0,0,0,10,10]; exclusion=[0,0,math.pi/4,2,2]
    assert not travel_allowed([0,0,0],boundary,exclusion,0,0,.35,.22)
    assert travel_allowed([0,0,0],boundary,None,0,0,.35,.22)
    assert travel_allowed([3,0,0],boundary,exclusion,0,0,.35,.22)
    assert not travel_allowed([5,0,0],boundary,None,0,0,.35,.22)
    assert not travel_allowed([2.,0,0],boundary,exclusion,-.2,0,.35,.22)


def test_navigation_map_masks_both_regions_without_changing_original():
    from third_impact.core import region_map
    cells=np.zeros((100,100),dtype=np.int8);cells[20,20]=-1
    boundary=[5,5,0,8,8]; exclusion=[5,5,math.pi/4,2,2]
    patrol=region_map(cells,.1,[0,0,0],boundary,exclusion)
    returning=region_map(cells,.1,[0,0,0],boundary)
    assert patrol[50,50]==100 and returning[50,50]==0
    assert patrol[0,0]==returning[0,0]==100
    assert patrol[20,20]==returning[20,20]==-1
    assert patrol[30,30]==0 and np.count_nonzero(cells==100)==0
    assert np.all(region_map(cells,.1,[0,0,0],None)==100)


def test_directional_zone_guard_allows_departure_but_blocks_approach():
    from third_impact.core import travel_allowed
    boundary=[0,0,0,10,10]; exclusion=[0,0,0,2,2]
    assert travel_allowed([1.55,0,0],boundary,exclusion,.6,0,.35,.22)
    assert not travel_allowed([1.55,0,0],boundary,exclusion,-.6,0,.35,.22)
    assert travel_allowed([4.4,0,0],boundary,None,-.6,0,.35,.22)
    assert not travel_allowed([4.4,0,0],boundary,None,.6,0,.35,.22)


def test_directional_guard_checks_curve_and_full_segment():
    from third_impact.core import travel_allowed
    boundary=[0,0,0,10,10]; exclusion=[0,0,0,1,1]
    assert not travel_allowed([-2,0,0],boundary,exclusion,4,0,.35,.22)
    assert travel_allowed([1.1,0,0],boundary,exclusion,0,.5,.35,.22)
    assert not travel_allowed([1.1,0,0],boundary,exclusion,0,.5,.35,.22,angular=2.)
