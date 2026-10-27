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
