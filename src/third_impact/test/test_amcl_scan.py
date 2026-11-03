import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from geometry_msgs.msg import Transform
from sensor_msgs.msg import LaserScan
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import LookupException
from third_impact.amcl_scan import AmclScan, apply_transform, scan_points, virtual_ranges


def test_urdf_sensor_mounts():
    root = ET.parse(Path(__file__).parents[2] / 'amr/description/amr_description.urdf')
    for name, sign in [('lidar1_joint', 1), ('lidar2_joint', -1)]:
        origin = root.find(f".//joint[@name='{name}']/origin")
        tf = Transform()
        tf.translation.x, tf.translation.y, tf.translation.z = map(float, origin.get('xyz').split())
        q = Rotation.from_euler('xyz', list(map(float, origin.get('rpy').split()))).as_quat()
        tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w = q
        points = apply_transform([[0, 0, 0], [-1, 0, 0]], tf)
        assert points[0, 0] == pytest.approx(sign * .311)
        assert points[1, 0] == pytest.approx(sign * 1.311, abs=.00001)
        assert points[0, 2] == pytest.approx(.0465)


def scan(stamp=10.):
    m = LaserScan()
    m.header.frame_id = 'lidar'
    m.header.stamp = Time(seconds=stamp).to_msg()
    m.angle_min = -1.; m.angle_increment = .1
    m.range_min = .05; m.range_max = 12.
    m.ranges = [1.] * 20
    return m


def harness():
    tf = Transform(); tf.rotation.w = 1.
    node = NS(p=dict(target_frame='base_link', fixed_frame='odom', max_age=.45,
                    max_pair_skew=.15, max_range=6., bins=720,
                    footprint_half_length=.35, footprint_half_width=.22),
              scans={'front': scan(10.), 'rear': scan(10.05)}, consumed={}, last_stamp=0,
              tf=Mock(), pub=Mock(), diag=Mock(), get_clock=lambda: NS(now=lambda: Time(seconds=10.1)))
    node.tf.lookup_transform.return_value = NS(transform=tf)
    node.tf.lookup_transform_full.return_value = NS(transform=tf)
    return node


def test_publish_and_consume_pair_with_motion_compensation():
    n = harness(); AmclScan.tick(n)
    m = n.pub.publish.call_args.args[0]
    assert m.header.frame_id == 'base_link'
    assert Time.from_msg(m.header.stamp).nanoseconds == 10050000000
    assert len(m.ranges) == 720
    assert n.tf.lookup_transform_full.call_count == 2
    assert n.tf.lookup_transform_full.call_args_list[0].args[3].nanoseconds == 10000000000
    AmclScan.tick(n)
    assert n.pub.publish.call_count == 1
    assert json.loads(n.diag.publish.call_args.args[0].data)['reason'] == 'waiting_for_new_pair'


@pytest.mark.parametrize('case,reason', [('missing','waiting_for_both_scans'),
    ('stale','stale_or_future_scan'), ('skew','scan_pair_skew'),
    ('tf','tf_unavailable'), ('empty','insufficient_sensor_returns')])
def test_reject_bad_inputs(case, reason):
    n = harness()
    if case == 'missing': del n.scans['rear']
    if case == 'stale': n.scans['rear'] = scan(9.)
    if case == 'skew': n.scans['rear'] = scan(9.8)
    if case == 'tf': n.tf.lookup_transform.side_effect = LookupException('missing')
    if case == 'empty': n.scans['rear'].ranges = [float('inf')] * 20
    AmclScan.tick(n)
    n.pub.publish.assert_not_called()
    assert json.loads(n.diag.publish.call_args.args[0].data)['reason'] == reason


def test_ranges_and_nearest_return():
    m = scan(); m.ranges = [float('nan'), float('inf'), .02, 8., 2.]
    assert len(scan_points(m, 6.)) == 1
    ranges = virtual_ranges(np.array([[1., 0, 0], [2., 0, 0], [7., 0, 0]]))
    assert ranges[360] == 1.
    assert np.isfinite(ranges).sum() == 1


def test_older_scan_is_shifted_to_newer_robot_pose():
    n = harness()
    old = Transform(); old.rotation.w = 1.; old.translation.x = -.2
    current = Transform(); current.rotation.w = 1.
    n.tf.lookup_transform_full.side_effect = [NS(transform=old), NS(transform=current)]
    AmclScan.tick(n)
    output = n.pub.publish.call_args.args[0]
    raw = scan_points(n.scans['front'], 6.)
    expected = virtual_ranges(np.vstack([raw + [-.2, 0, 0], raw]))
    np.testing.assert_allclose(output.ranges, expected, rtol=1e-6)
