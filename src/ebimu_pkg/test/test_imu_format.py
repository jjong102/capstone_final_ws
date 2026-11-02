import math
from types import SimpleNamespace
from builtin_interfaces.msg import Time
from ebimu_pkg.ebimu_publisher import EbimuPublisher


def parser():
    # Exercise conversion with recorded sensor text without opening serial or ROS nodes.
    p = SimpleNamespace(
        orientation_format='quaternion', quaternion_field_indices=[2, 1, 0, 3],
        rpy_field_indices=[0, 1, 2], gyro_field_indices=[4, 5, 6],
        accel_field_indices=[7, 8, 9], use_degrees=True, invert_yaw=False,
        invert_gyro_z_with_yaw=True, gyro_in_deg_s=True, accel_in_g=True,
        frame_id='imu_link', orientation_cov_roll_pitch=.03, orientation_cov_yaw=.15,
        angular_velocity_covariance_diag=.02, linear_acceleration_covariance_diag=.2,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
    )
    for method in ('normalize_tokens', 'token_to_float', 'vector_from_indices', 'build_imu_msg'):
        setattr(p, method, getattr(EbimuPublisher, method).__get__(p))
    return p


def test_recorded_quaternion_and_gyro_are_separate():
    p = parser()
    msg = p.build_imu_msg('*0.0522,0.8808,0.4699,-0.0241,3.33,-2.89,38.49,-0.007,-0.005,0.025,-0.151,-0.096,1.147')
    norm = math.sqrt(.0522**2 + .8808**2 + .4699**2 + .0241**2)
    assert math.isclose(msg.orientation.x, .4699 / norm)
    assert math.isclose(msg.orientation.y, .8808 / norm)
    assert math.isclose(msg.orientation.z, .0522 / norm)
    assert math.isclose(msg.orientation.w, -.0241 / norm)
    assert math.isclose(msg.angular_velocity.z, math.radians(38.49))
    assert msg.header.frame_id == 'imu_link'


def test_quaternion_invalid_and_known_rotation():
    p = parser()
    assert p.build_imu_msg('*0,0,0,0') is None
    assert p.build_imu_msg('*nan,0,0,1') is None
    assert p.build_imu_msg('*1,2') is None
    msg = p.build_imu_msg('*0.70710678,0,0,0.70710678')
    assert math.isclose(msg.orientation.z, math.sqrt(.5), rel_tol=1e-7)
    assert math.isclose(msg.orientation.w, math.sqrt(.5), rel_tol=1e-7)


def test_legacy_euler_is_explicit():
    p = parser()
    p.orientation_format = 'euler'
    msg = p.build_imu_msg('*0,0,90')
    assert math.isclose(msg.orientation.z, math.sqrt(.5))
    assert math.isclose(msg.orientation.w, math.sqrt(.5))
