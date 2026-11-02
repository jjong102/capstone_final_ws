from pathlib import Path
import math
from serial_test.robot_geometry import wheel_geometry


def test_actual_robot_dimensions_and_encoder_distance():
    path = Path(__file__).resolve().parents[2] / 'amr/description/amr_description.urdf'
    diameter, separation = wheel_geometry(path)
    assert math.isclose(diameter, 0.13)
    assert math.isclose(separation, 0.392)
    # One wheel revolution must represent the measured circumference, not 17 cm wheels.
    assert math.isclose(math.pi * diameter, 0.4084070449666731)
    # Recorded yaw discrepancy becomes consistent with IMU after geometry correction.
    corrected = -2113.125 * (diameter / .17) * (.272 / separation)
    assert abs(corrected - (-1106.46)) < 20
