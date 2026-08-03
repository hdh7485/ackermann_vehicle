import math

import pytest

from ackermann_vehicle_ros2.kinematics import twist_to_steering_angle


def test_straight_and_stationary_commands_have_zero_steering():
    assert twist_to_steering_angle(1.0, 0.0, 1.0) == 0.0
    assert twist_to_steering_angle(0.0, 1.0, 1.0) == 0.0


def test_turning_command_matches_ackermann_geometry():
    assert math.isclose(
        twist_to_steering_angle(2.0, 1.0, 1.0), math.atan(0.5)
    )


def test_wheelbase_must_be_positive():
    with pytest.raises(ValueError):
        twist_to_steering_angle(1.0, 1.0, 0.0)
