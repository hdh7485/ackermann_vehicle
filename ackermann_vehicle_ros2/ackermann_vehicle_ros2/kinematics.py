"""Distribution-independent Ackermann command conversions."""

import math


def twist_to_steering_angle(linear_velocity, angular_velocity, wheelbase):
    """Convert a planar Twist command to an Ackermann steering angle.

    A stationary vehicle and straight-line command have zero steering angle.
    The calculation intentionally stays independent of ROS messages so it can
    be unit tested on every supported ROS 2 distribution.
    """
    if wheelbase <= 0.0:
        raise ValueError("wheelbase must be greater than zero")

    if linear_velocity == 0.0 or angular_velocity == 0.0:
        return 0.0

    return math.atan(wheelbase * angular_velocity / linear_velocity)
