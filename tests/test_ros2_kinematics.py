import math
import sys
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "ackermann_vehicle_ros2"))

from ackermann_vehicle_ros2.kinematics import twist_to_steering_angle


class Ros2KinematicsTests(unittest.TestCase):
    def test_twist_conversion_preserves_ackermann_geometry(self):
        self.assertAlmostEqual(
            twist_to_steering_angle(2.0, 1.0, 1.0), math.atan(0.5)
        )
        self.assertEqual(twist_to_steering_angle(0.0, 1.0, 1.0), 0.0)
        self.assertEqual(twist_to_steering_angle(1.0, 0.0, 1.0), 0.0)

    def test_invalid_wheelbase_is_rejected(self):
        with self.assertRaises(ValueError):
            twist_to_steering_angle(1.0, 1.0, 0.0)
