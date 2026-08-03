#!/usr/bin/env python3

"""Exit successfully after observing the expected ROS 2 Ackermann command."""

import math
import sys
import time

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node


class AckermannProbe(Node):
    """Receive one command and assert the known smoke-test conversion."""

    def __init__(self):
        super().__init__("ackermann_vehicle_smoke_probe")
        self.received = False
        self.valid = False
        self.create_subscription(
            AckermannDriveStamped, "/ackermann_cmd", self._on_command, 10
        )

    def _on_command(self, command):
        self.received = True
        self.valid = (
            math.isclose(command.drive.speed, 2.0)
            and math.isclose(
                command.drive.steering_angle, math.atan(0.5), rel_tol=1e-6
            )
        )


def main():
    rclpy.init()
    probe = AckermannProbe()
    deadline = time.monotonic() + 10.0
    try:
        while time.monotonic() < deadline and not probe.received:
            rclpy.spin_once(probe, timeout_sec=0.2)
        return 0 if probe.valid else 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
