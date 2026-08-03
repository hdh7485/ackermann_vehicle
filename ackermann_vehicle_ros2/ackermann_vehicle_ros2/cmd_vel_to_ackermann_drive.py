"""ROS 2 Twist-to-Ackermann command bridge."""

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from ackermann_vehicle_ros2.kinematics import twist_to_steering_angle


class CmdVelToAckermannDrive(Node):
    """Publish AckermannDriveStamped commands for incoming Twist messages."""

    def __init__(self):
        super().__init__("cmd_vel_to_ackermann_drive")
        self.declare_parameter("twist_cmd_topic", "/cmd_vel")
        self.declare_parameter("ackermann_cmd_topic", "/ackermann_cmd")
        self.declare_parameter("wheelbase", 1.0)
        self.declare_parameter("frame_id", "odom")

        twist_topic = self.get_parameter("twist_cmd_topic").value
        ackermann_topic = self.get_parameter("ackermann_cmd_topic").value
        self._wheelbase = float(self.get_parameter("wheelbase").value)
        self._frame_id = self.get_parameter("frame_id").value

        if self._wheelbase <= 0.0:
            raise ValueError("wheelbase must be greater than zero")

        self._publisher = self.create_publisher(
            AckermannDriveStamped, ackermann_topic, 10
        )
        self._subscription = self.create_subscription(
            Twist, twist_topic, self._on_twist, 10
        )
        self.get_logger().info(
            "Listening to {} and publishing AckermannDriveStamped to {}".format(
                twist_topic, ackermann_topic
            )
        )

    def _on_twist(self, message):
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self._frame_id
        command.drive.speed = message.linear.x
        command.drive.steering_angle = twist_to_steering_angle(
            message.linear.x, message.angular.z, self._wheelbase
        )
        self._publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelToAckermannDrive()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
