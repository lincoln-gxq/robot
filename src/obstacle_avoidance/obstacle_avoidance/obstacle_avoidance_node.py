"""
obstacle_avoidance_node – ROS2 node for VFH local obstacle avoidance.

Subscribes to:
    /scan   (sensor_msgs/LaserScan)
    /goal_heading  (std_msgs/Float64) – desired heading in robot frame (rad)

Publishes:
    /cmd_vel_safe  (geometry_msgs/Twist)
"""

from __future__ import annotations

import math

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float64
    _ROS2_AVAILABLE = True
except ImportError:  # pragma: no cover
    _ROS2_AVAILABLE = False

from obstacle_avoidance.vfh import VFHController


if _ROS2_AVAILABLE:

    class ObstacleAvoidanceNode(Node):
        def __init__(self) -> None:
            super().__init__("obstacle_avoidance_node")

            self.declare_parameter("num_sectors", 72)
            self.declare_parameter("threshold", 3.0)
            self.declare_parameter("robot_radius", 0.2)
            self.declare_parameter("safety_distance", 0.3)
            self.declare_parameter("max_scan_range", 5.0)
            self.declare_parameter("linear_velocity", 0.3)

            ns = self.get_parameter("num_sectors").get_parameter_value().integer_value
            th = self.get_parameter("threshold").get_parameter_value().double_value
            rr = self.get_parameter("robot_radius").get_parameter_value().double_value
            sd = self.get_parameter("safety_distance").get_parameter_value().double_value
            mr = self.get_parameter("max_scan_range").get_parameter_value().double_value
            self._v = self.get_parameter("linear_velocity").get_parameter_value().double_value

            self._vfh = VFHController(
                num_sectors=ns,
                threshold=th,
                robot_radius_m=rr,
                safety_distance_m=sd,
                max_scan_range_m=mr,
            )
            self._goal_heading: float = 0.0

            self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
            self._scan_sub = self.create_subscription(
                LaserScan, "/scan", self._scan_callback, 10
            )
            self._heading_sub = self.create_subscription(
                Float64, "/goal_heading", self._heading_callback, 10
            )
            self.get_logger().info("ObstacleAvoidanceNode started.")

        def _heading_callback(self, msg: Float64) -> None:
            self._goal_heading = msg.data

        def _scan_callback(self, msg: LaserScan) -> None:
            v, omega = self._vfh.compute_command(
                scan_ranges=list(msg.ranges),
                scan_angle_min=msg.angle_min,
                scan_angle_increment=msg.angle_increment,
                goal_heading=self._goal_heading,
                linear_velocity=self._v,
            )
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = omega
            self._cmd_pub.publish(twist)


def main(args=None):  # pragma: no cover
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
