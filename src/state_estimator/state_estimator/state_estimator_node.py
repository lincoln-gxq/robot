"""
state_estimator_node – ROS2 node for differential-drive state estimation.

Subscribes to:
    /wheel_speeds  (geometry_msgs/Vector3)
        x = left-wheel linear speed (m/s)
        y = right-wheel linear speed (m/s)

Publishes:
    /odom   (nav_msgs/Odometry)
    /tf     (via tf2_ros TransformBroadcaster)
"""

from __future__ import annotations

import math
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped, Vector3
    from nav_msgs.msg import Odometry
    import tf2_ros
    _ROS2_AVAILABLE = True
except ImportError:  # pragma: no cover
    _ROS2_AVAILABLE = False

from state_estimator.kinematics import DifferentialDriveKinematics, RobotState


if _ROS2_AVAILABLE:

    class StateEstimatorNode(Node):
        def __init__(self) -> None:
            super().__init__("state_estimator_node")

            self.declare_parameter("wheel_track", 0.4)
            self.declare_parameter("wheel_radius", 0.1)
            self.declare_parameter("update_frequency", 50.0)

            L = self.get_parameter("wheel_track").get_parameter_value().double_value
            R = self.get_parameter("wheel_radius").get_parameter_value().double_value
            freq = self.get_parameter("update_frequency").get_parameter_value().double_value

            self._kinematics = DifferentialDriveKinematics(wheel_track_m=L, wheel_radius_m=R)
            self._state = RobotState()
            self._v_left: float = 0.0
            self._v_right: float = 0.0
            self._last_time = self.get_clock().now()

            self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self._wheel_sub = self.create_subscription(
                Vector3, "/wheel_speeds", self._wheel_callback, 10
            )
            self._timer = self.create_timer(1.0 / freq, self._update)
            self.get_logger().info("StateEstimatorNode started.")

        def _wheel_callback(self, msg: Vector3) -> None:
            self._v_left = msg.x
            self._v_right = msg.y

        def _update(self) -> None:
            now = self.get_clock().now()
            dt = (now - self._last_time).nanoseconds * 1e-9
            self._last_time = now
            if dt <= 0:
                return

            self._state = self._kinematics.step_from_wheels(
                self._state, self._v_left, self._v_right, dt
            )
            self._publish_odom(now)
            self._broadcast_tf(now)

        def _publish_odom(self, stamp) -> None:
            odom = Odometry()
            odom.header.stamp = stamp.to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self._state.x
            odom.pose.pose.position.y = self._state.y
            odom.pose.pose.orientation.z = math.sin(self._state.theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(self._state.theta / 2.0)
            odom.twist.twist.linear.x = self._state.vx
            odom.twist.twist.angular.z = self._state.omega
            self._odom_pub.publish(odom)

        def _broadcast_tf(self, stamp) -> None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp.to_msg()
            tf_msg.header.frame_id = "odom"
            tf_msg.child_frame_id = "base_link"
            tf_msg.transform.translation.x = self._state.x
            tf_msg.transform.translation.y = self._state.y
            tf_msg.transform.rotation.z = math.sin(self._state.theta / 2.0)
            tf_msg.transform.rotation.w = math.cos(self._state.theta / 2.0)
            self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):  # pragma: no cover
    rclpy.init(args=args)
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
