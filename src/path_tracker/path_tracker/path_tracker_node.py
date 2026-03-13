"""
path_tracker_node – ROS2 node for Pure Pursuit path tracking.

Subscribes to:
    /coverage_path   (nav_msgs/Path)
    /odom            (nav_msgs/Odometry)

Publishes:
    /cmd_vel         (geometry_msgs/Twist)
"""

from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, Path
    _ROS2_AVAILABLE = True
except ImportError:  # pragma: no cover
    _ROS2_AVAILABLE = False

from state_estimator.kinematics import RobotState
from path_tracker.pure_pursuit import PurePursuitController


if _ROS2_AVAILABLE:

    class PathTrackerNode(Node):
        def __init__(self) -> None:
            super().__init__("path_tracker_node")

            self.declare_parameter("lookahead_distance", 0.5)
            self.declare_parameter("linear_velocity", 0.3)
            self.declare_parameter("max_angular_velocity", 1.0)
            self.declare_parameter("goal_tolerance", 0.15)
            self.declare_parameter("control_frequency", 20.0)

            ld = self.get_parameter("lookahead_distance").get_parameter_value().double_value
            v = self.get_parameter("linear_velocity").get_parameter_value().double_value
            max_omega = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
            tol = self.get_parameter("goal_tolerance").get_parameter_value().double_value
            freq = self.get_parameter("control_frequency").get_parameter_value().double_value

            self._controller = PurePursuitController(
                lookahead_distance_m=ld,
                linear_velocity_m_s=v,
                max_angular_velocity=max_omega,
                goal_tolerance_m=tol,
            )
            self._state = RobotState()

            self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self._path_sub = self.create_subscription(
                Path, "/coverage_path", self._path_callback, 10
            )
            self._odom_sub = self.create_subscription(
                Odometry, "/odom", self._odom_callback, 10
            )
            self._timer = self.create_timer(1.0 / freq, self._control_loop)
            self.get_logger().info("PathTrackerNode started.")

        def _path_callback(self, msg: Path) -> None:
            waypoints = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
            self._controller.set_path(waypoints)
            self.get_logger().info(f"Received path with {len(waypoints)} waypoints.")

        def _odom_callback(self, msg: Odometry) -> None:
            import math
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            # Convert quaternion yaw
            siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self._state = RobotState(
                x=pos.x, y=pos.y, theta=yaw,
                vx=msg.twist.twist.linear.x,
                omega=msg.twist.twist.angular.z,
            )

        def _control_loop(self) -> None:
            v, omega = self._controller.compute_control(self._state)
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = omega
            self._cmd_pub.publish(twist)
            if self._controller.is_finished:
                self.get_logger().info("Path tracking complete.", once=True)


def main(args=None):  # pragma: no cover
    rclpy.init(args=args)
    node = PathTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
