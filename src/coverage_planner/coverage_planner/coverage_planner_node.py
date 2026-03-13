"""
coverage_planner_node – ROS2 node for coverage path planning.

Subscribes to:
    /map  (nav_msgs/OccupancyGrid)

Publishes:
    /coverage_path  (nav_msgs/Path)

Parameters (declared):
    algorithm       : str  – "boustrophedon" | "spiral"  (default "boustrophedon")
    robot_radius    : float – robot radius in metres      (default 0.20)
    linear_velocity : float – path execution speed (m/s)  (default 0.30)
"""

from __future__ import annotations

import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path
    _ROS2_AVAILABLE = True
except ImportError:  # pragma: no cover
    _ROS2_AVAILABLE = False

from map_tools.grid_map import GridMap
from coverage_planner.algorithms.boustrophedon import BoustrophedonPlanner
from coverage_planner.algorithms.spiral import SpiralPlanner


def _occupancy_grid_to_grid_map(msg) -> GridMap:
    """Convert a ROS2 OccupancyGrid message to a GridMap."""
    gm = GridMap(
        width_cells=msg.info.width,
        height_cells=msg.info.height,
        resolution=msg.info.resolution,
        origin_x=msg.info.origin.position.x,
        origin_y=msg.info.origin.position.y,
    )
    import numpy as np

    data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    # OccupancyGrid: 0=free, 100=occupied, -1=unknown
    occ = data.astype(int)
    grid_data = np.full_like(occ, GridMap.UNKNOWN, dtype="uint8")
    grid_data[occ == 0] = GridMap.FREE
    grid_data[occ == 100] = GridMap.OCCUPIED
    gm.data = grid_data
    return gm


if _ROS2_AVAILABLE:

    class CoveragePlannerNode(Node):
        def __init__(self) -> None:
            super().__init__("coverage_planner_node")

            self.declare_parameter("algorithm", "boustrophedon")
            self.declare_parameter("robot_radius", 0.20)
            self.declare_parameter("linear_velocity", 0.30)

            self._path_pub = self.create_publisher(Path, "/coverage_path", 10)
            self._map_sub = self.create_subscription(
                OccupancyGrid, "/map", self._map_callback, 10
            )
            self.get_logger().info("CoveragePlannerNode started.")

        def _map_callback(self, msg: OccupancyGrid) -> None:
            algorithm = self.get_parameter("algorithm").get_parameter_value().string_value
            robot_radius = (
                self.get_parameter("robot_radius").get_parameter_value().double_value
            )

            gm = _occupancy_grid_to_grid_map(msg)

            if algorithm == "spiral":
                planner = SpiralPlanner(gm, robot_radius_m=robot_radius)
            else:
                planner = BoustrophedonPlanner(gm, robot_radius_m=robot_radius)

            waypoints = planner.plan()
            path_msg = self._waypoints_to_path(waypoints, msg.header.frame_id)
            self._path_pub.publish(path_msg)
            self.get_logger().info(
                f"Published {len(waypoints)}-waypoint coverage path "
                f"(algorithm={algorithm})."
            )

        def _waypoints_to_path(self, waypoints, frame_id: str) -> Path:
            path = Path()
            path.header.frame_id = frame_id
            path.header.stamp = self.get_clock().now().to_msg()
            for wx, wy in waypoints:
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x = wx
                ps.pose.position.y = wy
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
            return path


def main(args=None):  # pragma: no cover
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
