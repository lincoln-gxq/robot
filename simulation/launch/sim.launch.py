"""
sim.launch.py – Unified launch file for the sweeping robot simulation.

Usage:
    ros2 launch simulation sim.launch.py world:=empty_room algorithm:=boustrophedon
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ------------------------------------------------------------------ #
    # Launch arguments
    # ------------------------------------------------------------------ #
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty_room",
        description="Gazebo world name (without .world extension)",
    )
    algorithm_arg = DeclareLaunchArgument(
        "algorithm",
        default_value="boustrophedon",
        description="Coverage algorithm: boustrophedon | spiral",
    )
    robot_radius_arg = DeclareLaunchArgument(
        "robot_radius",
        default_value="0.20",
        description="Robot radius in metres",
    )
    linear_velocity_arg = DeclareLaunchArgument(
        "linear_velocity",
        default_value="0.30",
        description="Robot forward speed in m/s",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 visualisation",
    )

    world = LaunchConfiguration("world")
    algorithm = LaunchConfiguration("algorithm")
    robot_radius = LaunchConfiguration("robot_radius")
    linear_velocity = LaunchConfiguration("linear_velocity")
    use_rviz = LaunchConfiguration("use_rviz")

    # ------------------------------------------------------------------ #
    # Paths
    # ------------------------------------------------------------------ #
    sim_share = FindPackageShare("simulation")
    config_share = FindPackageShare("simulation")  # config kept in same pkg

    world_file = PathJoinSubstitution([sim_share, "worlds", [world, ".world"]])
    rviz_config = PathJoinSubstitution([sim_share, "rviz", "sweeper.rviz"])
    robot_urdf = PathJoinSubstitution([sim_share, "models", "sweeper_robot", "robot.urdf"])
    params_file = PathJoinSubstitution([sim_share, "params", "planner_params.yaml"])

    # ------------------------------------------------------------------ #
    # Nodes
    # ------------------------------------------------------------------ #

    # Gazebo
    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_file, "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    # robot_state_publisher (URDF → TF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_urdf}],
    )

    # Spawn sweeper robot in Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_sweeper",
        arguments=[
            "-entity", "sweeper",
            "-file", robot_urdf,
            "-x", "1.0",
            "-y", "1.0",
            "-z", "0.05",
        ],
        output="screen",
    )

    # State estimator
    state_estimator_node = Node(
        package="state_estimator",
        executable="state_estimator_node",
        name="state_estimator_node",
        output="screen",
        parameters=[params_file],
    )

    # Coverage planner
    coverage_planner_node = Node(
        package="coverage_planner",
        executable="coverage_planner_node",
        name="coverage_planner_node",
        output="screen",
        parameters=[
            params_file,
            {"algorithm": algorithm},
            {"robot_radius": robot_radius},
            {"linear_velocity": linear_velocity},
        ],
    )

    # Path tracker
    path_tracker_node = Node(
        package="path_tracker",
        executable="path_tracker_node",
        name="path_tracker_node",
        output="screen",
        parameters=[
            params_file,
            {"linear_velocity": linear_velocity},
        ],
    )

    # Obstacle avoidance
    obstacle_avoidance_node = Node(
        package="obstacle_avoidance",
        executable="obstacle_avoidance_node",
        name="obstacle_avoidance_node",
        output="screen",
        parameters=[params_file],
    )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            algorithm_arg,
            robot_radius_arg,
            linear_velocity_arg,
            use_rviz_arg,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            state_estimator_node,
            coverage_planner_node,
            path_tracker_node,
            obstacle_avoidance_node,
            rviz_node,
        ]
    )
