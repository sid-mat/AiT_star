"""
demo.launch.py
────────────────────────────────────────────────────────────────────────────
Phase 1 (no Webots):
    ros2 launch ros2_ait_drone demo.launch.py use_webots:=false

Phase 2 (with Webots):
    ros2 launch ros2_ait_drone demo.launch.py use_webots:=true

Then in a separate terminal, set goal and switch planners:
    # Set goal
    ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
      '{header: {frame_id: world}, pose: {position: {x: 8.0, y: 8.0, z: 4.0}, orientation: {w: 1.0}}}'

    # Switch to RRT*
    ros2 topic pub --once /switch_planner std_msgs/String "data: 'rrt'"

    # Switch to Informed RRT*
    ros2 topic pub --once /switch_planner std_msgs/String "data: 'informed'"

    # Switch back to AIT*
    ros2 topic pub --once /switch_planner std_msgs/String "data: 'ait'"
"""

import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             ExecuteProcess, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('ros2_ait_drone')

    use_webots_arg = DeclareLaunchArgument(
        'use_webots', default_value='false',
        description='Launch Webots simulation (true) or pure ROS2 sim (false)')

    use_webots = LaunchConfiguration('use_webots')

    # ── OMPL Planner node (always launches) ────────────────────────────────────
    planner_node = Node(
        package='ros2_ait_drone',
        executable='planner_node',
        name='ait_planner_node',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'demo_params.yaml'),
            {'planner': 'ait'}
        ]
    )

    # ── Pure Python sim (Phase 1, no Webots) ───────────────────────────────────
    drone_sim_node = Node(
        package='ros2_ait_drone',
        executable='drone_sim.py',
        name='drone_sim',
        output='screen',
        condition=UnlessCondition(use_webots)
    )

    # ── Webots launch (Phase 2) ────────────────────────────────────────────────
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('webots_ros2_driver'),
                'launch', 'webots_launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'drone_demo.wbt'),
        }.items(),
        condition=IfCondition(use_webots)
    )

    webots_driver_node = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[{
            'robot_description':
                os.path.join(pkg, 'config', 'robot_description.yaml'),
        }],
        condition=IfCondition(use_webots)
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'config', 'rviz_config.rviz')],
        output='screen'
    )

    # ── Auto-publish initial goal after 2s (give planner time to start) ────────
    initial_goal = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once',
                     '/goal_pose', 'geometry_msgs/msg/PoseStamped',
                     '{"header": {"frame_id": "world"}, '
                     '"pose": {"position": {"x": 8.0, "y": 8.0, "z": 4.0}, '
                     '"orientation": {"w": 1.0}}}'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        use_webots_arg,
        planner_node,
        drone_sim_node,
        webots_launch,
        webots_driver_node,
        rviz_node,
        initial_goal,
    ])
