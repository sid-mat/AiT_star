import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('ros2_ait_drone')

    use_webots_arg = DeclareLaunchArgument(
        'use_webots', default_value='false',
        description='true = launch Webots, false = pure Python sim')

    use_webots = LaunchConfiguration('use_webots')

    # ── OMPL planner node (always runs) ─────────────────────────────────────
    planner_node = Node(
        package='ros2_ait_drone',
        executable='planner_node',
        name='ait_planner_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'demo_params.yaml')]
    )

    # ── Phase 1: pure Python sim ──────────────────────────────────────────────
    drone_sim_node = Node(
        package='ros2_ait_drone',
        executable='drone_sim.py',
        name='drone_sim',
        output='screen',
        condition=UnlessCondition(use_webots)
    )

    # ── Phase 2: launch Webots with the world ─────────────────────────────────
    # Webots reads the world file and runs supervisor_controller.py
    # which you must place in your Webots controllers directory (see README).
    webots_proc = ExecuteProcess(
        cmd=['webots', os.path.join(pkg, 'worlds', 'drone_demo.wbt')],
        output='screen',
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

    # ── Auto-publish initial goal after 3s ───────────────────────────────────
    initial_goal = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once',
                 '/goal_pose', 'geometry_msgs/msg/PoseStamped',
                 '{"header": {"frame_id": "world"}, '
                 '"pose": {"position": {"x": 8.0, "y": 8.0, "z": 4.0}, '
                 '"orientation": {"w": 1.0}}}'],
            output='screen'
        )]
    )

    return LaunchDescription([
        use_webots_arg,
        planner_node,
        drone_sim_node,
        webots_proc,
        rviz_node,
        initial_goal,
    ])
