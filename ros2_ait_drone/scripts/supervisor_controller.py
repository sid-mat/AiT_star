"""
supervisor_controller.py
────────────────────────────────────────────────────────────────────────────
Webots Supervisor controller — place this file in:
  ~/webots_project/controllers/supervisor_controller/supervisor_controller.py

In your Webots world, the Supervisor Robot node should have:
  controller: "supervisor_controller"
  supervisor: TRUE

This single script:
  1. Reads /planned_path from ROS2 and moves the main drone kinematically
  2. Moves 3 obstacle drones on Lissajous trajectories
  3. Publishes /drone_pose and /dynamic_obstacles back to ROS2

No webots_ros2_driver needed — pure Webots Python controller + rclpy.
"""

import sys
import os
import math
import rclpy
from rclpy.node import Node
import numpy as np

# Webots controller API
from controller import Supervisor

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Path


# ── Obstacle trajectories (Lissajous) ─────────────────────────────────────────
OBSTACLES = [
    dict(cx= 0.0, cy= 0.0, cz=3.0, ax=4.0, ay=0.0, az=1.0,
         wx=0.4,  wy=0.0, wz=0.2,  ph=0.0, radius=1.2),
    dict(cx= 2.0, cy= 2.0, cz=4.0, ax=0.0, ay=5.0, az=1.5,
         wx=0.0,  wy=0.3, wz=0.25, ph=1.0, radius=1.0),
    dict(cx=-3.0, cy= 3.0, cz=3.5, ax=3.0, ay=3.0, az=0.8,
         wx=0.5,  wy=0.5, wz=0.3,  ph=2.1, radius=1.1),
]

DRONE_SPEED  = 2.0    # m/s
WP_TOL       = 0.4    # m


def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())

    # ── Get DEF nodes ──────────────────────────────────────────────────────────
    main_node = robot.getFromDef('MAIN_DRONE')
    obs_nodes = [robot.getFromDef(f'OBS_DRONE_{i}') for i in range(3)]

    # Translation fields (Webots coords: X=East, Y=Up, Z=South)
    main_trans = main_node.getField('translation') if main_node else None
    obs_trans  = [n.getField('translation') if n else None for n in obs_nodes]
    main_rot   = main_node.getField('rotation') if main_node else None

    # ── Init ROS2 ─────────────────────────────────────────────────────────────
    rclpy.init()
    node = rclpy.create_node('webots_supervisor')

    path_waypoints = []
    wp_idx = [0]
    drone_pos = np.array([-8.0, -8.0, 1.5])  # ROS frame (x,y,z)

    def on_path(msg: Path):
        path_waypoints.clear()
        for p in msg.poses:
            path_waypoints.append(np.array([
                p.pose.position.x,
                p.pose.position.y,
                p.pose.position.z
            ]))
        wp_idx[0] = 0
        node.get_logger().info(f'New path: {len(path_waypoints)} waypoints')

    path_sub   = node.create_subscription(Path, '/planned_path', on_path, 10)
    drone_pub  = node.create_publisher(PoseStamped, '/drone_pose', 10)
    obs_pub    = node.create_publisher(PoseArray, '/dynamic_obstacles', 10)

    t_sim = 0.0
    dt    = timestep / 1000.0

    node.get_logger().info('Webots supervisor ready.')

    # ── Main loop ─────────────────────────────────────────────────────────────
    while robot.step(timestep) != -1:
        rclpy.spin_once(node, timeout_sec=0)

        t_sim += dt

        # ── Move main drone along path ─────────────────────────────────────────
        if path_waypoints and wp_idx[0] < len(path_waypoints):
            target = path_waypoints[wp_idx[0]]
            direction = target - drone_pos
            dist = np.linalg.norm(direction)

            if dist < WP_TOL:
                wp_idx[0] += 1
            else:
                step = (direction / dist) * min(DRONE_SPEED * dt, dist)
                drone_pos += step

                # Webots: Y=up, Z=-ROS_Y
                if main_trans:
                    main_trans.setSFVec3f([
                        float(drone_pos[0]),
                        float(drone_pos[2]),
                        float(-drone_pos[1])
                    ])
                # Yaw to face direction of travel
                if main_rot and dist > 0.01:
                    yaw = math.atan2(direction[1], direction[0])
                    main_rot.setSFRotation([0, 1, 0, yaw - math.pi / 2])

        # Publish drone pose to ROS2
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp    = node.get_clock().now().to_msg()
        ps.pose.position.x = float(drone_pos[0])
        ps.pose.position.y = float(drone_pos[1])
        ps.pose.position.z = float(drone_pos[2])
        ps.pose.orientation.w = 1.0
        drone_pub.publish(ps)

        # ── Move obstacle drones ───────────────────────────────────────────────
        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp    = node.get_clock().now().to_msg()

        for i, obs in enumerate(OBSTACLES):
            x = obs['cx'] + obs['ax'] * math.sin(obs['wx'] * t_sim + obs['ph'])
            y = obs['cy'] + obs['ay'] * math.sin(obs['wy'] * t_sim + obs['ph'] + 0.5)
            z = obs['cz'] + obs['az'] * math.sin(obs['wz'] * t_sim + obs['ph'] + 1.0)
            z = max(0.5, z)

            if obs_trans[i]:
                obs_trans[i].setSFVec3f([x, z, -y])   # Webots coords

            p = Pose()
            p.position.x  = x;  p.position.y = y;  p.position.z = z
            p.orientation.w = obs['radius']
            pa.poses.append(p)

        obs_pub.publish(pa)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
