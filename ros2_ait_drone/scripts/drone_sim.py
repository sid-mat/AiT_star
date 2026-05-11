#!/usr/bin/env python3
"""
drone_sim.py
────────────────────────────────────────────────────────────────────────────
Pure ROS2 Python node — NO Webots required.
Simulates:
  1. A drone following /planned_path using pure-pursuit in 3D
  2. N dynamic obstacle drones on random Lissajous trajectories
  3. Publishes everything needed for the planner node and RViz2

Run standalone (Phase 1):
    ros2 run ros2_ait_drone drone_sim.py

Pairs with Webots (Phase 2):
    Launch via demo.launch.py — the Webots driver replaces the drone
    movement part of this node, but obstacle simulation stays here.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String


# ── Obstacle trajectory parameters ────────────────────────────────────────────
OBSTACLES = [
    # (cx, cy, cz, ax, ay, az, wx, wy, wz, phase, radius)
    # Lissajous: pos = c + a * sin(w*t + phase)
    dict(cx= 0.0, cy= 0.0, cz=3.0,
         ax= 4.0, ay= 0.0, az=1.0,
         wx= 0.4, wy= 0.0, wz=0.2,
         ph= 0.0, radius=1.2),

    dict(cx= 2.0, cy= 2.0, cz=4.0,
         ax= 0.0, ay= 5.0, az=1.5,
         wx= 0.0, wy= 0.3, wz=0.25,
         ph= 1.0, radius=1.0),

    dict(cx=-3.0, cy= 3.0, cz=3.5,
         ax= 3.0, ay= 3.0, az=0.8,
         wx= 0.5, wy= 0.5, wz=0.3,
         ph= 2.1, radius=1.1),
]

DRONE_SPEED   = 1.8     # m/s along path
DRONE_RADIUS  = 0.35    # m
WAYPOINT_TOL  = 0.4     # m — switch to next waypoint


class DroneSim(Node):
    def __init__(self):
        super().__init__('drone_sim')

        # Drone state
        self.pos   = np.array([-8.0, -8.0, 1.5])
        self.path  = []          # list of np.array waypoints
        self.wp_idx = 0

        # ROS2
        self.drone_pub = self.create_publisher(PoseStamped, '/drone_pose', 10)
        self.obs_pub   = self.create_publisher(PoseArray,   '/dynamic_obstacles', 10)
        self.obsviz_pub= self.create_publisher(MarkerArray, '/obstacle_viz', 10)
        self.drone_viz = self.create_publisher(MarkerArray, '/drone_viz', 10)

        self.path_sub  = self.create_subscription(
            Path, '/planned_path', self.on_path, 10)

        # Timers
        self.create_timer(0.05,  self.step_drone)      # 20 Hz drone movement
        self.create_timer(0.05,  self.step_obstacles)  # 20 Hz obstacle movement
        self.create_timer(0.033, self.publish_viz)     # 30 Hz visualization

        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info('DroneSim running.')

    # ── Path follower ──────────────────────────────────────────────────────────
    def on_path(self, msg: Path):
        self.path = [np.array([p.pose.position.x,
                               p.pose.position.y,
                               p.pose.position.z])
                     for p in msg.poses]
        self.wp_idx = 0
        self.get_logger().info(f'New path received: {len(self.path)} waypoints')

    def step_drone(self):
        dt = 0.05

        if self.path and self.wp_idx < len(self.path):
            target = self.path[self.wp_idx]
            direction = target - self.pos
            dist = np.linalg.norm(direction)

            if dist < WAYPOINT_TOL:
                self.wp_idx += 1
            else:
                move = (direction / dist) * min(DRONE_SPEED * dt, dist)
                self.pos = self.pos + move

        # Publish drone pose
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.pose.position.x = float(self.pos[0])
        ps.pose.position.y = float(self.pos[1])
        ps.pose.position.z = float(self.pos[2])
        ps.pose.orientation.w = 1.0
        self.drone_pub.publish(ps)

    # ── Obstacle simulation ────────────────────────────────────────────────────
    def step_obstacles(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp    = self.get_clock().now().to_msg()

        for obs in OBSTACLES:
            x = obs['cx'] + obs['ax'] * math.sin(obs['wx'] * t + obs['ph'])
            y = obs['cy'] + obs['ay'] * math.sin(obs['wy'] * t + obs['ph'] + 0.5)
            z = obs['cz'] + obs['az'] * math.sin(obs['wz'] * t + obs['ph'] + 1.0)
            z = max(0.5, z)  # stay above ground

            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            # Pack radius into orientation.w (convention shared with planner_node.cpp)
            p.orientation.w = obs['radius']
            pa.poses.append(p)

        self.obs_pub.publish(pa)

    # ── Visualization ──────────────────────────────────────────────────────────
    def publish_viz(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        ma = MarkerArray()

        # Drone body — arrow pointing forward
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = 'drone'
        m.id = 0
        m.type   = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(self.pos[0])
        m.pose.position.y = float(self.pos[1])
        m.pose.position.z = float(self.pos[2])
        m.pose.orientation.w = 1.0
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.15
        m.color.r = 0.1; m.color.g = 0.9; m.color.b = 0.3; m.color.a = 1.0
        ma.markers.append(m)

        # Drone propellers (4 spheres)
        arm_offsets = [(0.3, 0.3, 0.05), (-0.3, 0.3, 0.05),
                       (0.3,-0.3, 0.05), (-0.3,-0.3, 0.05)]
        for i, (ox, oy, oz) in enumerate(arm_offsets):
            prop = Marker()
            prop.header = m.header
            prop.ns = 'propellers'; prop.id = i + 1
            prop.type = Marker.CYLINDER; prop.action = Marker.ADD
            prop.pose.position.x = float(self.pos[0]) + ox
            prop.pose.position.y = float(self.pos[1]) + oy
            prop.pose.position.z = float(self.pos[2]) + oz
            prop.pose.orientation.w = 1.0
            prop.scale.x = prop.scale.y = 0.22; prop.scale.z = 0.03
            prop.color.r = 0.9; prop.color.g = 0.9; prop.color.b = 0.9; prop.color.a = 0.9
            ma.markers.append(prop)

        self.drone_viz.publish(ma)

        # Dynamic obstacle spheres
        obs_ma = MarkerArray()
        for i, obs in enumerate(OBSTACLES):
            x = obs['cx'] + obs['ax'] * math.sin(obs['wx'] * t + obs['ph'])
            y = obs['cy'] + obs['ay'] * math.sin(obs['wy'] * t + obs['ph'] + 0.5)
            z = obs['cz'] + obs['az'] * math.sin(obs['wz'] * t + obs['ph'] + 1.0)
            z = max(0.5, z)

            om = Marker()
            om.header.frame_id = 'world'
            om.header.stamp    = self.get_clock().now().to_msg()
            om.ns = 'dyn_obs'; om.id = i
            om.type   = Marker.SPHERE
            om.action = Marker.ADD
            om.pose.position.x = x
            om.pose.position.y = y
            om.pose.position.z = z
            om.pose.orientation.w = 1.0
            d = obs['radius'] * 2
            om.scale.x = d; om.scale.y = d; om.scale.z = d
            om.color.r = 1.0; om.color.g = 0.15; om.color.b = 0.1; om.color.a = 0.75
            obs_ma.markers.append(om)

            # Trail — shrinking spheres
            for j in range(1, 5):
                trail_t = t - j * 0.3
                tx = obs['cx'] + obs['ax'] * math.sin(obs['wx'] * trail_t + obs['ph'])
                ty = obs['cy'] + obs['ay'] * math.sin(obs['wy'] * trail_t + obs['ph'] + 0.5)
                tz = max(0.5, obs['cz'] + obs['az'] * math.sin(obs['wz'] * trail_t + obs['ph'] + 1.0))
                tm = Marker()
                tm.header = om.header
                tm.ns = 'obs_trail'; tm.id = i * 10 + j
                tm.type = Marker.SPHERE; tm.action = Marker.ADD
                tm.pose.position.x = tx; tm.pose.position.y = ty; tm.pose.position.z = tz
                tm.pose.orientation.w = 1.0
                s = d * (1 - j * 0.18)
                tm.scale.x = tm.scale.y = tm.scale.z = max(s, 0.05)
                tm.color.r = 1.0; tm.color.g = 0.4; tm.color.b = 0.1
                tm.color.a = max(0.0, 0.4 - j * 0.08)
                obs_ma.markers.append(tm)

        self.obsviz_pub.publish(obs_ma)


def main():
    rclpy.init()
    node = DroneSim()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
