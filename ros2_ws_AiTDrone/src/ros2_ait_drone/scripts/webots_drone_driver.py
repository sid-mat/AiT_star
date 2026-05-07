"""
webots_drone_driver.py
────────────────────────────────────────────────────────────────────────────
Webots ROS2 driver for the main planning drone (Mavic 2 Pro).
Replaces drone_sim.py's movement logic when Webots is running.

The Supervisor teleports the drone along the OMPL planned path.
No attitude controller needed — purely kinematic for the demo.

webots_ros2_driver calls init() once then step() at each simulation timestep.

Webots world must have:
  DEF MAIN_DRONE Mavic2Pro { controller "ros2" ... }
  And a Robot node with: supervisor TRUE, controller "ros2"
  (use webots_obstacle_driver.py for the latter)
"""

import rclpy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class WebotsDroneDriver:
    """webots_ros2_driver plugin interface."""

    def init(self, webots_node, properties):
        self.__robot   = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Get the robot's own translation/rotation fields for kinematic movement
        self.__self_node = self.__robot.getSelf()
        self.__trans_field = self.__self_node.getField('translation')
        self.__rot_field   = self.__self_node.getField('rotation')

        # Path state
        self.__path      = []
        self.__wp_idx    = 0
        self.__speed     = float(properties.get('speed', '2.0'))
        self.__wp_tol    = float(properties.get('waypoint_tolerance', '0.4'))

        # Current pos (read from Webots at start)
        t = self.__trans_field.getSFVec3f()
        self.__pos = np.array([t[0], t[1], t[2]], dtype=float)

        # ROS2 within the driver
        rclpy.init(args=None)
        self.__node = rclpy.create_node('webots_drone_driver')

        self.__drone_pub = self.__node.create_publisher(
            PoseStamped, '/drone_pose', 10)
        self.__path_sub  = self.__node.create_subscription(
            Path, '/planned_path', self.__on_path, 10)

        self.__node.get_logger().info('Webots drone driver initialised.')

    def __on_path(self, msg: Path):
        self.__path = [
            np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])
            for p in msg.poses
        ]
        self.__wp_idx = 0

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        dt = self.__timestep / 1000.0   # seconds

        # Pure-pursuit: move toward next waypoint
        if self.__path and self.__wp_idx < len(self.__path):
            target = self.__path[self.__wp_idx]
            direction = target - self.__pos
            dist = np.linalg.norm(direction)

            if dist < self.__wp_tol:
                self.__wp_idx += 1
            else:
                step = (direction / dist) * min(self.__speed * dt, dist)
                self.__pos += step

                # Teleport drone in Webots (kinematic)
                self.__trans_field.setSFVec3f(
                    [float(self.__pos[0]),
                     float(self.__pos[2]),   # Webots Y = up
                     float(-self.__pos[1])]) # Webots Z = -ROS2_Y

                # Face direction of travel (yaw only)
                yaw = math.atan2(direction[1], direction[0])
                # Webots rotation: axis-angle [0,1,0, angle] for yaw
                self.__rot_field.setSFRotation([0, 1, 0, yaw - math.pi / 2])

        # Publish pose to ROS2
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp    = self.__node.get_clock().now().to_msg()
        ps.pose.position.x = float(self.__pos[0])
        ps.pose.position.y = float(self.__pos[1])
        ps.pose.position.z = float(self.__pos[2])
        ps.pose.orientation.w = 1.0
        self.__drone_pub.publish(ps)
