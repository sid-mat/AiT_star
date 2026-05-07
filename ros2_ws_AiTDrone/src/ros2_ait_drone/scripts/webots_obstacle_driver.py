"""
webots_obstacle_driver.py
────────────────────────────────────────────────────────────────────────────
Webots Supervisor driver — moves the obstacle drone nodes kinematically
and publishes their positions to ROS2 for the planner collision checker.

Webots world must have three robots DEF'd:
  DEF OBS_DRONE_0 Mavic2Pro { ... }
  DEF OBS_DRONE_1 Mavic2Pro { ... }
  DEF OBS_DRONE_2 Mavic2Pro { ... }
And a Robot with:
  supervisor TRUE
  controller "ros2"
  controllerArgs [ "--" ]
  (map to this driver in your webots_ros2 launcher params)

Obstacle trajectories are Lissajous curves — same parameters as drone_sim.py
so the RViz2 and Webots views stay in sync.
"""

import rclpy
import math

from geometry_msgs.msg import PoseArray, Pose

# Match drone_sim.py trajectories exactly
OBSTACLES = [
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

OBS_DEFS = ['OBS_DRONE_0', 'OBS_DRONE_1', 'OBS_DRONE_2']


class WebotsObstacleDriver:

    def init(self, webots_node, properties):
        self.__robot    = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__t        = 0.0

        # Get DEF'd nodes
        self.__obs_nodes = [
            self.__robot.getFromDef(d) for d in OBS_DEFS
        ]
        self.__trans_fields = [
            n.getField('translation') if n else None
            for n in self.__obs_nodes
        ]

        rclpy.init(args=None)
        self.__node = rclpy.create_node('webots_obstacle_driver')
        self.__pub  = self.__node.create_publisher(
            PoseArray, '/dynamic_obstacles', 10)

        self.__node.get_logger().info(
            f'Obstacle driver ready. Managing {len(OBS_DEFS)} obstacle drones.')

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        dt = self.__timestep / 1000.0
        self.__t += dt

        pa = PoseArray()
        pa.header.frame_id = 'world'
        pa.header.stamp    = self.__node.get_clock().now().to_msg()

        for i, obs in enumerate(OBSTACLES):
            x = obs['cx'] + obs['ax'] * math.sin(obs['wx'] * self.__t + obs['ph'])
            y = obs['cy'] + obs['ay'] * math.sin(obs['wy'] * self.__t + obs['ph'] + 0.5)
            z = obs['cz'] + obs['az'] * math.sin(obs['wz'] * self.__t + obs['ph'] + 1.0)
            z = max(0.5, z)

            # Move in Webots (Webots coord: Y=up, Z=-ROS_Y)
            if self.__trans_fields[i]:
                self.__trans_fields[i].setSFVec3f([x, z, -y])

            p = Pose()
            p.position.x  = x
            p.position.y  = y
            p.position.z  = z
            p.orientation.w = obs['radius']  # pack radius here
            pa.poses.append(p)

        self.__pub.publish(pa)
