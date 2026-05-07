import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math


class PathFollower(Node):
    """
    Pure Pursuit path follower.
    Instead of targeting one waypoint at a time, it looks ahead
    a fixed distance (lookahead) on the path and steers toward
    that point. This eliminates oscillation completely.
    """
    LOOKAHEAD  = 0.8   # metres ahead to target — increase if robot wobbles
    MAX_LINEAR = 0.22  # m/s  (TurtleBot3 Burger max is 0.22)
    MAX_ANGULAR= 1.2   # rad/s

    def __init__(self):
        super().__init__('path_follower')

        self.path_sub = self.create_subscription(
            Path, '/ait_star/path', self.path_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.path      = []    # list of (x, y)
        self.x = self.y = self.yaw = 0.0
        self.offset_x  = None
        self.offset_y  = None
        self.goal_done = False

        # Stuck detection
        self.stuck_count = 0
        self.last_x = self.last_y = 0.0
        self.recovering  = False
        self.recover_cnt = 0

        self.create_timer(0.1, self.loop)
        self.get_logger().info('Pure Pursuit follower ready')

    # ── Callbacks ─────────────────────────────────────────────────

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y)
                     for p in msg.poses]
        self.goal_done = False
        self.get_logger().info(
            f'Path received: {len(self.path)} waypoints. Moving!')

    def odom_cb(self, msg):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        if self.offset_x is None:
            self.offset_x = rx - 0.5
            self.offset_y = ry - 0.5
            self.get_logger().info(
                f'Offset locked: ({self.offset_x:.3f}, {self.offset_y:.3f})')
        self.x = rx - self.offset_x
        self.y = ry - self.offset_y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z))

    # ── Main control loop ──────────────────────────────────────────

    def loop(self):
        if self.offset_x is None or not self.path or self.goal_done:
            return

        # Check if we reached the final goal
        gx, gy = self.path[-1]
        if math.hypot(gx - self.x, gy - self.y) < 0.35:
            self.cmd_pub.publish(Twist())
            self.goal_done = True
            self.get_logger().info('Goal reached! Stopping.')
            return

        # Recovery behaviour
        if self.recovering:
            cmd = Twist()
            cmd.linear.x  = -0.1
            cmd.angular.z =  0.6
            self.cmd_pub.publish(cmd)
            self.recover_cnt += 1
            if self.recover_cnt > 20:
                self.recovering  = False
                self.recover_cnt = 0
                self.get_logger().info('Recovery done, resuming.')
            return

        # Stuck detection (check every 3 s)
        self.stuck_count += 1
        if self.stuck_count % 30 == 0:
            moved = math.hypot(self.x - self.last_x, self.y - self.last_y)
            self.last_x, self.last_y = self.x, self.y
            if moved < 0.04:
                self.get_logger().warn('Stuck! Triggering recovery.')
                self.recovering  = True
                self.recover_cnt = 0
                return

        # Find lookahead point on path
        target = self._lookahead_point()
        if target is None:
            return

        tx, ty   = target
        dx, dy   = tx - self.x, ty - self.y
        dist     = math.hypot(dx, dy)
        heading  = math.atan2(dy, dx)
        err      = heading - self.yaw

        # Normalise angle to [-pi, pi]
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi

        cmd = Twist()
        if abs(err) > 0.5:
            # Turn toward target before moving forward
            cmd.angular.z = max(-self.MAX_ANGULAR,
                                 min(self.MAX_ANGULAR, 1.5 * err))
            cmd.linear.x  = 0.05   # creep forward slightly while turning
        else:
            # Heading is good — drive and steer gently
            cmd.linear.x  = min(self.MAX_LINEAR, 0.4 * dist)
            cmd.angular.z = max(-self.MAX_ANGULAR,
                                 min(self.MAX_ANGULAR, 1.2 * err))

        self.cmd_pub.publish(cmd)

    # ── Pure pursuit: find the point LOOKAHEAD metres ahead ───────

    def _lookahead_point(self):
        """
        Walk along the path from the closest point forward until
        we find a point that is LOOKAHEAD metres from the robot.
        Returns that point as (x, y).
        """
        # Find index of closest waypoint
        closest_i = 0
        closest_d = float('inf')
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - self.x, py - self.y)
            if d < closest_d:
                closest_d = d
                closest_i = i

        # Walk forward from closest point to find lookahead target
        for i in range(closest_i, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - self.x, py - self.y) >= self.LOOKAHEAD:
                return (px, py)

        # If no point is far enough, just use the last waypoint
        return self.path[-1]


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathFollower())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
