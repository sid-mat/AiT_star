import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math


class PathFollower(Node):

    LOOKAHEAD   = 0.4   # metres — short so robot stays on path
    MAX_LINEAR  = 0.15  # m/s — slow and steady
    MAX_ANGULAR = 1.0   # rad/s
    WP_SPACING  = 0.15  # interpolation density (metres between points)

    def __init__(self):
        super().__init__('path_follower')

        self.path_sub = self.create_subscription(
            Path, '/ait_star/path', self.path_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.path      = []
        self.x = self.y = self.yaw = 0.0
        self.offset_x  = None
        self.offset_y  = None
        self.goal_done = False

        # Stuck detection
        self.stuck_count = 0
        self.last_x = self.last_y = 0.0
        self.recovering  = False
        self.recover_cnt = 0
        self.recover_dir = 1   # alternates so it doesn't always turn same way

        self.create_timer(0.1, self.loop)
        self.get_logger().info('Path Follower (dense interpolation) ready')

    # ── Callbacks ────────────────────────────────────────────────

    def path_cb(self, msg):
        raw = [(p.pose.position.x, p.pose.position.y)
               for p in msg.poses]

        # Interpolate raw waypoints into a dense path
        self.path = self._interpolate(raw, self.WP_SPACING)
        self.goal_done = False

        self.get_logger().info(
            f'Path: {len(raw)} waypoints → '
            f'{len(self.path)} dense points ({self.WP_SPACING}m spacing)')

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

    # ── Control loop ─────────────────────────────────────────────

    def loop(self):
        if self.offset_x is None or not self.path or self.goal_done:
            return

        # Goal check
        gx, gy = self.path[-1]
        if math.hypot(gx - self.x, gy - self.y) < 0.35:
            self.cmd_pub.publish(Twist())
            self.goal_done = True
            self.get_logger().info('Goal reached!')
            return

        # Recovery
        if self.recovering:
            cmd = Twist()
            cmd.linear.x  = -0.1
            cmd.angular.z =  self.recover_dir * 0.8
            self.cmd_pub.publish(cmd)
            self.recover_cnt += 1
            if self.recover_cnt > 25:   # ~2.5 s
                self.recovering  = False
                self.recover_cnt = 0
                self.recover_dir *= -1  # alternate direction next time
                self.get_logger().info('Recovery done, resuming')
            return

        # Stuck detection (every 3 s)
        self.stuck_count += 1
        if self.stuck_count % 30 == 0:
            moved = math.hypot(self.x - self.last_x,
                               self.y - self.last_y)
            self.last_x, self.last_y = self.x, self.y
            if moved < 0.04:
                self.get_logger().warn('Stuck! Recovering...')
                self.recovering  = True
                self.recover_cnt = 0
                return

        # Find lookahead point
        target = self._lookahead_point()
        if target is None:
            return

        tx, ty  = target
        dx, dy  = tx - self.x, ty - self.y
        dist    = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        err     = heading - self.yaw

        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi

        cmd = Twist()
        if abs(err) > 0.45:
            # Rotate in place toward target
            cmd.angular.z = max(-self.MAX_ANGULAR,
                                 min(self.MAX_ANGULAR, 1.8 * err))
            cmd.linear.x  = 0.0
        else:
            # Drive forward with gentle steering
            cmd.linear.x  = min(self.MAX_LINEAR, 0.4 * dist)
            cmd.angular.z = max(-self.MAX_ANGULAR,
                                 min(self.MAX_ANGULAR, 1.2 * err))

        self.cmd_pub.publish(cmd)

    # ── Helpers ──────────────────────────────────────────────────

    def _interpolate(self, waypoints, spacing):
        """
        Insert extra points between each pair of waypoints so that
        no two consecutive points are more than `spacing` metres apart.
        This keeps the lookahead target close to the robot at all times.
        """
        if len(waypoints) < 2:
            return waypoints
        dense = [waypoints[0]]
        for i in range(len(waypoints) - 1):
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i + 1]
            seg_len = math.hypot(x2 - x1, y2 - y1)
            n_steps = max(1, int(math.ceil(seg_len / spacing)))
            for j in range(1, n_steps + 1):
                t = j / n_steps
                dense.append((x1 + t * (x2 - x1),
                               y1 + t * (y2 - y1)))
        return dense

    def _lookahead_point(self):
        """
        Find the closest point on the dense path, then walk
        forward until we find one that is LOOKAHEAD metres away.
        """
        # Closest point index
        best_i, best_d = 0, float('inf')
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - self.x, py - self.y)
            if d < best_d:
                best_d, best_i = d, i

        # Walk forward from closest
        for i in range(best_i, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - self.x, py - self.y) >= self.LOOKAHEAD:
                return (px, py)

        return self.path[-1]   # fallback: final goal


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathFollower())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
