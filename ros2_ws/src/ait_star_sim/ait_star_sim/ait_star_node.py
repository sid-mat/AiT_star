import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import sys, os

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from ait_star_sim.planners.ait_star import AITStar
from ait_star_sim.utils.environment import Environment

# TurtleBot3 Burger body radius + safety margin
# Real robot radius = 0.105m, we add 0.1m buffer = 0.205m total
ROBOT_RADIUS = 0.25


class AITStarNode(Node):
    def __init__(self):
        super().__init__('ait_star_node')
        self.path_pub   = self.create_publisher(Path, '/ait_star/path', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/ait_star/tree_markers', 10)
        self.planned_path = None
        self.done = False
        self.create_timer(1.0, self.tick)
        self.get_logger().info('AIT* Node ready')

    def tick(self):
        if not self.done:
            self.done = True
            self.run_planner()
        if self.planned_path:
            self.path_pub.publish(self.planned_path)

    def run_planner(self):
        self.get_logger().info('Planning with inflated obstacles...')

        env = Environment.cluttered_room()

        # Inflate every obstacle by robot radius so the planner
        # keeps the path away from walls of cylinders
        for obs in env.obstacles:
            obs.radius += ROBOT_RADIUS

        self.get_logger().info(
            f'Obstacle radii inflated by {ROBOT_RADIUS}m for safety')

        planner = AITStar(env, (0.5, 0.5), (9.5, 9.5), batch_size=50)
        best_path = None

        for it, cost, path, tree, samples in planner.plan(max_iter=40):
            if path:
                best_path = path
            self.get_logger().info(f'Batch {it:02d}: cost={cost:.3f}')

        if not best_path:
            self.get_logger().warn('No path found! Try reducing ROBOT_RADIUS.')
            return

        self.get_logger().info(
            f'Safe path found! {len(best_path)} waypoints, cost={planner.best_cost:.3f}')

        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in best_path:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.planned_path = msg

        # Publish tree markers for visualisation
        array  = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.ns = 'ait_tree'; marker.id = 0
        marker.type   = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.6; marker.color.g = 0.2
        marker.color.b = 0.9; marker.color.a = 0.5
        for node in planner.T:
            if node.parent:
                marker.points.append(
                    Point(x=float(node.x), y=float(node.y), z=0.01))
                marker.points.append(
                    Point(x=float(node.parent.x),
                          y=float(node.parent.y), z=0.01))
        array.markers.append(marker)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AITStarNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
