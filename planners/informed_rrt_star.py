"""
Informed RRT* - Optimal sampling-based path planning via direct sampling
of an admissible ellipsoidal heuristic.
Gammell, Srinivasa, Barfoot. IROS 2014.
"""
import numpy as np
import math
from planners.rrt_star import RRTStar, Node


class InformedRRTStar(RRTStar):
    """
    Extension of RRT* that, once an initial solution is found, restricts
    new samples to a prolate hyperspheroid (ellipse in 2D) guaranteed to
    contain the optimal path.
    """

    def __init__(self, env, start, goal, **kwargs):
        super().__init__(env, start, goal, **kwargs)
        sx, sy = start
        gx, gy = goal
        self.c_min = math.hypot(gx - sx, gy - sy)
        # Rotation matrix to align ellipse with start-goal axis
        angle = math.atan2(gy - sy, gx - sx)
        self._cos_a = math.cos(angle)
        self._sin_a = math.sin(angle)
        self._cx = (sx + gx) / 2
        self._cy = (sy + gy) / 2

    def _sample_ellipse(self):
        """Sample uniformly from the informed ellipse (Gammell 2014, Alg 1)."""
        c_best = self.best_cost
        a = c_best / 2.0
        b = math.sqrt(max(c_best ** 2 - self.c_min ** 2, 1e-9)) / 2.0

        # Sample unit ball via rejection
        while True:
            r = math.sqrt(np.random.uniform(0, 1))
            theta = np.random.uniform(0, 2 * math.pi)
            u = r * math.cos(theta)
            v = r * math.sin(theta)

            # Scale to ellipse
            xe = a * u
            ye = b * v

            # Rotate + translate to world frame
            x = self._cx + xe * self._cos_a - ye * self._sin_a
            y = self._cy + xe * self._sin_a + ye * self._cos_a

            if self.env.in_bounds(x, y) and not self.env.in_collision_point(x, y):
                return Node(x, y)

    # ── Override plan to use ellipse sampling once solution exists ────────────

    def plan(self, max_iter=2000):
        for i in range(max_iter):
            # Informed sampling: ellipse if solution found, else uniform
            if self.best_cost < float('inf') and np.random.rand() > 0.05:
                sample = self._sample_ellipse()
            elif np.random.rand() < 0.05:
                sample = Node(self.goal.x, self.goal.y)
            else:
                x, y = self.env.sample_free()
                sample = Node(x, y)

            nearest = self._nearest(sample)
            new_node = self._steer(nearest, sample)

            if self.env.in_collision_point(new_node.x, new_node.y):
                continue
            if not self._collision_free(nearest, new_node):
                continue

            radius = self._rewire_radius()
            near_nodes = self._near(new_node, radius)

            best_parent, best_cost = nearest, nearest.cost + nearest.dist(new_node)
            for near in near_nodes:
                c = near.cost + near.dist(new_node)
                if c < best_cost and self._collision_free(near, new_node):
                    best_parent, best_cost = near, c

            self._attach(best_parent, new_node)
            self.nodes.append(new_node)
            self._rewire(new_node, near_nodes)
            self._update_goal(new_node)

            yield i, self.best_cost, self.best_path
