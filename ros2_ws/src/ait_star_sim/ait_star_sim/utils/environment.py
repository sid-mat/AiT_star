"""
2D Planning Environment
Circular obstacles, collision checking, visualization helpers.
"""
import numpy as np


class CircleObstacle:
    def __init__(self, cx, cy, radius):
        self.cx = cx
        self.cy = cy
        self.radius = radius

    def contains(self, x, y):
        return (x - self.cx) ** 2 + (y - self.cy) ** 2 <= self.radius ** 2

    def segment_collides(self, x1, y1, x2, y2, n_checks=20):
        for i in range(n_checks + 1):
            t = i / n_checks
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if self.contains(x, y):
                return True
        return False


class Environment:
    """
    Rectangular 2D workspace with circular obstacles.
    bounds: (x_min, x_max, y_min, y_max)
    """
    def __init__(self, bounds, obstacles):
        self.bounds = bounds  # (x_min, x_max, y_min, y_max)
        self.obstacles = obstacles

    def in_bounds(self, x, y):
        return (self.bounds[0] <= x <= self.bounds[1] and
                self.bounds[2] <= y <= self.bounds[3])

    def in_collision_point(self, x, y):
        return any(o.contains(x, y) for o in self.obstacles)

    def in_collision_segment(self, x1, y1, x2, y2):
        return any(o.segment_collides(x1, y1, x2, y2) for o in self.obstacles)

    def sample_free(self):
        """Uniform random sample in free space."""
        while True:
            x = np.random.uniform(self.bounds[0], self.bounds[1])
            y = np.random.uniform(self.bounds[2], self.bounds[3])
            if not self.in_collision_point(x, y):
                return x, y

    # ── Standard benchmark maps ──────────────────────────────────────────────

    @staticmethod
    def cluttered_room():
        bounds = (0, 10, 0, 10)
        obs = [
            CircleObstacle(2.5, 2.5, 0.8),
            CircleObstacle(5.0, 1.5, 0.7),
            CircleObstacle(7.5, 2.5, 0.8),
            CircleObstacle(2.0, 5.0, 0.9),
            CircleObstacle(5.0, 5.0, 1.0),
            CircleObstacle(8.0, 5.0, 0.9),
            CircleObstacle(2.5, 7.5, 0.8),
            CircleObstacle(5.0, 8.5, 0.7),
            CircleObstacle(7.5, 7.5, 0.8),
            CircleObstacle(4.0, 3.5, 0.6),
            CircleObstacle(6.0, 6.5, 0.6),
            CircleObstacle(3.5, 8.0, 0.5),
            CircleObstacle(7.0, 3.5, 0.5),
        ]
        return Environment(bounds, obs)

    @staticmethod
    def narrow_corridor():
        bounds = (0, 10, 0, 10)
        obs = [
            CircleObstacle(3.0, 2.0, 0.8),
            CircleObstacle(3.0, 4.0, 0.8),
            CircleObstacle(3.0, 6.0, 0.8),
            CircleObstacle(3.0, 8.5, 0.8),
            CircleObstacle(7.0, 1.5, 0.8),
            CircleObstacle(7.0, 3.5, 0.8),
            CircleObstacle(7.0, 5.5, 0.8),
            CircleObstacle(7.0, 7.5, 0.8),
            CircleObstacle(5.0, 0.8, 0.6),
            CircleObstacle(5.0, 9.2, 0.6),
        ]
        return Environment(bounds, obs)
