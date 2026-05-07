"""
RRT* - Asymptotically Optimal Rapidly-exploring Random Trees
Karaman & Frazzoli, IJRR 2011.
"""
import numpy as np
import math


class Node:
    __slots__ = ['x', 'y', 'cost', 'parent', 'children']

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None
        self.children = []

    def dist(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)


class RRTStar:
    """
    RRT* with near-neighbor rewiring.
    Yields (iteration, best_cost, path) at each iteration so convergence
    can be tracked externally.
    """

    GAMMA = 5.0   # rewiring radius constant (tune for environment)
    DIM   = 2     # C-space dimension

    def __init__(self, env, start, goal, goal_radius=0.3, step_size=0.5):
        self.env = env
        self.start_node = Node(*start)
        self.goal = Node(*goal)
        self.goal_radius = goal_radius
        self.step_size = step_size

        self.nodes = [self.start_node]
        self.best_cost = float('inf')
        self.best_path = None
        self.goal_node = None     # node closest to goal in tree

    # ── Core helpers ─────────────────────────────────────────────────────────

    def _rewire_radius(self):
        n = max(len(self.nodes), 2)
        return min(self.GAMMA * (math.log(n) / n) ** (1.0 / self.DIM), 2.5)

    def _nearest(self, node):
        return min(self.nodes, key=lambda n: n.dist(node))

    def _near(self, node, radius):
        return [n for n in self.nodes if n.dist(node) <= radius]

    def _steer(self, from_node, to_node):
        d = from_node.dist(to_node)
        if d <= self.step_size:
            return Node(to_node.x, to_node.y)
        ratio = self.step_size / d
        x = from_node.x + ratio * (to_node.x - from_node.x)
        y = from_node.y + ratio * (to_node.y - from_node.y)
        return Node(x, y)

    def _collision_free(self, n1, n2):
        return not self.env.in_collision_segment(n1.x, n1.y, n2.x, n2.y)

    def _attach(self, parent, child):
        child.parent = parent
        child.cost = parent.cost + parent.dist(child)
        parent.children.append(child)

    def _rewire(self, new_node, near_nodes):
        for near in near_nodes:
            candidate = new_node.cost + new_node.dist(near)
            if candidate < near.cost and self._collision_free(new_node, near):
                # Detach from old parent
                if near.parent:
                    near.parent.children.remove(near)
                # Reattach
                near.parent = new_node
                near.cost = candidate
                new_node.children.append(near)
                self._propagate_cost(near)

    def _propagate_cost(self, node):
        for child in node.children:
            child.cost = node.cost + node.dist(child)
            self._propagate_cost(child)

    def _extract_path(self, end_node):
        path, n = [], end_node
        while n:
            path.append((n.x, n.y))
            n = n.parent
        return path[::-1]

    def _update_goal(self, new_node):
        d = new_node.dist(self.goal)
        if d <= self.goal_radius:
            total = new_node.cost + d
            if total < self.best_cost and self._collision_free(new_node, self.goal):
                self.best_cost = total
                self.goal_node = new_node
                self.best_path = self._extract_path(new_node) + [(self.goal.x, self.goal.y)]

    # ── Main planning loop ────────────────────────────────────────────────────

    def plan(self, max_iter=2000):
        for i in range(max_iter):
            # Sample (bias toward goal 5% of the time)
            if np.random.rand() < 0.05:
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

            # Choose best parent from near neighbors
            best_parent = nearest
            best_cost = nearest.cost + nearest.dist(new_node)
            for near in near_nodes:
                c = near.cost + near.dist(new_node)
                if c < best_cost and self._collision_free(near, new_node):
                    best_parent = near
                    best_cost = c

            self._attach(best_parent, new_node)
            self.nodes.append(new_node)
            self._rewire(new_node, near_nodes)
            self._update_goal(new_node)

            yield i, self.best_cost, self.best_path
