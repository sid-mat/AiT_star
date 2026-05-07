"""
Adaptively Informed Trees (AIT*)
Strub & Gammell, ICRA 2020 / IJRR 2022.

Key ideas:
  1. Batch sampling with ellipsoidal pruning (informed set).
  2. Reverse Dijkstra from goal over sample graph -> adaptive heuristic h_hat.
  3. Forward search ordered by f = g + h_hat with lazy collision checking.
  4. Bidirectional: reverse tree continuously informs forward search.
"""

import heapq
import math
import numpy as np


class Node:
    _ctr = 0

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = float('inf')
        self.parent = None
        self.children = []
        Node._ctr += 1
        self._id = Node._ctr

    def dist(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)

    def __lt__(self, other):
        return self._id < other._id

    def __hash__(self):
        return self._id

    def __eq__(self, other):
        return isinstance(other, Node) and self._id == other._id


class AITStar:
    GAMMA  = 8.0   # RGG radius constant
    DIM    = 2
    R_MIN  = 1.5   # minimum connection radius

    def __init__(self, env, start, goal, batch_size=50, goal_radius=0.4):
        self.env         = env
        self.batch_size  = batch_size
        self.goal_radius = goal_radius

        self.start      = Node(*start)
        self.start.g    = 0.0
        self.goal_node  = Node(*goal)

        self.V      = [self.start, self.goal_node]
        self.T      = {self.start}

        self.best_cost = float('inf')
        self.best_path = None

        # Heuristic cache: node index -> h_hat
        self.h_hat = {}
        self._update_h_euclidean()

        # Informed-set geometry
        sx, sy = start
        gx, gy = goal
        self.c_min  = math.hypot(gx - sx, gy - sy)
        ang         = math.atan2(gy - sy, gx - sx)
        self._cos   = math.cos(ang)
        self._sin   = math.sin(ang)
        self._cx    = (sx + gx) / 2.0
        self._cy    = (sy + gy) / 2.0

        # Numpy cache
        self._coords = None   # (n, 2) array mirroring self.V
        self._dirty  = True

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _update_h_euclidean(self):
        gx, gy = self.goal_node.x, self.goal_node.y
        for v in self.V:
            self.h_hat[v] = math.hypot(v.x - gx, v.y - gy)

    def _rebuild_coords(self):
        self._coords = np.array([[v.x, v.y] for v in self.V], dtype=np.float64)
        self._dirty  = False

    def _rgg_radius(self):
        n = max(len(self.V), 2)
        r = self.GAMMA * (math.log(n) / n) ** (1.0 / self.DIM)
        return max(min(r, 4.0), self.R_MIN)

    def _neighbors_of(self, idx, r):
        """Return list of (neighbor_node, distance) within r of V[idx]."""
        if self._dirty:
            self._rebuild_coords()
        cx, cy = self._coords[idx]
        diff   = self._coords - np.array([cx, cy])
        dists  = np.hypot(diff[:, 0], diff[:, 1])
        mask   = (dists <= r) & (dists > 1e-9)
        idxs   = np.where(mask)[0]
        return [(self.V[i], float(dists[i])) for i in idxs]

    def _neighbors_of_xy(self, x, y, r):
        """Return list of (node, distance) within r of (x,y)."""
        if self._dirty:
            self._rebuild_coords()
        diff  = self._coords - np.array([x, y])
        dists = np.hypot(diff[:, 0], diff[:, 1])
        mask  = (dists <= r) & (dists > 1e-9)
        idxs  = np.where(mask)[0]
        return [(self.V[i], float(dists[i])) for i in idxs]

    def _idx(self, node):
        """Find index of node in V (by identity)."""
        for i, v in enumerate(self.V):
            if v is node:
                return i
        return -1

    def _propagate(self, node):
        for child in node.children:
            child.g = node.g + node.dist(child)
            self._propagate(child)

    def _extract_path(self):
        path, n = [], self.goal_node
        while n:
            path.append((n.x, n.y))
            n = n.parent
        return path[::-1]

    # ── Informed sampling ─────────────────────────────────────────────────────

    def _in_informed_set(self, x, y):
        if self.best_cost == float('inf'):
            return True
        d_s = math.hypot(x - self.start.x,     y - self.start.y)
        d_g = math.hypot(x - self.goal_node.x,  y - self.goal_node.y)
        return (d_s + d_g) < self.best_cost

    def _sample_batch(self):
        b0, b1, b2, b3 = self.env.bounds
        added, attempts = 0, 0
        while added < self.batch_size and attempts < self.batch_size * 30:
            attempts += 1
            x = np.random.uniform(b0, b1)
            y = np.random.uniform(b2, b3)
            if self._in_informed_set(x, y) and not self.env.in_collision_point(x, y):
                self.V.append(Node(x, y))
                added += 1
        self._dirty = True

    # ── Reverse heuristic (AIT* key step) ────────────────────────────────────

    def _compute_reverse_heuristic(self):
        """
        Dijkstra from goal over the implicit RGG (no collision check).
        h_hat(v) = graph distance to goal, always admissible because
        graph distances underestimate true path costs (no obstacles ignored).
        Tightens each batch as the graph becomes denser.
        """
        if self._dirty:
            self._rebuild_coords()
        r  = self._rgg_radius()
        n  = len(self.V)
        gi = self._idx(self.goal_node)

        dist_arr = np.full(n, np.inf)
        dist_arr[gi] = 0.0
        visited  = np.zeros(n, dtype=bool)
        pq       = [(0.0, gi)]

        while pq:
            d, ui = heapq.heappop(pq)
            if visited[ui]:
                continue
            visited[ui] = True
            for v, edge_d in self._neighbors_of(ui, r):
                vi = self._idx(v)
                nd = d + edge_d
                if nd < dist_arr[vi]:
                    dist_arr[vi] = nd
                    heapq.heappush(pq, (nd, vi))

        # Update h_hat: use graph estimate if finite, else Euclidean fallback
        gx, gy = self.goal_node.x, self.goal_node.y
        for i, v in enumerate(self.V):
            gh = dist_arr[i]
            eh = math.hypot(v.x - gx, v.y - gy)
            self.h_hat[v] = gh if gh < np.inf else eh

    # ── Forward search ────────────────────────────────────────────────────────

    def _forward_search(self):
        """
        Connect unvisited samples to tree ordered by f = g(parent)+edge+h_hat.
        Lazy collision checking: only performed when edge is chosen as best.
        """
        r     = self._rgg_radius()
        T_set = self.T
        gx, gy = self.goal_node.x, self.goal_node.y

        # Build edge queue: for each non-tree node, best parent candidate
        eq = []
        for child in self.V:
            if child in T_set:
                continue
            h = self.h_hat.get(child, math.hypot(child.x - gx, child.y - gy))
            for parent, d in self._neighbors_of_xy(child.x, child.y, r):
                if parent not in T_set:
                    continue
                f = parent.g + d + h
                if f < self.best_cost:
                    heapq.heappush(eq, (f, parent, child, d))

        added   = set()
        rewired = set()

        while eq:
            f, parent, child, edge_d = heapq.heappop(eq)
            if f >= self.best_cost:
                break

            if child in T_set:
                # Rewire check
                if id(child) in rewired:
                    continue
                new_g = parent.g + parent.dist(child)
                if new_g < child.g and not self.env.in_collision_segment(
                        parent.x, parent.y, child.x, child.y):
                    if child.parent:
                        try: child.parent.children.remove(child)
                        except ValueError: pass
                    child.parent = parent
                    child.g      = new_g
                    parent.children.append(child)
                    self._propagate(child)
                    rewired.add(id(child))
                continue

            if id(child) in added:
                continue

            # Lazy collision check
            if self.env.in_collision_segment(parent.x, parent.y, child.x, child.y):
                continue

            new_g = parent.g + edge_d
            if new_g >= self.best_cost:
                continue

            # Add child to tree
            if child.parent:
                try: child.parent.children.remove(child)
                except ValueError: pass
            child.parent = parent
            child.g      = new_g
            parent.children.append(child)
            T_set.add(child)
            added.add(id(child))
            self._dirty = True  # coords didn't change but T changed

            # Check if goal reachable from child
            d_goal = child.dist(self.goal_node)
            if d_goal <= r:
                total = child.g + d_goal
                if total < self.best_cost and not self.env.in_collision_segment(
                        child.x, child.y, gx, gy):
                    self.best_cost = total
                    if self.goal_node.parent:
                        try: self.goal_node.parent.children.remove(self.goal_node)
                        except ValueError: pass
                    self.goal_node.parent = child
                    self.goal_node.g      = total
                    child.children.append(self.goal_node)
                    T_set.add(self.goal_node)
                    self.best_path = self._extract_path()

    # ── Main planning loop ────────────────────────────────────────────────────

    def plan(self, max_iter=30):
        for i in range(max_iter):
            self._sample_batch()
            self._compute_reverse_heuristic()
            self._forward_search()
            yield (i,
                   self.best_cost,
                   self.best_path,
                   list(self.T),
                   list(self.V))
