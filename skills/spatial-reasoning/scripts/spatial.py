# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Helpers extracted from spatial-reasoning/SKILL.md.

Each section header below corresponds to where this block was originally
embedded. Import individual functions as needed.
"""

# ======================================================================
# Block 1
# ======================================================================


def bake_waypoints(xform_op, waypoints, speed_mps, fps=30, mpu=1.0, max_frames=3600):
    """Bake linear waypoint interpolation as timeSamples."""
    dists = [
        math.sqrt((waypoints[i + 1][0] - waypoints[i][0]) ** 2 + (waypoints[i + 1][1] - waypoints[i][1]) ** 2)
        for i in range(len(waypoints) - 1)
    ]
    total = sum(dists)
    if total == 0:
        return
    loop_time = total / speed_mps
    loop_frames = int(loop_time * fps)

    for frame in range(0, min(loop_frames * 3, max_frames), 2):
        t = (frame / fps) % loop_time
        elapsed = t * speed_mps
        acc, si = 0, 0
        for idx, sd in enumerate(dists):
            if acc + sd >= elapsed:
                si = idx
                break
            acc += sd
        else:
            si = len(dists) - 1
        lt = (elapsed - acc) / dists[si] if dists[si] > 0 else 0
        x = waypoints[si][0] + (waypoints[si + 1][0] - waypoints[si][0]) * lt
        y = waypoints[si][1] + (waypoints[si + 1][1] - waypoints[si][1]) * lt
        heading = math.degrees(
            math.atan2(waypoints[si + 1][1] - waypoints[si][1], waypoints[si + 1][0] - waypoints[si][0])
        )
        S = Gf.Matrix4d().SetScale(Gf.Vec3d(mpu, mpu, mpu))
        R = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), heading))
        T = Gf.Matrix4d().SetTranslate(Gf.Vec3d(x, y, 0))
        xform_op.Set(T * R * S, frame)


# ======================================================================
# Block 2
# ======================================================================

import math


def euler_to_quat(rx, ry, rz):
    """Convert Euler XYZ (degrees) to quaternion (w, x, y, z)."""
    rx, ry, rz = math.radians(rx), math.radians(ry), math.radians(rz)
    cx, sx = math.cos(rx / 2), math.sin(rx / 2)
    cy, sy = math.cos(ry / 2), math.sin(ry / 2)
    cz, sz = math.cos(rz / 2), math.sin(rz / 2)
    return (
        cx * cy * cz + sx * sy * sz,  # w
        sx * cy * cz - cx * sy * sz,  # x
        cx * sy * cz + sx * cy * sz,  # y
        cx * cy * sz - sx * sy * cz,  # z
    )


def quat_to_matrix(q):
    """Convert quaternion to 3×3 rotation matrix."""
    w, x, y, z = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def slerp(q0, q1, t):
    """Spherical linear interpolation between two quaternions."""
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0:  # Take shorter path
        q1 = tuple(-c for c in q1)
        dot = -dot
    dot = min(dot, 1.0)
    theta = math.acos(dot)
    if theta < 1e-6:  # Nearly identical — linear interpolation
        return tuple(a + t * (b - a) for a, b in zip(q0, q1))
    sin_t = math.sin(theta)
    w0 = math.sin((1 - t) * theta) / sin_t
    w1 = math.sin(t * theta) / sin_t
    return tuple(w0 * a + w1 * b for a, b in zip(q0, q1))


# ======================================================================
# Block 3
# ======================================================================

import numpy as np


def decompose_transform(matrix_4x4):
    """Decompose 4x4 transform into translation, rotation, scale."""
    M = np.array(matrix_4x4)
    translation = M[3, :3]  # USD convention: translation in row 3

    # Extract 3x3 upper-left
    M3 = M[:3, :3].copy()

    # Scale = column magnitudes
    sx = np.linalg.norm(M3[:, 0])
    sy = np.linalg.norm(M3[:, 1])
    sz = np.linalg.norm(M3[:, 2])
    scale = np.array([sx, sy, sz])

    # Rotation = normalized columns
    R = M3.copy()
    R[:, 0] /= sx
    R[:, 1] /= sy
    R[:, 2] /= sz

    # Fix reflection if det < 0
    if np.linalg.det(R) < 0:
        R[:, 0] *= -1
        scale[0] *= -1

    return translation, R, scale


# ======================================================================
# Block 4
# ======================================================================


class RTreeNode:
    """Simplified 2D R-tree for warehouse spatial queries."""

    MAX_ENTRIES = 8

    def __init__(self):
        self.entries = []  # [(bbox, data)] for leaves, [(bbox, child)] for internal
        self.is_leaf = True

    def insert(self, bbox, data):
        if self.is_leaf:
            self.entries.append((bbox, data))
            if len(self.entries) > self.MAX_ENTRIES:
                return self._split()
        else:
            # Choose child with least enlargement
            best = min(self.entries, key=lambda e: self._enlargement(e[0], bbox))
            overflow = best[1].insert(bbox, data)
            if overflow:
                self.entries.remove(best)
                self.entries.extend(overflow)
                if len(self.entries) > self.MAX_ENTRIES:
                    return self._split()
        return None

    def query_range(self, query_bbox, results=None):
        """Find all entries overlapping query_bbox."""
        if results is None:
            results = []
        for bbox, item in self.entries:
            if self._overlaps(bbox, query_bbox):
                if self.is_leaf:
                    results.append((bbox, item))
                else:
                    item.query_range(query_bbox, results)
        return results

    @staticmethod
    def _overlaps(a, b):
        """Check if two bboxes (x_min, y_min, x_max, y_max) overlap."""
        return a[0] <= b[2] and a[2] >= b[0] and a[1] <= b[3] and a[3] >= b[1]

    @staticmethod
    def _enlargement(bbox, new_bbox):
        merged = (
            min(bbox[0], new_bbox[0]),
            min(bbox[1], new_bbox[1]),
            max(bbox[2], new_bbox[2]),
            max(bbox[3], new_bbox[3]),
        )
        orig_area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
        new_area = (merged[2] - merged[0]) * (merged[3] - merged[1])
        return new_area - orig_area


# ======================================================================
# Block 5
# ======================================================================


class SpatialGrid:
    """Fast 2D grid for warehouse collision detection."""

    def __init__(self, width, height, cell_size=2.0):
        self.cell_size = cell_size
        self.cols = int(math.ceil(width / cell_size))
        self.rows = int(math.ceil(height / cell_size))
        self.cells = {}  # (col, row) → [objects]

    def insert(self, x, y, w, d, obj):
        """Insert object with position (x,y) and size (w,d)."""
        c0 = int(x / self.cell_size)
        r0 = int(y / self.cell_size)
        c1 = int((x + w) / self.cell_size)
        r1 = int((y + d) / self.cell_size)
        for c in range(c0, c1 + 1):
            for r in range(r0, r1 + 1):
                self.cells.setdefault((c, r), []).append(obj)

    def query(self, x, y, w, d):
        """Find all objects overlapping query rectangle."""
        results = set()
        c0 = int(x / self.cell_size)
        r0 = int(y / self.cell_size)
        c1 = int((x + w) / self.cell_size)
        r1 = int((y + d) / self.cell_size)
        for c in range(c0, c1 + 1):
            for r in range(r0, r1 + 1):
                for obj in self.cells.get((c, r), []):
                    results.add(id(obj))
        return results

    def check_placement(self, x, y, w, d, buffer=0.5):
        """Check if a new placement would collide with existing objects."""
        return len(self.query(x - buffer, y - buffer, w + 2 * buffer, d + 2 * buffer)) > 0


# ======================================================================
# Block 6
# ======================================================================


def gjk_overlap_2d(shape_a, shape_b):
    """GJK collision test for 2D convex polygons (vertex lists)."""

    def support(shape, direction):
        return max(shape, key=lambda v: v[0] * direction[0] + v[1] * direction[1])

    def minkowski_support(d):
        sa = support(shape_a, d)
        sb = support(shape_b, (-d[0], -d[1]))
        return (sa[0] - sb[0], sa[1] - sb[1])

    d = (1, 0)  # Initial search direction
    simplex = [minkowski_support(d)]
    d = (-simplex[0][0], -simplex[0][1])  # Toward origin

    for _ in range(20):  # Max iterations
        a = minkowski_support(d)
        if a[0] * d[0] + a[1] * d[1] < 0:
            return False  # No collision
        simplex.append(a)

        if len(simplex) == 3:
            # Check if origin is inside triangle
            # (simplified — full impl needs edge-case handling)
            ao = (-a[0], -a[1])
            ab = (simplex[0][0] - a[0], simplex[0][1] - a[1])
            ac = (simplex[1][0] - a[0], simplex[1][1] - a[1])
            ab_perp = (ab[1], -ab[0])
            if ab_perp[0] * ao[0] + ab_perp[1] * ao[1] > 0:
                simplex.remove(simplex[1])
                d = ab_perp
            else:
                ac_perp = (-ac[1], ac[0])
                if ac_perp[0] * ao[0] + ac_perp[1] * ao[1] > 0:
                    simplex.remove(simplex[0])
                    d = ac_perp
                else:
                    return True  # Origin inside simplex — collision
        else:
            ao = (-a[0], -a[1])
            ab = (simplex[0][0] - a[0], simplex[0][1] - a[1])
            d = (
                ab[1] * (-1 if ab[0] * ao[1] - ab[1] * ao[0] < 0 else 1),
                ab[0] * (1 if ab[0] * ao[1] - ab[1] * ao[0] < 0 else -1),
            )
    return False


# ======================================================================
# Block 7
# ======================================================================

import heapq


def astar_warehouse(grid, start, goal, cell_size=0.1):
    """A* pathfinding on occupancy grid. Returns list of (x,y) waypoints."""
    rows, cols = len(grid), len(grid[0])
    sr, sc = int(start[1] / cell_size), int(start[0] / cell_size)
    gr, gc = int(goal[1] / cell_size), int(goal[0] / cell_size)

    # Heuristic: octile distance (allows diagonal movement)
    SQRT2 = 1.414

    def h(r, c):
        dr, dc = abs(r - gr), abs(c - gc)
        return max(dr, dc) + (SQRT2 - 1) * min(dr, dc)

    open_set = [(h(sr, sc), 0, sr, sc)]
    came_from = {}
    g_score = {(sr, sc): 0}

    # 8-connected neighbors
    DIRS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    COSTS = [SQRT2, 1, SQRT2, 1, 1, SQRT2, 1, SQRT2]

    while open_set:
        f, _, r, c = heapq.heappop(open_set)
        if (r, c) == (gr, gc):
            # Reconstruct path
            path = []
            while (r, c) in came_from:
                path.append((c * cell_size, r * cell_size))
                r, c = came_from[(r, c)]
            path.append((sc * cell_size, sr * cell_size))
            return path[::-1]

        for (dr, dc), cost in zip(DIRS, COSTS):
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                # Check diagonal doesn't cut through walls
                if abs(dr) + abs(dc) == 2:
                    if grid[r + dr][c] == 1 or grid[r][c + dc] == 1:
                        continue

                new_g = g_score[(r, c)] + cost
                if new_g < g_score.get((nr, nc), float("inf")):
                    g_score[(nr, nc)] = new_g
                    came_from[(nr, nc)] = (r, c)
                    heapq.heappush(open_set, (new_g + h(nr, nc), new_g, nr, nc))

    return None  # No path found


# ======================================================================
# Block 8
# ======================================================================


def catmull_rom_point(p0, p1, p2, p3, t, alpha=0.5):
    """Centripetal Catmull-Rom spline point at parameter t ∈ [0,1]."""

    def tj(ti, pi, pj):
        dx, dy = pj[0] - pi[0], pj[1] - pi[1]
        return ti + (dx * dx + dy * dy) ** (alpha / 2)

    t0 = 0
    t1 = tj(t0, p0, p1)
    t2 = tj(t1, p1, p2)
    t3 = tj(t2, p2, p3)
    t = t1 + t * (t2 - t1)

    def lerp(a, b, ta, tb, tc):
        f = (tc - ta) / (tb - ta) if abs(tb - ta) > 1e-10 else 0
        return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)

    a1 = lerp(p0, p1, t0, t1, t)
    a2 = lerp(p1, p2, t1, t2, t)
    a3 = lerp(p2, p3, t2, t3, t)
    b1 = lerp(a1, a2, t0, t2, t)
    b2 = lerp(a2, a3, t1, t3, t)
    return lerp(b1, b2, t1, t2, t)


def smooth_path(waypoints, points_per_segment=10):
    """Smooth a waypoint list with Catmull-Rom splines."""
    if len(waypoints) < 4:
        return waypoints
    smoothed = []
    # Pad endpoints
    pts = [waypoints[0]] + waypoints + [waypoints[-1]]
    for i in range(1, len(pts) - 2):
        for j in range(points_per_segment):
            t = j / points_per_segment
            smoothed.append(catmull_rom_point(pts[i - 1], pts[i], pts[i + 1], pts[i + 2], t))
    smoothed.append(waypoints[-1])
    return smoothed


# ======================================================================
# Block 9
# ======================================================================


class MaxRectsBinPack:
    """2D bin packing using maximal rectangles heuristic.
    Best-Short-Side-Fit (BSSF) — O(n²) but excellent packing density."""

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.free_rects = [(0, 0, width, height)]  # (x, y, w, h)
        self.placed = []

    def insert(self, w, h):
        """Place a rectangle of size (w,h). Returns (x,y) or None."""
        best_x, best_y = None, None
        best_short_side = float("inf")
        best_idx = -1

        for i, (fx, fy, fw, fh) in enumerate(self.free_rects):
            # Try normal orientation
            if w <= fw and h <= fh:
                leftover_h = fh - h
                leftover_w = fw - w
                short_side = min(leftover_h, leftover_w)
                if short_side < best_short_side:
                    best_short_side = short_side
                    best_x, best_y = fx, fy
                    best_idx = i
            # Try rotated (90°)
            if h <= fw and w <= fh:
                leftover_h = fh - w
                leftover_w = fw - h
                short_side = min(leftover_h, leftover_w)
                if short_side < best_short_side:
                    best_short_side = short_side
                    best_x, best_y = fx, fy
                    best_idx = i
                    w, h = h, w  # Use rotated

        if best_x is None:
            return None

        # Split free rectangle
        self._split_free_rect(best_idx, best_x, best_y, w, h)
        self.placed.append((best_x, best_y, w, h))
        return (best_x, best_y)

    def _split_free_rect(self, idx, px, py, pw, ph):
        """Split the free rect at idx after placing (px,py,pw,ph)."""
        fx, fy, fw, fh = self.free_rects.pop(idx)
        # Right remainder
        if px + pw < fx + fw:
            self.free_rects.append((px + pw, fy, fx + fw - px - pw, fh))
        # Top remainder
        if py + ph < fy + fh:
            self.free_rects.append((fx, py + ph, fw, fy + fh - py - ph))
        # Remove any overlapping free rects and merge
        self._prune_free_rects((px, py, pw, ph))

    def _prune_free_rects(self, placed):
        px, py, pw, ph = placed
        new_free = []
        for fx, fy, fw, fh in self.free_rects:
            if not (fx < px + pw and fx + fw > px and fy < py + ph and fy + fh > py):
                new_free.append((fx, fy, fw, fh))
            else:
                # Clip to non-overlapping regions
                if fx < px:
                    new_free.append((fx, fy, px - fx, fh))
                if fx + fw > px + pw:
                    new_free.append((px + pw, fy, fx + fw - px - pw, fh))
                if fy < py:
                    new_free.append((fx, fy, fw, py - fy))
                if fy + fh > py + ph:
                    new_free.append((fx, py + ph, fw, fy + fh - py - ph))
        self.free_rects = new_free

    def utilization(self):
        placed_area = sum(w * h for _, _, w, h in self.placed)
        return placed_area / (self.width * self.height)


# ======================================================================
# Block 10
# ======================================================================


def frustum_planes(eye, forward, up, fov_h, fov_v, near, far):
    """Compute 6 frustum planes for view culling. Returns [(normal, d), ...]"""
    right = cross(forward, up)
    planes = []

    # Near and far
    planes.append((forward, -dot(forward, eye) - near))
    planes.append((neg(forward), dot(forward, eye) + far))

    # Left and right
    half_h = math.tan(math.radians(fov_h / 2))
    left_normal = normalize(add(forward, scale(right, half_h)))
    right_normal = normalize(sub(forward, scale(right, half_h)))
    planes.append((left_normal, -dot(left_normal, eye)))
    planes.append((right_normal, -dot(right_normal, eye)))

    # Top and bottom
    half_v = math.tan(math.radians(fov_v / 2))
    top_normal = normalize(add(forward, scale(up, half_v)))
    bot_normal = normalize(sub(forward, scale(up, half_v)))
    planes.append((top_normal, -dot(top_normal, eye)))
    planes.append((bot_normal, -dot(bot_normal, eye)))

    return planes


def bbox_in_frustum(bbox_min, bbox_max, planes):
    """Test if AABB is potentially visible (conservative)."""
    for normal, d in planes:
        # P-vertex: bbox corner most along normal direction
        px = bbox_max[0] if normal[0] >= 0 else bbox_min[0]
        py = bbox_max[1] if normal[1] >= 0 else bbox_min[1]
        pz = bbox_max[2] if normal[2] >= 0 else bbox_min[2]
        if normal[0] * px + normal[1] * py + normal[2] * pz + d < 0:
            return False  # Entirely outside this plane
    return True


# ======================================================================
# Block 11
# ======================================================================

import math


def compute_camera_distance(obj_height, obj_width, focal_mm=40.0, aperture_mm=36.0, margin=1.3):
    """Compute minimum camera distance to capture full bounding box.

    Args:
        obj_height: object height in meters
        obj_width: object width in meters
        focal_mm: focal length in mm
        aperture_mm: horizontal aperture in mm (36mm = full frame)
        margin: multiplier for safety margin (1.3 = 30% extra)

    Returns:
        distance in meters
    """
    # Vertical FOV (assuming 16:9 → sensor_h = aperture * 9/16)
    sensor_h = aperture_mm * 9.0 / 16.0  # 20.25mm for 36mm aperture
    fov_v = 2 * math.atan(sensor_h / (2 * focal_mm))

    # Horizontal FOV
    fov_h = 2 * math.atan(aperture_mm / (2 * focal_mm))

    # Distance to fit vertically and horizontally
    dist_v = (obj_height / 2) / math.tan(fov_v / 2)
    dist_h = (obj_width / 2) / math.tan(fov_h / 2)

    return max(dist_v, dist_h) * margin


# Example: stacked pallet (1.424m tall, 1.2m wide)
# compute_camera_distance(1.424, 1.2, focal_mm=40) → ~3.1m
# For 3/4 view: eye at (dist*0.7, -dist*0.7, obj_height*0.5)
