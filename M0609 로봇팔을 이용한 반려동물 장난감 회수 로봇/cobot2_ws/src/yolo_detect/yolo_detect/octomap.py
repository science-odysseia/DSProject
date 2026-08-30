"""점유 옥트리(OctoMap) — depth 스트림을 8분할 재귀 점유격자로 유지한다."""
import numpy as np

FREE = 0
OCCUPIED = 1
MIXED = 2

LOG_ODDS_HIT = 0.85
LOG_ODDS_MISS = -0.4
LOG_ODDS_MIN = -2.0
LOG_ODDS_MAX = 3.5
LOG_ODDS_OCCUPIED_THRESHOLD = 0.85

DEFAULT_RESOLUTION_MM = 10.0
DEFAULT_DEPTH = 6
DEFAULT_MAX_RANGE_MM = 3000.0
DEFAULT_MIN_RANGE_MM = 450.0
RAY_STEP_RATIO = 1.0


class _Node:
    __slots__ = ("state", "children")

    def __init__(self, state=FREE):
        self.state = state
        self.children = None


class OctoMap:
    """고정 해상도 점유 옥트리."""

    def __init__(self, origin_mm, resolution_mm=DEFAULT_RESOLUTION_MM, depth=DEFAULT_DEPTH,
                 max_range_mm=DEFAULT_MAX_RANGE_MM, min_range_mm=DEFAULT_MIN_RANGE_MM):
        self.resolution = float(resolution_mm)
        self.depth = int(depth)
        self.size = self.resolution * (2 ** self.depth)
        self.origin = np.asarray(origin_mm, dtype=np.float64)
        self.max_range = float(max_range_mm)
        self.min_range = float(min_range_mm)
        self._log_odds = {}
        self._root = _Node(FREE)
        self._dirty = False

    @property
    def num_occupied(self):
        return sum(1 for v in self._log_odds.values() if v >= LOG_ODDS_OCCUPIED_THRESHOLD)

    @property
    def num_leaves(self):
        return len(self._log_odds)

    def bounds(self):
        return self.origin, self.origin + self.size

    def _keys(self, points_mm):
        pts = np.asarray(points_mm, dtype=np.float64).reshape(-1, 3)
        idx = np.floor((pts - self.origin) / self.resolution).astype(np.int64)
        n = 2 ** self.depth
        inside = np.all((idx >= 0) & (idx < n), axis=1)
        return idx[inside]

    def _apply(self, keys, delta):
        for key in map(tuple, keys):
            value = self._log_odds.get(key, 0.0) + delta
            self._log_odds[key] = float(np.clip(value, LOG_ODDS_MIN, LOG_ODDS_MAX))

    def insert_points(self, points_mm, sensor_origin_mm=None):
        """관측점을 점유로, 센서에서 그 점까지의 광선을 자유공간으로 갱신한다."""
        pts = np.asarray(points_mm, dtype=np.float64).reshape(-1, 3)
        if len(pts) == 0:
            return 0

        if sensor_origin_mm is not None:
            origin = np.asarray(sensor_origin_mm, dtype=np.float64)
            vec = pts - origin
            dist = np.linalg.norm(vec, axis=1)
            keep = (dist > self.min_range) & (dist < self.max_range)
            pts, vec, dist = pts[keep], vec[keep], dist[keep]
            if len(pts) == 0:
                return 0
            self._apply(self._free_keys(origin, pts, vec, dist), LOG_ODDS_MISS)

        hit_keys = self._keys(pts)
        self._apply(hit_keys, LOG_ODDS_HIT)
        self._dirty = True
        return len(hit_keys)

    def _free_keys(self, origin, points, vec, dist):
        """광선을 해상도 간격으로 훑어 자유공간 칸을 모은다."""
        step = self.resolution * RAY_STEP_RATIO
        n_steps = int(np.ceil(float(dist.max()) / step))
        if n_steps <= 1:
            return np.zeros((0, 3), dtype=np.int64)
        unit = vec / dist[:, None]
        ratios = (np.arange(1, n_steps) * step)[:, None] / dist[None, :]
        ratios = np.clip(ratios, 0.0, 1.0)
        keep = ratios < 1.0
        rows, cols = np.nonzero(keep)
        samples = origin + unit[cols] * (ratios[rows, cols] * dist[cols])[:, None]
        keys = self._keys(samples)
        return np.unique(keys, axis=0) if len(keys) else keys

    def clear(self):
        self._log_odds.clear()
        self._root = _Node(FREE)
        self._dirty = False

    def build(self):
        """점유 칸을 옥트리로 올리고 같은 상태의 8형제를 하나로 병합한다."""
        self._root = _Node(FREE)
        for key, value in self._log_odds.items():
            if value < LOG_ODDS_OCCUPIED_THRESHOLD:
                continue
            self._insert_key(key)
        self._prune(self._root)
        self._dirty = False
        return self._root

    def _insert_key(self, key):
        node = self._root
        for level in range(self.depth - 1, -1, -1):
            if node.children is None:
                node.children = [_Node(FREE) for _ in range(8)]
                node.state = MIXED
            node.state = MIXED
            bit = ((key[0] >> level) & 1) | (((key[1] >> level) & 1) << 1) \
                | (((key[2] >> level) & 1) << 2)
            node = node.children[bit]
        node.state = OCCUPIED
        node.children = None

    def _prune(self, node):
        if node.children is None:
            return node.state
        states = [self._prune(c) for c in node.children]
        if all(s == states[0] and s != MIXED for s in states):
            node.state = states[0]
            node.children = None
        else:
            node.state = MIXED
        return node.state

    def hits_capsule(self, p0, p1, radius_mm):
        """선분과 반지름으로 정의한 캡슐이 점유 칸과 겹치는지 옥트리 하강으로 판정한다."""
        if self._dirty:
            self.build()
        a = np.asarray(p0, dtype=np.float64)
        b = np.asarray(p1, dtype=np.float64)
        return self._descend(self._root, self.origin, self.size, a, b, float(radius_mm))

    def _descend(self, node, corner, size, a, b, radius):
        if node.state == FREE:
            return False
        if not _capsule_hits_box(a, b, radius, corner, corner + size):
            return False
        if node.children is None:
            return node.state == OCCUPIED
        half = size / 2.0
        for bit, child in enumerate(node.children):
            if child.state == FREE:
                continue
            offset = np.array([bit & 1, (bit >> 1) & 1, (bit >> 2) & 1], dtype=np.float64)
            if self._descend(child, corner + offset * half, half, a, b, radius):
                return True
        return False

    def occupied_centers(self):
        """점유 칸 중심 좌표(mm) 배열 — 시각화·검증용."""
        keys = [k for k, v in self._log_odds.items() if v >= LOG_ODDS_OCCUPIED_THRESHOLD]
        if not keys:
            return np.zeros((0, 3), dtype=np.float64)
        return self.origin + (np.asarray(keys, dtype=np.float64) + 0.5) * self.resolution


def _capsule_hits_box(a, b, radius, lo, hi):
    """선분 캡슐과 축정렬 상자의 교차 판정."""
    lo = lo - radius
    hi = hi + radius
    d = b - a
    t_lo, t_hi = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-9:
            if a[i] < lo[i] or a[i] > hi[i]:
                return False
            continue
        inv = 1.0 / d[i]
        t0 = (lo[i] - a[i]) * inv
        t1 = (hi[i] - a[i]) * inv
        if t0 > t1:
            t0, t1 = t1, t0
        t_lo = max(t_lo, t0)
        t_hi = min(t_hi, t1)
        if t_lo > t_hi:
            return False
    return True


def covering_octomap(corner1_mm, corner2_mm, corner3_mm, depth_mm,
                     resolution_mm=DEFAULT_RESOLUTION_MM, margin_mm=100.0):
    """박스 세 모서리와 깊이로부터 작업공간을 덮는 옥트리를 만든다."""
    p1 = np.asarray(corner1_mm, dtype=np.float64)
    p2 = np.asarray(corner2_mm, dtype=np.float64)
    p3 = np.asarray(corner3_mm, dtype=np.float64)
    ex = (p2 - p1) / np.linalg.norm(p2 - p1)
    ey = (p3 - p1) / np.linalg.norm(p3 - p1)
    ez = np.cross(ex, ey)
    ez = ez / np.linalg.norm(ez)
    corners = [p1 + ex * u + ey * v + ez * w
               for u in (0.0, float(np.linalg.norm(p2 - p1)))
               for v in (0.0, float(np.linalg.norm(p3 - p1)))
               for w in (0.0, float(depth_mm))]
    lo = np.min(corners, axis=0) - margin_mm
    hi = np.max(corners, axis=0) + margin_mm
    need = float(np.max(hi - lo))
    depth_levels = DEFAULT_DEPTH
    while resolution_mm * (2 ** depth_levels) < need:
        depth_levels += 1
    size = resolution_mm * (2 ** depth_levels)
    center = (lo + hi) / 2.0
    return OctoMap(center - size / 2.0, resolution_mm=resolution_mm, depth=depth_levels)
