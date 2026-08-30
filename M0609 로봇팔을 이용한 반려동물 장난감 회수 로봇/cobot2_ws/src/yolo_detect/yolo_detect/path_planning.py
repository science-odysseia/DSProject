"""자체 구현 경로계획 — 로봇 링크 충돌모델과 관절공간 RRT-Connect."""
import math
import random

import numpy as np

from yolo_detect import config, transforms

JOINT_ORDER = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

LINK_RADII_MM = (90.0, 80.0, 62.0, 58.0, 52.0, 50.0, 50.0)

DEFAULT_STEP_RAD = 0.20
DEFAULT_RESOLUTION_RAD = 0.05
DEFAULT_MAX_ITER = 4000
DEFAULT_GOAL_BIAS = 0.1
SHORTCUT_ROUNDS = 200
DENSIFY_STEP_RAD = 0.08


class Obb:
    """중심, 회전, 반치수로 정의한 방향성 직육면체 장애물."""

    def __init__(self, center_mm, rotation, half_extents_mm, name=""):
        self.center = np.asarray(center_mm, dtype=np.float64)
        self.rot = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
        self.half = np.asarray(half_extents_mm, dtype=np.float64)
        self.name = name

    def hits_capsule(self, p0, p1, radius_mm):
        """선분과 반지름으로 정의한 캡슐이 이 상자와 겹치는지 판정함."""
        a = self.rot.T @ (np.asarray(p0, dtype=np.float64) - self.center)
        b = self.rot.T @ (np.asarray(p1, dtype=np.float64) - self.center)
        h = self.half + float(radius_mm)
        d = b - a
        t_lo, t_hi = 0.0, 1.0
        for i in range(3):
            if abs(d[i]) < 1e-9:
                if a[i] < -h[i] or a[i] > h[i]:
                    return False
                continue
            inv = 1.0 / d[i]
            t0 = (-h[i] - a[i]) * inv
            t1 = (h[i] - a[i]) * inv
            if t0 > t1:
                t0, t1 = t1, t0
            t_lo = max(t_lo, t0)
            t_hi = min(t_hi, t1)
            if t_lo > t_hi:
                return False
        return True


class Sphere:
    """중심과 반지름으로 정의한 구 장애물."""

    def __init__(self, center_mm, radius_mm, name=""):
        self.center = np.asarray(center_mm, dtype=np.float64)
        self.radius = float(radius_mm)
        self.name = name

    def hits_capsule(self, p0, p1, radius_mm):
        """선분과 반지름으로 정의한 캡슐이 이 구와 겹치는지 판정함."""
        a = np.asarray(p0, dtype=np.float64)
        ab = np.asarray(p1, dtype=np.float64) - a
        denom = float(ab @ ab)
        t = 0.0 if denom < 1e-12 else float(np.clip((self.center - a) @ ab / denom, 0.0, 1.0))
        closest = a + ab * t
        return float(np.linalg.norm(self.center - closest)) <= self.radius + float(radius_mm)


class World:
    """충돌 판정 대상 장애물 모음 — 박스 5벽 + 점유 옥트리 + 구."""

    def __init__(self):
        self.walls = []
        self.spheres = []
        self.octomap = None

    def set_octomap(self, octree):
        """점유 옥트리를 planning scene에 등록함."""
        self.octomap = octree

    def clear_walls(self):
        """등록된 벽을 모두 지움."""
        self.walls = []

    def clear_spheres(self):
        """등록된 구 장애물을 모두 지움."""
        self.spheres = []

    def add_wall(self, obb):
        """벽 하나를 추가함."""
        self.walls.append(obb)

    def set_spheres(self, centers_mm, radius_mm):
        """구 장애물 목록을 통째로 교체함."""
        self.spheres = [Sphere(c, radius_mm, f"obstacle_{i}")
                        for i, c in enumerate(centers_mm)]

    def first_hit(self, p0, p1, radius_mm):
        """캡슐과 처음 겹치는 장애물 이름. 없으면 None."""
        for w in self.walls:
            if w.hits_capsule(p0, p1, radius_mm):
                return w.name
        if self.octomap is not None and self.octomap.hits_capsule(p0, p1, radius_mm):
            return "octomap"
        for s in self.spheres:
            if s.hits_capsule(p0, p1, radius_mm):
                return s.name
        return None


def link_points_mm(q):
    """관절각 배열로부터 링크 경계점들을 base 좌표(mm)로 계산함."""
    T = np.eye(4)
    pts = [T[:3, 3].copy()]
    for name, theta in zip(JOINT_ORDER, q):
        xyz, rpy = transforms._JOINT_ORIGINS_M[name]
        T = T @ transforms._urdf_transform(xyz, rpy) @ transforms._joint_z_rotation(float(theta))
        pts.append(T[:3, 3].copy())
    tip = T @ transforms._urdf_transform((0.0, 0.0, config.FLANGE_TO_FINGERTIP_MM / 1000.0), (0, 0, 0))
    pts.append(tip[:3, 3].copy())
    return [p * 1000.0 for p in pts]


def segment_distance_mm(p0, p1, q0, q1):
    """두 선분 사이의 최단거리."""
    u = np.asarray(p1, dtype=np.float64) - np.asarray(p0, dtype=np.float64)
    v = np.asarray(q1, dtype=np.float64) - np.asarray(q0, dtype=np.float64)
    w = np.asarray(p0, dtype=np.float64) - np.asarray(q0, dtype=np.float64)
    a, b, c = float(u @ u), float(u @ v), float(v @ v)
    d, e = float(u @ w), float(v @ w)
    denom = a * c - b * b
    if denom < 1e-12:
        s = 0.0
        t = (e / c) if c > 1e-12 else 0.0
    else:
        s = (b * e - c * d) / denom
        t = (a * e - b * d) / denom
    s = min(max(s, 0.0), 1.0)
    t = min(max(t, 0.0), 1.0)
    closest_p = np.asarray(p0, dtype=np.float64) + u * s
    closest_q = np.asarray(q0, dtype=np.float64) + v * t
    return float(np.linalg.norm(closest_p - closest_q))


def _always_colliding_pairs(samples=400, min_gap_ratio=1.0):
    """무작위 자세에서 항상 겹치는 링크 쌍을 자기충돌 검사에서 제외함."""
    lower = np.array([transforms._JOINT_LIMITS_RAD[j][0] for j in JOINT_ORDER])
    upper = np.array([transforms._JOINT_LIMITS_RAD[j][1] for j in JOINT_ORDER])
    n_link = len(LINK_RADII_MM)
    candidates = {(i, j) for i in range(n_link) for j in range(i + 2, n_link)}
    always = set(candidates)
    rng = np.random.default_rng(0)
    for _ in range(samples):
        pts = link_points_mm(rng.uniform(lower, upper))
        for (i, j) in list(always):
            gap = segment_distance_mm(pts[i], pts[i + 1], pts[j], pts[j + 1])
            if gap > (LINK_RADII_MM[i] + LINK_RADII_MM[j]) * min_gap_ratio:
                always.discard((i, j))
        if not always:
            break
    return always


SELF_COLLISION_PAIRS = sorted(
    {(i, j) for i in range(len(LINK_RADII_MM)) for j in range(i + 2, len(LINK_RADII_MM))}
    - _always_colliding_pairs())


class JointSpacePlanner:
    """관절공간에서 충돌 없는 경로를 찾는 RRT-Connect 계획기."""

    def __init__(self, world=None, logger=None, check_self_collision=True):
        self.world = world or World()
        self._log = logger
        self.check_self_collision = bool(check_self_collision)
        self.lower = np.array([transforms._JOINT_LIMITS_RAD[j][0] for j in JOINT_ORDER])
        self.upper = np.array([transforms._JOINT_LIMITS_RAD[j][1] for j in JOINT_ORDER])
        self.last_reason = ""

    def in_limits(self, q):
        """관절각이 한계 안인지 봄."""
        return bool(np.all(q >= self.lower) and np.all(q <= self.upper))

    def self_collision_hit(self, pts):
        """서로 닿는 링크 쌍 이름. 없으면 None."""
        for (i, j) in SELF_COLLISION_PAIRS:
            gap = segment_distance_mm(pts[i], pts[i + 1], pts[j], pts[j + 1])
            if gap < LINK_RADII_MM[i] + LINK_RADII_MM[j]:
                return f"link_{i + 1}↔link_{j + 1}"
        return None

    def state_hit(self, q):
        """이 관절자세에서 충돌하는 대상 이름. 없으면 None."""
        pts = link_points_mm(q)
        for i in range(len(pts) - 1):
            hit = self.world.first_hit(pts[i], pts[i + 1], LINK_RADII_MM[i])
            if hit is not None:
                return f"link_{i + 1}↔{hit}"
        if self.check_self_collision:
            return self.self_collision_hit(pts)
        return None

    def is_valid(self, q):
        """관절자세가 한계 안이고 충돌하지 않는지 봄."""
        q = np.asarray(q, dtype=np.float64)
        return self.in_limits(q) and self.state_hit(q) is None

    def motion_valid(self, qa, qb, resolution_rad=DEFAULT_RESOLUTION_RAD):
        """두 관절자세를 잇는 직선 구간 전체가 유효한지 봄."""
        qa = np.asarray(qa, dtype=np.float64)
        qb = np.asarray(qb, dtype=np.float64)
        n = max(2, int(math.ceil(float(np.max(np.abs(qb - qa))) / resolution_rad)) + 1)
        for i in range(n + 1):
            if not self.is_valid(qa + (qb - qa) * (i / n)):
                return False
        return True

    def _steer(self, frm, to, step_rad):
        d = to - frm
        dist = float(np.linalg.norm(d))
        if dist <= step_rad:
            return to.copy()
        return frm + d / dist * step_rad

    def _extend(self, tree, parents, target, step_rad, resolution_rad):
        idx = int(np.argmin([np.linalg.norm(node - target) for node in tree]))
        new = self._steer(tree[idx], target, step_rad)
        if not self.in_limits(new) or not self.motion_valid(tree[idx], new, resolution_rad):
            return None
        tree.append(new)
        parents.append(idx)
        return len(tree) - 1

    @staticmethod
    def _trace(tree, parents, idx):
        path = []
        while idx != -1:
            path.append(tree[idx])
            idx = parents[idx]
        return path[::-1]

    def plan(self, q_start, q_goal, max_iter=DEFAULT_MAX_ITER,
             step_rad=DEFAULT_STEP_RAD, resolution_rad=DEFAULT_RESOLUTION_RAD):
        """시작 자세에서 목표 자세까지 충돌 없는 관절 경로를 찾음."""
        q0 = np.asarray(q_start, dtype=np.float64)
        q1 = np.asarray(q_goal, dtype=np.float64)

        hit = self.state_hit(q0)
        if hit is not None:
            self.last_reason = f"시작 자세가 충돌: {hit}"
            return None
        if not self.in_limits(q1):
            self.last_reason = "목표 자세가 관절 한계 밖"
            return None
        hit = self.state_hit(q1)
        if hit is not None:
            self.last_reason = f"목표 자세가 충돌: {hit}"
            return None

        if self.motion_valid(q0, q1, resolution_rad):
            self.last_reason = "직선 구간으로 도달"
            return self._densify([q0, q1])

        tree_a, par_a = [q0], [-1]
        tree_b, par_b = [q1], [-1]
        for it in range(max_iter):
            if random.random() < DEFAULT_GOAL_BIAS:
                sample = q1 if (it % 2 == 0) else q0
            else:
                sample = np.random.uniform(self.lower, self.upper)

            a_new = self._extend(tree_a, par_a, sample, step_rad, resolution_rad)
            if a_new is not None:
                b_new = self._extend(tree_b, par_b, tree_a[a_new], step_rad, resolution_rad)
                if b_new is not None and self.motion_valid(
                        tree_a[a_new], tree_b[b_new], resolution_rad):
                    path = self._trace(tree_a, par_a, a_new) + self._trace(tree_b, par_b, b_new)[::-1]
                    if not np.allclose(path[0], q0):
                        path = path[::-1]
                    self.last_reason = f"RRT-Connect 성공(확장 {it + 1}회, 노드 {len(tree_a) + len(tree_b)}개)"
                    return self._densify(self._shortcut(path, resolution_rad))
            tree_a, tree_b = tree_b, tree_a
            par_a, par_b = par_b, par_a

        self.last_reason = f"경로를 찾지 못함(확장 {max_iter}회)"
        return None

    def _shortcut(self, path, resolution_rad):
        if len(path) < 3:
            return path
        path = list(path)
        for _ in range(SHORTCUT_ROUNDS):
            if len(path) < 3:
                break
            i = random.randrange(0, len(path) - 2)
            j = random.randrange(i + 2, len(path))
            if self.motion_valid(path[i], path[j], resolution_rad):
                path = path[:i + 1] + path[j:]
        return path

    @staticmethod
    def _densify(path, step_rad=DENSIFY_STEP_RAD):
        out = [np.asarray(path[0], dtype=np.float64)]
        for nxt in path[1:]:
            nxt = np.asarray(nxt, dtype=np.float64)
            prev = out[-1]
            n = max(1, int(math.ceil(float(np.linalg.norm(nxt - prev)) / step_rad)))
            for k in range(1, n + 1):
                out.append(prev + (nxt - prev) * (k / n))
        return out


def joints_dict(q):
    """관절각 배열을 이름별 dict로 바꿈."""
    return {name: float(v) for name, v in zip(JOINT_ORDER, q)}


def joints_array(joint_positions_by_name):
    """이름별 dict를 관절각 배열로 바꿈."""
    return np.array([float(joint_positions_by_name[j]) for j in JOINT_ORDER])


def path_to_posx(path):
    """관절 경로를 DSR posx 리스트로 바꿈."""
    return [tuple(transforms.forward_kinematics_posx_mm_deg(joints_dict(q))) for q in path]
