"""좌표 변환 유틸."""
import struct
import warnings

import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

from yolo_detect import config

def robot_pose_to_matrix(x, y, z, rx, ry, rz):
    """base->gripper 4x4 변환행렬."""
    R = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

def backproject_bbox_median(depth_frame, box, intrinsics):
    """bbox 내부 유효 depth 픽셀을 전부 각자 역투영한 뒤, 3D 점들의 축별 median을 반환."""
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    x1, y1 = max(x1, 0), max(y1, 0)
    x2 = min(x2, depth_frame.shape[1] - 1)
    y2 = min(y2, depth_frame.shape[0] - 1)
    if x2 <= x1 or y2 <= y1:
        return None
    region = depth_frame[y1:y2, x1:x2].astype(np.float64)
    vs, us = np.nonzero(region > 0)
    if vs.size == 0:
        return None
    depths = region[vs, us]
    fx, fy = intrinsics['fx'], intrinsics['fy']
    ppx, ppy = intrinsics['ppx'], intrinsics['ppy']
    xs = (us + x1 - ppx) * depths / fx
    ys = (vs + y1 - ppy) * depths / fy
    return np.array([np.median(xs), np.median(ys), np.median(depths)], dtype=np.float64)

def backproject_mask_median(depth_frame, mask, intrinsics):
    """bbox 대신 분할 마스크."""
    vs, us = np.nonzero(mask & (depth_frame > 0))
    if vs.size == 0:
        return None
    depths = depth_frame[vs, us].astype(np.float64)
    fx, fy = intrinsics['fx'], intrinsics['fy']
    ppx, ppy = intrinsics['ppx'], intrinsics['ppy']
    xs = (us - ppx) * depths / fx
    ys = (vs - ppy) * depths / fy
    return np.array([np.median(xs), np.median(ys), np.median(depths)], dtype=np.float64)

def pixelwise_median_depth(frames):
    """여러 depth 프레임을 픽셀별 중앙값으로 합쳐."""
    stack = np.stack(frames).astype(np.float32)
    stack[stack == 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        med = np.nanmedian(stack, axis=0)
    return np.nan_to_num(med, nan=0.0)

def backproject_mask_points(depth_frame, mask, intrinsics, stride=1,
                            z_min=1.0, z_max=2000.0):
    """mask 안의 유효 depth 픽셀을 전부 역투영해."""
    m = mask
    if stride > 1:
        m = np.zeros_like(mask)
        m[::stride, ::stride] = mask[::stride, ::stride]

    vs, us = np.nonzero(m)
    if vs.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    depths = depth_frame[vs, us].astype(np.float32)
    valid = (depths >= z_min) & (depths <= z_max)
    us, vs, depths = us[valid], vs[valid], depths[valid]

    fx, fy = intrinsics['fx'], intrinsics['fy']
    ppx, ppy = intrinsics['ppx'], intrinsics['ppy']
    xs = (us - ppx) * depths / fx
    ys = (vs - ppy) * depths / fy
    return np.stack([xs, ys, depths], axis=1).astype(np.float32)

def camera_to_base_batch(camera_points_mm, T_gripper2camera, robot_posx):
    """camera_to_base의 점군 판."""
    pts = np.asarray(camera_points_mm, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    x, y, z, rx, ry, rz = robot_posx[:6]
    base2cam = robot_pose_to_matrix(x, y, z, rx, ry, rz) @ T_gripper2camera
    pts_h = np.hstack([pts, np.ones((len(pts), 1))])
    return (base2cam @ pts_h.T).T[:, :3]

_LOOK_AT_MAX_ITERATIONS = 60
_LOOK_AT_CONVERGENCE_MM = 1e-6
_LOOK_AT_DAMPING = 0.5
_LOOK_AT_RESIDUAL_TOLERANCE_DEG = 5.0

def _camera_look_at_rotation(eye_pos_base, target_pos_base, world_up=(0.0, 0.0, 1.0)):
    """카메라가 eye_pos_base에서 target_pos_base."""
    forward = np.asarray(target_pos_base, dtype=np.float64) - np.asarray(eye_pos_base, dtype=np.float64)
    norm = np.linalg.norm(forward)
    if norm < 1e-6:
        raise ValueError("eye_pos_base와 target_pos_base가 너무 가까움 — 방향 계산 불가")
    forward = forward / norm

    up_ref = np.asarray(world_up, dtype=np.float64)
    if np.linalg.norm(np.cross(forward, up_ref)) < 1e-6:
        up_ref = np.array([1.0, 0.0, 0.0])

    x_cam = np.cross(forward, up_ref)
    x_cam /= np.linalg.norm(x_cam)
    y_cam = np.cross(forward, x_cam)
    y_cam /= np.linalg.norm(y_cam)

    return np.column_stack([x_cam, y_cam, forward])

def look_at_orientation(eye_pos_base, target_pos_base, T_gripper2camera, world_up=(0.0, 0.0, 1.0)):
    """카메라가 eye_pos_base에서 target_pos_base."""
    R_ce = T_gripper2camera[:3, :3]
    t_ce = T_gripper2camera[:3, 3]
    p_flange = np.asarray(eye_pos_base, dtype=np.float64)

    radius = float(np.linalg.norm(t_ce))

    def gripper_rotation_for(cam_pos):
        return _camera_look_at_rotation(cam_pos, target_pos_base, world_up) @ R_ce.T

    cam_pos = p_flange
    for _ in range(_LOOK_AT_MAX_ITERATIONS):
        next_cam = p_flange + gripper_rotation_for(cam_pos) @ t_ce
        step = np.linalg.norm(next_cam - cam_pos)
        cam_pos = cam_pos + _LOOK_AT_DAMPING * (next_cam - cam_pos)
        if step < _LOOK_AT_CONVERGENCE_MM:
            break

    def sphere_point(angles):
        theta, phi = angles
        return p_flange + radius * np.array([
            np.cos(phi) * np.cos(theta), np.cos(phi) * np.sin(theta), np.sin(phi)])

    def residual(angles):
        c = sphere_point(angles)
        return c - (p_flange + gripper_rotation_for(c) @ t_ce)

    u = cam_pos - p_flange
    if np.linalg.norm(u) < 1e-9:
        u = t_ce
    u = u / np.linalg.norm(u)
    guess = [np.arctan2(u[1], u[0]), np.arcsin(float(np.clip(u[2], -1.0, 1.0)))]
    solution = least_squares(residual, guess, method='lm', xtol=1e-12, ftol=1e-12, gtol=1e-12)
    R_final = gripper_rotation_for(sphere_point(solution.x))

    rx, ry, rz = Rotation.from_matrix(R_final).as_euler('ZYZ', degrees=True)
    residual_deg, _ = camera_aim_error_deg(
        [p_flange[0], p_flange[1], p_flange[2], rx, ry, rz], target_pos_base, T_gripper2camera)
    if residual_deg > _LOOK_AT_RESIDUAL_TOLERANCE_DEG:
        raise ValueError(
            f"카메라 지향 방향이 수렴하지 않음(잔차 {residual_deg:.2f}° > "
            f"{_LOOK_AT_RESIDUAL_TOLERANCE_DEG}°) — 플랜지 {np.round(p_flange, 1).tolist()}에서 "
            f"목표 {np.round(np.asarray(target_pos_base, dtype=np.float64), 1).tolist()}까지의 "
            f"기하가 퇴화했을 수 있음(물체가 플랜지 바로 아래 등)")
    return float(rx), float(ry), float(rz)

def min_camera_aim_distance_mm(T_gripper2camera, ratio=2.0):
    """카메라 지향 자세를 안정적으로 풀 수 있는 최소 목표 거리."""
    return float(ratio * np.linalg.norm(np.asarray(T_gripper2camera)[:3, 3]))

def camera_aim_error_deg(posx, target_pos_base, T_gripper2camera):
    """주어진 로봇 자세."""
    x, y, z, rx, ry, rz = posx[:6]
    base2gripper = robot_pose_to_matrix(x, y, z, rx, ry, rz)
    base2cam = base2gripper @ T_gripper2camera
    cam_pos = base2cam[:3, 3]
    optical_axis = base2cam[:3, 2]
    v = np.asarray(target_pos_base, dtype=np.float64) - cam_pos
    dist = np.linalg.norm(v)
    if dist < 1e-6:
        raise ValueError("카메라와 목표가 너무 가까움 — 조준각 계산 불가")
    cos = float(np.clip(np.dot(optical_axis, v / dist), -1.0, 1.0))
    return float(np.degrees(np.arccos(cos))), float(dist)

_JOINT_ORIGINS_M = {
    'joint_1': ((0, 0, 0.1345), (0, 0, 0)),
    'joint_2': ((0, 0.0062, 0), (0, -1.571, -1.571)),
    'joint_3': ((0.411, 0, 0), (0, 0, 1.571)),
    'joint_4': ((0, -0.368, 0), (1.571, 0, 0)),
    'joint_5': ((0, 0, 0), (-1.571, 0, 0)),
    'joint_6': ((0, -0.121, 0), (1.571, 0, 0)),
}
_TOOL0_OFFSET_M = ((0, 0, 0), (3.1415926535, -1.570796327, 0))
_FK_JOINT_ORDER = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

def _urdf_transform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    T[:3, 3] = xyz
    return T

def _joint_z_rotation(theta):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('z', theta).as_matrix()
    return T

def forward_kinematics_base_link_m(joint_positions_by_name):
    """관절각."""
    T = np.eye(4)
    for jname in _FK_JOINT_ORDER:
        xyz, rpy = _JOINT_ORIGINS_M[jname]
        T = T @ _urdf_transform(xyz, rpy) @ _joint_z_rotation(joint_positions_by_name[jname])
    T = T @ _urdf_transform(*_TOOL0_OFFSET_M)
    return T

def ordered_joint_degrees(joint_positions_by_name):
    """joint_states 구독값."""
    return [float(np.degrees(joint_positions_by_name[j])) for j in _FK_JOINT_ORDER]

_TOOL_TO_DSR_ROTATION_CORRECTION = Rotation.from_quat([
    0.7053469925271197, -0.0007025857174631245, 0.708858432185709, -0.002202186521496839,
]).as_matrix()

def camera_look_at_tool0_quat(eye_pos_base, target_pos_base, T_gripper2camera, world_up=(0.0, 0.0, 1.0)):
    """look_at_orientation과 같은 '카메라가 target을 보게' 계산이지만, DSR posx."""
    try:
        rx, ry, rz = look_at_orientation(eye_pos_base, target_pos_base, T_gripper2camera, world_up)
        R_world_dsr_tcp = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
        exact = True
    except ValueError:
        R_world_camera = _camera_look_at_rotation(eye_pos_base, target_pos_base, world_up)
        R_world_dsr_tcp = R_world_camera @ np.asarray(T_gripper2camera)[:3, :3].T
        exact = False
    R_world_tool0 = R_world_dsr_tcp @ _TOOL_TO_DSR_ROTATION_CORRECTION.T
    return Rotation.from_matrix(R_world_tool0).as_quat(), exact

def forward_kinematics_posx_mm_deg(joint_positions_by_name):
    """관절각."""
    T = forward_kinematics_base_link_m(joint_positions_by_name)
    xyz_mm = T[:3, 3] * 1000.0
    R = T[:3, :3] @ _TOOL_TO_DSR_ROTATION_CORRECTION
    rx, ry, rz = Rotation.from_matrix(R).as_euler('ZYZ', degrees=True)
    return [float(xyz_mm[0]), float(xyz_mm[1]), float(xyz_mm[2]), float(rx), float(ry), float(rz)]

_JOINT_LIMITS_RAD = {
    'joint_1': (-6.2832, 6.2832),
    'joint_2': (-6.2832, 6.2832),
    'joint_3': (-2.618, 2.618),
    'joint_4': (-6.2832, 6.2832),
    'joint_5': (-6.2832, 6.2832),
    'joint_6': (-6.2832, 6.2832),
}

def inverse_kinematics_joints(target_posx_mm_deg, seed_joint_positions_by_name,
                               pos_tol_mm=1.0, ori_tol_deg=0.5):
    """DSR posx [x,y,z,rx,ry,rz]."""
    from scipy.optimize import least_squares

    target = np.asarray(target_posx_mm_deg, dtype=np.float64)
    t_xyz_m = target[:3] / 1000.0
    R_target = Rotation.from_euler('ZYZ', target[3:6], degrees=True).as_matrix()
    q0 = np.array([float(seed_joint_positions_by_name[j]) for j in _FK_JOINT_ORDER])
    lo = np.array([_JOINT_LIMITS_RAD[j][0] for j in _FK_JOINT_ORDER])
    hi = np.array([_JOINT_LIMITS_RAD[j][1] for j in _FK_JOINT_ORDER])

    def _pose(q):
        T = forward_kinematics_base_link_m(dict(zip(_FK_JOINT_ORDER, q)))
        return T[:3, 3], T[:3, :3] @ _TOOL_TO_DSR_ROTATION_CORRECTION

    def _residual(q):
        xyz, R = _pose(q)
        e_pos = (xyz - t_xyz_m) * 1000.0
        e_rot = Rotation.from_matrix(R @ R_target.T).as_rotvec() * 200.0
        return np.concatenate([e_pos, e_rot])

    sol = least_squares(_residual, np.clip(q0, lo, hi), bounds=(lo, hi),
                        xtol=1e-12, ftol=1e-12)
    xyz, R = _pose(sol.x)
    pos_err_mm = float(np.linalg.norm(xyz * 1000.0 - target[:3]))
    ori_err_deg = float(np.degrees(Rotation.from_matrix(R @ R_target.T).magnitude()))
    if pos_err_mm > pos_tol_mm or ori_err_deg > ori_tol_deg:
        return None
    return dict(zip(_FK_JOINT_ORDER, [float(v) for v in sol.x]))

def slerp_posx(from_posx, to_posx, fraction):
    """두 posx 사이를 보간한 posx."""
    a = np.asarray(from_posx, dtype=np.float64)
    b = np.asarray(to_posx, dtype=np.float64)
    xyz = a[:3] + (b[:3] - a[:3]) * fraction
    key = Rotation.from_euler('ZYZ', [a[3:6], b[3:6]], degrees=True)
    R = key[0] * Rotation.from_rotvec((key[0].inv() * key[1]).as_rotvec() * fraction)
    rx, ry, rz = R.as_euler('ZYZ', degrees=True)
    return [float(xyz[0]), float(xyz[1]), float(xyz[2]), float(rx), float(ry), float(rz)]

def rotation_geodesic_error_deg(rx1, ry1, rz1, rx2, ry2, rz2):
    """두 ZYZ 오일러각."""
    R1 = Rotation.from_euler('ZYZ', [rx1, ry1, rz1], degrees=True).as_matrix()
    R2 = Rotation.from_euler('ZYZ', [rx2, ry2, rz2], degrees=True).as_matrix()
    rel = R1.T @ R2
    return float(Rotation.from_matrix(rel).magnitude() * 180.0 / np.pi)

def camera_to_base(camera_coords_mm, T_gripper2camera, robot_posx):
    """camera 좌표계 3D점 -> base 좌표계."""
    coord_h = np.append(np.asarray(camera_coords_mm, dtype=np.float64), 1.0)
    x, y, z, rx, ry, rz = robot_posx[:6]
    base2gripper = robot_pose_to_matrix(x, y, z, rx, ry, rz)
    base2cam = base2gripper @ T_gripper2camera
    base_coord = base2cam @ coord_h
    return base_coord[:3]

def find_checkerboard_pose(image, camera_matrix, dist_coeffs,
                            board_size=config.CHECKERBOARD_SIZE,
                            square_size=config.CHECKERBOARD_SQUARE_MM):
    """이미지에서 체커보드를 찾아 camera->checker 변환."""
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_size

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, board_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found:
        return None, None

    corners_sub = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )
    ok, rvec, tvec = cv2.solvePnP(objp, corners_sub, camera_matrix, dist_coeffs)
    if not ok:
        return None, None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec

def checker_to_base(R_cam2checker, t_cam2checker, T_gripper2camera, robot_posx):
    """T_checker2base = T_base2gripper x T_gripper2camera x T_camera2checker."""
    T_cam2checker = np.eye(4)
    T_cam2checker[:3, :3] = R_cam2checker
    T_cam2checker[:3, 3] = t_cam2checker.flatten()

    x, y, z, rx, ry, rz = robot_posx[:6]
    T_base2gripper = robot_pose_to_matrix(x, y, z, rx, ry, rz)
    T_checker2base = T_base2gripper @ T_gripper2camera @ T_cam2checker
    return T_checker2base

def build_colored_pointcloud2(header, points_xyz_mm, colors_01):
    """Open3D 누적 맵."""
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    points = []
    rgb_u8 = (np.clip(colors_01, 0.0, 1.0) * 255).astype(np.uint8)
    for (x, y, z), (r, g, b) in zip(points_xyz_mm, rgb_u8):
        rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
        rgb_float = struct.unpack('f', struct.pack('I', rgb_uint32))[0]
        points.append([float(x), float(y), float(z), rgb_float])
    return pc2.create_cloud(header, fields, points)

def cluster_points(points, weights, threshold_mm=config.CLUSTER_THRESHOLD_MM):
    """단순 거리 기준 클러스터링."""
    n = len(points)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj

    for i in range(n):
        for j in range(i + 1, n):
            if np.linalg.norm(points[i] - points[j]) <= threshold_mm:
                union(i, j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)

    results = []
    for idxs in groups.values():
        pts = np.array([points[i] for i in idxs])
        ws = np.array([weights[i] for i in idxs], dtype=np.float64)
        if ws.sum() <= 0:
            centroid = pts.mean(axis=0)
        else:
            centroid = (pts * ws[:, None]).sum(axis=0) / ws.sum()
        merged_confidence = float(ws.max())
        results.append((centroid, merged_confidence))
    return results
