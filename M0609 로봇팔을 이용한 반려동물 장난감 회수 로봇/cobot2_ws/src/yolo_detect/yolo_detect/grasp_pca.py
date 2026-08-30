"""부위 점군에서 RG2 파지 자세."""
import numpy as np
from scipy.spatial.transform import Rotation

from yolo_detect import config, transforms

MIN_POINT_COUNT = 30
PCA_AXIS_LOW_CONFIDENCE_RATIO = 1.20
PCA_AXIS_HIGH_CONFIDENCE_RATIO = 1.35
MIN_TOOL_Z_TILT_DEG = 5.0
MAX_TOOL_Z_TILT_DEG = 80.0
ORIENTATION_SAMPLE_COUNT_PER_PART = 2000
BODY_TOOL_Z_OFFSET_MM = 3.0

GRASP_Z_OFFSET_MM = config.FLANGE_TO_FINGERTIP_MM
PREGRASP_DISTANCE_MM = 50.0
GRASP_DEPTH_MM = 5.0
GRASP_CLEARANCE_MM = 10.0
GRASP_ALONG_AXIS_DEEPER_MM = 20.0
FLOOR_Z_MM = 0.0
FLOOR_CLEARANCE_MM = 30.0
BASE_Z_MIN_MM = -300.0
BASE_Z_MAX_MM = 1500.0

WIDTH_PERCENTILE = 2.5
RG2_MAX_WIDTH_MM = 110.0
PART_MAX_EXTENT_MM = 120.0

class GraspTooWideError(ValueError):
    """부위가 그리퍼 최대 개방보다 넓어 파지할 수 없을 때 던짐."""

def _normalize(vector):
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-9:
        raise ValueError("PCA 축을 정규화할 수 없습니다(길이 0).")
    return np.asarray(vector, dtype=float) / norm

def is_part_family(part_name, family):
    """arm1/arm_1처럼 family 뒤에 번호가 붙은 파트명을 판별함."""
    name = str(part_name).strip().lower()
    if name == family:
        return True
    if not name.startswith(family):
        return False
    suffix = name[len(family):].lstrip("_-")
    return bool(suffix) and suffix.isdigit()

def split_parts(part_names, part_num_points, points):
    """이어 붙은 점군을 부위 이름별 점군으로 나눔."""
    points = np.asarray(points, dtype=np.float64)
    counts = [int(c) for c in part_num_points]
    if sum(counts) != len(points):
        raise ValueError(
            f"부위 경계 합({sum(counts)})이 점 개수({len(points)})와 다릅니다.")
    duplicated = {n for n in part_names if list(part_names).count(n) > 1}
    used, grouped, start = {}, {}, 0
    for name, count in zip(part_names, counts):
        key = str(name).strip().lower()
        if name in duplicated:
            used[name] = used.get(name, 0) + 1
            key = f"{key}{used[name]}"
        grouped[key] = points[start:start + count]
        start += count
    return grouped

def resolve_target_part(part_points, target_part, part_confidences=None):
    """요청한 부위 이름을 split_parts가 만든 실제 키로 바꿈."""
    key = str(target_part or "").strip().lower()
    if not key:
        return None
    if key in part_points:
        return key
    family_keys = [k for k in part_points if is_part_family(k, key)]
    if not family_keys:
        raise ValueError(
            f"요청한 부위 {key!r}가 점군에 없습니다. 있는 부위: {sorted(part_points)}")
    if len(family_keys) == 1:
        return family_keys[0]

    conf_by_key = _confidence_by_key(part_points, part_confidences)
    if conf_by_key:
        return max(family_keys, key=lambda k: conf_by_key.get(k, -1.0))
    return max(family_keys, key=lambda k: len(part_points[k]))

def _confidence_by_key(part_points, part_confidences):
    """part_confidences를 {split_parts 키: conf} dict로 정규화."""
    if part_confidences is None:
        return {}
    if isinstance(part_confidences, dict):
        return {str(k).strip().lower(): float(v) for k, v in part_confidences.items()}
    confs = list(part_confidences)
    if len(confs) != len(part_points):
        return {}
    return {k: float(c) for k, c in zip(part_points.keys(), confs)}

def combine_part_family(part_points, family):
    """leg, leg1, leg2처럼 같은 family 점군을 하나로 합침."""
    clouds = [cloud for name, cloud in part_points.items()
              if is_part_family(name, family)]
    return np.vstack(clouds) if clouds else None

def balance_orientation_groups(named_groups,
                               sample_count=ORIENTATION_SAMPLE_COUNT_PER_PART):
    """파트별 점 개수를 동일하게 맞춰 특정 파트의 PCA 지배를 막음."""
    valid = [(name, np.asarray(pts, dtype=np.float64))
             for name, pts in named_groups if pts is not None and len(pts) > 0]
    if not valid:
        return None, [], 0
    per_group = min(sample_count, min(len(pts) for _, pts in valid))
    rng = np.random.default_rng(0)
    chunks, names = [], []
    for name, pts in valid:
        if len(pts) > per_group:
            pts = pts[rng.choice(len(pts), per_group, replace=False)]
        chunks.append(pts)
        names.append(name)
    return np.vstack(chunks), names, per_group

def select_orientation_strategy(part_points, target_part, fallback_points):
    """장축 계산에 쓸 점군과, 있으면 body→사지 기준축을 고름."""
    target_part = str(target_part or "").strip().lower()
    if not part_points or not target_part:
        return {"points": fallback_points, "preferred_axis": None,
                "prefer_preferred_axis": False,
                "groups": [target_part or "unlabeled"]}

    body_points = part_points.get("body")
    head_points = part_points.get("head")
    leg_points = combine_part_family(part_points, "leg")
    target_cloud = part_points.get(target_part, fallback_points)
    target_is_arm = is_part_family(target_part, "arm")
    target_is_leg = is_part_family(target_part, "leg")

    if target_is_arm:
        named_groups = [("body", body_points), (target_part, target_cloud)]
    elif target_part in ("head", "body") or target_is_leg:
        named_groups = [("head", head_points), ("body", body_points),
                        ("leg", leg_points)]
    else:
        named_groups = [(target_part, target_cloud)]

    orientation_points, groups, sample_count = balance_orientation_groups(named_groups)
    if orientation_points is None:
        orientation_points, groups, sample_count = fallback_points, [target_part], 0

    preferred_axis = None
    prefer_preferred_axis = False
    is_numbered_limb = ((target_is_arm and target_part != "arm")
                        or (target_is_leg and target_part != "leg"))
    if is_numbered_limb and body_points is not None and len(body_points):
        preferred_axis = np.mean(target_cloud, axis=0) - np.mean(body_points, axis=0)
        if np.linalg.norm(preferred_axis) >= 1.0e-9:
            prefer_preferred_axis = True
        else:
            preferred_axis = None

    return {"points": orientation_points, "preferred_axis": preferred_axis,
            "prefer_preferred_axis": prefer_preferred_axis, "groups": groups,
            "sample_per_group": sample_count}

def enforce_minimum_tool_z_tilt(tool_z, current_rotation, minimum_tilt_deg):
    """Tool Z가 Base Z와 평행해지지 않도록 최소 기울기를 적용함."""
    tool_z = _normalize(tool_z)
    base_z = np.array([0.0, 0.0, 1.0])
    vertical_sign = 1.0 if np.dot(tool_z, base_z) >= 0.0 else -1.0
    nearest_vertical = vertical_sign * base_z
    tilt_deg = float(np.degrees(np.arccos(
        np.clip(np.dot(tool_z, nearest_vertical), -1.0, 1.0))))
    minimum_tilt_deg = max(0.0, min(89.0, float(minimum_tilt_deg)))
    if tilt_deg + 1.0e-6 >= minimum_tilt_deg:
        return tool_z, tilt_deg, False

    horizontal = tool_z - np.dot(tool_z, base_z) * base_z
    for column in (0, 1):
        if np.linalg.norm(horizontal) >= 1.0e-9:
            break
        horizontal = np.asarray(current_rotation, dtype=float)[:, column]
        horizontal = horizontal - np.dot(horizontal, base_z) * base_z
    if np.linalg.norm(horizontal) < 1.0e-9:
        horizontal = np.array([1.0, 0.0, 0.0])
    horizontal = _normalize(horizontal)

    tilt_rad = np.radians(minimum_tilt_deg)
    tilted = np.cos(tilt_rad) * nearest_vertical + np.sin(tilt_rad) * horizontal
    return _normalize(tilted), minimum_tilt_deg, True

def orient_tool_z_toward_floor(tool_z, current_rotation, minimum_tilt_deg):
    """Tool Z를 Base -Z 반구로 보내 물체·바닥 방향을 향하게 함."""
    tool_z = _normalize(tool_z)
    base_z = np.array([0.0, 0.0, 1.0])
    floor_forced = False

    if np.dot(tool_z, base_z) > 1.0e-9:
        tool_z = -tool_z
        floor_forced = True

    if abs(float(np.dot(tool_z, base_z))) <= 1.0e-9:
        horizontal = tool_z - np.dot(tool_z, base_z) * base_z
        if np.linalg.norm(horizontal) < 1.0e-9:
            horizontal = np.asarray(current_rotation, dtype=float)[:, 0]
            horizontal = horizontal - np.dot(horizontal, base_z) * base_z
        if np.linalg.norm(horizontal) < 1.0e-9:
            horizontal = np.array([1.0, 0.0, 0.0])
        horizontal = _normalize(horizontal)
        tilt_rad = np.radians(max(0.0, min(89.0, float(minimum_tilt_deg))))
        tool_z = np.sin(tilt_rad) * horizontal - np.cos(tilt_rad) * base_z
        floor_forced = True

    return _normalize(tool_z), floor_forced

def construct_floor_aligned_grasp_axes(major_axis, desired_tool_z,
                                       current_rotation, minimum_tool_z_tilt_deg,
                                       approach_rotation_deg=0.0,
                                       closing_tilt_deg=0.0):
    """Tool X를 바닥과 평행하고 장축과 수직으로 구성함."""
    base_z = np.array([0.0, 0.0, 1.0])
    major_axis = _normalize(major_axis)
    major_horizontal = major_axis - np.dot(major_axis, base_z) * base_z
    if np.linalg.norm(major_horizontal) < 1.0e-9:
        major_horizontal = np.asarray(current_rotation, dtype=float)[:, 1]
        major_horizontal = major_horizontal - np.dot(major_horizontal, base_z) * base_z
    if np.linalg.norm(major_horizontal) < 1.0e-9:
        major_horizontal = np.array([1.0, 0.0, 0.0])
    major_horizontal = _normalize(major_horizontal)

    tool_x = _normalize(np.cross(base_z, major_horizontal))

    if abs(float(closing_tilt_deg)) > 1.0e-9:
        rot_x = Rotation.from_rotvec(np.radians(float(closing_tilt_deg)) * major_horizontal)
        tool_x = _normalize(rot_x.apply(tool_x))

    desired_tool_z = _normalize(desired_tool_z)
    tool_z = desired_tool_z - np.dot(desired_tool_z, tool_x) * tool_x
    if np.linalg.norm(tool_z) < 1.0e-9:
        tool_z = -base_z
    tool_z = _normalize(tool_z)
    if np.dot(tool_z, base_z) > 0.0:
        tool_z = -tool_z

    minimum_tool_z_tilt_deg = max(0.0, min(89.0, float(minimum_tool_z_tilt_deg)))
    tilt_deg = float(np.degrees(np.arccos(np.clip(np.dot(tool_z, -base_z), -1.0, 1.0))))
    tilt_adjusted = False
    if tilt_deg + 1.0e-6 < minimum_tool_z_tilt_deg:
        approach_horizontal = major_horizontal
        desired_horizontal = desired_tool_z - np.dot(desired_tool_z, base_z) * base_z
        if (np.linalg.norm(desired_horizontal) >= 1.0e-9
                and np.dot(approach_horizontal, desired_horizontal) < 0.0):
            approach_horizontal = -approach_horizontal
        tilt_rad = np.radians(minimum_tool_z_tilt_deg)
        tool_z = _normalize(np.sin(tilt_rad) * approach_horizontal
                            - np.cos(tilt_rad) * base_z)
        tilt_deg = minimum_tool_z_tilt_deg
        tilt_adjusted = True

    if abs(float(approach_rotation_deg)) > 1.0e-9:
        rot = Rotation.from_rotvec(np.radians(float(approach_rotation_deg)) * tool_x)
        tool_z = _normalize(rot.apply(tool_z))
        if np.dot(tool_z, base_z) > -np.cos(np.radians(MAX_TOOL_Z_TILT_DEG)):
            raise ValueError(
                f"접근축 회전 {approach_rotation_deg:+.0f}°에서 접근 방향이 수평/위쪽으로 "
                f"넘어감(허용 기울기 상한 {MAX_TOOL_Z_TILT_DEG:.0f}°)")
        tilt_deg = float(np.degrees(np.arccos(np.clip(np.dot(tool_z, -base_z), -1.0, 1.0))))
        if tilt_deg + 1.0e-6 < minimum_tool_z_tilt_deg:
            raise ValueError(
                f"접근축 회전 {approach_rotation_deg:+.0f}°에서 기울기가 {tilt_deg:.1f}°로 "
                f"최소 기울기 {minimum_tool_z_tilt_deg:.0f}° 미만(손목 특이점 방지 하한)")

    tool_y = _normalize(np.cross(tool_z, tool_x))
    tool_x = _normalize(np.cross(tool_y, tool_z))
    tool_x_floor_angle_deg = float(np.degrees(np.arcsin(
        np.clip(abs(np.dot(tool_x, base_z)), 0.0, 1.0))))
    return {"major_axis": major_axis, "major_horizontal": major_horizontal,
            "tool_x": tool_x, "tool_y": tool_y, "tool_z": tool_z,
            "tool_z_tilt_deg": tilt_deg, "tool_z_tilt_adjusted": tilt_adjusted,
            "tool_x_floor_angle_deg": tool_x_floor_angle_deg}

def calculate_3d_pca(points, camera_origin, current_rotation,
                     major_axis_points=None, preferred_major_axis=None,
                     prefer_preferred_axis=False,
                     min_tool_z_tilt_deg=MIN_TOOL_Z_TILT_DEG,
                     low_ratio=PCA_AXIS_LOW_CONFIDENCE_RATIO,
                     high_ratio=PCA_AXIS_HIGH_CONFIDENCE_RATIO,
                     approach_rotation_deg=0.0,
                     closing_tilt_deg=0.0):
    """파지점 점군과 선택적 장축 점군으로 Tool frame을 만듦."""
    points = np.asarray(points, dtype=np.float64)
    current_rotation = np.asarray(current_rotation, dtype=np.float64)
    centroid = np.mean(points, axis=0)
    covariance = np.cov(points - centroid, rowvar=False)
    target_eigenvalues, target_eigenvectors = np.linalg.eigh(covariance)
    if target_eigenvalues[-1] < 1.0e-9:
        raise ValueError("점군 covariance가 퇴화했습니다(한 점에 뭉쳐 있음).")

    surface_normal = _normalize(target_eigenvectors[:, 0])
    if np.dot(surface_normal, np.asarray(camera_origin) - centroid) < 0.0:
        surface_normal = -surface_normal

    if major_axis_points is None:
        major_axis_points = points
    major_axis_points = np.asarray(major_axis_points, dtype=np.float64)
    major_axis_centroid = np.mean(major_axis_points, axis=0)
    eigenvalues, _ = np.linalg.eigh(
        np.cov(major_axis_points - major_axis_centroid, rowvar=False))
    if eigenvalues[-1] < 1.0e-9:
        raise ValueError("장축 계산용 점군 covariance가 퇴화했습니다.")

    horizontal_points = major_axis_points[:, :2]
    horizontal_covariance = np.cov(
        horizontal_points - np.mean(horizontal_points, axis=0), rowvar=False)
    horizontal_eigenvalues, horizontal_eigenvectors = np.linalg.eigh(horizontal_covariance)
    horizontal_shape_ratio = float(
        horizontal_eigenvalues[-1] / max(horizontal_eigenvalues[-2], 1.0e-9))
    pca_horizontal_axis = np.array(
        [horizontal_eigenvectors[0, -1], horizontal_eigenvectors[1, -1], 0.0])

    valid_preferred_axis = False
    if preferred_major_axis is not None:
        preferred_major_axis = np.asarray(preferred_major_axis, dtype=np.float64)
        valid_preferred_axis = bool(
            preferred_major_axis.shape == (3,)
            and np.all(np.isfinite(preferred_major_axis))
            and np.linalg.norm(preferred_major_axis[:2]) >= 1.0e-9)

    if prefer_preferred_axis and valid_preferred_axis:
        selected_major_axis = preferred_major_axis
        major_axis_source = "part_center_connection"
    elif horizontal_shape_ratio < low_ratio:
        if valid_preferred_axis:
            selected_major_axis = preferred_major_axis
            major_axis_source = "low_confidence_center_fallback"
        else:
            selected_major_axis = current_rotation[:, 1]
            major_axis_source = "low_confidence_current_tool_y"
    else:
        selected_major_axis = pca_horizontal_axis
        major_axis_source = ("pca_high_confidence"
                             if horizontal_shape_ratio >= high_ratio
                             else "pca_medium_confidence")

    tool_z = _normalize(current_rotation[:, 2])
    if np.dot(tool_z, centroid - np.asarray(camera_origin)) < 0.0:
        tool_z = -tool_z
    tool_z, tool_z_floor_forced = orient_tool_z_toward_floor(
        tool_z, current_rotation, min_tool_z_tilt_deg)
    tool_z, tool_z_tilt_deg, tool_z_tilt_adjusted = enforce_minimum_tool_z_tilt(
        tool_z, current_rotation, min_tool_z_tilt_deg)

    axes = construct_floor_aligned_grasp_axes(
        selected_major_axis, tool_z, current_rotation, min_tool_z_tilt_deg,
        approach_rotation_deg=approach_rotation_deg,
        closing_tilt_deg=closing_tilt_deg)
    tool_x, tool_y, tool_z = axes["tool_x"], axes["tool_y"], axes["tool_z"]
    tool_z_tilt_adjusted = bool(tool_z_tilt_adjusted or axes["tool_z_tilt_adjusted"])

    candidates = (np.column_stack((tool_x, tool_y, tool_z)),
                  np.column_stack((-tool_x, -tool_y, tool_z)))
    rotation_matrix = min(candidates, key=lambda c: Rotation.from_matrix(
        current_rotation.T @ c).magnitude())
    gripper_rotation_deg = float(np.degrees(Rotation.from_matrix(
        current_rotation.T @ rotation_matrix).magnitude()))

    normal_projection = (points - centroid) @ surface_normal
    contact_point = centroid + surface_normal * np.percentile(normal_projection, 85.0)

    grasp_width_mm, width_diag = _measure_grasp_width(points, centroid, rotation_matrix)

    return {"grasp_width_mm": grasp_width_mm,
            "part_extent_mm": width_diag["part_extent_mm"],
            "width_trustworthy": width_diag["trustworthy"],
            "width_warning": width_diag["warning"],
            "centroid": centroid, "contact_point": contact_point,
            "surface_normal": surface_normal, "major_axis": axes["major_axis"],
            "tool_x": rotation_matrix[:, 0], "tool_y": rotation_matrix[:, 1],
            "tool_z": tool_z, "rotation_matrix": rotation_matrix,
            "tool_z_floor_forced": tool_z_floor_forced,
            "tool_z_tilt_deg": axes["tool_z_tilt_deg"],
            "approach_rotation_deg": float(approach_rotation_deg),
            "closing_tilt_deg": float(closing_tilt_deg),
            "tool_z_tilt_adjusted": tool_z_tilt_adjusted,
            "tool_x_floor_angle_deg": axes["tool_x_floor_angle_deg"],
            "gripper_rotation_deg": gripper_rotation_deg,
            "eigenvalues": eigenvalues, "target_eigenvalues": target_eigenvalues,
            "horizontal_eigenvalues": horizontal_eigenvalues,
            "horizontal_shape_ratio": horizontal_shape_ratio,
            "major_axis_source": major_axis_source}

def _measure_grasp_width(points, centroid, rotation_matrix):
    """닫힘축."""
    tool_x = rotation_matrix[:, 0]
    centered = points - centroid

    def span(axis):
        proj = centered @ axis
        return float(np.percentile(proj, 100.0 - WIDTH_PERCENTILE)
                     - np.percentile(proj, WIDTH_PERCENTILE))

    width_mm = span(tool_x)
    part_extent_mm = max(span(rotation_matrix[:, i]) for i in range(3))

    trustworthy = part_extent_mm <= PART_MAX_EXTENT_MM
    warning = ""
    if not trustworthy:
        warning = (f"부위 extent가 {part_extent_mm:.0f}mm — 장난감 전체(약 100mm)보다 넓다. "
                   f"마스크가 부위 밖(몸통/배경)까지 번진 것으로 판단")
    return width_mm, {"part_extent_mm": part_extent_mm,
                      "trustworthy": trustworthy, "warning": warning}

def create_grasp_poses(result, part_tool_z_offset_mm=0.0,
                       grasp_z_offset_mm=GRASP_Z_OFFSET_MM,
                       pregrasp_distance_mm=PREGRASP_DISTANCE_MM,
                       grasp_depth_mm=GRASP_DEPTH_MM,
                       grasp_clearance_mm=GRASP_CLEARANCE_MM,
                       floor_z_mm=FLOOR_Z_MM, floor_clearance_mm=FLOOR_CLEARANCE_MM,
                       along_axis_deeper_mm=GRASP_ALONG_AXIS_DEEPER_MM):
    """접촉점에서 플랜지."""
    part_tool_z_offset_mm = max(0.0, float(part_tool_z_offset_mm))
    effective_grasp_depth_mm = grasp_depth_mm + part_tool_z_offset_mm
    tool_z = result["tool_z"]

    grasp_position = (result["contact_point"]
                      - tool_z * (grasp_z_offset_mm + grasp_clearance_mm)
                      + tool_z * effective_grasp_depth_mm)

    along_axis_shift_mm = float(along_axis_deeper_mm)
    axis_shift = np.zeros(3)
    if abs(along_axis_shift_mm) > 1.0e-9:
        axis = _normalize(result["major_axis"])
        outward = np.array([result["contact_point"][0], result["contact_point"][1], 0.0])
        if np.linalg.norm(outward) > 1.0e-9:
            outward = _normalize(outward)
            if np.dot(axis, outward) < 0.0:
                axis = -axis
        axis_shift = axis * along_axis_shift_mm
        grasp_position = grasp_position + axis_shift

    fingertip_grasp_position = grasp_position + tool_z * grasp_z_offset_mm

    minimum_fingertip_z = floor_z_mm + floor_clearance_mm
    floor_guard_raise_mm = max(0.0, minimum_fingertip_z - fingertip_grasp_position[2])
    if floor_guard_raise_mm > 0.0:
        grasp_position = grasp_position.copy()
        fingertip_grasp_position = fingertip_grasp_position.copy()
        grasp_position[2] += floor_guard_raise_mm
        fingertip_grasp_position[2] += floor_guard_raise_mm

    pregrasp_position = grasp_position - tool_z * pregrasp_distance_mm

    for name, position in (("grasp", grasp_position), ("pregrasp", pregrasp_position)):
        if not np.all(np.isfinite(position)):
            raise ValueError(f"{name} position이 유한하지 않습니다.")
        if not BASE_Z_MIN_MM < position[2] < BASE_Z_MAX_MM:
            raise ValueError(f"{name} Z가 작업 범위를 벗어났습니다: {position[2]:.2f}")

    euler = Rotation.from_matrix(result["rotation_matrix"]).as_euler("ZYZ", degrees=True)
    return {"grasp": np.concatenate((grasp_position, euler)),
            "pregrasp": np.concatenate((pregrasp_position, euler)),
            "fingertip_grasp_position": fingertip_grasp_position,
            "floor_guard_raise_mm": floor_guard_raise_mm,
            "minimum_fingertip_z": minimum_fingertip_z,
            "part_tool_z_offset_mm": part_tool_z_offset_mm,
            "effective_grasp_depth_mm": effective_grasp_depth_mm}

def compute_grasp(points_base_mm, part_names, part_num_points, target_part,
                  capture_posx, T_gripper2camera, part_confidences=None,
                  start_posx=None, **kwargs):
    """진입점."""
    points = np.asarray(points_base_mm, dtype=np.float64)
    if len(points) < MIN_POINT_COUNT:
        raise ValueError(f"PCA를 계산하기에 점이 부족합니다: {len(points)} < {MIN_POINT_COUNT}")

    base_to_flange = transforms.robot_pose_to_matrix(*capture_posx)
    camera_origin = (base_to_flange @ np.asarray(T_gripper2camera, dtype=np.float64))[:3, 3]
    start_pose = capture_posx if start_posx is None else start_posx
    current_rotation = transforms.robot_pose_to_matrix(*start_pose)[:3, :3]

    part_points = split_parts(part_names, part_num_points, points)
    target_key = resolve_target_part(part_points, target_part, part_confidences)
    grasp_points = points if target_key is None else part_points[target_key]
    if len(grasp_points) < MIN_POINT_COUNT:
        raise ValueError(
            f"target_part={target_key!r} 점이 부족합니다: {len(grasp_points)}")

    along_axis_deeper_mm = kwargs.pop("along_axis_deeper_mm", GRASP_ALONG_AXIS_DEEPER_MM)

    strategy = select_orientation_strategy(part_points, target_key, grasp_points)
    result = calculate_3d_pca(
        grasp_points, camera_origin, current_rotation,
        major_axis_points=strategy["points"],
        preferred_major_axis=strategy["preferred_axis"],
        prefer_preferred_axis=strategy["prefer_preferred_axis"], **kwargs)
    result["orientation_groups"] = strategy["groups"]
    result["target_part"] = target_key
    result["num_grasp_points"] = len(grasp_points)

    result["graspable"] = bool(result["grasp_width_mm"] < RG2_MAX_WIDTH_MM)
    if not result["graspable"]:
        raise GraspTooWideError(
            f"부위 '{target_key}'의 닫힘축 방향 폭 {result['grasp_width_mm']:.1f}mm가 "
            f"RG2 최대 개방 {RG2_MAX_WIDTH_MM:.0f}mm 이상입니다 — 이 부위는 잡을 수 없습니다.")

    part_tool_z_offset_mm = BODY_TOOL_Z_OFFSET_MM if target_key == "body" else 0.0
    poses = create_grasp_poses(result, part_tool_z_offset_mm=part_tool_z_offset_mm,
                               along_axis_deeper_mm=along_axis_deeper_mm)
    return result, poses

def format_result(result, poses):
    """로그 한 줄 요약."""
    return (f"[PCA] target={result['target_part']} 점={result['num_grasp_points']} "
            f"파지폭={result['grasp_width_mm']:.1f}mm"
            f"(부위extent {result['part_extent_mm']:.0f}mm) "
            f"장축출처={result['major_axis_source']} "
            f"XY분산비={result['horizontal_shape_ratio']:.2f} "
            f"묶음={result['orientation_groups']} "
            f"접근축회전={result.get('approach_rotation_deg', 0.0):+.0f}° "
            f"Tool Z기울기={result['tool_z_tilt_deg']:.1f}° "
            f"손목회전={result['gripper_rotation_deg']:.1f}° | "
            f"grasp(플랜지)={[round(v, 1) for v in poses['grasp']]} "
            f"pregrasp={[round(v, 1) for v in poses['pregrasp']]} | "
            f"손끝예상={[round(v, 1) for v in poses['fingertip_grasp_position']]} "
            f"(플랜지→손끝 {GRASP_Z_OFFSET_MM:.1f}mm 가정)")
