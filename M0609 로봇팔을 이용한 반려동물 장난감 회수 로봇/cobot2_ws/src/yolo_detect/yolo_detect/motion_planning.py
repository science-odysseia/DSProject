"""접근 경로 계획 — 자체 구현 RRT-Connect로 충돌 회피 경로를 뽑음."""
import math
import os
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32

from yolo_detect import config, octomap, path_planning, transforms

BASE_FRAME = "base_link"
MM_PER_M = 1000.0

WALL_NAMES = ("box_wall_1", "box_wall_2", "box_wall_3", "box_wall_4", "box_wall_5")


def _inflate(center_mm, radius_mm, resolution_mm):
    """점 하나를 반지름만큼 부풀린 칸 좌표 목록으로 바꿈."""
    n = int(math.ceil(float(radius_mm) / float(resolution_mm)))
    if n <= 0:
        return center_mm.reshape(1, 3)
    axis = np.arange(-n, n + 1) * resolution_mm
    gx, gy, gz = np.meshgrid(axis, axis, axis, indexing="ij")
    offsets = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    offsets = offsets[np.linalg.norm(offsets, axis=1) <= float(radius_mm)]
    return center_mm + offsets


class ApproachPathPlanner:
    """장애물을 등록하고 목표까지의 충돌 회피 경로를 계산함."""

    def __init__(self, node):
        self._node = node
        self._world = path_planning.World()
        self._planner = path_planning.JointSpacePlanner(self._world)
        self._n_applied_obstacles = 0
        self._last_plan_final_joints = None
        self._latest_joints = None
        self._octomap = None
        self._bridge = CvBridge()
        self._intrinsics = None
        self._last_octomap_update = 0.0
        self._octomap_frames = 0
        self._T_gripper2camera = (np.load(config.T_GRIPPER2CAMERA_PATH)
                                  if os.path.exists(config.T_GRIPPER2CAMERA_PATH) else None)
        node.create_subscription(
            JointState, f'/{config.ROBOT_ID}/joint_states', self._joint_state_cb, 10)
        node.create_subscription(
            CameraInfo, config.TOPIC_CAMERA_INFO_FILTERED, self._camera_info_cb,
            qos_profile_sensor_data)
        node.create_subscription(
            Image, config.TOPIC_DEPTH_FILTERED, self._depth_cb, qos_profile_sensor_data)
        viz_qos = QoSProfile(depth=config.MAP_QOS_DEPTH,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self._path_viz_pub = node.create_publisher(
            PoseArray, config.TOPIC_APPROACH_PATH, viz_qos)
        self._progress_pub = node.create_publisher(
            Int32, config.TOPIC_APPROACH_PROGRESS, viz_qos)

    def _joint_state_cb(self, msg):
        """현재 관절값을 캐시함."""
        self._latest_joints = dict(zip(list(msg.name), [float(v) for v in msg.position]))

    def _camera_info_cb(self, msg):
        """옥트리 갱신에 쓸 카메라 내부 파라미터를 캐시함."""
        self._intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def _depth_cb(self, msg):
        """마스킹된 depth 프레임을 점유 옥트리에 반영함."""
        if self._octomap is None or self._intrinsics is None or self._T_gripper2camera is None:
            return
        now = time.monotonic()
        if now - self._last_octomap_update < 1.0 / max(config.OCTOMAP_MAX_UPDATE_HZ, 1e-3):
            return
        q = self._current_joints()
        if q is None:
            return
        self._last_octomap_update = now

        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        posx = transforms.forward_kinematics_posx_mm_deg(path_planning.joints_dict(q))
        base2cam = transforms.robot_pose_to_matrix(*posx) @ self._T_gripper2camera
        points = self._backproject(depth)
        if len(points) == 0:
            return
        points_h = np.hstack([points, np.ones((len(points), 1))])
        base_points = (base2cam @ points_h.T).T[:, :3]
        n = self._octomap.insert_points(base_points, sensor_origin_mm=base2cam[:3, 3])
        self._octomap_frames += 1
        self._node.get_logger().info(
            f"[octomap] 프레임 {self._octomap_frames}회 반영 — 점유 칸 "
            f"{self._octomap.num_occupied}개(해상도 {self._octomap.resolution:.0f}mm, "
            f"이번 프레임 {n}점)", throttle_duration_sec=5.0)

    def _backproject(self, depth_mm):
        """depth 이미지를 카메라 좌표 3D 점으로 역투영함."""
        stride = config.OCTOMAP_PIXEL_STRIDE
        h, w = depth_mm.shape[:2]
        us, vs = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
        d = depth_mm[vs, us].astype(np.float32)
        valid = (d > config.OCTOMAP_MIN_RANGE_MM) & (d < config.OCTOMAP_MAX_RANGE_MM)
        if not np.any(valid):
            return np.zeros((0, 3), dtype=np.float64)
        fx, fy = self._intrinsics["fx"], self._intrinsics["fy"]
        ppx, ppy = self._intrinsics["ppx"], self._intrinsics["ppy"]
        u, v, dv = us[valid].astype(np.float32), vs[valid].astype(np.float32), d[valid]
        return np.stack([(u - ppx) * dv / fx, (v - ppy) * dv / fy, dv], axis=1).astype(np.float64)

    def _current_joints(self):
        """계획 시작점으로 쓸 현재 관절값. 없으면 None."""
        if self._latest_joints and all(j in self._latest_joints
                                       for j in path_planning.JOINT_ORDER):
            return path_planning.joints_array(self._latest_joints)
        if self._last_plan_final_joints:
            return path_planning.joints_array(self._last_plan_final_joints)
        return None

    def register_box_walls(self, corner1_mm, corner2_mm, corner3_mm, interior_depth_mm,
                           thickness_mm, ceiling_margin_mm=None, floor_margin_mm=None,
                           side_margin_mm=None):
        """박스의 5개 벽을 충돌 판정용 직육면체로 등록함."""
        if ceiling_margin_mm is None:
            ceiling_margin_mm = config.BOX_CEILING_MARGIN_MM
        if floor_margin_mm is None:
            floor_margin_mm = config.BOX_FLOOR_MARGIN_MM
        if side_margin_mm is None:
            side_margin_mm = config.BOX_SIDE_MARGIN_MM

        p1 = np.asarray(corner1_mm, dtype=np.float64)
        p2 = np.asarray(corner2_mm, dtype=np.float64)
        p3 = np.asarray(corner3_mm, dtype=np.float64)
        ex = p2 - p1
        width = float(np.linalg.norm(ex))
        ex = ex / width
        ey = p3 - p1
        height = float(np.linalg.norm(ey))
        ey = ey / height
        ez = np.cross(ex, ey)
        ez = ez / np.linalg.norm(ez)
        rot = np.column_stack([ex, ey, ez])

        t = float(thickness_mm)
        exterior_depth = float(interior_depth_mm) + t
        ceil_th = t + ceiling_margin_mm
        floor_th = t + floor_margin_mm
        side_th = t + side_margin_mm
        ceil_c = -t / 2 + ceiling_margin_mm / 2
        floor_c = height + t / 2 - floor_margin_mm / 2
        side_a_c = -t / 2 + side_margin_mm / 2
        side_b_c = width + t / 2 - side_margin_mm / 2

        specs = [
            ((width / 2, height / 2, float(interior_depth_mm) + t / 2), (width, height, t)),
            ((width / 2, ceil_c, exterior_depth / 2), (width, ceil_th, exterior_depth)),
            ((width / 2, floor_c, exterior_depth / 2), (width, floor_th, exterior_depth)),
            ((side_a_c, height / 2, exterior_depth / 2), (side_th, height, exterior_depth)),
            ((side_b_c, height / 2, exterior_depth / 2), (side_th, height, exterior_depth)),
        ]

        self._world.clear_walls()
        for name, (local_c, dims) in zip(WALL_NAMES, specs):
            center = p1 + rot @ np.asarray(local_c, dtype=np.float64)
            half = np.asarray(dims, dtype=np.float64) / 2.0
            self._world.add_wall(path_planning.Obb(center, rot, half, name))

        self._octomap = octomap.covering_octomap(
            corner1_mm, corner2_mm, corner3_mm, interior_depth_mm,
            resolution_mm=config.OCTOMAP_RESOLUTION_MM, margin_mm=config.OCTOMAP_MARGIN_MM)
        self._octomap.min_range = config.OCTOMAP_MIN_RANGE_MM
        self._octomap.max_range = config.OCTOMAP_MAX_RANGE_MM
        self._world.set_octomap(self._octomap)

        self._node.get_logger().info(
            f"[motion_planning] 박스 벽 {len(self._world.walls)}개 + 점유 옥트리 등록"
            f"(폭 {width:.1f} x 높이 {height:.1f} x 깊이 {float(interior_depth_mm):.1f}mm, "
            f"옥트리 {self._octomap.size:.0f}mm 정육면체 / 해상도 "
            f"{self._octomap.resolution:.0f}mm / 깊이 {self._octomap.depth}단계)")
        return True

    def purge_stale_obstacles(self):
        """등록된 장애물을 모두 지움."""
        self._world.clear_spheres()
        if self._octomap is not None:
            self._octomap.clear()
        self._n_applied_obstacles = 0

    def _apply_obstacles(self, obstacle_points_mm):
        """정찰에서 뽑은 장애물 점을 점유 옥트리에 반영함."""
        points = list(obstacle_points_mm or [])
        if not points or self._octomap is None:
            self._n_applied_obstacles = 0
            return True
        total = 0
        for p in points:
            radius = float(p[3]) if len(p) >= 4 else config.MAP_OBSTACLE_SAFETY_RADIUS_MM
            total += self._octomap.insert_points(
                _inflate(np.asarray(p[:3], dtype=np.float64), radius,
                         self._octomap.resolution))
        self._n_applied_obstacles = len(points)
        self._node.get_logger().info(
            f"[motion_planning] 정찰 장애물 {len(points)}개를 옥트리에 반영 — "
            f"점유 칸 {self._octomap.num_occupied}개")
        return True

    def clear_octomap(self, settle_sec=None):
        """계획 직전에 옥트리를 비우고 새 depth 프레임이 들어올 때까지 기다림."""
        if self._octomap is None:
            return False
        limit = config.OCTOMAP_CLEAR_SETTLE_SEC if settle_sec is None else float(settle_sec)
        before = self._octomap_frames
        self._octomap.clear()
        self._last_octomap_update = 0.0
        self._node.get_logger().info("[octomap] 소거 완료 — 재축적 대기 시작")

        deadline = time.monotonic() + limit
        while time.monotonic() < deadline:
            if (self._octomap_frames > before
                    and self._octomap.num_occupied >= config.OCTOMAP_MIN_OCCUPIED_CELLS):
                self._node.get_logger().info(
                    f"[octomap] 재축적 확인 — 점유 칸 {self._octomap.num_occupied}개 "
                    f"({limit - (deadline - time.monotonic()):.1f}초)")
                return True
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._node.get_logger().warn(
            f"[octomap] ⚠️ 옥트리가 안 찼음(점유 칸 {self._octomap.num_occupied}개) — "
            f"실시간 장애물 없이 계획한다(박스 5벽은 유효)")
        return False

    def _goal_orientation(self, target_xyz_mm, look_at_xyz_mm, T_gripper2camera):
        """접근 목표에서 카메라가 물체를 보는 자세의 방향각. 못 구하면 None."""
        if look_at_xyz_mm is None or T_gripper2camera is None:
            return None
        try:
            return transforms.look_at_orientation(
                target_xyz_mm, look_at_xyz_mm, T_gripper2camera)
        except ValueError:
            quat, _ = transforms.camera_look_at_tool0_quat(
                target_xyz_mm, look_at_xyz_mm, T_gripper2camera)
            r = (Rotation.from_quat(quat).as_matrix()
                 @ transforms._TOOL_TO_DSR_ROTATION_CORRECTION)
            rx, ry, rz = Rotation.from_matrix(r).as_euler('ZYZ', degrees=True)
            return float(rx), float(ry), float(rz)

    def _plan_to_joints(self, q_goal, label):
        """현재 자세에서 목표 관절값까지 계획해 posx 리스트로 돌려줌."""
        q_start = self._current_joints()
        if q_start is None:
            self._node.get_logger().error(
                f"[motion_planning] {label}: 현재 관절값을 못 받아 계획 불가")
            return None

        path = self._planner.plan(q_start, q_goal)
        if path is None:
            self._node.get_logger().error(
                f"[motion_planning] {label} 계획 실패 — {self._planner.last_reason}")
            return None

        self._last_plan_final_joints = path_planning.joints_dict(path[-1])
        waypoints = path_planning.path_to_posx(path)
        self._node.get_logger().info(
            f"[motion_planning] {label} 계획 성공: 웨이포인트 {len(waypoints)}개 "
            f"({self._planner.last_reason}, 장애물 {len(self._world.spheres)}개 + "
            f"벽 {len(self._world.walls)}개)")
        return waypoints

    def _ik_or_none(self, target_posx, label):
        """목표 posx의 관절해를 현재 자세에서 이어 품."""
        q_start = self._current_joints()
        if q_start is None:
            self._node.get_logger().error(f"[motion_planning] {label}: 현재 관절값 없음")
            return None
        seed = path_planning.joints_dict(q_start)
        joints = transforms.inverse_kinematics_joints(list(target_posx), seed)
        if joints is None:
            self._node.get_logger().error(
                f"[motion_planning] {label}: 목표 자세 IK 수렴 실패 — 도달 불가")
            return None
        return path_planning.joints_array(joints)

    def plan_dense_waypoints_mm(self, target_xyz_mm, obstacle_points_mm, goal_tolerance_mm=10.0,
                                look_at_xyz_mm=None, T_gripper2camera=None,
                                orientation_tolerance_deg=None):
        """목표 좌표까지의 충돌 회피 경로를 posx 리스트로 계산함."""
        self._apply_obstacles(obstacle_points_mm)

        rpy = self._goal_orientation(target_xyz_mm, look_at_xyz_mm, T_gripper2camera)
        if rpy is None:
            q_start = self._current_joints()
            if q_start is None:
                self._node.get_logger().error("[motion_planning] 접근: 현재 관절값 없음")
                return None
            cur = transforms.forward_kinematics_posx_mm_deg(
                path_planning.joints_dict(q_start))
            rpy = (cur[3], cur[4], cur[5])

        target_posx = [float(target_xyz_mm[0]), float(target_xyz_mm[1]),
                       float(target_xyz_mm[2]), rpy[0], rpy[1], rpy[2]]
        q_goal = self._ik_or_none(target_posx, "접근")
        if q_goal is None:
            return None
        return self._plan_to_joints(q_goal, "접근 경로")

    def plan_dense_waypoints_to_joints(self, target_joints_by_name, tolerance_rad=None):
        """관절공간 목표로 계획함."""
        try:
            q_goal = path_planning.joints_array(target_joints_by_name)
        except KeyError:
            self._node.get_logger().error("[motion_planning] 파지 경로: 관절 이름이 모자람")
            return None
        return self._plan_to_joints(q_goal, "파지 경로(관절목표)")

    def plan_dense_waypoints_to_posx(self, target_posx, position_tolerance_mm=None,
                                     orientation_tolerance_deg=None):
        """목표 posx의 위치와 방향을 둘 다 지키는 경로를 계획함."""
        q_goal = self._ik_or_none(target_posx, "파지 경로")
        if q_goal is None:
            return None
        return self._plan_to_joints(q_goal, "파지 경로")

    def publish_path_for_viz(self, waypoints_mm_deg):
        """계획된 경로를 웹 3D 뷰어용으로 발행함."""
        msg = PoseArray()
        msg.header.frame_id = BASE_FRAME
        msg.header.stamp = self._node.get_clock().now().to_msg()
        for wp in waypoints_mm_deg:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (
                float(wp[0]), float(wp[1]), float(wp[2]))
            if len(wp) >= 6:
                q = Rotation.from_euler('ZYZ', [wp[3], wp[4], wp[5]], degrees=True).as_quat()
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w) = (
                    float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            else:
                pose.orientation.w = 1.0
            msg.poses.append(pose)
        self._path_viz_pub.publish(msg)
        self.publish_progress(0)
        self._node.get_logger().info(
            f"[motion_planning] 웹 시각화용 접근 경로 발행: {len(msg.poses)}개 지점")

    def publish_progress(self, reached_count):
        """도달을 마친 웨이포인트 개수를 발행함."""
        self._progress_pub.publish(Int32(data=int(reached_count)))

    def check_joint_state_valid(self, joint_positions_by_name):
        """관절자세 하나가 등록된 장애물과 충돌하는지 봄."""
        try:
            q = path_planning.joints_array(joint_positions_by_name)
        except KeyError:
            return None, "관절 이름이 모자라 충돌검사 불가"
        if not self._planner.in_limits(q):
            return False, "관절 한계 초과"
        hit = self._planner.state_hit(q)
        if hit is None:
            return True, "충돌 없음"
        return False, hit

    @staticmethod
    def _resolve_sample_count(requested, dist_mm=0.0, rot_deg=0.0):
        """구간 크기에 맞춰 실제로 쓸 샘플 수를 정함."""
        need_mm = math.ceil(dist_mm / config.SEGMENT_VALIDATION_MAX_STEP_MM) if dist_mm > 0 else 0
        need_deg = math.ceil(rot_deg / config.SEGMENT_VALIDATION_MAX_STEP_DEG) if rot_deg > 0 else 0
        n = max(int(requested), need_mm, need_deg, 1)
        return min(n, config.SEGMENT_VALIDATION_MAX_SAMPLES)

    def validate_in_place_rotation(self, from_posx, to_posx, seed_joints=None, samples=None):
        """제자리 재조정을 실행 전에 검사해 안전하게 회전할 수 있는 비율을 돌려줌."""
        if samples is None:
            samples = config.REORIENT_VALIDATION_SAMPLES
        if seed_joints is None:
            seed_joints = self._last_plan_final_joints or self._latest_joints
        if seed_joints is None:
            return None, "IK seed 없음 — 검사 불가"

        total_deg = transforms.rotation_geodesic_error_deg(*from_posx[3:6], *to_posx[3:6])
        if total_deg <= config.ORIENTATION_TOLERANCE_DEG:
            return 1.0, f"회전량 {total_deg:.1f}° — 허용오차 이내라 검사 생략"

        samples = self._resolve_sample_count(samples, rot_deg=total_deg)

        seed = dict(seed_joints)
        last_safe = 0.0
        for i in range(1, samples + 1):
            f = i / samples
            posx = transforms.slerp_posx(from_posx, to_posx, f)
            joints = transforms.inverse_kinematics_joints(posx, seed)
            if joints is None:
                return last_safe, (f"{f * 100:.0f}% 지점({total_deg * f:.1f}°)에서 IK 수렴 실패 "
                                   f"— 그 자세는 도달 불가로 판단, 안전구간 {last_safe * 100:.0f}%")
            seed = joints
            valid, why = self.check_joint_state_valid(joints)
            if valid is None:
                return None, why
            if not valid:
                return last_safe, (f"{f * 100:.0f}% 지점({total_deg * f:.1f}° 회전)에서 충돌: {why} "
                                   f"— 안전구간 {last_safe * 100:.0f}%({total_deg * last_safe:.1f}°)")
            last_safe = f

        return 1.0, f"{samples}개 샘플 전부 충돌 없음(총 {total_deg:.1f}° 회전)"

    def validate_posx_segment(self, from_posx, to_posx, seed_joints=None, samples=None):
        """위치가 바뀌는 직선 구간 하나를 실행 전에 검사함."""
        if samples is None:
            samples = config.REORIENT_VALIDATION_SAMPLES
        if seed_joints is None:
            seed_joints = self._last_plan_final_joints or self._latest_joints
        if seed_joints is None:
            return None, "IK seed 없음 — 검사 불가"

        dist_mm = float(np.linalg.norm(np.asarray(to_posx[:3], dtype=np.float64)
                                       - np.asarray(from_posx[:3], dtype=np.float64)))
        rot_deg = transforms.rotation_geodesic_error_deg(*from_posx[3:6], *to_posx[3:6])
        samples = self._resolve_sample_count(samples, dist_mm=dist_mm, rot_deg=rot_deg)

        seed = dict(seed_joints)
        last_safe = 0.0
        for i in range(1, samples + 1):
            f = i / samples
            posx = transforms.slerp_posx(from_posx, to_posx, f)
            joints = transforms.inverse_kinematics_joints(posx, seed)
            if joints is None:
                return last_safe, (f"{f * 100:.0f}% 지점({dist_mm * f:.0f}mm)에서 IK 수렴 실패 "
                                   f"— 도달 불가로 판단, 안전구간 {last_safe * 100:.0f}%")
            seed = joints
            valid, why = self.check_joint_state_valid(joints)
            if valid is None:
                return None, why
            if not valid:
                return last_safe, (f"{f * 100:.0f}% 지점({dist_mm * f:.0f}mm 이동)에서 충돌: {why} "
                                   f"— 안전구간 {last_safe * 100:.0f}%({dist_mm * last_safe:.0f}mm)")
            last_safe = f

        return 1.0, (f"{samples}개 샘플 전부 충돌 없음"
                     f"(이동 {dist_mm:.0f}mm, 회전 {rot_deg:.1f}°)")
