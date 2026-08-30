"""정찰 -> 탐지 물체 접근 -> 재촬영 -> 파지 -> 복귀까지 자체완결 실기 스크립트."""
import datetime
import glob
import json
import os
import sys
import threading
import time
import traceback
import types

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import DR_init
from dsr_msgs2.srv import GetCurrentPosx
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

from yolo_detect import config, grasp_pca, rg2_gripper, transforms
from yolo_detect.motion_planning import ApproachPathPlanner
from yolo_detect_msgs.action import RunRecon
from yolo_detect_msgs.msg import Waypoint

VELOCITY, ACC = 30, 30
ARRIVAL_TIMEOUT_SEC = 20.0
HOME_ARRIVAL_TIMEOUT_SEC = 30.0
RESULTS_DIR = config.RECON_RESULTS_DIR

def _recorded_waypoint_joints():
    """정찰 웨이포인트별 실측 관절값을 스냅샷에서 읽어 번호별로 돌려줌."""
    wps = [[p["x"], p["y"], p["z"]] for p in config.RECON_WAYPOINTS]
    out = {}
    root = config.SNAPSHOT_SAVE_DIR
    for snap in sorted(glob.glob(os.path.join(root, "2*"))):
        if not os.path.isdir(snap):
            continue
        for idx in range(len(wps)):
            meta_path = os.path.join(snap, f"waypoint_{idx}_meta.json")
            if not os.path.exists(meta_path):
                continue
            try:
                meta = json.load(open(meta_path))
            except (OSError, ValueError):
                continue
            js = meta.get("joint_state") or {}
            if not js.get("name") or not js.get("position"):
                continue
            gap = float(np.linalg.norm(
                np.asarray(meta["posx_mm_deg"][:3], dtype=np.float64)
                - np.asarray(wps[idx], dtype=np.float64)))
            if gap > config.RECON_SEED_POSX_TOLERANCE_MM:
                continue
            out[idx] = (os.path.basename(snap), dict(zip(js["name"], js["position"])))
    return out

def _branch_drift_deg(from_posx, to_posx, seed_joints, recorded_end_joints):
    """구간을 seed에서 이어 푼 도착 관절값 vs 끝점 실측 관절값의 최대 관절차."""
    seed = dict(seed_joints)
    samples = config.GRASP_VALIDATION_SAMPLES
    for i in range(1, samples + 1):
        posx = transforms.slerp_posx(from_posx, to_posx, i / samples)
        nxt = transforms.inverse_kinematics_joints(posx, seed)
        if nxt is None:
            return None
        seed = nxt
    shared = [k for k in recorded_end_joints if k in seed]
    if not shared:
        return None
    return max(abs(np.degrees(seed[k] - recorded_end_joints[k])) for k in shared)

class RobotControlNode(Node):
    """메인 노드."""

    def __init__(self):
        super().__init__("robot_control_node")
        self._recon_client = ActionClient(self, RunRecon, f"/{config.ACTION_RUN_RECON}")
        self._approach_planner = ApproachPathPlanner(self)
        self._approach_planner.register_box_walls(
            config.BOX_CORNER_1_MM, config.BOX_CORNER_2_MM, config.BOX_CORNER_3_MM,
            config.BOX_DEPTH_MM, config.BOX_WALL_THICKNESS_MM)
        self._T_gripper2camera = np.load(config.T_GRIPPER2CAMERA_PATH)
        self._latest_joints = None
        self._grasp_path_final_joints = None
        self.create_subscription(
            JointState, f"/{config.ROBOT_ID}/joint_states", self._joint_state_cb, 10)

    def _joint_state_cb(self, msg):
        self._latest_joints = dict(zip(msg.name, msg.position))

    def run_recon(self, executor):
        """정찰 액션 호출 + 결과 대기."""
        self.get_logger().info(
            f"object_detect_node action server({config.ACTION_RUN_RECON}) 대기 중...")
        if not self._recon_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("object_detect_node action server를 찾을 수 없음 — 켜져 있는지 확인")
            return None

        goal = RunRecon.Goal()
        for wp in config.RECON_WAYPOINTS:
            msg = Waypoint()
            msg.x, msg.y, msg.z = wp["x"], wp["y"], wp["z"]
            msg.rx, msg.ry, msg.rz = wp["rx"], wp["ry"], wp["rz"]
            msg.has_orientation = True
            goal.waypoints.append(msg)

        self.get_logger().info(f"RunRecon goal 전송: 웨이포인트 {len(goal.waypoints)}개")
        send_future = self._recon_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, executor=executor)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("goal이 거부됨")
            return None

        self.get_logger().info("goal accepted — 정찰 진행 중 (mover 스레드가 이동시킴)")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, executor=executor)
        return result_future.result().result

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"stage={fb.stage}  snapshots={fb.snapshots_taken}/{fb.snapshots_total}")

    def _save_results_to_file(self, result):
        """정찰 결과를 recon_results/recon_<타임스탬프>.txt로 저장."""
        col_id, col_cls, col_num = 16, 8, 10

        def header_row(id_label):
            return (
                f"# {id_label:<{col_id - 2}}{'class':<{col_cls}}{'x_mm':<{col_num}}"
                f"{'y_mm':<{col_num}}{'z_mm':<{col_num}}confidence\n"
            )

        os.makedirs(RESULTS_DIR, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(RESULTS_DIR, f"recon_{timestamp}.txt")
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(f"# 정찰 결과 — {timestamp}\n")
            f.write(f"# success={result.success}, 물체 {len(result.objects)}개\n")
            f.write(header_row("object_id"))
            for obj in result.objects:
                f.write(
                    f"{obj.object_id:<{col_id}}{obj.object_class:<{col_cls}}"
                    f"{obj.coords_base[0]:<{col_num}.1f}{obj.coords_base[1]:<{col_num}.1f}"
                    f"{obj.coords_base[2]:<{col_num}.1f}{obj.confidence:.3f}\n"
                )
            f.write(f"\n# 원시 탐지(클러스터링 전) — {len(result.raw_detections)}건, 웨이포인트별\n")
            f.write(header_row("waypoint_idx"))
            raw_sorted = sorted(result.raw_detections, key=lambda d: d.waypoint_idx)
            for d in raw_sorted:
                f.write(
                    f"{d.waypoint_idx:<{col_id}}{d.object_class:<{col_cls}}"
                    f"{d.coords_base[0]:<{col_num}.1f}{d.coords_base[1]:<{col_num}.1f}"
                    f"{d.coords_base[2]:<{col_num}.1f}{d.confidence:.3f}\n"
                )
        self.get_logger().info(f"결과를 파일로 저장함: {out_path}")

    def set_target_filter(self, target_obj):
        """depth_target_filter_node에 타겟 좌표를 설정해 마스킹 활성화."""
        client = self.create_client(SetParameters, "/depth_target_filter_node/set_parameters")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "depth_target_filter_node/set_parameters 서비스를 찾을 수 없음 — "
                "그 노드가 안 떠 있으면 depth 마스킹이 동작하지 않음")
            return False

        x, y, z = target_obj.coords_base

        def _param(name, value, ptype, field):
            p = Parameter()
            p.name = name
            pv = ParameterValue()
            pv.type = ptype
            setattr(pv, field, value)
            p.value = pv
            return p

        req = SetParameters.Request()
        req.parameters = [
            _param("target_x_mm", float(x), ParameterType.PARAMETER_DOUBLE, "double_value"),
            _param("target_y_mm", float(y), ParameterType.PARAMETER_DOUBLE, "double_value"),
            _param("target_z_mm", float(z), ParameterType.PARAMETER_DOUBLE, "double_value"),
            _param("enabled", True, ParameterType.PARAMETER_BOOL, "bool_value"),
        ]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        ok = future.done() and all(r.successful for r in future.result().results)
        if ok:
            self.get_logger().info(
                f"depth_target_filter_node 타겟 설정 완료: [{x:.1f}, {y:.1f}, {z:.1f}]mm, "
                f"반경 {config.DEPTH_TARGET_MASK_RADIUS_MM}mm")
        else:
            self.get_logger().error("depth_target_filter_node 파라미터 설정 실패")
        return ok

    def plan_approach(self, obj, all_objects, static_obstacle_points):
        """접근경로 계획."""
        x, y, z = obj.coords_base
        target_xyz = (x, y, z + config.OBJECT_APPROACH_OFFSET_MM)
        obstacle_points = [
            (o.coords_base[0], o.coords_base[1], o.coords_base[2], config.OBSTACLE_SAFETY_RADIUS_MM)
            for o in all_objects if o.object_id != obj.object_id
        ] + [
            (p.x, p.y, p.z, config.MAP_OBSTACLE_SAFETY_RADIUS_MM)
            for p in static_obstacle_points
        ]
        self.get_logger().info(
            f"탐지 좌표 [{x:.1f}, {y:.1f}, {z:.1f}] -> 접근경로 계획 "
            f"(목표 z+{config.OBJECT_APPROACH_OFFSET_MM:.0f}mm, 장애물 {len(obstacle_points)}개)")
        min_aim_mm = transforms.min_camera_aim_distance_mm(self._T_gripper2camera)
        if config.OBJECT_APPROACH_OFFSET_MM < min_aim_mm:
            self.get_logger().warn(
                f"⚠️ 접근 오프셋 {config.OBJECT_APPROACH_OFFSET_MM:.0f}mm가 카메라 지향 하한 "
                f"{min_aim_mm:.0f}mm보다 작다 — 도착해도 카메라 조준이 수렴하지 않아 "
                f"재조정/재촬영이 실패할 수 있다(config.OBJECT_APPROACH_OFFSET_MM 주석 참고)")
        ladder = config.APPROACH_ORIENTATION_TOLERANCE_LADDER_DEG
        for i, tol in enumerate(ladder):
            points = self._approach_planner.plan_dense_waypoints_mm(
                target_xyz, obstacle_points, config.GOAL_TOLERANCE_MM,
                look_at_xyz_mm=obj.coords_base, T_gripper2camera=self._T_gripper2camera,
                orientation_tolerance_deg=tol)
            if points is not None:
                self.get_logger().info(f"방향제약 {tol:.0f}°로 계획 성공 (사다리 {i + 1}/{len(ladder)})")
                return points
            if i + 1 < len(ladder):
                self.get_logger().warning(
                    f"방향제약 {tol:.0f}° 계획 실패 — {ladder[i + 1]:.0f}°로 완화해 재시도")
        return None

    def check_pose_reachable(self, posx):
        """posx 자세 하나가 등록된 장애물과 충돌하는지 봄."""
        if self._latest_joints is None:
            return None, f"/{config.ROBOT_ID}/joint_states 수신 없음"
        joints = transforms.inverse_kinematics_joints(list(posx), dict(self._latest_joints))
        if joints is None:
            return None, "IK 수렴 실패(도달 불가일 수 있음)"
        return self._approach_planner.check_joint_state_valid(joints)

    def validate_grasp_path(self, start_posx, poses, skip_first_segment=False):
        """파지 이동."""
        if not config.GRASP_VALIDATE_BEFORE_EXECUTE:
            self.get_logger().warn(
                "⚠️ 파지 이동 사전검사가 꺼져 있음(GRASP_VALIDATE_BEFORE_EXECUTE=False) — "
                "박스 안에서 검사 없이 뻗는다")
            return True, "검사 비활성화"
        if self._latest_joints is None:
            return False, (f"/{config.ROBOT_ID}/joint_states 수신 없음 — IK seed가 없어 "
                           f"검사 불가")

        seed = dict(self._latest_joints)
        segments = [("도착->pregrasp", list(start_posx), list(poses["pregrasp"])),
                    ("pregrasp->grasp", list(poses["pregrasp"]), list(poses["grasp"]))]
        if skip_first_segment:
            self.get_logger().info(
                "  ⏭ 도착->pregrasp: 계획된 경로라 직선 검사 생략(계획이 이미 검사됨)")
            seed = transforms.inverse_kinematics_joints(list(poses["pregrasp"]), seed) or seed
            segments = segments[1:]
        for label, frm, to in segments:
            fraction, why = self._approach_planner.validate_posx_segment(
                frm, to, seed_joints=seed, samples=config.GRASP_VALIDATION_SAMPLES)
            if fraction is None:
                return False, f"{label} 검사 불가({why})"
            if fraction < 1.0:
                return False, f"{label} 충돌 예상 — {why}"
            self.get_logger().info(f"  ✅ {label}: {why}")
            seed = transforms.inverse_kinematics_joints(to, seed) or seed
        self._grasp_path_final_joints = dict(seed)
        return True, "두 구간 전부 충돌 없음"

    def preflight_validate_fixed_segments(self):
        """정찰 구간의 고정 좌표 이동을 움직이기 전에 한 번 검사함."""
        if not config.RECON_PREFLIGHT_VALIDATE:
            return
        recorded = _recorded_waypoint_joints()
        if self._latest_joints is None and not recorded:
            self.get_logger().warn(
                f"정찰 구간 사전검사 생략 — 실측 관절값 스냅샷도 없고 "
                f"/{config.ROBOT_ID}/joint_states 수신도 없어 IK seed를 만들 수 없음")
            return

        h = config.HOME_POSE
        home = [h["x"], h["y"], h["z"], h["rx"], h["ry"], h["rz"]]
        w = config.HOME_TO_OBJECT_WAYPOINT
        waypoint = [w["x"], w["y"], w["z"], w["rx"], w["ry"], w["rz"]]
        w2 = config.HOME_TO_OBJECT_WAYPOINT_2
        waypoint2 = (None if w2 is None else
                     [w2["x"], w2["y"], w2["z"], w2["rx"], w2["ry"], w2["rz"]])
        wps = [[p["x"], p["y"], p["z"], p["rx"], p["ry"], p["rz"]]
               for p in config.RECON_WAYPOINTS]

        segments = [("홈->WP0", home, wps[0], None, 0)] if wps else []
        segments += [(f"WP{i}->WP{i + 1}", wps[i], wps[i + 1], i, i + 1)
                     for i in range(len(wps) - 1)]
        segments.append(("홈->경유점", home, waypoint, None, None))
        if waypoint2 is not None:
            segments.append(("경유점1->경유점2", waypoint, waypoint2, None, None))

        src = (f"실측 관절값 WP{sorted(recorded)}" if recorded
               else "실측 관절값 없음 — 현재 관절값 폴백")
        self.get_logger().info(f"정찰 구간 사전 충돌검사(고정 좌표 구간, 경고만) — seed: {src}")
        n_bad = 0
        for label, frm, to, from_idx, to_idx in segments:
            seed, seed_kind = None, ""
            if from_idx is not None and from_idx in recorded:
                seed, seed_kind = dict(recorded[from_idx][1]), "실측"
            elif self._latest_joints is not None:
                seed, seed_kind = (
                    transforms.inverse_kinematics_joints(frm, dict(self._latest_joints)), "현재")
            if seed is None:
                self.get_logger().warn(f"  ❓ {label}: 시작 자세 IK seed를 만들 수 없음 — 검사 불가")
                continue

            fraction, why = self._approach_planner.validate_posx_segment(
                frm, to, seed_joints=seed, samples=config.GRASP_VALIDATION_SAMPLES)

            drift = None
            if to_idx is not None and to_idx in recorded:
                drift = _branch_drift_deg(frm, to, seed, recorded[to_idx][1])
            drift_s = f", 분기드리프트 {drift:.1f}°" if drift is not None else ""
            unreliable = (drift is not None
                          and drift > config.RECON_SEED_BRANCH_DRIFT_MAX_DEG)

            if fraction is None:
                self.get_logger().warn(f"  ❓ {label}[{seed_kind}]: 검사 불가({why}){drift_s}")
            elif unreliable:
                self.get_logger().warn(
                    f"  ❓ {label}[{seed_kind}]: 판정 신뢰 불가 — IK가 로봇과 다른 분기로 "
                    f"갔다(드리프트 {drift:.1f}° > "
                    f"{config.RECON_SEED_BRANCH_DRIFT_MAX_DEG:.0f}°). 참고용 결과: {why}")
            elif fraction < 1.0:
                n_bad += 1
                self.get_logger().warn(f"  🔴 {label}[{seed_kind}]: {why}{drift_s}")
            else:
                self.get_logger().info(f"  ✅ {label}[{seed_kind}]: {why}{drift_s}")
        if n_bad:
            self.get_logger().warn(
                f"⚠️ 정찰 구간 {n_bad}개가 박스와 충돌 예상 — 그대로 진행하지만(경고만), "
                f"해당 좌표를 조그로 재측정하는 것을 검토할 것. seed가 [실측]이면 로봇의 "
                f"실제 분기로 검사한 결과이므로 분기 탓으로 넘기지 말 것")

    def validate_return_path(self, poses):
        """복귀 이동."""
        if not config.RETURN_VALIDATE_BEFORE_EXECUTE:
            self.get_logger().warn(
                "⚠️ 복귀 경로 사전검사가 꺼져 있음(RETURN_VALIDATE_BEFORE_EXECUTE=False)")
            return True, "검사 비활성화"
        if self._latest_joints is None:
            return False, f"/{config.ROBOT_ID}/joint_states 수신 없음 — IK seed가 없어 검사 불가"

        w = config.HOME_TO_OBJECT_WAYPOINT
        waypoint = [w["x"], w["y"], w["z"], w["rx"], w["ry"], w["rz"]]
        h = config.HOME_POSE
        home = [h["x"], h["y"], h["z"], h["rx"], h["ry"], h["rz"]]
        grasp = list(poses["grasp"])
        pregrasp = list(poses["pregrasp"])

        seed = getattr(self, "_grasp_path_final_joints", None)
        if seed is None:
            seed = transforms.inverse_kinematics_joints(grasp, dict(self._latest_joints))
        if seed is None:
            return False, "grasp 자세 IK 수렴 실패 — 복귀 검사 불가"
        seed = dict(seed)

        segments = (("grasp->pregrasp", grasp, pregrasp),
                    ("pregrasp->경유점", pregrasp, waypoint),
                    ("경유점->홈", waypoint, home))
        for label, frm, to in segments:
            fraction, why = self._approach_planner.validate_posx_segment(
                frm, to, seed_joints=seed, samples=config.GRASP_VALIDATION_SAMPLES)
            if fraction is None:
                return False, f"{label} 검사 불가({why})"
            if fraction < 1.0:
                return False, f"{label} 충돌 예상 — {why}"
            self.get_logger().info(f"  ✅ {label}: {why}")
            seed = transforms.inverse_kinematics_joints(to, seed) or seed
        return True, "복귀 3구간 전부 충돌 없음"

    def validated_reorient_fraction(self, from_posx, to_posx):
        """제자리 재조정을 실행 전에 검사해 "안전하게 회전할 수 있는 비율"을 돌려줌."""
        if not config.REORIENT_VALIDATE_BEFORE_EXECUTE:
            self.get_logger().warn("⚠️ 재조정 사전검사가 꺼져 있음(REORIENT_VALIDATE_BEFORE_EXECUTE=False)")
            return 1.0
        if self._latest_joints is None:
            self.get_logger().error(
                f"재조정 사전검사 불가 — /{config.ROBOT_ID}/joint_states 수신 없음. 재조정을 건너뜀")
            return 0.0

        fraction, why = self._approach_planner.validate_in_place_rotation(
            from_posx, to_posx, self._latest_joints)
        if fraction is None:
            self.get_logger().error(f"재조정 사전검사 실패({why}) — 안전하게 재조정을 건너뜀")
            return 0.0
        if fraction >= 1.0:
            self.get_logger().info(f"재조정 사전검사 통과: {why}")
            return 1.0

        self.get_logger().warn(f"⚠️ 재조정 사전검사에서 충돌 예상 — {why}")
        if not config.REORIENT_ALLOW_PARTIAL or fraction < config.REORIENT_MIN_PARTIAL_FRACTION:
            self.get_logger().warn("재조정을 건너뜀(도착 자세 그대로 유지 — 90° 느슨제약은 이미 만족한 상태)")
            return 0.0
        self.get_logger().warn(f"안전한 {fraction * 100:.0f}%만 부분 재조정함")
        return fraction

def _query_posx(mover_node, posx_client, executor):
    """현재 posx를 조회함."""
    if not posx_client.wait_for_service(timeout_sec=1.0):
        return None
    future = posx_client.call_async(GetCurrentPosx.Request())
    rclpy.spin_until_future_complete(mover_node, future, executor=executor, timeout_sec=2.0)
    if not future.done():
        return None
    result = future.result()
    if result is None or not result.success or not result.task_pos_info:
        return None
    return list(result.task_pos_info[0].data[:6])

def _wait_for_arrival(mover_node, posx_client, target_pos, executor, check_orientation=False,
                      timeout_sec=None):
    """도착 대기."""
    start = time.time()
    dwell = 0
    has_ori = len(target_pos) >= 6
    limit = ARRIVAL_TIMEOUT_SEC if timeout_sec is None else timeout_sec
    while time.time() - start < limit:
        posx = _query_posx(mover_node, posx_client, executor)
        if posx is not None:
            pos_err = max(abs(posx[i] - target_pos[i]) for i in range(3))
            ok = pos_err <= config.POSITION_TOLERANCE_MM

            ori_err = None
            if has_ori:
                ori_err = transforms.rotation_geodesic_error_deg(
                    posx[3], posx[4], posx[5], target_pos[3], target_pos[4], target_pos[5])
                if check_orientation:
                    ok = ok and ori_err <= config.ORIENTATION_TOLERANCE_DEG

            elapsed = time.time() - start
            if elapsed >= 2.0:
                mover_node.get_logger().info(
                    f"도착 대기 {elapsed:.1f}s: pos_err={pos_err:.1f}mm "
                    f"ori_err={f'{ori_err:.1f}deg' if ori_err is not None else 'n/a'} "
                    f"(방향게이팅={'ON' if check_orientation else 'OFF'}) ok={ok}",
                    throttle_duration_sec=1.0)

            if ok:
                dwell += 1
                if dwell >= config.DWELL_CHECKS:
                    return True
            else:
                dwell = 0
        time.sleep(0.1)
    return False

def _wait_until_stopped(mover_node, posx_client, executor,
                        tol_mm=0.5, stable_polls=4, timeout_sec=5.0):
    """posx가 더 이상 변하지 않을 때까지 기다림."""
    start = time.time()
    prev, stable = None, 0
    while time.time() - start < timeout_sec:
        posx = _query_posx(mover_node, posx_client, executor)
        if posx is not None:
            if prev is not None:
                moved = max(abs(posx[i] - prev[i]) for i in range(3))
                if moved <= tol_mm:
                    stable += 1
                    if stable >= stable_polls:
                        mover_node.get_logger().info(
                            f"정지 확인 — 최근 {stable_polls}회 이동 {moved:.2f}mm 이하 "
                            f"({time.time() - start:.1f}s)")
                        return True
                else:
                    stable = 0
            prev = posx
        time.sleep(0.1)
    mover_node.get_logger().warn(
        f"⚠️ 정지 확인 타임아웃({timeout_sec:.0f}s) — 아직 움직이는 중일 수 있다")
    return False

def _wait_for_snapshot(mover_node, received_snapshots, waypoint_idx, executor):
    """object_detect_node가 TOPIC_SNAPSHOT_TAKEN으로 waypoint_idx를 발행할 때까지 대기."""
    start = time.time()
    while time.time() - start < config.SNAPSHOT_HANDSHAKE_TIMEOUT_SEC:
        if waypoint_idx in received_snapshots:
            return True
        rclpy.spin_once(mover_node, executor=executor, timeout_sec=0.1)
    return False

def _move_and_wait(mover_node, posx_client, amovel, target, label, executor,
                   check_orientation=False, timeout_sec=None):
    """amovel 명령을 보내고 실제 도착까지 기다림."""
    limit = ARRIVAL_TIMEOUT_SEC if timeout_sec is None else timeout_sec
    mover_node.get_logger().info(f"{label}: {[round(v, 1) for v in target]}로 이동")
    amovel(list(target), vel=VELOCITY, acc=ACC)
    if _wait_for_arrival(mover_node, posx_client, target, executor, check_orientation, limit):
        mover_node.get_logger().info(f"{label} 도착 완료")
        return True
    mover_node.get_logger().warning(f"⚠️ {limit}s 안에 {label} 도착 확인 못 함")
    return False

def _run_recon_mover_thread(mover_node, posx_client, amovel, received_snapshots, executor,
                            home_done):
    """정찰 웨이포인트 실제 이동."""
    log = mover_node.get_logger()
    waypoints = config.RECON_WAYPOINTS
    log.info(f"[mover] {len(waypoints)}개 웨이포인트로 순차 이동 시작 (vel={VELOCITY}, acc={ACC})")
    log.warning("[mover] ⚠️ Web UI 정지/비상정지 버튼 옆에 켜두고 지켜볼 것")
    _move_to_home(mover_node, posx_client, amovel, executor)
    home_done.set()
    for i, wp in enumerate(waypoints):
        pos = [wp["x"], wp["y"], wp["z"], wp["rx"], wp["ry"], wp["rz"]]
        log.info(f"[mover] [{i + 1}/{len(waypoints)}] movel -> {pos}")
        amovel(pos, vel=VELOCITY, acc=ACC)
        if _wait_for_arrival(mover_node, posx_client, pos, executor):
            log.info("[mover]   도착 완료 — object_detect_node 스냅샷 신호 대기 중...")
            if _wait_for_snapshot(mover_node, received_snapshots, i, executor):
                log.info(f"[mover]   스냅샷 확인됨(waypoint {i}) — 다음으로 이동")
            else:
                log.warning(f"[mover]   ⚠️ {config.SNAPSHOT_HANDSHAKE_TIMEOUT_SEC}s 안에 스냅샷 신호 못 받음 — 그냥 진행")
        else:
            log.warning(f"[mover]   ⚠️ {ARRIVAL_TIMEOUT_SEC}s 안에 도착 확인 못 함 — 다음 웨이포인트로 계속 진행")
    log.info("[mover] 전체 웨이포인트 이동 완료")

def _move_to_home(mover_node, posx_client, amovel, executor):
    """정찰을 항상 같은 자세에서 시작하기 위한 홈 이동."""
    h = config.HOME_POSE
    return _move_and_wait(mover_node, posx_client, amovel,
                          [h["x"], h["y"], h["z"], h["rx"], h["ry"], h["rz"]], "홈 자세",
                          executor, timeout_sec=HOME_ARRIVAL_TIMEOUT_SEC)

def _transit_waypoints():
    """홈->물체 사이 경유점 목록 [."""
    out = [("홈->물체 경유점", config.HOME_TO_OBJECT_WAYPOINT)]
    if config.HOME_TO_OBJECT_WAYPOINT_2 is not None:
        out = [("홈->물체 경유점1", config.HOME_TO_OBJECT_WAYPOINT),
               ("경유점1->경유점2", config.HOME_TO_OBJECT_WAYPOINT_2)]
    return out

def load_recon_part_clouds(object_id="obj_001", expect_coords_base=None,
                           max_centroid_gap_mm=None):
    """정찰이 저장한 부위 점군을 읽어옴."""
    return _load_recon_part_clouds_impl(object_id, expect_coords_base, max_centroid_gap_mm)

def _load_recon_part_clouds_impl(object_id, expect_coords_base, max_centroid_gap_mm):
    """정찰이 저장한 부위 점군 파일을 읽어 파지 입력으로 만듦."""
    dirs = sorted(d for d in glob.glob(os.path.join(config.SNAPSHOT_SAVE_DIR, "2*"))
                  if os.path.isdir(d))
    for snap_dir in reversed(dirs):
        path = os.path.join(snap_dir, f"parts_{object_id}.npz")
        if not os.path.exists(path):
            continue
        z = np.load(path, allow_pickle=False)
        meta = json.loads(str(z["meta"]))
        names, confs, counts, chunks, posx_by_part = [], [], [], [], {}
        for name, info in meta["parts"].items():
            key = f"points_{name}"
            if key not in z:
                continue
            pts = np.asarray(z[key], dtype=np.float64)
            names.append(name)
            confs.append(float(info["score"]))
            counts.append(int(len(pts)))
            chunks.append(pts)
            posx_by_part[name] = list(info["posx_mm_deg"])
        if not chunks:
            continue
        allpts = np.vstack(chunks)

        if expect_coords_base is not None:
            tol = (config.CLUSTER_THRESHOLD_MM if max_centroid_gap_mm is None
                   else max_centroid_gap_mm)
            gap = float(np.linalg.norm(allpts.mean(axis=0)
                                       - np.asarray(expect_coords_base, dtype=np.float64)))
            if gap > tol:
                print(f"[part_clouds] {path} 버림 — 부위 중심이 이번 정찰 좌표와 "
                      f"{gap:.0f}mm 어긋남(허용 {tol:.0f}mm). 물체가 옮겨졌거나 낡은 파일이다")
                continue

        class _Pt:
            __slots__ = ("x", "y", "z")

            def __init__(self, v):
                self.x, self.y, self.z = float(v[0]), float(v[1]), float(v[2])

        seg = types.SimpleNamespace(
            success=True, error_message="",
            part_names=names, part_confidences=confs, part_num_points=counts,
            part_sam2_applied=[False] * len(names),
            points=[_Pt(v) for v in allpts],
            num_parts=len(names), frame_id="base_link", unit="mm",
            source=path)
        return seg, posx_by_part
    return None, None

def _pick_target_part(seg, requested):
    """잡을 부위를 확정함."""
    names = list(seg.part_names)
    if not requested:
        raise ValueError(
            "잡을 부위가 지정되지 않았습니다 — `--part <부위>`로 지시할 것"
            f"(검출된 부위: {names}). "
            "confidence로 부위를 대신 고르지 않는다")
    key = requested.strip().lower()
    if key not in [n.strip().lower() for n in names]:
        raise ValueError(f"요청한 부위 '{key}'가 검출 결과에 없습니다: {names}")
    return key

def _execute_grasp(main_node, mover_node, posx_client, amovel, executor,
                   seg, target_obj, requested_part, approach_posx=None,
                   capture_posx_override=None):
    """계산한 파지 자세로 물체를 잡음."""
    log = main_node.get_logger()

    if capture_posx_override is not None:
        capture_posx = list(capture_posx_override)
        log.info(f"촬영 자세: 정찰 스냅샷 자세 사용 "
                 f"{[round(v, 1) for v in capture_posx[:6]]} (재촬영 없음)")
    else:
        capture_posx = _query_posx(mover_node, posx_client, executor)
    if capture_posx is None:
        log.error("촬영 자세(posx)를 못 읽음 — 파지 중단")
        return False

    if capture_posx_override is not None:
        start_posx = _query_posx(mover_node, posx_client, executor)
        if start_posx is None:
            log.error("현재 자세(posx)를 못 읽음 — 파지 중단")
            return False
        log.info(f"파지 출발 자세: 현재 자세 {[round(v, 1) for v in start_posx[:6]]} "
                 f"(촬영용 꺾기가 없으므로 복귀 단계 없음)")
        approach_posx = None
    else:
        start_posx = list(capture_posx)

    if config.RESTORE_POSE_AFTER_CAPTURE and approach_posx is not None:
        fraction = main_node.validated_reorient_fraction(list(capture_posx), list(approach_posx))
        if fraction >= config.RESTORE_POSE_MIN_SAFE_FRACTION:
            log.info("촬영 자세 → 접근 자세로 복귀(꺾은 각도 상쇄) — 파지는 여기서 출발한다")
            if _move_and_wait(mover_node, posx_client, amovel, list(approach_posx),
                              "촬영자세→접근자세 복귀", executor, check_orientation=True):
                start_posx = list(approach_posx)
            else:
                log.warn("복귀 이동이 도착 확인에 실패 — 현재 자세에서 파지를 계산한다")
                start_posx = _query_posx(mover_node, posx_client, executor) or list(capture_posx)
        else:
            log.warn(f"복귀 회전 사전검사 {fraction:.0%} < "
                     f"{config.RESTORE_POSE_MIN_SAFE_FRACTION:.0%} — 복귀를 건너뛴다"
                     f"(꺾인 자세 그대로 파지를 시도한다)")

    points = np.array([[p.x, p.y, p.z] for p in seg.points], dtype=np.float64)

    def _plan_and_validate(part, approach_rotation_deg=0.0):
        """부위와 접근축 후보 하나로 파지 자세를 계산하고 실행 가능한지 검사함."""
        try:
            res, ps = grasp_pca.compute_grasp(
                points, list(seg.part_names), list(seg.part_num_points), part,
                capture_posx, main_node._T_gripper2camera,
                part_confidences=list(seg.part_confidences),
                start_posx=start_posx,
                approach_rotation_deg=approach_rotation_deg)
        except grasp_pca.GraspTooWideError as e:
            return None, f"RG2 최대 개방 초과({e})"
        except ValueError as e:
            return None, f"자세 계산 실패({e})"

        log.info(grasp_pca.format_result(res, ps))
        if res.get("width_warning"):
            log.warn(f"[폭] {res['width_warning']}")
        if not res.get("width_trustworthy", True):
            return None, (f"폭 신뢰 불가(부위 extent {res.get('part_extent_mm', 0.0):.0f}mm > "
                          f"{grasp_pca.PART_MAX_EXTENT_MM:.0f}mm — 마스크 번짐)")

        mat = config.OBJECT_MATERIAL_MAP.get(target_obj.object_class, config.DEFAULT_MATERIAL)
        pl = rg2_gripper.plan_grasp(mat, res["grasp_width_mm"])
        if not pl["known_material"]:
            log.warn(f"'{mat}'는 파지 정책에 없는 재질 — "
                     f"'{pl['applied_material']}'({pl['force_n']:.1f}N)로 대체")
        log.info(f"[파지정책] {target_obj.object_class} -> {mat} | "
                 f"실측 {pl['measured_width_mm']:.1f}mm -> 개방 {pl['open_width_mm']:.1f}mm, "
                 f"목표 {pl['target_width_mm']:.1f}mm, 힘 {pl['force_n']:.1f}N")

        for label, key in (("pregrasp", "pregrasp"), ("grasp", "grasp")):
            valid, why_v = main_node.check_pose_reachable(list(ps[key]))
            if valid is False:
                return None, (f"{label} 자세 자체가 박스와 충돌({why_v}) — 경로 문제가 아니라 "
                              f"이 부위를 이 자리에서 잡는 자세가 존재하지 않는다. "
                              f"물체를 옮기거나 다른 부위를 지시할 것")
            if valid is None:
                log.warn(f"{label} 자세 유효성 검사 불가({why_v}) — 계획으로 진행")

        grasp_wps = None
        if config.GRASP_PLAN_WITH_PLANNER:
            if config.GRASP_PLAN_USE_JOINT_GOAL:
                if main_node._latest_joints is None:
                    return None, (f"/{config.ROBOT_ID}/joint_states 수신 없음 — "
                                  f"관절목표 계획의 IK seed가 없다")
                pre_joints = transforms.inverse_kinematics_joints(
                    list(ps["pregrasp"]), dict(main_node._latest_joints))
                if pre_joints is None:
                    return None, "pregrasp 자세 IK 수렴 실패(도달 불가)"
                log.info("도착->pregrasp 경로 계획(관절공간 목표)...")
                grasp_wps = main_node._approach_planner.plan_dense_waypoints_to_joints(
                    pre_joints)
            else:
                log.info("도착->pregrasp 경로 계획(위치+방향 목표)...")
                grasp_wps = main_node._approach_planner.plan_dense_waypoints_to_posx(
                    list(ps["pregrasp"]))
            if grasp_wps is None:
                if not config.GRASP_PLAN_FALLBACK_TO_STRAIGHT:
                    return None, ("도착->pregrasp 경로 계획 실패 — 직선 폴백이 꺼져 있다"
                                  "(config.GRASP_PLAN_FALLBACK_TO_STRAIGHT). 직선은 이미"
                                  " 막히는 것이 확인된 경로다")
                log.warn("경로 계획 실패 — 직선으로 폴백한다(옛 실패를 재현할 수 있음)")
            else:
                log.info(f"도착->pregrasp 계획 성공: dense waypoint {len(grasp_wps)}개")

        log.info("파지 이동 사전 충돌검사(도착 -> pregrasp -> grasp)...")
        ok, why_ = main_node.validate_grasp_path(
            start_posx, ps, skip_first_segment=grasp_wps is not None)
        if not ok:
            return None, f"파지 이동 충돌({why_})"
        log.info(f"파지 이동 사전검사 통과: {why_}")

        log.info("복귀 경로 사전 충돌검사(grasp -> pregrasp -> 경유점 -> 홈)...")
        ok, why_ = main_node.validate_return_path(ps)
        if not ok:
            return None, f"복귀 경로 충돌({why_})"
        log.info(f"복귀 경로 사전검사 통과: {why_}")
        return (res, ps, pl, grasp_wps), None

    try:
        first = _pick_target_part(seg, requested_part)
    except ValueError as e:
        log.error(f"파지 자세 계산 실패: {e}")
        return False

    candidates = [first]
    if config.GRASP_AUTO_RETRY_OTHER_PARTS:
        by_conf = sorted(zip([n.strip().lower() for n in seg.part_names],
                             list(seg.part_confidences)),
                         key=lambda x: -x[1])
        for name, _c in by_conf:
            fam = "".join(ch for ch in name if not ch.isdigit())
            if fam not in candidates:
                candidates.append(fam)
        log.warn("⚠️ 부위 자동 재시도가 켜져 있다 — 지시한 부위가 아닌 것을 집을 수 있다")

    rotations = list(config.GRASP_APPROACH_ROTATION_CANDIDATES_DEG) or [0.0]
    chosen, failures = None, []
    for part in candidates:
        for j, rot_deg in enumerate(rotations, 1):
            log.info(f"── 파지 후보 '{part}' 접근축 {rot_deg:+.0f}° ({j}/{len(rotations)})")
            got, reason = _plan_and_validate(part, approach_rotation_deg=rot_deg)
            if got:
                chosen = got
                break
            failures.append((part, rot_deg, reason))
            log.warn(f"  '{part}' 접근축 {rot_deg:+.0f}° 불가 — {reason}")
        if chosen:
            break

    if chosen is None:
        log.error("파지 가능한 자세가 없어 회수 중단(로봇은 현재 자세 유지, 그리퍼 미접촉)")
        log.error(f"  지시 부위 '{candidates[0]}'에 대해 접근축 {len(rotations)}개 후보를 "
                  f"전부 시도했으나 모두 거부됨:")
        for part, rot_deg, reason in failures:
            log.error(f"  - {part} {rot_deg:+.0f}°: {reason}")
        log.error("  물체를 세우거나 입구 쪽으로 당긴 뒤 다시 시도할 것")
        _log_graspable_parts(log, seg, capture_posx, main_node._T_gripper2camera)
        return False
    result, poses, plan, grasp_wps = chosen
    if failures:
        log.info(f"'{result['target_part']}' 접근축 "
                 f"{result.get('approach_rotation_deg', 0.0):+.0f}°로 진행 "
                 f"(앞서 {len(failures)}개 후보가 막힘)")

    grasp_path_viz = ([list(w) for w in grasp_wps] if grasp_wps
                      else [list(poses["pregrasp"])])
    grasp_path_viz.append(list(poses["grasp"]))
    main_node._approach_planner.publish_path_for_viz(grasp_path_viz)

    gripper = rg2_gripper.Rg2Gripper(logger=mover_node.get_logger())
    try:
        if not gripper.connect():
            log.error("RG2 연결 실패 — 파지 중단(로봇은 현재 자세 유지)")
            return False
        if config.CLOSE_GRIPPER_AT_START:
            try:
                w0, _r0 = gripper.close_fingers()
                log.info(f"파지 이동 전 그리퍼 닫음(시작 상태 확정) — 폭={w0:.1f}mm")
            except (TimeoutError, OSError) as e:
                log.warn(f"⚠️ 시작 닫기 실패({e}) — 벌어진 상태일 수 있다. 이동을 계속한다")

        if grasp_wps:
            log.info(f"파지 접근 경로 {len(grasp_wps)}개 지점 실행(계획된 우회)")
            for i, wp in enumerate(grasp_wps[:-1]):
                log.info(f"파지 접근 {i + 1}/{len(grasp_wps)}: {[round(v, 1) for v in wp]}로 이동")
                amovel(list(wp), vel=VELOCITY, acc=ACC)
                main_node._approach_planner.publish_progress(i + 1)
            if not _move_and_wait(mover_node, posx_client, amovel, list(grasp_wps[-1]),
                                  f"파지 접근 {len(grasp_wps)}/{len(grasp_wps)}(pregrasp)",
                                  executor, check_orientation=True):
                log.error("pregrasp 도착 실패 — 파지 중단")
                return False
            main_node._approach_planner.publish_progress(len(grasp_path_viz) - 1)
        elif not _move_and_wait(mover_node, posx_client, amovel, list(poses["pregrasp"]),
                                "pregrasp", executor):
            log.error("pregrasp 도착 실패 — 파지 중단")
            return False
        else:
            main_node._approach_planner.publish_progress(len(grasp_path_viz) - 1)
        log.info(f"pregrasp 도착 — 여기서 개방(목표 {plan['open_width_mm']:.1f}mm)")
        try:
            w_open, _r = gripper.open_for(plan)
            log.info(f"[개방] 폭={w_open:.1f}mm")
        except (TimeoutError, OSError) as e:
            log.error(f"개방 실패({e}) — 파지 중단(로봇은 pregrasp 유지)")
            return False

        if not _move_and_wait(mover_node, posx_client, amovel, list(poses["grasp"]),
                              "grasp", executor):
            log.error("grasp 도착 실패 — 파지 중단")
            return False
        main_node._approach_planner.publish_progress(len(grasp_path_viz))
        _wait_until_stopped(mover_node, posx_client, executor)

        grip = gripper.grasp(plan)
        log.info(f"[파지] 최종폭={grip['final_width_mm']:.1f}mm "
                 f"정지사유={grip['stop_reason']} grip플래그={grip['grip_flag']} "
                 f"-> {'물었음' if grip['gripped'] else '⚠️ 헛집음'}")

        log.info("복귀 시작 — pregrasp -> 경유점 -> 홈")
        w = config.HOME_TO_OBJECT_WAYPOINT
        h = config.HOME_POSE
        return_path_viz = [list(poses["pregrasp"]),
                           [w["x"], w["y"], w["z"], w["rx"], w["ry"], w["rz"]],
                           [h["x"], h["y"], h["z"], h["rx"], h["ry"], h["rz"]]]
        main_node._approach_planner.publish_path_for_viz(return_path_viz)
        _move_and_wait(mover_node, posx_client, amovel, list(poses["pregrasp"]),
                       "복귀(pregrasp)", executor)
        main_node._approach_planner.publish_progress(1)
        _move_and_wait(mover_node, posx_client, amovel,
                       [w["x"], w["y"], w["z"], w["rx"], w["ry"], w["rz"]],
                       "복귀(경유점)", executor)
        main_node._approach_planner.publish_progress(2)
        _move_to_home(mover_node, posx_client, amovel, executor)
        main_node._approach_planner.publish_progress(3)

        if config.RELEASE_AT_HOME:
            log.info("홈 도착 — 물체 놓기(그리퍼 개방)")
            try:
                final_width_mm, stop_reason = gripper.release()
                log.info(f"[놓기] 최종폭={final_width_mm:.1f}mm 정지사유={stop_reason}")
            except (TimeoutError, OSError) as e:
                log.error(f"⚠️ 놓기 실패({e}) — 물체를 들고 있을 수 있다. "
                          f"수동으로 그리퍼를 열 것")
            if config.CLOSE_GRIPPER_AFTER_RELEASE:
                try:
                    w2_mm, reason2 = gripper.close_fingers()
                    log.info(f"[종료 자세 정리] 그리퍼 닫음 — 최종폭={w2_mm:.1f}mm "
                             f"정지사유={reason2}")
                except (TimeoutError, OSError) as e:
                    log.warn(f"⚠️ 종료 닫기 실패({e}) — 다음 실행 전에 수동으로 닫을 것")
        else:
            log.warn("⚠️ 홈에서 놓기가 꺼져 있음(RELEASE_AT_HOME=False) — 물체를 든 채로 끝난다")
        return bool(grip["gripped"])
    finally:
        gripper.close()

def _log_graspable_parts(log, seg, capture_posx, T_gripper2camera):
    """어느 부위가 RG2 개방 폭에 들어가는지 표로 남김."""
    points = np.array([[p.x, p.y, p.z] for p in seg.points], dtype=np.float64)
    log.info("부위별 폭 판정 (이동 경로는 별도 게이트 — 위 충돌 사유를 볼 것):")
    for name in dict.fromkeys(seg.part_names):
        try:
            r, _ = grasp_pca.compute_grasp(
                points, list(seg.part_names), list(seg.part_num_points),
                name, capture_posx, T_gripper2camera,
                part_confidences=list(seg.part_confidences))
            log.info(f"  {name:6s} 폭 {r['grasp_width_mm']:6.1f}mm  ✅ 가능")
        except grasp_pca.GraspTooWideError as e:
            log.info(f"  {name:6s} {str(e).split('의 ')[-1]}")
        except ValueError as e:
            log.info(f"  {name:6s} 계산 불가: {e}")

def main(args=None):
    rclpy.init(args=args)
    requested_part = ""
    for i, arg in enumerate(sys.argv):
        if arg == "--part" and i + 1 < len(sys.argv):
            requested_part = sys.argv[i + 1]

    voice_class = None
    if "--voice" in sys.argv:
        from yolo_detect.voice_command import VoiceCommandListener
        try:
            listener = VoiceCommandListener()
        except Exception as e:
            print(f"❌ 음성 입력 초기화 실패({type(e).__name__}: {e})")
            sys.exit(1)
        cmd = listener.listen()
        if cmd is None:
            print("❌ 음성 지시를 못 받아 시작하지 않습니다")
            sys.exit(1)
        requested_part = cmd.part
        voice_class = cmd.object_class

    node = RobotControlNode()

    mover_node = rclpy.create_node("robot_control_mover", namespace=config.ROBOT_ID)
    DR_init.__dsr__id = config.ROBOT_ID
    DR_init.__dsr__model = config.ROBOT_MODEL
    DR_init.__dsr__node = mover_node
    try:
        from DSR_ROBOT2 import amovel
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        sys.exit(1)
    posx_client = mover_node.create_client(
        GetCurrentPosx, f"/{config.ROBOT_ID}/aux_control/get_current_posx")

    mover_executor = rclpy.executors.SingleThreadedExecutor()
    main_executor = rclpy.executors.SingleThreadedExecutor()

    received_snapshots = set()
    mover_node.create_subscription(
        Int32, f"/{config.TOPIC_SNAPSHOT_TAKEN}",
        lambda msg: received_snapshots.add(msg.data), 10)

    for _ in range(50):
        if node._latest_joints is not None:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    node.preflight_validate_fixed_segments()

    home_done = threading.Event()
    mover_thread = threading.Thread(
        target=_run_recon_mover_thread,
        args=(mover_node, posx_client, amovel, received_snapshots, mover_executor, home_done),
        daemon=True)
    mover_thread.start()

    try:
        if not home_done.wait(timeout=HOME_ARRIVAL_TIMEOUT_SEC + 30.0):
            node.get_logger().warning("⚠️ 홈 이동 완료 신호를 못 받음 — 그대로 정찰 goal 전송")
        result = node.run_recon(main_executor)
        mover_thread.join()
        if result is None:
            return

        node.get_logger().info(
            f"정찰 완료: success={result.success}, 물체 {len(result.objects)}개 탐지됨")
        for obj in result.objects:
            node.get_logger().info(
                f"  object_id={obj.object_id}  class={obj.object_class}  "
                f"coords_base(mm)=[{obj.coords_base[0]:.1f}, {obj.coords_base[1]:.1f}, "
                f"{obj.coords_base[2]:.1f}]  confidence={obj.confidence:.2f}")

        node._save_results_to_file(result)

        if not result.objects:
            node.get_logger().error("탐지된 물체 없음 — 이동 취소 (YOLO 모델/조명/카메라 각도 확인 필요)")
            return

        target_class = voice_class or config.RECON_TARGET_CLASS
        candidates = list(result.objects)
        if target_class:
            want = str(target_class).strip().lower()
            matching = [o for o in candidates
                        if str(o.object_class).strip().lower() == want]
            if matching:
                if len(matching) != len(candidates):
                    node.get_logger().info(
                        f"회수 대상 '{want}' {len(matching)}개만 후보로 사용"
                        f"(전체 {len(candidates)}개 중)")
                candidates = matching
            else:
                node.get_logger().warning(
                    f"⚠️ 회수 대상 '{want}'가 탐지 목록에 없음 — 탐지된 클래스: "
                    f"{sorted({o.object_class for o in candidates})}. 전체에서 고른다")
        target_obj = max(candidates, key=lambda o: o.confidence)
        if len(candidates) > 1:
            node.get_logger().warning(
                f"대상 후보가 {len(candidates)}개 — confidence 가장 높은 "
                f"{target_obj.object_id}({target_obj.confidence:.2f})로만 이동")

        seg, posx_by_part = load_recon_part_clouds(
            target_obj.object_id, expect_coords_base=list(target_obj.coords_base))
        if seg is None:
            node.get_logger().error(
                f"{target_obj.object_id}의 부위 점군을 정찰에서 확보하지 못함 — 회수 중단")
            return

        want = (requested_part or "").strip().lower()
        have = [n.strip().lower() for n in seg.part_names]
        if want and want not in have:
            node.get_logger().error(
                f"지시 부위 '{want}'가 정찰 부위 점군에 없음(있는 것: {have}) — 회수 중단")
            return
        capture_posx_override = posx_by_part[want if want else seg.part_names[0]]

        node.get_logger().info(f"정찰 부위 점군 사용: {seg.source}")
        transit = _transit_waypoints()
        for label, wp in transit:
            _move_and_wait(mover_node, posx_client, amovel,
                           [wp["x"], wp["y"], wp["z"], wp["rx"], wp["ry"], wp["rz"]],
                           label, mover_executor)
        w2 = transit[-1][1]
        node.set_target_filter(target_obj)
        if config.OCTOMAP_CLEAR_BEFORE_GRASP:
            node._approach_planner.clear_octomap()
        approach_posx = [w2["x"], w2["y"], w2["z"], w2["rx"], w2["ry"], w2["rz"]]

        node.get_logger().info(
            f"파지 재료 확보: 부위 {seg.num_parts}개, 총 {len(seg.points)}점 "
            f"({seg.frame_id}, {seg.unit})")
        node.get_logger().info(
            f"  (정찰 좌표: [{target_obj.coords_base[0]:.1f}, "
            f"{target_obj.coords_base[1]:.1f}, {target_obj.coords_base[2]:.1f}])")
        offset = 0
        for name, conf, sam2, n in zip(seg.part_names, seg.part_confidences,
                                       seg.part_sam2_applied, seg.part_num_points):
            c = np.mean([[p.x, p.y, p.z] for p in seg.points[offset:offset + n]], axis=0)
            node.get_logger().info(
                f"  {name:5s} conf={conf:.2f} sam2={str(sam2):5s} {n:6d}점  "
                f"중심(mm)=[{c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}]")
            offset += n

        gripped = _execute_grasp(node, mover_node, posx_client, amovel, mover_executor,
                                 seg, target_obj, requested_part,
                                 approach_posx=approach_posx,
                                 capture_posx_override=capture_posx_override)
        node.get_logger().info(f"=== 회수 {'성공' if gripped else '실패'} ===")
    except KeyboardInterrupt:
        node.get_logger().warning("중단됨(Ctrl-C)")
    except Exception:
        node.get_logger().fatal("파이프라인 예외로 중단:\n" + traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        mover_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
