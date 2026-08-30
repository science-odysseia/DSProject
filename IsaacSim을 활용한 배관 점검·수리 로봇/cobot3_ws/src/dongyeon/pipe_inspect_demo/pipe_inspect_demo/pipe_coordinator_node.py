"""수리로봇의 용접봉 잔량과 수리 요청을 결합해 최종 수리 판정을 발행한다."""

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from pipe_inspect_demo.pipe_report_node import DEFAULT_OUTPUT_ROOT
from pipe_inspect_demo.repair_decision import decide_repair
from pipe_inspect_demo.repair_target_registry import RepairTargetRegistry

# 🔑 결함 기록 자리는 `pipe_report_node` 가 정한 것을 그대로 쓴다 — 두 노드가
#    같은 폴더를 봐야 하므로 규칙을 두 곳에 적지 않는다.
DEFAULT_REPORT_OUTPUT_ROOT = DEFAULT_OUTPUT_ROOT


class PipeCoordinatorNode(Node):
    """최신 결함과 실제 용접봉 잔량을 관리하고 요청된 결함의 수리 가능 여부를 판단한다."""

    def __init__(self):
        """관련 토픽 파라미터와 구독자·판정 발행자를 초기화한다."""
        super().__init__("pipe_coordinator")
        defaults = {
            "defect_topic": "/defect/report_json",
            "aligned_defect_topic": "/defect/aligned_report_json",
            "rod_status_topic": "/repair_robot/welding_rod_status",
            "repair_request_topic": "/repair/request_json",
            "decision_topic": "/repair/decision_json",
            "target_request_topic": "/repair/target_request_json",
            "target_json_topic": "/repair/target_json",
            "target_pose_topic": "/repair/target_pose",
            "repair_result_topic": "/repair/result_json",
            "repair_status_topic": "/repair/status_json",
            "report_output_root": str(DEFAULT_REPORT_OUTPUT_ROOT),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.defects = {}
        self.target_registry = RepairTargetRegistry()
        self.rod_status = None
        self.decision_pub = self.create_publisher(String, self.get_parameter("decision_topic").value, 10)
        self.target_json_pub = self.create_publisher(String, self.get_parameter("target_json_topic").value, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, self.get_parameter("target_pose_topic").value, 10)
        self.repair_status_pub = self.create_publisher(String, self.get_parameter("repair_status_topic").value, 10)
        self.defect_sub = self.create_subscription(String, self.get_parameter("defect_topic").value, self.on_defect, 10)
        self.aligned_defect_sub = self.create_subscription(String, self.get_parameter("aligned_defect_topic").value, self.on_aligned_defect, 10)
        self.rod_status_sub = self.create_subscription(String, self.get_parameter("rod_status_topic").value, self.on_rod_status, 10)
        self.repair_request_sub = self.create_subscription(String, self.get_parameter("repair_request_topic").value, self.on_repair_request, 10)
        self.target_request_sub = self.create_subscription(String, self.get_parameter("target_request_topic").value, self.on_target_request, 10)
        self.repair_result_sub = self.create_subscription(String, self.get_parameter("repair_result_topic").value, self.on_repair_result, 10)
        loaded = self._load_latest_summary()
        self.get_logger().info(f"수리 판단 노드 시작: 결함·목표·용접봉 상태 대기 중, 저장 목표={loaded}개")

    def on_defect(self, message):
        """결함 보고를 ID별 최신 상태로 보관한다."""
        report = self._decode_object(message.data, "결함 보고")
        if report is None:
            return
        defect_id = report.get("defect_id")
        if not defect_id:
            self.get_logger().warning("defect_id가 없는 결함 보고를 무시했어.")
            return
        self.defects[str(defect_id)] = report

    def on_aligned_defect(self, message):
        """정렬 완료 결함의 유효한 수리 목표를 ID별로 보관한다."""
        report = self._decode_object(message.data, "정렬 결함 보고")
        if report is None:
            return
        stored, reason = self.target_registry.add_aligned_report(report)
        if not stored:
            self.get_logger().warning(f"수리 목표를 저장하지 않았어: reason={reason}")
            return
        self.defects[str(report["defect_id"])] = report
        self.get_logger().info(f"정렬 수리 목표 저장: id={report['defect_id']}")

    def on_rod_status(self, message):
        """수리로봇이 발행한 최신 용접봉 잔량 상태를 검증해 보관한다."""
        status = self._decode_object(message.data, "용접봉 상태")
        if status is None:
            return
        try:
            remaining_mm = float(status["remaining_length_mm"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning("remaining_length_mm가 유효하지 않은 용접봉 상태를 무시했어.")
            return
        if remaining_mm < 0.0:
            self.get_logger().warning("용접봉 잔량이 음수인 상태를 무시했어.")
            return
        self.rod_status = status
        self.get_logger().info(f"용접봉 잔량 갱신: remaining={remaining_mm:.3f}mm, valid={bool(status.get('sensor_valid', True))}")

    def on_repair_request(self, message):
        """수리 요청의 결함을 최신 실제 잔량으로 판정해 JSON으로 발행한다."""
        request = self._decode_object(message.data, "수리 요청")
        if request is None:
            return
        defect_id = str(request.get("defect_id", ""))
        if defect_id not in self.defects:
            self._publish_unavailable(defect_id, "defect_not_found")
            return
        if self.rod_status is None:
            self._publish_unavailable(defect_id, "welding_rod_status_unavailable")
            return
        if not bool(self.rod_status.get("sensor_valid", True)):
            self._publish_unavailable(defect_id, "welding_rod_sensor_invalid")
            return
        decision = decide_repair(self.defects[defect_id], self.rod_status["remaining_length_mm"])
        self._publish_decision(defect_id, decision)

    def on_target_request(self, message):
        """결함 ID로 저장 목표를 조회해 JSON과 PoseStamped로 다시 발행한다."""
        request = self._decode_object(message.data, "수리 목표 요청")
        if request is None:
            return
        defect_id = str(request.get("defect_id", "")).strip()
        report, status = self.target_registry.get_target(defect_id)
        if report is None and status == "target_not_found" and self._load_latest_summary() > 0:
            report, status = self.target_registry.get_target(defect_id)
        if report is None:
            self._publish_target_unavailable(defect_id, status)
            return
        target = report["repair_target"]
        now = self.get_clock().now().to_msg()
        payload = {"schema_version": "1.0", "event": "repair_target", "timestamp": {"sec": now.sec, "nanosec": now.nanosec}, "defect_id": defect_id, "status": status, "frame_id": target["frame_id"], "navigation_goal_xyz_m": target["navigation_goal_xyz_m"], "orientation_xyzw": target["orientation_xyzw"], "defect_pose": report.get("defect_pose"), "measurement": report.get("measurement")}
        self.target_json_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = now, target["frame_id"]
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, target["navigation_goal_xyz_m"])
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, target["orientation_xyzw"])
        self.target_pose_pub.publish(pose)
        self.get_logger().info(f"저장 수리 목표 발행: id={defect_id}, status={status}")

    def on_repair_result(self, message):
        """수리로봇의 진행·완료·실패 결과를 목표 상태에 반영하고 발행한다."""
        result = self._decode_object(message.data, "수리 결과")
        if result is None:
            return
        defect_id, status = str(result.get("defect_id", "")).strip(), str(result.get("status", "")).strip()
        updated, reason = self.target_registry.update_result(defect_id, status)
        now = self.get_clock().now().to_msg()
        payload = {"schema_version": "1.0", "event": "repair_status", "timestamp": {"sec": now.sec, "nanosec": now.nanosec}, "defect_id": defect_id, "status": reason if updated else "rejected", "reason": None if updated else reason, "detail": result.get("detail")}
        self.repair_status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))
        log = self.get_logger().info if updated else self.get_logger().warning
        log(f"수리 결과 {'반영' if updated else '거부'}: id={defect_id}, status={status}, reason={reason}")

    def _publish_target_unavailable(self, defect_id, reason):
        """요청한 수리 목표가 없거나 완료된 상태를 JSON으로 알린다."""
        payload = {"schema_version": "1.0", "event": "repair_target_unavailable", "defect_id": defect_id, "reason": reason}
        self.target_json_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))
        self.get_logger().warning(f"수리 목표 발행 불가: id={defect_id}, reason={reason}")

    def _load_latest_summary(self):
        """보고 출력 폴더에서 가장 최근 임무 요약의 정렬 목표를 불러온다."""
        output_root = Path(str(self.get_parameter("report_output_root").value)).expanduser()
        summaries = sorted(output_root.glob("mission_*/defect_summary.json"), key=lambda path: path.stat().st_mtime, reverse=True) if output_root.is_dir() else []
        if not summaries:
            return 0
        try:
            return self.target_registry.load_summary(summaries[0])
        except (OSError, json.JSONDecodeError, TypeError, ValueError) as error:
            self.get_logger().warning(f"최근 임무 수리 목표를 불러오지 못했어: {error}")
            return 0

    def _publish_unavailable(self, defect_id, reason):
        """필수 입력이 없을 때 수리 금지 판정을 발행한다."""
        decision = {"repairable": False, "action": "wait_and_report", "report_required": True, "reason": reason}
        self._publish_decision(defect_id, decision)

    def _publish_decision(self, defect_id, decision):
        """판정 시각과 용접봉 상태를 포함한 최종 수리 판정을 발행한다."""
        now = self.get_clock().now().to_msg()
        payload = {
            "schema_version": "1.0",
            "event": "repair_decision",
            "timestamp": {"sec": now.sec, "nanosec": now.nanosec},
            "defect_id": defect_id,
            "robot_id": None if self.rod_status is None else self.rod_status.get("robot_id"),
            "repair_decision": decision,
            "welding_rod_status": self.rod_status,
        }
        self.decision_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))
        self.get_logger().info(f"수리 판정 발행: id={defect_id}, action={decision['action']}, reason={decision['reason']}")

    def _decode_object(self, data, label):
        """문자열 메시지를 JSON 객체로 변환하고 잘못된 입력을 경고한다."""
        try:
            value = json.loads(data)
            if not isinstance(value, dict):
                raise ValueError("최상위 형식이 객체가 아니야.")
            return value
        except (json.JSONDecodeError, ValueError) as error:
            self.get_logger().warning(f"{label} JSON을 처리하지 못했어: {error}")
            return None


def main(args=None):
    """ROS를 초기화하고 수리 판단 노드를 종료 요청까지 실행한다."""
    rclpy.init(args=args)
    node = PipeCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
