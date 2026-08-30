"""저장된 수리 목표 요청과 완료 응답 흐름을 실제 로봇 없이 시험한다."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RepairTargetTestNode(Node):
    """결함 ID를 요청하고 목표 수신 시 가상 완료 결과를 발행한다."""

    def __init__(self):
        """시험 결함 ID와 요청·응답 토픽 및 단발 타이머를 초기화한다."""
        super().__init__("repair_target_test")
        self.declare_parameter("defect_id", "crack_0001")
        self.declare_parameter("auto_complete", True)
        self.defect_id = str(self.get_parameter("defect_id").value)
        self.auto_complete = bool(self.get_parameter("auto_complete").value)
        self.request_pub = self.create_publisher(String, "/repair/target_request_json", 10)
        self.result_pub = self.create_publisher(String, "/repair/result_json", 10)
        self.target_sub = self.create_subscription(String, "/repair/target_json", self.on_target, 10)
        self.request_sent = self.result_sent = False
        self.timer = self.create_timer(1.0, self.send_request)

    def send_request(self):
        """설정한 결함 ID의 수리 목표를 한 번 요청한다."""
        if self.request_sent or self.request_pub.get_subscription_count() == 0:
            return
        self.request_pub.publish(String(data=json.dumps({"defect_id": self.defect_id}, separators=(",", ":"))))
        self.request_sent = True
        self.get_logger().info(f"수리 목표 요청: id={self.defect_id}")

    def on_target(self, message):
        """목표 JSON을 확인하고 설정에 따라 가상 완료 결과를 발행한다."""
        try:
            target = json.loads(message.data)
        except json.JSONDecodeError as error:
            self.get_logger().warning(f"수리 목표 JSON 오류: {error}")
            return
        if target.get("defect_id") != self.defect_id:
            return
        if target.get("event") != "repair_target":
            self.get_logger().warning(f"수리 목표를 받지 못했어: reason={target.get('reason')}")
            return
        self.get_logger().info(f"수리 목표 수신: id={self.defect_id}, position={target['navigation_goal_xyz_m']}")
        if self.auto_complete and not self.result_sent:
            result = {"defect_id": self.defect_id, "status": "completed", "detail": "mock_target_reached"}
            self.result_pub.publish(String(data=json.dumps(result, ensure_ascii=False, separators=(",", ":"))))
            self.result_sent = True
            self.get_logger().info(f"가상 수리 완료 발행: id={self.defect_id}")


def main(args=None):
    """ROS를 초기화하고 수리 목표 mock 노드를 실행한다."""
    rclpy.init(args=args)
    node = RepairTargetTestNode()
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
