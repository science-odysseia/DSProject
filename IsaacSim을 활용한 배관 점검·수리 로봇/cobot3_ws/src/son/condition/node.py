"""[ROS 3.10] 관 상태 판정 ROS 2 노드.

전방 Depth 로 관 상태를 판정해 발행한다. 결함(크랙·구멍) 검출은 YOLO 가
별도로 담당하며 이 노드는 주행 안전 판정 전용이다.

발행 토픽은 조율 노드를 거치지 않고 주행 제어 노드로 직행한다. 정지 판단에
지연이 있으면 안 되기 때문이다.

pipe_msgs/PipeCondition 이 아직 없으므로 있으면 그것을, 없으면 JSON 문자열을
발행한다. 인터페이스가 생기면 USE_MSG 분기만 남기고 정리할 것.

실행:
  python3 node.py --ros-args --params-file config/pipe_condition.yaml
  python3 node.py --ros-args -r __ns:=/scout
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pyver import require_ros

require_ros(__file__)

import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String

import detector as D
from detector import DEFAULTS, PipeConditionDetector
from odometry import DEFAULTS as VO_DEFAULTS, VisualOdometry

try:
    from pipe_msgs.msg import PipeCondition
    USE_MSG = True
except ImportError:
    PipeCondition = None
    USE_MSG = False

BLOCKAGE = {"NORMAL": 0, "DISCONNECTED": 1, "MISALIGNMENT": 2,
            "UNDETERMINED": 3}


def sensor_qos(depth=5):
    """이미지는 반드시 BEST_EFFORT. Isaac Sim 이 기본 BEST_EFFORT 라
    구독자가 RELIABLE 이면 토픽이 아예 수신되지 않는다."""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


class PipeConditionNode(Node):
    def __init__(self):
        super().__init__("pipe_condition_node")
        for key, val in DEFAULTS.items():
            self.declare_parameter(key, val)
        for key, val in (("depth_topic", "front/depth"),
                         ("rgb_topic", "front/rgb"),
                         ("camera_info_topic", "front/camera_info"),
                         ("joint_topic", "joint_angle"),
                         ("condition_topic", "condition"),
                         ("visual_topic", "visual_speed"),
                         ("enable_odometry", True),
                         ("publish_hz", 10.0),
                         ("fallback_fx", 523.9), ("fallback_fy", 523.9),
                         ("fallback_ppx", 640.0), ("fallback_ppy", 360.0)):
            self.declare_parameter(key, val)

        p = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.intr = dict(fx=self.get_parameter("fallback_fx").value,
                         fy=self.get_parameter("fallback_fy").value,
                         ppx=self.get_parameter("fallback_ppx").value,
                         ppy=self.get_parameter("fallback_ppy").value)
        self.intr["f_fish"] = self.intr["fx"]
        self.det = PipeConditionDetector(self.intr, p)
        # 시각 오도메트리도 여기 둔다. 이미지는 인지 노드 밖으로 나가지 않고
        # 주행 노드로는 속도 스칼라만 보낸다(설계 원칙 1).
        self.vo = VisualOdometry(f_px=self.intr["f_fish"])
        self.use_vo = bool(self.get_parameter("enable_odometry").value)
        self.rgb = None
        self.last_t = None
        self.bridge = CvBridge()
        self.depth = None
        self._warned_mm = False
        self.joint_deg = 0.0
        self.have_info = False

        self.create_subscription(
            Image, self.get_parameter("depth_topic").value,
            self.on_depth, sensor_qos())
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self.on_info, sensor_qos())
        self.create_subscription(
            Float32, self.get_parameter("joint_topic").value,
            self.on_joint, 10)
        self.create_subscription(
            Image, self.get_parameter("rgb_topic").value,
            self.on_rgb, sensor_qos())
        self.pub_vo = self.create_publisher(
            Float32, self.get_parameter("visual_topic").value, 10)

        topic = self.get_parameter("condition_topic").value
        if USE_MSG:
            self.pub = self.create_publisher(PipeCondition, topic, 10)
        else:
            self.pub = self.create_publisher(String, topic, 10)
            self.get_logger().warn(
                "pipe_msgs/PipeCondition 없음 — JSON 문자열로 발행한다")

        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(hz, 0.1), self.tick)
        self.get_logger().info(
            f"관 상태 판정 시작 — {hz:.0f} Hz, invalid_mode="
            f"{self.det.k['invalid_mode']}")

    def on_info(self, msg):
        if self.have_info:
            return
        self.det.fx, self.det.fy = msg.k[0], msg.k[4]
        self.det.ppx, self.det.ppy = msg.k[2], msg.k[5]
        # 어안이라 K 는 엄밀한 핀홀 행렬이 아니다. K[0] 에 등거리 어안의
        # f (r = f*theta) 를 실어 보내는 규약을 쓴다. 이 줄을 빠뜨리면
        # f_fish 가 fallback 에 머물러 오프셋이 배율만큼 틀어진다.
        self.det.f_fish = msg.k[0]
        self.have_info = True
        self.get_logger().info(
            f"camera_info 수신 f={self.det.fx:.1f} "
            f"pp=({self.det.ppx:.1f},{self.det.ppy:.1f}) "
            f"projection={self.det.k['projection']}")

    def on_joint(self, msg):
        self.joint_deg = float(msg.data)

    def on_rgb(self, msg):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warn(f"rgb 변환 실패: {exc}")

    def on_depth(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"depth 변환 실패: {exc}")
            return
        # 단위 판단은 detector.normalize_depth 가 한다 — 판정 로직이라
        # 오프라인에서 시험되어야 하고, 여기 두면 rclpy 없이는 검증이 안 된다.
        # 예전에 이 자리에 있던 np.nanmax 판정은 inf 를 못 걸러 Depth 전체를
        # 1/1000 로 만들었다 (detector.normalize_depth 주석 참조).
        img, scaled = D.normalize_depth(img)
        if scaled and not self._warned_mm:
            self._warned_mm = True
            self.get_logger().warn("depth 가 mm 로 들어온다 — m 로 환산 중")
        self.depth = img

    def tick(self):
        if self.depth is None:
            return
        c = self.det.run(self.depth, joint_angle_deg=self.joint_deg)

        if self.use_vo and self.rgb is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = (now - self.last_t) if self.last_t else 0.0
            self.last_t = now
            if dt > 0:
                fr = self.vo.step(self.rgb, self.depth, dt)
                if fr.valid:
                    self.pub_vo.publish(Float32(data=float(fr.speed_mps)))

        if USE_MSG:
            m = PipeCondition()
            m.header.stamp = self.get_clock().now().to_msg()
            m.circularity = float(c.circularity)
            m.forward_range_m = float(c.forward_range_m)
            m.offset_mm = float(c.offset_mm)
            m.passable = bool(c.passable)
            m.blockage_type = BLOCKAGE.get(c.state, 3)
            self.pub.publish(m)
        else:
            self.pub.publish(String(data=json.dumps(c.as_dict(),
                                                    ensure_ascii=False)))
        if not c.passable:
            self.get_logger().warn(f"{c.state} — {c.reason}")


def main():
    rclpy.init()
    node = PipeConditionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
