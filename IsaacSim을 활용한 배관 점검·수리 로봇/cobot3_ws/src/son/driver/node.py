"""[ROS 3.10] 주행 제어 ROS 2 노드.

관 상태 판정을 받아 전진 속도를 낸다. 이미지는 보지 않는다 — 설계 원칙상
이미지는 인지 노드 밖으로 나가지 않으며 여기로는 스칼라만 들어온다.

조향이 없다. 관절은 수동이고 6륜이 관벽에 압착되어 횡방향은 기계가 잡는다.
그래서 발행하는 것은 linear.x 하나뿐이다.

구독
  ~/condition       관 상태 판정 (JSON 문자열 또는 pipe_msgs/PipeCondition)
  ~/wheel_speed     휠 엔코더 속도 (m/s)
  ~/visual_speed    시각 오도메트리 속도 (m/s) — 끼임 판정에 쓴다
  ~/start           출발 지시
  ~/recall          복귀 지시

발행
  ~/cmd_vel         Twist. linear.x 만 사용
  ~/drive_state     FSM 상태 (JSON 문자열)

실행:
  python3 node.py --ros-args --params-file config/driver.yaml
  python3 node.py --ros-args -r __ns:=/scout
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pyver import require_ros

require_ros(__file__)

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from geometry_msgs.msg import Twist

from control import DEFAULTS, DriveController

try:
    from pipe_msgs.msg import PipeCondition
    USE_MSG = True
except ImportError:
    PipeCondition = None
    USE_MSG = False

BLOCKAGE_NAME = {0: "NORMAL", 1: "DISCONNECTED", 2: "MISALIGNMENT",
                 3: "UNDETERMINED"}


class DriverNode(Node):
    def __init__(self):
        super().__init__("driver_node")
        for key, val in DEFAULTS.items():
            self.declare_parameter(key, val)
        for key, val in (("condition_topic", "condition"),
                         ("wheel_topic", "wheel_speed"),
                         ("visual_topic", "visual_speed"),
                         ("start_topic", "start"),
                         ("recall_topic", "recall"),
                         ("cmd_topic", "cmd_vel"),
                         ("state_topic", "drive_state"),
                         ("rate_hz", 20.0)):
            self.declare_parameter(key, val)

        p = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.ctl = DriveController(p)
        self.cond = None
        self.wheel = 0.0
        self.visual = None
        self.start = False
        self.recall = False

        g = self.get_parameter
        if USE_MSG:
            self.create_subscription(PipeCondition, g("condition_topic").value,
                                     self.on_condition_msg, 10)
        else:
            self.create_subscription(String, g("condition_topic").value,
                                     self.on_condition_json, 10)
            self.get_logger().warn(
                "pipe_msgs/PipeCondition 없음 — JSON 문자열로 수신한다")
        self.create_subscription(Float32, g("wheel_topic").value,
                                 lambda m: setattr(self, "wheel", float(m.data)), 10)
        self.create_subscription(Float32, g("visual_topic").value,
                                 lambda m: setattr(self, "visual", float(m.data)), 10)
        self.create_subscription(Bool, g("start_topic").value,
                                 lambda m: setattr(self, "start", bool(m.data)), 10)
        self.create_subscription(Bool, g("recall_topic").value,
                                 lambda m: setattr(self, "recall", bool(m.data)), 10)

        self.pub_cmd = self.create_publisher(Twist, g("cmd_topic").value, 10)
        self.pub_state = self.create_publisher(String, g("state_topic").value, 10)

        self.hz = float(g("rate_hz").value)
        self.dt = 1.0 / max(self.hz, 1.0)
        self.create_timer(self.dt, self.tick)
        self.prev_state = None
        self.get_logger().info(
            f"주행 제어 시작 — {self.hz:.0f} Hz, "
            f"최대 {self.ctl.k['v_max_mps']:.3f} m/s, "
            f"임무 {self.ctl.k['mission_range_m']:.0f} m")

    def on_condition_json(self, msg):
        try:
            self.cond = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().error(f"condition 파싱 실패: {exc}")

    def on_condition_msg(self, msg):
        self.cond = dict(
            state=BLOCKAGE_NAME.get(int(msg.blockage_type), "UNDETERMINED"),
            passable=bool(msg.passable),
            speed="full" if msg.passable else "stop",
            circularity=float(msg.circularity),
            forward_range_m=float(msg.forward_range_m),
            offset_mm=float(msg.offset_mm),
            reason="")

    def tick(self):
        s = self.ctl.step(self.dt, cond=self.cond, wheel_mps=self.wheel,
                          visual_mps=self.visual, start=self.start,
                          recall=self.recall)
        # 판정은 매 주기 새로 받아야 한다. 남겨 두면 두절을 못 잡는다.
        self.cond = None

        t = Twist()
        t.linear.x = float(s.v_cmd)
        self.pub_cmd.publish(t)
        self.pub_state.publish(String(data=json.dumps(s.as_dict(),
                                                      ensure_ascii=False)))
        if s.state != self.prev_state:
            self.get_logger().info(
                f"{self.prev_state} → {s.state}  ({s.reason})")
            self.prev_state = s.state


def main():
    rclpy.init()
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
