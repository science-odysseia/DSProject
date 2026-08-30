"""[ROS 3.10] 추측 항법 ROS 2 노드.

로봇은 배관 도면을 갖지 않는다. 바퀴 회전과 관절 꺾임만으로 지나온 길을
기록해 **상대 경로**를 발행한다. 도면 매칭은 관리자 쪽 몫이다.

구독
  ~/wheel_speed    휠 6개 평균 접지 속도 (m/s)
  ~/visual_speed   시각 오도메트리 (m/s) — 휠이 헛도는 구간을 잡는다
  ~/joint_angle    관절 엔코더 (deg) — 곡관 진입·이탈과 회전 방향
  ~/imu_yaw_rate   요 각속도 (deg/s) — 회전량 적분
  ~/imu_roll       중력 기준 절대 롤 (deg) — 결함의 원주 위치
  ~/mark_defect    결함 표시 요청 (JSON 문자열)

발행
  ~/pose           현재 호 위치·누적 회전·롤 (JSON)
  ~/path           구간 열 (JSON) — 관리자가 도면에 얹는다
  ~/defect_marks   결함 목록 (JSON)

실행:
  python3 node.py --ros-args --params-file config/localization.yaml
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pyver import require_ros

require_ros(__file__)

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from deadreckon import DEFAULTS, DeadReckoner


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node")
        for key, val in DEFAULTS.items():
            self.declare_parameter(key, val)
        for key, val in (("wheel_topic", "wheel_speed"),
                         ("visual_topic", "visual_speed"),
                         ("joint_topic", "joint_angle"),
                         ("yaw_topic", "imu_yaw_rate"),
                         ("roll_topic", "imu_roll"),
                         ("mark_topic", "mark_defect"),
                         ("pose_topic", "pose"),
                         ("path_topic", "path"),
                         ("marks_topic", "defect_marks"),
                         ("rate_hz", 20.0),
                         ("path_every", 10),
                         ("input_timeout_s", 0.5)):
            self.declare_parameter(key, val)

        p = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.dr = DeadReckoner(p)
        self.wheel = 0.0
        self.visual = None
        self.joint = 0.0
        self.yaw = 0.0
        self.roll = None
        # 신호가 끊겼는데 마지막 값을 계속 적분하면 거리가 무한정 늘어난다.
        # 실측으로 잡았다 — 발행이 멈춘 뒤 +1254mm 가 더 쌓였다.
        self._last_wheel_t = None
        self._last_visual_t = None

        g = self.get_parameter
        self.create_subscription(Float32, g("wheel_topic").value,
                                 self.on_wheel, 10)
        self.create_subscription(Float32, g("visual_topic").value,
                                 self.on_visual, 10)
        self.create_subscription(Float32, g("joint_topic").value,
                                 lambda m: setattr(self, "joint", float(m.data)), 10)
        self.create_subscription(Float32, g("yaw_topic").value,
                                 lambda m: setattr(self, "yaw", float(m.data)), 10)
        self.create_subscription(Float32, g("roll_topic").value,
                                 lambda m: setattr(self, "roll", float(m.data)), 10)
        self.create_subscription(String, g("mark_topic").value,
                                 self.on_mark, 10)

        self.pub_pose = self.create_publisher(String, g("pose_topic").value, 10)
        self.pub_path = self.create_publisher(String, g("path_topic").value, 10)
        self.pub_marks = self.create_publisher(String, g("marks_topic").value, 10)

        hz = float(g("rate_hz").value)
        self.dt = 1.0 / max(hz, 1.0)
        self.every = int(g("path_every").value)
        self.n = 0
        self.turns = 0
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f"추측 항법 시작 — {hz:.0f} Hz. 도면 없이 상대 경로만 낸다")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_wheel(self, msg):
        self.wheel = float(msg.data)
        self._last_wheel_t = self._now()

    def on_visual(self, msg):
        self.visual = float(msg.data)
        self._last_visual_t = self._now()

    def _fresh(self, t):
        if t is None:
            return False
        return (self._now() - t) <= float(
            self.get_parameter("input_timeout_s").value)

    def on_mark(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().error(f"mark 파싱 실패: {exc}")
            return
        m = self.dr.mark_defect(d.get("defect_id", f"D{len(self.dr.defects) + 1}"),
                                kind=d.get("kind", ""),
                                size_mm=float(d.get("size_mm", 0.0)),
                                confidence=float(d.get("confidence", 0.0)))
        self.get_logger().info(
            f"결함 {m.defect_id} 기록 — 호 {m.arc_mm:.1f}mm 롤 {m.roll_deg:+.1f}°")
        self.pub_marks.publish(String(data=json.dumps(
            [x.as_dict() for x in self.dr.defects], ensure_ascii=False)))

    def tick(self):
        # 신호가 오래됐으면 정지로 본다. 안 그러면 발행이 멈춰도 거리가 계속 쌓인다.
        w = self.wheel if self._fresh(self._last_wheel_t) else 0.0
        v = self.visual if self._fresh(self._last_visual_t) else None
        if self._last_wheel_t is not None and not self._fresh(self._last_wheel_t):
            if not getattr(self, "_warned", False):
                self.get_logger().warn("wheel_speed 두절 — 적산을 멈춘다")
                self._warned = True
        elif self._fresh(self._last_wheel_t):
            self._warned = False
        p = self.dr.step(self.dt, wheel_mps=w, visual_mps=v,
                         joint_deg=self.joint, yaw_rate_dps=self.yaw,
                         roll_deg=self.roll)
        self.pub_pose.publish(String(data=json.dumps(dict(
            arc_mm=p.arc_mm, heading_deg=p.heading_deg, roll_deg=p.roll_deg,
            kind=p.kind, turns=p.turns, slip_ratio=p.slip_ratio,
            wheel_mm=p.wheel_mm, corrected_mm=p.corrected_mm),
            ensure_ascii=False)))
        self.n += 1
        if self.n % self.every == 0:
            self.pub_path.publish(String(data=json.dumps(
                [s.as_dict() for s in self.dr.path()], ensure_ascii=False)))
        if p.turns != self.turns:
            self.turns = p.turns
            last = p.segments[-1]
            self.get_logger().info(
                f"곡관 {p.turns}번째 통과 — {last.direction} "
                f"{last.turn_deg:+.1f}°, 호 {last.length_mm:.1f}mm "
                f"(측정차 {last.spec_gap_mm:+.1f}mm)")


def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
