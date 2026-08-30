"""주행/정지 알림 수신 — 로봇이 지금 어디서 무엇을 하는지 한 줄로 찍는다.

로봇 여러 대를 **한 노드에서** 본다. `map_test_demo.py` 는 코스마다 로봇이
한 대씩(elbow_v / elbow_h / tee) 이라 셋을 동시에 봐야 한다.

받는 것 (로봇마다)
  moving        std_msgs/Bool       주행 중인가 — 전환될 때만 알림을 낸다
  drive_state   std_msgs/String     JSON 상태 10Hz (state·s·이탈·왕복·끼임)
  event         std_msgs/String     JSON 사건 (출발·도착·끼임·이탈·단절 …)
  odom          nav_msgs/Odometry   진행거리·속도 (표준 소비자용)
  imu           sensor_msgs/Imu     롤 — 결함의 시계각 기준

🔑 **사건(event)과 상태(drive_state)를 둘 다 받는 이유.** 상태는 10Hz 표본이라
   그 사이에 일어난 전환은 통째로 사라진다 — 끼였다가 즉시 빠져나온 경우가
   그렇다. 놓치면 안 되는 것은 RELIABLE 인 `event` 로 따로 온다.

🚨 **타임아웃 감시가 이 노드의 핵심이다.** Isaac 이 죽거나 물리가 멈추면
   토픽이 조용해질 뿐 에러는 안 난다. 마지막 수신에서 `timeout_sec` 이 지나면
   **[두절]** 로 찍는다 — 안 그러면 마지막 줄이 화면에 남아 "잘 가고 있다"로
   보인다.

실행:
  ros2 run pipe_comm drive_monitor                                   # /robot
  ros2 run pipe_comm drive_monitor --ros-args -p ns:="[elbow_v,elbow_h,tee]"
  ros2 run pipe_comm drive_monitor --ros-args -p ns:="[elbow_v]" -p period_sec:=0.5
"""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

from pipe_comm import contract
from pipe_comm.contract import Topics

# 이 시간 동안 drive_state 가 안 오면 두절로 본다.
DEFAULT_TIMEOUT = 3.0


def _quat_to_roll(q):
    """쿼터니언 → 롤(도). 관 축이 X 이므로 X 회전이 곧 결함의 시계각이다.

    🚨 시계각 0° = 천장, 180° = 바닥. 부호가 뒤집히면 토치 링을 **정반대로**
       돌려 엉뚱한 곳을 용접한다. 발행 쪽(state_bridge)의 부호 규약과 반드시
       같이 봐야 한다.
    """
    s = 2.0 * (q.w * q.x + q.y * q.z)
    c = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.degrees(math.atan2(s, c))


class RobotView:
    """로봇 한 대의 최신 수신 상태."""

    def __init__(self, ns):
        self.ns = ns
        self.state = None          # drive_state JSON dict
        self.moving = None         # Bool
        self.odom_s_m = None       # 진행거리 (m)
        self.odom_v = None         # 속도 (m/s)
        self.roll_deg = None
        self.last_rx = 0.0         # 마지막 drive_state 수신 시각
        self.n_state = 0
        self.n_event = 0
        self.warned_lost = False

    def line(self, now, timeout):
        """콘솔 한 줄. 못 받고 있으면 그 사실이 먼저 보여야 한다."""
        if self.last_rx and now - self.last_rx > timeout:
            return (f"{self.ns:9s} ⛔ 두절 {now - self.last_rx:4.1f}초 "
                    f"(마지막 상태 {(self.state or {}).get('state', '?')})")
        if self.state is None:
            return f"{self.ns:9s} … 수신 대기"
        d = self.state
        arrow = "→" if d.get("dir", 1) > 0 else "←"
        tot = d.get("s_total_mm") or 0.0
        pct = f"{100.0 * d.get('s_mm', 0.0) / tot:3.0f}%" if tot else "  - "
        bits = [f"{self.ns:9s}",
                f"{'주행' if d.get('moving') else '정지'}",
                f"{d.get('state', '?'):7s}",
                f"s {d.get('s_mm', 0.0):6.0f}mm {arrow} {pct}",
                f"이탈 {d.get('off_mm', 0.0):4.1f}mm",
                f"왕복 {d.get('lap', 0)}",
                f"끼임 {d.get('stuck', 0)}"]
        if self.roll_deg is not None:
            bits.append(f"롤 {self.roll_deg:+6.1f}°")
        if self.odom_v is not None:
            bits.append(f"v {self.odom_v * 1000:4.0f}mm/s")
        if d.get("reason"):
            bits.append(f"— {d['reason']}")
        return "  ".join(bits)


class DriveMonitor(Node):

    def __init__(self):
        super().__init__("drive_monitor")
        # 문자열 배열 파라미터. 한 대만 볼 때도 배열로 준다.
        self.declare_parameter("ns", [contract.DEFAULT_NS])
        self.declare_parameter("period_sec", 1.0)
        self.declare_parameter("timeout_sec", DEFAULT_TIMEOUT)

        names = list(self.get_parameter("ns").value) or [contract.DEFAULT_NS]
        self.timeout = float(self.get_parameter("timeout_sec").value)
        self.views = {n: RobotView(n) for n in names}

        sq, cq = contract.sensor_qos(), contract.command_qos()
        for n in names:
            t = Topics(n)
            # 🔑 람다 기본인자로 n 을 묶는다. 안 묶으면 마지막 로봇 하나에
            #    전부 몰린다(파이썬 늦은 바인딩 — 조용히 틀린다).
            self.create_subscription(
                String, t.drive_state,
                lambda m, k=n: self._on_state(k, m), cq)
            self.create_subscription(
                String, t.event, lambda m, k=n: self._on_event(k, m), cq)
            self.create_subscription(
                Bool, t.moving, lambda m, k=n: self._on_moving(k, m), cq)
            self.create_subscription(
                Odometry, t.odom, lambda m, k=n: self._on_odom(k, m), sq)
            self.create_subscription(
                Imu, t.imu, lambda m, k=n: self._on_imu(k, m), sq)

        self.create_timer(float(self.get_parameter("period_sec").value),
                          self._report)
        contract.log_env(self.get_logger())
        self.get_logger().info(
            f"주행 감시 시작 — 로봇 {len(names)}대 {names} "
            f"(domain {contract.ROS_DOMAIN_ID}, 두절 판정 {self.timeout:.0f}초)")

    # ── 수신 ────────────────────────────────────────────────────
    def _on_state(self, ns, msg):
        d = contract.parse(msg.data)
        if d is None:
            self.get_logger().warn(f"[{ns}] drive_state JSON 이 깨졌다: "
                                   f"{msg.data[:80]!r}")
            return
        v = self.views[ns]
        v.state, v.last_rx, v.n_state = d, time.time(), v.n_state + 1
        if v.warned_lost:
            self.get_logger().info(f"[{ns}] ✅ 수신 복구")
            v.warned_lost = False

    def _on_event(self, ns, msg):
        d = contract.parse(msg.data)
        if d is None:
            return
        self.views[ns].n_event += 1
        kind, detail = d.get("event", "?"), d.get("detail", "")
        line = (f"[{ns}] ▶ {kind}  s={d.get('s_mm', 0.0):.0f}mm"
                + (f"  {detail}" if detail else ""))
        # 알림 사건은 경고로 올린다 — 로그가 흐를 때 눈에 걸려야 한다.
        (self.get_logger().warn if d.get("alert")
         else self.get_logger().info)(line)

    def _on_moving(self, ns, msg):
        v = self.views[ns]
        # 전환될 때만 찍는다. 매번 찍으면 로그가 이것만으로 찬다.
        if v.moving is not None and bool(msg.data) != v.moving:
            self.get_logger().info(
                f"[{ns}] {'▶ 주행 시작' if msg.data else '■ 정지'}")
        v.moving = bool(msg.data)

    def _on_odom(self, ns, msg):
        v = self.views[ns]
        # 🔑 진행거리는 pose.position.x 가 아니라 **중심선 호길이** 다.
        #    발행 쪽이 x 에 s 를 싣는 규약이다(곡관에서 직선거리는 뒤로 간다).
        v.odom_s_m = float(msg.pose.pose.position.x)
        v.odom_v = float(msg.twist.twist.linear.x)

    def _on_imu(self, ns, msg):
        self.views[ns].roll_deg = _quat_to_roll(msg.orientation)

    # ── 보고 ────────────────────────────────────────────────────
    def _report(self):
        now = time.time()
        for v in self.views.values():
            if v.last_rx and now - v.last_rx > self.timeout and not v.warned_lost:
                v.warned_lost = True
                self.get_logger().error(
                    f"[{v.ns}] ⛔ 상태 두절 {self.timeout:.0f}초 — Isaac 이 "
                    f"살아 있는지, Play 상태인지 확인할 것")
        self.get_logger().info(
            "\n".join("  " + v.line(now, self.timeout)
                      for v in self.views.values()))


def main(args=None):
    rclpy.init(args=args)
    node = DriveMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
