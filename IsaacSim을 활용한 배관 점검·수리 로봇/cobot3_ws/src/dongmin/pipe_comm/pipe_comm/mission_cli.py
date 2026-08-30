"""임무 지령 발행 — ROS 노드 쪽에서 Isaac 쪽 로봇에게 출발·정지·복귀를 시킨다.

한 번 쏘고 끝나는 CLI 다. 대화형 콘솔이 아니라 스크립트에서 부르기 쉽게 뒀다.

  ros2 run pipe_comm mission_cli -- START
  ros2 run pipe_comm mission_cli -- STOP  --ns elbow_v
  ros2 run pipe_comm mission_cli -- SPEED --mps 0.05
  ros2 run pipe_comm mission_cli -- RECALL --reason "관 단절 감지"
  ros2 run pipe_comm mission_cli -- ESTOP  --ns all

지령
  START   주행 시작 / HOLD 에서 재개
  STOP    정지(HOLD). 재개 가능
  RECALL  복귀 — 방향을 뒤집어 출발점으로
  RETRY   끼임 탈출 재시도 (방향 전환)
  SPEED   주행 속도 변경.  --mps 를 같이 줄 것
  ESTOP   비상 정지. **재개 불가** — 사람이 풀어야 한다

🚨 **RELIABLE 로 보내지만 도착이 보장되는 것은 아니다.** 구독자가 아직 안
   붙었으면 그냥 사라진다(latch 가 아니다). 그래서 이 도구는 발행 뒤
   **구독자 수를 확인하고, 0 이면 경고와 함께 0 이 아닌 종료코드**를 낸다.
   `--wait` 를 주면 구독자가 붙을 때까지 기다렸다 쏜다.

🚨 `--ns all` 은 `map_test_demo.py` 의 세 코스 전부에 같은 지령을 뿌린다.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pipe_comm import contract
from pipe_comm.contract import Topics


class MissionCli(Node):

    def __init__(self, namespaces):
        super().__init__("mission_cli")
        self.pubs = {n: self.create_publisher(String, Topics(n).mission,
                                              contract.command_qos())
                     for n in namespaces}

    def n_peers(self):
        return {n: p.get_subscription_count() for n, p in self.pubs.items()}

    def wait_peers(self, timeout):
        """모든 대상에 구독자가 붙을 때까지 기다린다. 붙었으면 True."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if all(v > 0 for v in self.n_peers().values()):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def send(self, payload):
        msg = String(data=contract.dumps(payload))
        for n, p in self.pubs.items():
            p.publish(msg)
        # 발행 직후 바로 끝내면 DDS 가 아직 안 내보낸 상태다.
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.02)


def build_parser():
    p = argparse.ArgumentParser(
        prog="mission_cli", description="배관 로봇 임무 지령 발행")
    p.add_argument("cmd", choices=contract.COMMANDS, help="보낼 지령")
    p.add_argument("--ns", default=contract.DEFAULT_NS,
                   help="대상 네임스페이스. 'all' 이면 코스 3종 전부 "
                        f"({','.join(contract.COURSE_NS)})")
    p.add_argument("--mps", type=float, default=None,
                   help="SPEED 지령의 목표 속도 (m/s)")
    p.add_argument("--reason", default="", help="로그에 남길 사유")
    p.add_argument("--wait", type=float, default=0.0, metavar="SEC",
                   help="구독자가 붙을 때까지 최대 이만큼 기다렸다 쏜다")
    return p


def main(argv=None):
    # ros2 run 이 넘기는 --ros-args 이후는 우리 것이 아니다.
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    args = build_parser().parse_args(argv)

    if args.cmd == contract.CMD_SPEED and args.mps is None:
        print("[중단] SPEED 지령에는 --mps 가 있어야 한다", file=sys.stderr)
        return 2

    names = (list(contract.COURSE_NS) if args.ns == "all" else [args.ns])
    payload = contract.mission(args.cmd, mps=args.mps, reason=args.reason)

    rclpy.init()
    node = MissionCli(names)
    try:
        if args.wait > 0 and not node.wait_peers(args.wait):
            print(f"[경고] {args.wait:.0f}초를 기다려도 구독자가 안 붙었다",
                  file=sys.stderr)
        node.send(payload)
        peers = node.n_peers()
        for n in names:
            print(f"  {Topics(n).mission}  ← {contract.dumps(payload)}"
                  f"   구독자 {peers[n]}")
        if not any(peers.values()):
            print("\n[경고] 구독자가 0 이다 — 지령은 아무 데도 안 갔다.\n"
                  "       ① Isaac 쪽 시연이 떠 있는가\n"
                  f"       ② ROS_DOMAIN_ID 가 {contract.ROS_DOMAIN_ID} 인가\n"
                  f"       ③ --ns 가 맞는가 (지금 {names})",
                  file=sys.stderr)
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
