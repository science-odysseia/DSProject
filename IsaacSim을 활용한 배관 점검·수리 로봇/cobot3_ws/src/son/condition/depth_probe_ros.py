"""[ROS 3.10] Depth 반환 방식 확인 진단 — Isaac Sim 연결 후 1회 실행.

빈 공간의 Depth 를 시뮬레이터가 어떻게 돌려주는지 확인한다.
  0 으로 반환      → invalid_mode: empty_is_zero
  최대값/inf 로 채움 → invalid_mode: empty_is_far  (max_range_m 도 같이 설정)

이 확인 없이 판정 노드를 돌리면 DISCONNECTED 가 전혀 작동하지 않는다.

RealSense 실물 최소 측정 거리도 함께 본다. 관 내벽까지가 25mm 뿐이라
근접 벽면이 무효로 나오면 무효율이 상시 높아 오탐이 발생한다.

실행:
  python3 depth_probe.py --ros-args -r __ns:=/scout
  python3 depth_probe.py --topic /scout/front/depth --frames 60
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pyver import require_ros

require_ros(__file__)

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class DepthProbe(Node):
    def __init__(self, topic, frames):
        super().__init__("depth_probe")
        self.bridge = CvBridge()
        self.want = frames
        self.seen = 0
        self.acc = []
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, topic, self.on_depth, qos)
        self.get_logger().info(f"{topic} 구독, {frames} 프레임 수집")
        self.get_logger().info(
            "프레임이 0 이면: headless 여부 확인. headless 에서는 카메라 "
            "토픽이 목록에 보여도 프레임이 발행되지 않는다.")

    def on_depth(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        a = np.asarray(img, dtype=np.float64)
        self.acc.append(a)
        self.seen += 1
        if self.seen == 1:
            self.get_logger().info(
                f"첫 프레임 {a.shape} dtype={img.dtype} "
                f"encoding={msg.encoding}")

    def report(self):
        if not self.acc:
            print("\n[실패] 프레임을 하나도 받지 못했다.")
            print("  - GUI 로 띄웠는가 (headless 는 카메라 프레임 0)")
            print("  - 구독 QoS 가 BEST_EFFORT 인가")
            print("  - ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY=0 확인")
            return 1

        a = np.stack(self.acc)
        finite = np.isfinite(a)
        n = a.size
        zero = int((a == 0).sum())
        nan = int(np.isnan(a).sum())
        inf = int(np.isinf(a).sum())
        v = a[finite & (a > 0)]

        print("\n" + "=" * 70)
        print(f"수집 {len(self.acc)} 프레임, 픽셀 {n:,}")
        print("=" * 70)
        print(f"  0 인 픽셀     {zero:10,}  ({zero / n * 100:6.2f}%)")
        print(f"  NaN           {nan:10,}  ({nan / n * 100:6.2f}%)")
        print(f"  inf           {inf:10,}  ({inf / n * 100:6.2f}%)")
        if v.size:
            print(f"  유효값 범위   {v.min():.4f} ~ {v.max():.4f}")
            print(f"  분위수 1/50/99  {np.percentile(v, 1):.4f} / "
                  f"{np.percentile(v, 50):.4f} / {np.percentile(v, 99):.4f}")
            hi = np.percentile(v, 99.5)
            at_max = int((v > hi * 0.999).sum())
            print(f"  상위값 뭉침   {at_max:,} 픽셀이 {hi:.4f} 부근")

        print("-" * 70)
        if zero / n > 0.01:
            mode, note = "empty_is_zero", "0 이 유의미하게 존재"
        elif nan + inf > 0:
            mode, note = "empty_is_zero", "NaN/inf 사용 — 코드가 0 과 동일 취급"
        else:
            mode, note = "empty_is_far", "0 도 NaN 도 없음 — 먼 값으로 채우는 방식"
        print(f"  권고 invalid_mode: {mode}   ({note})")
        if mode == "empty_is_far" and v.size:
            print(f"  권고 max_range_m : {np.percentile(v, 99) * 0.98:.3f}")

        print("-" * 70)
        print("근접 벽면 확인 (관 내벽 25mm 가 유효하게 나오는가)")
        if v.size:
            near = int((v < 0.030).sum())
            print(f"  30mm 미만 유효 픽셀 {near:,} "
                  f"({near / max(v.size, 1) * 100:.2f}%)")
            if near == 0:
                print("  [경고] 근접 벽면 Depth 가 없다. 무효율이 상시 높아져")
                print("         DISCONNECTED 오탐이 발생한다. 결함 치수 실측을")
                print("         RGB 기반으로 전환할지 검토할 것.")
            else:
                print("  [OK] 근접 벽면이 유효하게 측정된다")
        print("=" * 70)
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="front/depth")
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--timeout", type=float, default=40.0)
    args, rest = ap.parse_known_args()

    rclpy.init(args=rest)
    node = DepthProbe(args.topic, args.frames)
    t0 = node.get_clock().now()
    while rclpy.ok() and node.seen < node.want:
        rclpy.spin_once(node, timeout_sec=0.2)
        if (node.get_clock().now() - t0).nanoseconds / 1e9 > args.timeout:
            node.get_logger().warn("시간 초과")
            break
    code = node.report()
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
