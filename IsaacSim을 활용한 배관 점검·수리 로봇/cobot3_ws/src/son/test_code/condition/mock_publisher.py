"""[오프라인] 모의 발행기

Isaac Sim 도 실기도 없이 pipe_condition_node 를 돌려볼 수 있다.
scenes/ 의 합성 장면을 순서대로 또는 지정 장면만 반복 발행한다.

실행:
  python3 mock_publisher.py                      # 전 장면 순회
  python3 mock_publisher.py --scene disconnected  # 한 장면만
  python3 mock_publisher.py --joint 20            # 곡관 예외 확인
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

HERE = Path(__file__).resolve().parent
SCENES = HERE / "scenes"


class MockPublisher(Node):
    def __init__(self, names, meta, hz, joint_deg, dwell):
        super().__init__("mock_publisher")
        self.bridge = CvBridge()
        self.meta = meta
        self.names = names
        self.joint_deg = joint_deg
        self.dwell = dwell
        self.idx = 0
        self.count = 0
        self.cache = {}

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub_d = self.create_publisher(Image, "front/depth", qos)
        self.pub_c = self.create_publisher(Image, "front/rgb", qos)
        self.pub_i = self.create_publisher(CameraInfo, "front/camera_info", qos)
        self.pub_j = self.create_publisher(Float32, "joint_angle", 10)
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(
            f"{len(names)} 장면 발행, 장면당 {dwell} 프레임, 관절 {joint_deg}°")

    def load(self, name):
        if name not in self.cache:
            self.cache[name] = (
                np.load(SCENES / f"{name}_depth.npy").astype(np.float32),
                np.load(SCENES / f"{name}_rgb.npy").astype(np.uint8))
        return self.cache[name]

    def info_msg(self, stamp):
        m = CameraInfo()
        m.header.stamp = stamp
        m.header.frame_id = "front_camera"
        m.width = int(self.meta["width"])
        m.height = int(self.meta["height"])
        fx, fy = self.meta["fx"], self.meta["fy"]
        cx, cy = self.meta["ppx"], self.meta["ppy"]
        m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        m.distortion_model = "plumb_bob"
        m.d = [0.0] * 5
        return m

    def tick(self):
        name = self.names[self.idx]
        depth, rgb = self.load(name)
        stamp = self.get_clock().now().to_msg()

        dm = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        dm.header.stamp = stamp
        dm.header.frame_id = "front_camera"
        self.pub_d.publish(dm)

        cm = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        cm.header.stamp = stamp
        cm.header.frame_id = "front_camera"
        self.pub_c.publish(cm)

        self.pub_i.publish(self.info_msg(stamp))
        self.pub_j.publish(Float32(data=float(self.joint_deg)))

        self.count += 1
        if self.count == 1:
            truth = self.meta["scenes"][name]["truth"]
            self.get_logger().info(f"[{name}] 정답 {truth}")
        if self.count >= self.dwell:
            self.count = 0
            self.idx = (self.idx + 1) % len(self.names)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=None)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--joint", type=float, default=0.0)
    ap.add_argument("--dwell", type=int, default=30)
    args, rest = ap.parse_known_args()

    meta_path = SCENES / "scenes_meta.json"
    if not meta_path.is_file():
        print("scenes/ 가 없다. python3 make_scenes.py 를 먼저 실행할 것.")
        return 1
    meta = json.loads(meta_path.read_text())
    names = [args.scene] if args.scene else list(meta["scenes"])
    for n in names:
        if n not in meta["scenes"]:
            print(f"장면 없음: {n}   가능: {list(meta['scenes'])}")
            return 1

    rclpy.init(args=rest)
    node = MockPublisher(names, meta, args.hz, args.joint, args.dwell)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
