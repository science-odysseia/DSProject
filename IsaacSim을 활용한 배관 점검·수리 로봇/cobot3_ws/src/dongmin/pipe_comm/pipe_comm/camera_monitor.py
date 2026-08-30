"""카메라 수신 검증 — Isaac 이 보낸 RGB·Depth·CameraInfo 가 쓸 만한지 본다.

1초마다 토픽별 수신율(Hz)과 통계를 찍는다. 결함 검출을 붙이기 **전에** 이걸로
그림이 오는지부터 가린다 — YOLO 가 아무것도 못 찾을 때 "모델이 나쁜 건지
영상이 검은 건지" 를 여기서 먼저 갈라야 한다.

  rgb    해상도, 평균 밝기.  5/255 아래면 조명 문제다(관 내부는 로봇 조명이
         유일한 광원이라 intensity 를 안 올리면 통째로 검다)
  depth  유효 픽셀 비율, min/중앙값.  유효율이 낮으면 near clip 이 관벽을
         자르고 있거나 카메라가 본체 안에 파묻혀 있다
  info   fx/cx/cy.  어안이라 fx 는 등거리 모델의 f (r = f·theta) 다

프레임이 0 일 때 보는 순서:
  1. Isaac 이 **GUI** 로 떠 있는가 — headless 는 토픽만 있고 프레임이 0 이다
  2. **Play** 상태인가
  3. 양쪽 `ROS_DOMAIN_ID` 가 같은가 (우리 팀 143)
  4. 구독 QoS 가 BEST_EFFORT 인가 — 이 노드는 `contract.sensor_qos()` 를 쓴다

실행:
  ros2 run pipe_comm camera_monitor
  ros2 run pipe_comm camera_monitor --ros-args -p ns:=elbow_v
  ros2 run pipe_comm camera_monitor --ros-args -p ns:=tee -p save_dir:=/tmp/shots
"""

import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from pipe_comm import contract
from pipe_comm.contract import Topics
from pipe_comm import image_codec as codec

# 이 밝기 아래면 조명이 안 켜진 것으로 본다.
DARK_MEAN = 5.0


class CameraMonitor(Node):

    def __init__(self):
        super().__init__("camera_monitor")
        self.declare_parameter("ns", contract.DEFAULT_NS)
        self.declare_parameter("period_sec", 1.0)
        # 비워 두면 저장하지 않는다. 경로를 주면 주기마다 최신 프레임을 굽는다.
        self.declare_parameter("save_dir", "")

        ns = str(self.get_parameter("ns").value)
        self.t = Topics(ns)
        self.save_dir = str(self.get_parameter("save_dir").value).strip()
        if self.save_dir:
            Path(self.save_dir).mkdir(parents=True, exist_ok=True)

        qos = contract.sensor_qos()
        self.n = {"rgb": 0, "depth": 0, "depth_raw": 0, "rear_rgb": 0}
        self.last = {}
        self.info = None
        self.t0 = time.time()
        # 🚨 "한 번도 못 받았다" 와 "받다가 끊겼다" 는 **원인이 전혀 다르다.**
        #    앞은 설정 문제(도메인·QoS·ns), 뒤는 Isaac 이 멈춘 것이다. 누적
        #    수신 수와 마지막 수신 시각을 따로 들고 있어야 구분할 수 있다.
        #    (안 그러면 잘 받다 끊겼을 때 "GUI 인가?" 라는 틀린 진단이 나온다)
        self.total = 0
        self.last_rx = 0.0

        self.create_subscription(CompressedImage, self.t.rgb,
                                 lambda m: self._take("rgb", m), qos)
        self.create_subscription(CompressedImage, self.t.depth,
                                 lambda m: self._take("depth", m), qos)
        # 32FC1 원본도 같이 본다 — 압축이 깊이를 얼마나 깎았는지 대조용이다.
        self.create_subscription(Image, self.t.depth_raw,
                                 lambda m: self._take("depth_raw", m), qos)
        self.create_subscription(CompressedImage, self.t.rear_rgb,
                                 lambda m: self._take("rear_rgb", m), qos)
        self.create_subscription(CameraInfo, self.t.camera_info,
                                 self._on_info, qos)

        self.create_timer(float(self.get_parameter("period_sec").value),
                          self._report)
        contract.log_env(self.get_logger())
        self.get_logger().info(
            f"구독 시작 [{ns}] — {self.t.rgb} / {self.t.depth} / "
            f"{self.t.camera_info}  (BEST_EFFORT, domain "
            f"{contract.ROS_DOMAIN_ID})")

    def _take(self, key, msg):
        self.n[key] += 1
        self.last[key] = msg
        self.total += 1
        self.last_rx = time.time()

    def _on_info(self, msg):
        self.info = msg

    # ── 통계 ────────────────────────────────────────────────────
    def _rgb_line(self, key, msg):
        try:
            a = codec.jpeg_to_rgb(msg)
        except ValueError as exc:
            return f"  {key:9s} 디코딩 실패 — {exc}"
        mean = float(a.mean())
        warn = "   ⚠ 조명 확인 (관 내부는 로봇 조명이 유일한 광원)" \
            if mean < DARK_MEAN else ""
        return (f"  {key:9s} {a.shape[1]}x{a.shape[0]} jpeg "
                f"{len(msg.data) / 1024:.0f}KB  평균밝기 {mean:5.1f}/255{warn}")

    def _depth_line(self, key, msg, decode):
        try:
            d = decode(msg)
        except ValueError as exc:
            return f"  {key:9s} 디코딩 실패 — {exc}"
        fin = d[np.isfinite(d)]
        ratio = 100.0 * fin.size / d.size if d.size else 0.0
        if not fin.size:
            return f"  {key:9s} {d.shape[1]}x{d.shape[0]}  유효 픽셀 없음  ⚠"
        warn = "   ⚠ near clip / 카메라 매몰 확인" if ratio < 20.0 else ""
        return (f"  {key:9s} {d.shape[1]}x{d.shape[0]}  유효 {ratio:3.0f}%  "
                f"min {fin.min():.4f}  중앙 {float(np.median(fin)):.4f}  "
                f"max {fin.max():.4f} m{warn}")

    def _report(self):
        now = time.time()
        live = any(self.n.values())
        # 지금 프레임이 안 들어오면 아래 통계는 **낡은 값**이다. 현재 값처럼
        # 보이면 "멈춘 화면을 보며 잘 되고 있다" 고 오판한다.
        tag = "" if live else "  (낡음)"

        lines = ["  " + "  ".join(f"{k} {v}Hz" for k, v in self.n.items())]

        if "rgb" in self.last:
            lines.append(self._rgb_line("rgb", self.last["rgb"]) + tag)
        if "rear_rgb" in self.last:
            lines.append(self._rgb_line("rear_rgb", self.last["rear_rgb"]) + tag)
        if "depth" in self.last:
            lines.append(self._depth_line("depth", self.last["depth"],
                                          codec.png16_to_depth_m) + tag)
        if "depth_raw" in self.last:
            lines.append(self._depth_line("depth32", self.last["depth_raw"],
                                          codec.image_to_depth_m) + tag)
        if self.info is not None:
            k = self.info.k
            lines.append(f"  info      {self.info.width}x{self.info.height}  "
                         f"fx {k[0]:.1f}  cx {k[2]:.1f}  cy {k[5]:.1f}  "
                         f"모델 {self.info.distortion_model or '(없음)'}")

        if live:
            self._save()
        elif self.total == 0:
            # 한 번도 못 받았다 → **설정 문제**다
            lines.append(f"  프레임 0 ({now - self.t0:.0f}초째) — ① Isaac 이 "
                         f"떠 있고 Play 인가 ② ROS_DOMAIN_ID 가 "
                         f"{contract.ROS_DOMAIN_ID} 인가 ③ ns 파라미터가 "
                         f"맞는가 (지금 {self.t.ns!r}) ④ 발행 QoS 가 "
                         f"BEST_EFFORT 인가")
        else:
            # 받다가 끊겼다 → **Isaac 이 멈춘 것**이다. 설정은 이미 맞았다
            lines.append(f"  ⛔ 프레임 끊김 {now - self.last_rx:.0f}초 "
                         f"(누적 {self.total}장 받았다) — Isaac 이 살아 있는지, "
                         f"Play 상태인지 확인할 것")

        self.get_logger().info("\n".join(lines))
        for k in self.n:
            self.n[k] = 0

    def _save(self):
        """최신 프레임을 파일로 굽는다. 눈으로 봐야 할 때만 쓴다."""
        if not self.save_dir:
            return
        import cv2
        stamp = time.strftime("%H%M%S")
        if "rgb" in self.last:
            a = codec.jpeg_to_rgb(self.last["rgb"])
            cv2.imwrite(f"{self.save_dir}/rgb_{stamp}.png", a[:, :, ::-1])
        if "depth" in self.last:
            d = codec.png16_to_depth_m(self.last["depth"])
            fin = d[np.isfinite(d)]
            if fin.size:
                # 보이라고 정규화하는 것이다 — 이 png 로 거리를 재면 안 된다.
                v = np.nan_to_num((d - fin.min()) /
                                  max(fin.max() - fin.min(), 1e-6))
                cv2.imwrite(f"{self.save_dir}/depth_{stamp}.png",
                            (v * 255).astype(np.uint8))


def main(args=None):
    rclpy.init(args=args)
    node = CameraMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
