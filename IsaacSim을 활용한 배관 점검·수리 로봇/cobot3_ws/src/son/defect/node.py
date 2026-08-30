"""[ROS 3.10] 결함(구멍·비드) OpenCV 검출 ROS 2 노드 — 학습 모델 불필요.

**`pipe_inspect_demo`의 `pipe_vision_node`(YOLO Seg)를 건드리지 않는
완전히 별도의 노드다.** 같은 카메라 토픽을 보되, 무거운 추론 없이 순수
`cv2`(`defect/opencv_hole_detector.py`, `repair_demo.py`의 find_wall_hole
/find_weld_bead 이식)로만 검출한다 — 학습·GPU·torch 로딩이 없어 PC B 에서
가볍게 상시로 돌릴 수 있다.

🔑 **좌표 저장은 이 노드가 직접 하지 않는다.** 결함을 새로 확정하면
`~/mark_defect` 로 `{defect_id, kind, size_mm, confidence}` 만 요청하고,
**"지금 관 위의 어디냐"(호 위치 arc_mm + 원주 각도 roll_deg)는
`localization/node.py` 의 `DeadReckoner.mark_defect()` 가 그 시점의 자기
누적 상태로 채운다.** 이미 동작·검증된 메커니즘이라(test_code/localization/
test_deadreckon.py) 이쪽에서 오도메트리·IMU 쿼터니언을 다시 계산하지 않는다
(`pipe_inspect_demo` 쪽이 `/odom`·`/imu` 를 아무도 발행 안 하는 메시지
타입으로 구독하는 문제를 그대로 물려받지 않으려는 의도이기도 하다 —
`../pipe_inspect_demo/archive/yolo_vision_pipeline_backup_20260806/README.md`
참고).

🚨 **원주 각도는 근사치다.** 결함의 정확한 시계 위치를 로봇 롤 + 화면 내
결함 방위각으로 계산하지 않고, **결함이 시야에 들어온 순간의 로봇 절대 롤
(`imu_roll`)을 그대로 결함의 시계 위치로 쓴다.** 로봇이 결함에 근접해
INSPECT 하는 상황에서는 화면 내 편차가 각도 몇 도 수준이라 이 근사가
허용된다고 보지만, 화면 가장자리에서 처음 포착된 결함은 실제보다 몇 도
어긋날 수 있다 — 정밀 정렬(ALIGN)이 필요한 실제 용접 단계에서는
`repair_demo.py` 의 어안 역투영을 따로 써야 한다. **이 노드는 "검출해서
대략의 위치를 남긴다"까지만 하고, 로봇을 그 지점으로 이동시키는 로직은
의도적으로 넣지 않았다** — 다음 단계 작업이다.

수신
  ~/front/rgb           RGB (필수)
  ~/front/depth         Depth (선택 — 있으면 size_mm 을 mm 단위로 추정)
  ~/front/camera_info   카메라 내부 파라미터 (선택 — 없으면 fallback 상수)
  ~/imu_roll            결함의 원주 위치 근사에 쓰는 로봇 절대 롤 (deg)

발행
  ~/mark_defect         결함 확정 요청 (JSON) → localization 노드가 저장

실행:
  python3 node.py --ros-args --params-file config/defect_detect.yaml
"""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pyver import require_ros

require_ros(__file__)

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String

from opencv_hole_detector import OpenCvHoleDetector

DEFAULTS = dict(
    hole_dark_frac=0.12, hole_min_px_ref=300.0,
    bead_sat_min=60, bead_val_min=40, bead_min_px_ref=100.0,
    find_holes=True, find_beads=True,
    match_dist_px=40.0,          # 이전 프레임 같은 결함으로 볼 중심 거리
    confirm_frames=2,            # 이 프레임 수만큼 연속 관측돼야 확정
    track_timeout_s=1.0,         # 이 시간 이상 안 보이면 트랙을 버린다(재검출 시 새 ID)
    publish_hz=5.0,              # YOLO 대비 가벼우니 10Hz 도 무리 없지만 기본은 절반
)


def sensor_qos(depth=5):
    """이미지·IMU 스칼라 모두 Isaac Sim 기본이 BEST_EFFORT 라 맞춰야 수신된다."""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


class _Track:
    __slots__ = ("kind", "cx", "cy", "hits", "last_t", "confirmed", "defect_id")

    def __init__(self, kind, cx, cy):
        self.kind, self.cx, self.cy = kind, cx, cy
        self.hits, self.confirmed, self.defect_id = 1, False, None
        self.last_t = time.monotonic()


class DefectVisionNode(Node):
    """OpenCV 로 결함을 찾고, 연속 확인 후 `mark_defect` 로 좌표 저장을 요청한다."""

    def __init__(self):
        super().__init__("defect_vision_node")
        for key, val in DEFAULTS.items():
            self.declare_parameter(key, val)
        for key, val in (("rgb_topic", "front/rgb"),
                         ("depth_topic", "front/depth"),
                         ("camera_info_topic", "front/camera_info"),
                         ("roll_topic", "imu_roll"),
                         ("mark_topic", "mark_defect"),
                         ("fallback_fx", 523.9), ("fallback_fy", 523.9),
                         ("fallback_ppx", 640.0), ("fallback_ppy", 360.0)):
            self.declare_parameter(key, val)

        p = self.get_parameter
        self.detector = OpenCvHoleDetector(
            hole_dark_frac=p("hole_dark_frac").value,
            hole_min_px_ref=p("hole_min_px_ref").value,
            bead_sat_min=p("bead_sat_min").value,
            bead_val_min=p("bead_val_min").value,
            bead_min_px_ref=p("bead_min_px_ref").value)
        self.find_holes = bool(p("find_holes").value)
        self.find_beads = bool(p("find_beads").value)
        self.match_dist_px = float(p("match_dist_px").value)
        self.confirm_frames = int(p("confirm_frames").value)
        self.track_timeout_s = float(p("track_timeout_s").value)

        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.roll_deg = 0.0
        self.f_fish = float(p("fallback_fx").value)
        self.have_info = False
        self._tracks = []       # list[_Track]
        self._next_id = {"hole": 1, "bead": 1}

        self.create_subscription(Image, p("rgb_topic").value, self._on_rgb, sensor_qos())
        self.create_subscription(Image, p("depth_topic").value, self._on_depth, sensor_qos())
        self.create_subscription(CameraInfo, p("camera_info_topic").value, self._on_info, sensor_qos())
        self.create_subscription(Float32, p("roll_topic").value,
                                 lambda m: setattr(self, "roll_deg", float(m.data)), 10)
        self.mark_pub = self.create_publisher(String, p("mark_topic").value, 10)

        hz = float(p("publish_hz").value)
        self.create_timer(1.0 / max(hz, 0.1), self._tick)
        self.get_logger().info(
            f"결함 OpenCV 검출 시작 — {hz:.1f}Hz, "
            f"hole={self.find_holes} bead={self.find_beads}, "
            f"confirm_frames={self.confirm_frames} (YOLO 미사용, torch 로딩 없음)")

    def _on_rgb(self, msg):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warn(f"rgb 변환 실패: {exc}")

    def _on_depth(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"depth 변환 실패: {exc}")

    def _on_info(self, msg):
        if self.have_info:
            return
        # 어안이라 K 는 엄밀한 핀홀 행렬이 아니다. condition/node.py 와 같은 규약 —
        # K[0] 에 등거리 어안의 f(r=f*theta) 를 실어 보낸다고 본다.
        self.f_fish = float(msg.k[0])
        self.have_info = True

    def _size_mm(self, detection):
        """depth 가 있으면 등가반경(px)·f_fish 로 물리 지름(mm) 을 근사한다."""
        if self.depth is None:
            return 0.0
        cx, cy = int(round(detection.center_pixel[0])), int(round(detection.center_pixel[1]))
        h, w = self.depth.shape[:2]
        if not (0 <= cy < h and 0 <= cx < w):
            return 0.0
        region = self.depth[max(0, cy - 3):cy + 4, max(0, cx - 3):cx + 4]
        valid = region[np.isfinite(region) & (region > 0)]
        if valid.size == 0:
            return 0.0
        depth_m = float(np.median(valid))
        return round(2.0 * detection.r_eq_px * depth_m / max(self.f_fish, 1e-6) * 1000.0, 2)

    def _match_track(self, kind, cx, cy):
        best, best_d = None, self.match_dist_px
        for track in self._tracks:
            if track.kind != kind:
                continue
            d = math.hypot(cx - track.cx, cy - track.cy)
            if d <= best_d:
                best, best_d = track, d
        return best

    def _purge_stale(self, now):
        self._tracks = [t for t in self._tracks if now - t.last_t <= self.track_timeout_s]

    def _tick(self):
        if self.rgb is None:
            return
        detections = self.detector.detect(self.rgb, self.find_holes, self.find_beads)
        now = time.monotonic()
        for det in detections:
            cx, cy = det.center_pixel
            track = self._match_track(det.kind, cx, cy)
            if track is None:
                track = _Track(det.kind, cx, cy)
                self._tracks.append(track)
            else:
                track.cx, track.cy, track.hits = cx, cy, track.hits + 1
            track.last_t = now
            if track.confirmed or track.hits < self.confirm_frames:
                continue
            track.confirmed = True
            track.defect_id = f"{det.kind}_{self._next_id[det.kind]:04d}"
            self._next_id[det.kind] += 1
            self._publish_mark(track.defect_id, det)
        self._purge_stale(now)

    def _publish_mark(self, defect_id, detection):
        payload = {"defect_id": defect_id, "kind": detection.kind,
                  "size_mm": self._size_mm(detection),
                  "confidence": round(detection.confidence, 3)}
        self.mark_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(
            f"결함 확정 → mark_defect: id={defect_id} area={detection.area_px}px "
            f"size≈{payload['size_mm']}mm conf={payload['confidence']} "
            f"(현재 롤 {self.roll_deg:+.1f}° 를 원주 위치 근사로 씀)")


def main():
    rclpy.init()
    node = DefectVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
