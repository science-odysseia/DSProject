"""real_map_demo.py(Isaac Sim)가 발행하는 활성 카메라 스트림을 실시간으로 본다.

`integration_test/tools/view_active_cam.py`(패키지 없는 일반 스크립트)를
`ros2 run`으로 돌릴 수 있게 이 패키지 노드로 옮긴 것 — 로직은 동일하다.

구독 토픽
    /repair_robot/active_cam/rgb/compressed   sensor_msgs/CompressedImage
    /repair_robot/active_cam/which             std_msgs/String  (현재 활성 카메라 이름)
    /repair_robot/opencv_judgement/json        std_msgs/String  (아래 참고)

창 하나에 active_cam 실시간 영상만 띄우고, 그 위에 OpenCV 판정을 **매
프레임 그대로 겹쳐 그린다**. real_map_demo.py 가 front_camera 가 활성일
때마다(최대 10Hz) find_wall_hole/find_weld_bead 를 그 프레임에 대해 돌려
좌표·판정 근거만 JSON 으로 발행하고(이미 렌더된 이미지가 아니다), 이 노드가
그 좌표를 지금 들어오는 raw 프레임 위에 직접 그린다 — 그래서 판정이 나는
"순간의 정지 이미지"가 아니라 로봇이 결함에 다가가는 동안 원이 실시간으로
따라온다. **표시 전용이다** — 결함을 실제로 등록·확정하는 시점은 지금도
INSPECT/RECHECK/VERIFY 세 번뿐이고, 이 오버레이는 그 판정을 바꾸지 않는다.

m 키 — 원본→엣지검출(Canny) 순환. q 또는 Esc 로 종료한다.

실행:
    ros2 run pipe_inspect_demo view_active_cam
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

MODES = ("raw", "edge")
WINDOW_NAME = "active_cam"
PLACEHOLDER_H, PLACEHOLDER_W = 360, 640
# opencv_judgement 가 이 시간 안에 들어온 것만 "지금 유효한 판정"으로 겹쳐
# 그린다 — real_map_demo.py 가 front_camera 활성일 때만 최대 10Hz 로 계속
# 보내므로(연속 스트림), 짧게 잡아도 끊기지 않는다. 카메라가 back/torch 로
# 바뀌거나 결함이 없으면 발행이 멈추고 이 창에서도 곧바로 사라진다.
JUDGE_FRESH_S = 1.0


def _placeholder(text, h=PLACEHOLDER_H, w=PLACEHOLDER_W):
    """프레임이 아직 안 온 쪽을 빈 화면 대신 안내 문구로 채운다."""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    cv2.putText(img, text, (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    return img


def _process(frame, mode):
    """frame(BGR) 을 mode 에 맞게 처리해 표시용 BGR 이미지를 돌려준다."""
    if mode == "edge":
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 80, 160)
        return cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    return frame.copy()


def _label(img, pos, text, color):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def _speed_color(speed):
    return {"stop": (0, 0, 255), "slow": (0, 200, 255), "full": (0, 255, 0)}.get(speed, (200, 200, 200))


def draw_judgement(frame_bgr, judgement):
    """real_map_demo.py::_judgement_payload() 가 보낸 판정을 프레임 위에 그린다.

    frame_bgr 은 cv2.imdecode 로 받은 진짜 BGR 배열이라(Isaac 쪽이 발행 전에
    이미 뒤집어서 encode 함) 여기서는 cv2 관례대로 (0,0,255)=빨강 그대로
    쓴다 — real_map_demo.py::draw_inspect_debug() 는 반대로 RGB 배열에
    그리기 때문에 거기서는 (255,0,0)이 빨강이다(헷갈리기 쉬운 부분).
    """
    img = frame_bgr.copy()
    px = judgement.get("px")
    win_px = int(judgement.get("win_px") or 70)
    if px is not None:
        ex, ey = int(px[0]), int(px[1])
        cv2.rectangle(img, (ex - win_px, ey - win_px), (ex + win_px, ey + win_px), (160, 160, 160), 1)
        cv2.drawMarker(img, (ex, ey), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)

    hole = judgement.get("hole")
    if hole:
        cx, cy = int(hole["cx"]), int(hole["cy"])
        r = max(4, int(hole["r_eq_px"]))
        matched = hole.get("matched", True)
        color = (0, 255, 0) if matched else (0, 0, 255)
        cv2.circle(img, (cx, cy), r, color, 2)
        text = (f"hole {hole['area_px']}px thr={hole['thr']:.0f} "
               f"d={hole.get('dist_px', 0):.0f}px "
               f"{'MATCH' if matched else 'REJECT'}")
        if hole.get("rim_m"):
            text += f" rim={hole['rim_m'] * 1000:.0f}mm"
        _label(img, (cx + r + 4, max(12, cy)), text, color)

    bead = judgement.get("bead")
    if bead:
        cx, cy = int(bead["cx"]), int(bead["cy"])
        r = max(4, int(bead["r_eq_px"]))
        matched = bead.get("matched", True)
        color = (0, 140, 255) if matched else (0, 0, 255)
        cv2.circle(img, (cx, cy), r, color, 2)
        text = f"bead {bead['area_px']}px d={bead.get('dist_px', 0):.0f}px"
        if "matched" in bead:
            text += " MATCH" if matched else " REJECT"
        _label(img, (cx + r + 4, max(12, cy) + 16), text, color)

    # 🚨 cv2.putText(Hershey 폰트)는 한글을 못 그린다 — 화면에 물음표로
    #    깨진다(2026-08-08 실측, 스크린샷으로 확인). 이 오버레이 텍스트는
    #    전부 영문/숫자만 쓴다. cond["reason"]은 PipeConditionDetector 가
    #    한글로 채우므로 아예 표시하지 않는다(state/circularity/roughness/
    #    offset_mm 로 이미 같은 정보가 영문으로 나간다).
    cond = judgement.get("cond")
    joint_deg = judgement.get("joint_deg")
    if cond is not None or joint_deg is not None:
        y = 48
        # 🚨 2026-08-10 — curve_ahead(관절 비틀림이 문턱을 넘어 원형도 판정을
        #    건너뛴 시점)의 근거를 화면에 남긴다(사용자 지시). 이때
        #    circularity/roughness 는 detector.run() 이 재지 않고 0.0 그대로
        #    돌려준 값이라(위 조기 반환) 그대로 보여주면 "실측 0"으로
        #    오해하기 쉽다 — 대신 문턱 비교식을 보여준다.
        curve_ahead = bool(cond.get("curve_ahead")) if cond is not None else False
        if curve_ahead:
            # 텍스트만으로는 눈에 잘 안 띈다는 지적(사용자) — 화면 전체에
            # 굵은 테두리를 둘러 프레임 단위로 한눈에 보이게 한다.
            fh, fw = img.shape[:2]
            cv2.rectangle(img, (0, 0), (fw - 1, fh - 1), (0, 140, 255), 8)
        if joint_deg is not None:
            jcolor = (0, 140, 255) if curve_ahead else (0, 255, 255)
            _label(img, (10, y), f"joint(bellows) bend {joint_deg:.1f}deg", jcolor)
            y += 18
        if cond is not None:
            color = _speed_color(cond.get("speed"))
            _label(img, (10, y), f"pipe {cond.get('state')}  ({cond.get('speed')})", color)
            y += 18
            if curve_ahead:
                thr = cond.get("bend_threshold_deg", 0.0)
                _label(img, (10, y),
                      f"CURVE AHEAD: joint {abs(joint_deg or 0.0):.1f}deg >= {thr:.1f}deg "
                      f"-> circularity check skipped", (0, 140, 255))
            else:
                _label(img, (10, y),
                      f"circularity {cond.get('circularity', 0):.3f}  roughness {cond.get('roughness', 0):.4f}  "
                      f"offset {cond.get('offset_mm', 0):.1f}mm", color)

    # 🚨 2026-08-10 — VERIFY(2차 검증)에서만 weld_done=True 로 온다(사용자
    #    지시). bead 오버레이는 그대로 두고 그 위에 완료 배너만 얹는다.
    if judgement.get("weld_done"):
        h, w = img.shape[:2]
        bw, bh = 220, 26
        bx, by = (w - bw) // 2, 8
        cv2.rectangle(img, (bx, by), (bx + bw, by + bh), (0, 200, 0), -1)
        cv2.putText(img, "WELD COMPLETE", (bx + 12, by + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)

    return img


class ActiveCamViewerNode(Node):
    """active_cam 영상을 구독해 창 하나에 실시간 표시하고 OpenCV 판정을 겹쳐 그린다."""

    def __init__(self):
        """토픽 parameter, 구독자, OpenCV 창을 초기화한다."""
        super().__init__("view_active_cam")
        defaults = {
            "rgb_topic": "/repair_robot/active_cam/rgb/compressed",
            "which_topic": "/repair_robot/active_cam/which",
            "opencv_judgement_topic": "/repair_robot/opencv_judgement/json",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.which = "?"
        self.frame = None
        self.n_frames = 0
        self.judgement = None
        self.judgement_t = 0.0
        self.n_judgements = 0
        self.mode_idx = 0
        self.create_subscription(String, str(self.get_parameter("which_topic").value), self._on_which, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, str(self.get_parameter("rgb_topic").value), self._on_image, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("opencv_judgement_topic").value), self._on_judgement, qos_profile_sensor_data)
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.create_timer(1.0 / 30.0, self._refresh_windows)
        self.get_logger().info(f"active_cam 뷰어 시작 — m 키로 처리 모드 순환({' → '.join(MODES)}), q/Esc 로 종료")

    def _on_which(self, message):
        """지금 active_cam 이 내보내는 카메라 이름(front/back/torch)을 기록한다."""
        self.which = message.data

    def _on_image(self, message):
        """active_cam 압축 RGB 를 디코딩해 최신 프레임으로 들고 있는다."""
        buf = np.frombuffer(message.data, dtype=np.uint8)
        # Isaac 쪽이 BGR 로 뒤집어서 imencode 했으므로(camera/rig.py 와 같은
        # 관례) imdecode 결과도 그대로 BGR — cv2.imshow 에 바로 넣으면 된다.
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            self.get_logger().warn("active_cam 압축 영상을 디코딩할 수 없어.", throttle_duration_sec=5.0)
            return
        self.frame = bgr
        self.n_frames += 1

    def _on_judgement(self, message):
        """OpenCV(find_wall_hole/find_weld_bead) 실시간 판정 좌표를 받는다."""
        try:
            self.judgement = json.loads(message.data)
        except (ValueError, TypeError):
            return
        self.judgement_t = time.time()
        self.n_judgements += 1

    def _refresh_windows(self):
        """새 프레임 유무와 무관하게 창과 키 입력을 계속 처리한다."""
        if self.frame is not None:
            mode = MODES[self.mode_idx]
            disp = _process(self.frame, mode)
        else:
            disp = _placeholder("waiting for active_cam...")

        if self.judgement is not None and (time.time() - self.judgement_t) <= JUDGE_FRESH_S:
            disp = draw_judgement(disp, self.judgement)

        cv2.imshow(WINDOW_NAME, disp)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            self.get_logger().info(f"종료 — 받은 프레임 {self.n_frames}개, 마지막 활성 카메라 '{self.which}'")
            rclpy.shutdown()
        elif key == ord("m"):
            self.mode_idx = (self.mode_idx + 1) % len(MODES)

    def close_windows(self):
        """노드가 사용한 OpenCV 표시 창을 닫는다."""
        cv2.destroyWindow(WINDOW_NAME)
        cv2.waitKey(1)


def main(args=None):
    """ROS 2를 초기화하고 active_cam 뷰어를 종료할 때까지 실행한다."""
    rclpy.init(args=args)
    node = None
    try:
        node = ActiveCamViewerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_windows()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
