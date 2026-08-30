"""배관 로봇 ROS 2 통신 규약 — **토픽 이름과 JSON 스키마의 단일 출처.**

🚨 **PC 번호로 부르지 않는다.** 팀 관례는 `PC1 = 카메라·검출 쪽`,
   `PC2 = Isaac Sim` 인데 이 규약을 처음 쓸 때 정반대로 적었다가 걸렸다.
   번호는 읽는 사람마다 뒤집히므로 이 파일은 **역할 이름**만 쓴다 —
   **`Isaac 쪽`(발행, Python 3.11)** 과 **`ROS 노드 쪽`(수신·지령, 3.10)**.

🔑 **시연은 RTX 5080 MSI PC 한 대에서 양쪽을 같이 돌린다** (2026-08-06 확정).
   같은 PC 안이라 DDS 가 루프백으로 통하고 멀티캐스트·NAT 문제가 없다.
   EC2 서버는 개발·검증용이다. PC 를 갈라야 하면 `docs/SETUP_EC2.md` 참조.

🔑 이 파일은 **순수 stdlib 다.** rclpy 도, numpy 도, ROS 도 import 하지 않는다.
   이유가 있다 — Isaac Sim 은 **Python 3.11**, 우리 ROS 2 Humble 은 **3.10** 이라
   발행하는 쪽과 받는 쪽이 같은 인터프리터를 못 쓴다. 규약을 양쪽에 손으로
   베껴 적으면 반드시 갈라지므로(이미 팀 안에서 `/rgb` `front/rgb`
   `/rgb/compressed` 세 갈래로 갈렸다), 이 파일 하나만 양쪽이 읽는다.

   Isaac 쪽(3.11)에서 쓰는 법 — 설치 없이 소스를 그대로 읽는다.
   🚨 절대경로를 박지 말 것. 워크스페이스 위치는 PC 마다 다르다:

       import sys
       from pathlib import Path
       # <ws>/src/son/xxx.py 에서 <ws>/src/dongmin/pipe_comm 을 찾는다
       WS = Path(__file__).resolve().parents[2]        # src/ 의 부모
       sys.path.insert(0, str(WS / "src" / "dongmin" / "pipe_comm"))
       from pipe_comm.contract import Topics, drive_state, event

   또는 환경변수로 — 레이아웃이 달라도 견딘다:

       sys.path.insert(0, os.environ["PIPE_COMM_DIR"])

   ROS 노드 쪽(3.10)은 colcon 으로 설치된 것을 그냥 import 한다.

━━ 왜 경계에는 표준 메시지만 쓰는가 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

**Isaac ↔ ROS 노드를 건너는 토픽은 전부 `std_msgs` / `sensor_msgs` /
`geometry_msgs` / `nav_msgs` 다. 커스텀 msg 를 쓰지 않는다.**

커스텀 msg 는 rosidl 이 만든 3.10 용 `.so` 에 묶여 있다. 경계에 쓰면 Isaac
쪽(3.11)에서도 그 인터페이스를 빌드해야 하는데, Isaac 5.1 내장 rclpy 는
`isaacsim.ros2.bridge/humble` 에 박제된 3.11 ABI 라 우리 워크스페이스를
그대로 못 얹는다. 표준 메시지는 그 번들에 이미 들어 있으므로 빌드가 0 이다.

부수 효과가 하나 더 있다 — **받는 쪽이 `pipe_comm` 을 빌드하지 않아도 된다.**
`ros2 topic echo` 로 바로 보이고, 팀원이 자기 노드를 붙일 때 우리 패키지에
의존하지 않는다. 규약 상수를 쓰고 싶을 때만 import 하면 된다.

대신 **구조가 필요한 값은 `std_msgs/String` 에 JSON 으로 싣는다.**
아래 `drive_state()` / `event()` / `mission()` / `repair_target()` 가 그
직렬화기이고, 파싱은 `parse()` 하나로 끝난다. 성능이 문제되는 양이 아니다
(상태 10Hz, 사건은 1회성).

🚨 영상만은 예외적으로 무겁다 — 그래서 JSON 이 아니라 `CompressedImage` 다.

━━ 토픽 지도 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

로봇 한 대가 하나의 네임스페이스를 갖는다. `map_test_demo.py` 는 코스마다
로봇이 한 대씩(3대) 이므로 `/elbow_v` `/elbow_h` `/tee` 로 갈라지고,
`repair_demo.py` / `real_map_demo.py` 는 한 대라 `/robot` 하나다.

🔑 **층별 2대 동시 주행**(2026-08-10)은 `/floor1` `/floor2` 두 네임스페이스다
   (`FLOOR_NS`). 1·2층은 **연결하지 않는다** — 각자 자기 층에서 따로 출발해
   따로 끝난다. 시연은 `ROS_NS` 를 달리한 두 프로세스이고, 관제 패널은
   `-p ns:=floor1,floor2` 로 둘을 나란히 띄운다(왼쪽 1층 / 오른쪽 2층).
   🚨 층이 다르면 **월드 좌표 프레임도 다르다** — `course()` 의 `z_shift_mm`
   설명을 볼 것.

    Isaac 쪽 (3.11)                             ROS 노드 쪽 (3.10)
    ───────────────                             ──────────────────
                          ── 센서 ──────────▶
      카메라 RGB           rgb/compressed        pipe_comm/camera_monitor
      카메라 Depth         depth/compressed      dongyeon/pipe_vision_node
      카메라 내부파라미터  camera_info                 └ YOLO 결함 검출
      후방 카메라          rear/rgb/compressed   pipe_comm/web_panel Camera 화면
      토치 카메라          torch/rgb/compressed        └ 용접부 근접
                          ── 상태 ──────────▶
      주행 오도메트리      odom                  pipe_comm/drive_monitor
      IMU(롤·요)          imu                   son/localization/node.py
      관절 12휠+서스펜션   joint_states
                          ── 알림 ──────────▶
      주행/정지 플래그     moving  (Bool)        pipe_comm/drive_monitor
      FSM 상태 10Hz        drive_state (JSON)          └ 사람이 보는 콘솔
      1회성 사건           event  (JSON)         dongyeon/pipe_coordinator
      코스 중심선(latched) course (JSON)         pipe_comm/web_panel 3D 맵
      CAD 메시(latched)    mesh (UInt8MultiArray) pipe_comm/web_panel 3D 맵
                                                       └ 맵 .webmesh, 0.8MB 1회
                          ◀── 지령 ──────────
      주행 속도            cmd_vel (Twist)       pipe_comm/mission_cli
      임무 지령            mission (JSON)        son/driver/node.py
      수리 대상            repair_target (JSON)  dongyeon/pipe_coordinator

🚨 **영상 QoS 는 반드시 BEST_EFFORT 다.** RELIABLE 로 구독하면 발행자와
   호환되지 않아 **토픽이 보이는데 프레임이 0** 이다(에러도 안 난다).
   `sensor_qos()` 를 쓰면 그 실수를 할 수 없다.

🚨 **양쪽 `ROS_DOMAIN_ID` 가 같아야 한다 — 우리 팀은 143 이다.**
   다르면 `ros2 topic list` 에 아무것도 안 보인다.
"""

import json
import time

# 팀 공용 도메인. 이 값이 다르면 통신 자체가 성립하지 않는다.
ROS_DOMAIN_ID = 143

# 네임스페이스가 안 주어졌을 때의 기본값. 로봇이 한 대인 시연이 여기 해당한다.
DEFAULT_NS = "robot"

# `map_test_demo.py` 가 굴리는 코스 3종 = 로봇 3대의 네임스페이스.
COURSE_NS = ("elbow_v", "elbow_h", "tee")

# 층별 2대 동시 주행의 네임스페이스. **순서가 곧 관제 화면의 왼→오른쪽**이다.
FLOOR_NS = ("floor1", "floor2")
# 네임스페이스 → 사람이 읽는 이름. 여기 없는 이름은 그대로 쓴다.
NS_LABEL = {"floor1": "1층", "floor2": "2층", "robot": "로봇"}


def ns_label(ns):
    """네임스페이스의 표시 이름. `floor1` → `1층`, 모르는 것은 그대로."""
    return NS_LABEL.get(str(ns).strip("/"), str(ns).strip("/"))


# 네임스페이스를 붙이기 전의 상대 토픽 이름. 오른쪽이 메시지 타입이다.
RELATIVE = {
    # ── Isaac → ROS 노드 : 센서 ────────────────────────────────────────
    "rgb": "rgb/compressed",                 # sensor_msgs/CompressedImage jpeg
    "depth": "depth/compressed",             # sensor_msgs/CompressedImage 16UC1 png(mm)
    "depth_raw": "depth",                    # sensor_msgs/Image 32FC1 (m) — 선택
    "camera_info": "camera_info",            # sensor_msgs/CameraInfo
    "rear_rgb": "rear/rgb/compressed",       # sensor_msgs/CompressedImage
    "rear_camera_info": "rear/camera_info",  # sensor_msgs/CameraInfo
    "torch_rgb": "torch/rgb/compressed",     # sensor_msgs/CompressedImage
    "torch_camera_info": "torch/camera_info",  # sensor_msgs/CameraInfo
    # dongyeon 검출의 opencv 디버그 오버레이. pc2(floor2)가 활성 카메라 위에
    # 검출 결과를 그려 계속 발행한다 — 웹의 floor2 카메라 칸은 이걸 쓴다.
    # 🔑 real_map_demo_v1_3.py 가 발행하는 실제 이름에 맞춘다
    #    (`dy_pub["debug"]` → `/{ns}/repair_robot/opencv_debug/compressed`).
    #    타입은 이름과 무관하게 **CompressedImage jpeg** 다.
    "debug_rgb": "repair_robot/opencv_debug/compressed",  # sensor_msgs/CompressedImage (jpeg)
    # ── Isaac → ROS 노드 : 상태 ────────────────────────────────────────
    "odom": "odom",                          # nav_msgs/Odometry
    "imu": "imu",                            # sensor_msgs/Imu
    "joint_states": "joint_states",          # sensor_msgs/JointState
    # ── Isaac → ROS 노드 : 알림 ────────────────────────────────────────
    "moving": "moving",                      # std_msgs/Bool    주행 중인가
    "drive_state": "drive_state",            # std_msgs/String  JSON, 10Hz
    "event": "event",                        # std_msgs/String  JSON, 사건 때만
    "course": "course",                      # std_msgs/String  JSON, latched 1회
    "mesh": "mesh",                          # std_msgs/UInt8MultiArray  .webmesh, latched 1회
    # ── ROS 노드 → Isaac : 지령 ────────────────────────────────────────
    "cmd_vel": "cmd_vel",                    # geometry_msgs/Twist  linear.x 만
    "mission": "mission",                    # std_msgs/String  JSON
    "repair_target": "repair_target",        # std_msgs/String  JSON
}


class Topics:
    """네임스페이스를 씌운 토픽 이름을 만든다.

    >>> Topics("elbow_v").rgb
    '/elbow_v/rgb/compressed'
    >>> Topics().cmd_vel
    '/robot/cmd_vel'

    🔑 이름을 문자열로 직접 적지 말 것. 오타는 **에러가 아니라 침묵**이다 —
       발행은 성공하고 아무도 못 받는다. 진단이 제일 오래 걸리는 부류다.
       여기 없는 이름을 물으면 AttributeError 로 바로 걸린다.
    """

    __slots__ = ("ns",)

    def __init__(self, ns=DEFAULT_NS):
        self.ns = "/" + str(ns).strip("/") if ns else ""

    def __getattr__(self, name):
        # __slots__ 라 인스턴스 사전이 없다 → RELATIVE 에 없는 이름만 여기 온다.
        try:
            return f"{self.ns}/{RELATIVE[name]}"
        except KeyError:
            raise AttributeError(
                f"{name!r} 은 규약에 없는 토픽이다 — {sorted(RELATIVE)}") from None

    def all(self):
        """{별칭: 절대 토픽 이름} 전부. 진단 출력에 쓴다."""
        return {k: f"{self.ns}/{v}" for k, v in RELATIVE.items()}


# ── 프레임 이름 (CameraInfo.header.frame_id, Odometry 의 child_frame_id) ──
FRAME_FRONT_CAM = "front_camera"
FRAME_REAR_CAM = "rear_camera"
FRAME_TORCH_CAM = "torch_camera"   # 토치 링에 달린 용접부 근접 카메라
FRAME_BASE = "base_link"
FRAME_PIPE = "pipe"          # 관 중심선을 따라가는 좌표계. odom 의 부모다


# ── 주행 FSM 상태값 ────────────────────────────────────────────────
# `map_test_demo.py` 의 FSM 과 `driver/control.py` 를 함께 덮는 집합이다.
STATE_SETTLE = "SETTLE"        # 투입 직후 피스톤이 관벽을 잡는 과도 구간
STATE_RUN = "RUN"              # 주행 중
STATE_HOLD = "HOLD"            # 지령으로 정지(멀쩡함). 재개 가능
STATE_STUCK = "STUCK"          # 끼임 감지 — 방향 전환 재시도 중
STATE_INSPECT = "INSPECT"      # 결함 앞 정지, 카메라 판정 중
STATE_REPAIR = "REPAIR"        # 용접 중
STATE_RETURN = "RETURN"        # 복귀 주행
STATE_DONE = "DONE"            # 임무 완료
STATE_DEAD = "DEAD"            # 코스 이탈·복구 불가. 그 로봇만 멈춘다

STATES = (STATE_SETTLE, STATE_RUN, STATE_HOLD, STATE_STUCK, STATE_INSPECT,
          STATE_REPAIR, STATE_RETURN, STATE_DONE, STATE_DEAD)

# 주행 중으로 볼 상태 — `moving` Bool 과 `drive_state.moving` 이 같은 판정을
# 쓰도록 여기 한 곳에서 정한다.
MOVING_STATES = (STATE_RUN, STATE_STUCK, STATE_RETURN)


# ── 1회성 사건 ─────────────────────────────────────────────────────
EV_START = "START"             # 안착 끝 → 주행 시작
EV_ARRIVE = "ARRIVE"           # 코스 끝 도달
EV_HOME = "HOME"               # 출발점 복귀
EV_STUCK = "STUCK"             # 끼임 → 방향 전환
EV_OFF_COURSE = "OFF_COURSE"   # 관 밖으로 이탈 (복구 안 함)
EV_DISCONNECT = "DISCONNECT"   # 관 단절 감지
EV_BRANCH = "BRANCH"           # T 분기 감지
EV_DEFECT = "DEFECT"           # 결함 발견
EV_WELD_BEGIN = "WELD_BEGIN"
EV_WELD_DONE = "WELD_DONE"
EV_ESTOP = "ESTOP"             # 비상 정지
EV_DONE = "DONE"               # 임무 완료

EVENTS = (EV_START, EV_ARRIVE, EV_HOME, EV_STUCK, EV_OFF_COURSE,
          EV_DISCONNECT, EV_BRANCH, EV_DEFECT, EV_WELD_BEGIN, EV_WELD_DONE,
          EV_ESTOP, EV_DONE)

# 사람이 바로 알아야 하는 사건 — 모니터가 굵게 찍는다.
EV_ALERT = (EV_OFF_COURSE, EV_DISCONNECT, EV_ESTOP, EV_STUCK)


# ── 임무 지령 ──────────────────────────────────────────────────────
CMD_START = "START"            # 주행 시작 / 재개
CMD_STOP = "STOP"              # 정지 (HOLD). 재개 가능
CMD_RECALL = "RECALL"          # 복귀 — 방향을 뒤집어 출발점으로
CMD_FORWARD = "FORWARD"        # 복귀 취소 — 다시 전진하며 점검한다
CMD_RETRY = "RETRY"            # 끼임 탈출 재시도 (방향 전환)
CMD_SPEED = "SPEED"            # 주행 속도 변경. `mps` 를 같이 준다
CMD_ESTOP = "ESTOP"            # 비상 정지 — 재개 불가, 사람이 풀어야 한다

# 🔑 RECALL 의 짝은 START 가 아니라 FORWARD 다. START 는 "서 있던 것을 다시
#    간다" 일 뿐 **방향을 안 바꾼다** — 복귀 중에 START 를 눌러도 계속 뒤로
#    간다. 앞으로 돌려세우는 지령이 따로 있어야 한다(2026-08-08 추가).
COMMANDS = (CMD_START, CMD_STOP, CMD_RECALL, CMD_FORWARD, CMD_RETRY,
            CMD_SPEED, CMD_ESTOP)


# ── JSON 직렬화 ────────────────────────────────────────────────────
# 🔑 전부 `dict` 를 돌려준다. 발행하는 쪽이 `String(data=json.dumps(...))` 로
#    감싸면 되고, ROS 를 안 쓰는 시험 코드에서도 그대로 검사할 수 있다.

def _stamp(t=None):
    return round(float(t if t is not None else time.time()), 3)


def drive_state(ns, state, *, moving=None, direction=1, speed_mps=0.0,
                s_mm=0.0, s_total_mm=0.0, off_mm=0.0, lap=0, stuck=0,
                step=0, reason="", t=None):
    """10Hz 로 흘리는 주행 상태.  `Topics.DRIVE_STATE` 에 싣는다.

    s_mm       관 **중심선을 따라 잰 진행거리**. 직선거리가 아니다 —
               곡관에서 직선거리를 쓰면 위치가 뒤로 가는 것처럼 보인다.
    off_mm     중심선에서 벗어난 거리. 관 내반경(50mm)을 넘으면 이탈이다.
    direction  +1 전진 / -1 후진. 속도 부호와 분리해 둔다(후진 중에도
               speed_mps 는 양수로 두고 이 값으로 방향을 읽는다).
    """
    if state not in STATES:
        raise ValueError(f"모르는 상태 {state!r} — {STATES} 중 하나여야 한다")
    return {
        "stamp": _stamp(t), "robot": ns, "step": int(step),
        "state": state,
        "moving": bool(state in MOVING_STATES if moving is None else moving),
        "dir": int(direction), "speed_mps": round(float(speed_mps), 4),
        "s_mm": round(float(s_mm), 1), "s_total_mm": round(float(s_total_mm), 1),
        "off_mm": round(float(off_mm), 2),
        "lap": int(lap), "stuck": int(stuck), "reason": str(reason),
    }


def event(ns, kind, *, s_mm=0.0, detail="", t=None, **extra):
    """사건이 일어난 그 순간에만 한 번 발행한다.  `Topics.EVENT`.

    🚨 상태 스트림(`drive_state`)으로 사건을 유추하지 말 것. 10Hz 샘플 사이에
       일어난 전환은 통째로 사라진다 — 끼임 후 즉시 빠져나온 경우가 그렇다.
    """
    if kind not in EVENTS:
        raise ValueError(f"모르는 사건 {kind!r} — {EVENTS} 중 하나여야 한다")
    d = {"stamp": _stamp(t), "robot": ns, "event": kind,
         "s_mm": round(float(s_mm), 1), "detail": str(detail),
         "alert": kind in EV_ALERT}
    d.update(extra)
    return d


def mission(cmd, *, mps=None, reason="", t=None):
    """ROS 노드 → Isaac 임무 지령.  `Topics.MISSION`."""
    if cmd not in COMMANDS:
        raise ValueError(f"모르는 지령 {cmd!r} — {COMMANDS} 중 하나여야 한다")
    d = {"stamp": _stamp(t), "cmd": cmd, "reason": str(reason)}
    if cmd == CMD_SPEED:
        if mps is None:
            raise ValueError("SPEED 지령에는 mps 가 있어야 한다")
        d["mps"] = round(float(mps), 4)
    elif mps is not None:
        d["mps"] = round(float(mps), 4)
    return d


def course(ns, pts_m, *, ir_m, bend_r_m=0.0, z_shift_mm=None, t=None):
    """코스 중심선 표본 — 시연이 기동할 때 **한 번** latched 로 발행한다.
    `Topics.COURSE` + `latched_qos()`.

    pts_m      [[s, x, y, z], ...] — 호길이 s 와 월드 좌표(전부 **m**).
               s 오름차순, 간격 불균일 허용(직선은 성기게, 곡관은 조밀하게).
    ir_m       관 내반경(m). 웹이 관 튜브·벽면 좌표(시계각)를 그릴 때 쓴다.
    z_shift_mm 이 좌표가 원본 맵(USD)에 대해 얼마나 z 로 올라가 있는지.
               시연의 `-Z_NET` 을 그대로 준다 (`publish_mesh` 와 같은 값).

    🔑 이것이 있기 전에는 web_panel JS 에 코너 좌표가 하드코딩되어 있어서
       맵이 바뀌면 시연 코드와 웹을 **따로** 고쳐야 했다. 이제 기하의 단일
       출처는 시연 쪽 CenterLine 하나다 — 웹은 받은 표본을 그대로 그린다.

    🚨 **로봇이 여러 대면 `z_shift_mm` 을 반드시 실어야 한다.** 시연은 활성
       층의 수평망을 월드 z=0 으로 올려 놓고 좌표를 내므로, 층이 다른 두 대는
       **서로 다른 프레임**으로 말한다(floor2 +250 / floor1 +2740.23 →
       2.49m). 이 값이 있어야 관제 화면이 둘을 한 좌표계로 겹쳐 그린다.
       한 대뿐이면 없어도 된다(기준이 곧 그 로봇이다).
    """
    pts = [[round(float(v), 4) for v in p] for p in pts_m]
    if any(len(p) != 4 for p in pts):
        raise ValueError("pts_m 은 [s,x,y,z] 의 목록이어야 한다")
    d = {"stamp": _stamp(t), "robot": ns, "ir_m": round(float(ir_m), 4),
         "bend_r_m": round(float(bend_r_m), 4),
         "s_total_m": pts[-1][0] if pts else 0.0, "pts": pts}
    if z_shift_mm is not None:
        d["z_shift_mm"] = round(float(z_shift_mm), 3)
    return d


def repair_target(defect_id, *, s_mm, clock_deg, width_mm=0.0, depth_mm=0.0,
                  confidence=1.0, t=None):
    """ROS 노드 → Isaac 수리 대상.  `Topics.REPAIR_TARGET`.

    clock_deg  관 단면의 **시계각**. 0° = 천장, 180° = 바닥(중력 방향).
               토치 링 J1 을 여기로 돌린다 — 부호가 틀리면 정반대를 용접한다.
               기준은 IMU 롤(`Topics.IMU`)이고 후방 세그먼트로 통일한다.
    """
    return {"stamp": _stamp(t), "id": str(defect_id),
            "s_mm": round(float(s_mm), 1),
            "clock_deg": round(float(clock_deg) % 360.0, 1),
            "width_mm": round(float(width_mm), 2),
            "depth_mm": round(float(depth_mm), 2),
            "confidence": round(float(confidence), 3)}


def dumps(d):
    """dict → String.data 로 실을 문자열. 한글 detail 을 안 깨뜨린다."""
    return json.dumps(d, ensure_ascii=False, separators=(",", ":"))


def parse(data):
    """String.data → dict. 깨진 것은 예외 대신 None 이다.

    🔑 통신 경로에서 온 값은 언제든 깨질 수 있다. 모니터 노드가 JSON 하나
       때문에 죽으면 정작 봐야 할 로그를 못 본다.
    """
    try:
        d = json.loads(data)
        return d if isinstance(d, dict) else None
    except (ValueError, TypeError):
        return None


# ── QoS ────────────────────────────────────────────────────────────
# 🚨 여기를 함수로 둔 이유: 영상 QoS 를 RELIABLE 로 구독하면 발행자와 호환되지
#    않아 **토픽은 보이는데 프레임이 0** 이다. 에러가 안 나서 진단이 오래 걸린다.
#    직접 QoSProfile 을 적지 말고 이 둘 중 하나를 쓸 것.

def sensor_qos(depth=5):
    """영상·IMU 등 흘려보내는 신호용. BEST_EFFORT / KEEP_LAST."""
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def command_qos(depth=10):
    """지령·사건용. RELIABLE — 한 번 놓치면 안 되는 것들이다."""
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def latched_qos():
    """1회성 선언용(코스 기하 등). TRANSIENT_LOCAL — 발행이 구독보다 먼저라도
    나중에 붙은 구독자가 마지막 값을 받는다. **양쪽이 같이 써야 한다** —
    구독만 TRANSIENT_LOCAL 이고 발행이 VOLATILE 이면 못 받는다(반대는 된다)."""
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


# ── 환경 점검 ──────────────────────────────────────────────────────

def check_env():
    """도메인·미들웨어가 팀 값과 맞는지 본다.  [(심각도, 문구), ...] 를 돌려준다.

    🚨 **이 셋이 틀리면 에러가 안 난다.** 노드는 멀쩡히 뜨고 토픽 목록도
       비어 있지 않은데 데이터만 영영 안 온다 — 원인을 찾는 데 제일 오래
       걸리는 부류다. 그래서 각 노드가 기동할 때 이걸 불러 먼저 찍는다.

    ROS 를 import 하지 않으므로(환경변수만 본다) 3.11 Isaac 쪽에서도 쓸 수 있다.
    """
    import os
    out = []

    dom = os.environ.get("ROS_DOMAIN_ID")
    if dom is None:
        out.append(("error", f"ROS_DOMAIN_ID 가 설정되지 않았다 (기본 0 으로 "
                             f"동작한다). `export ROS_DOMAIN_ID="
                             f"{ROS_DOMAIN_ID}` 할 것"))
    elif dom.strip() != str(ROS_DOMAIN_ID):
        out.append(("error", f"ROS_DOMAIN_ID={dom} 인데 팀 값은 "
                             f"{ROS_DOMAIN_ID} 다 — 상대 토픽이 안 보인다"))

    rmw = os.environ.get("RMW_IMPLEMENTATION")
    if rmw is None:
        out.append(("warn", "RMW_IMPLEMENTATION 이 없다 — 양쪽이 같은 구현이어야 "
                            "서로를 찾는다. `export RMW_IMPLEMENTATION="
                            "rmw_fastrtps_cpp` 를 권한다"))
    elif rmw != "rmw_fastrtps_cpp":
        out.append(("warn", f"RMW_IMPLEMENTATION={rmw} — Isaac 쪽은 "
                            f"rmw_fastrtps_cpp 다. 다르면 서로 못 찾는다"))

    if os.environ.get("ROS_LOCALHOST_ONLY") == "1":
        out.append(("warn", "ROS_LOCALHOST_ONLY=1 — 다른 PC 와는 통신하지 "
                            "않는다 (한 PC 안이면 문제없다)"))

    return out


def log_env(logger):
    """`check_env()` 결과를 rclpy 로거로 찍는다. 노드 기동 때 한 번 부른다."""
    for level, msg in check_env():
        (logger.error if level == "error" else logger.warn)(f"[환경] {msg}")
