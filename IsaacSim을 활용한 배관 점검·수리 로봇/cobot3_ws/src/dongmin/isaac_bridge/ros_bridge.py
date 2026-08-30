"""[Isaac 3.11] 시연 스크립트를 ROS 2 규약에 물리는 브리지.  **테스트용 판이다.**

규약은 `src/dongmin/pipe_comm/pipe_comm/contract.py` 가 단일 출처다. 이 파일은
그것을 읽어 발행만 한다 — 토픽 이름도 JSON 스키마도 여기서 새로 정하지 않는다.

━━ 🚨 이것은 통신을 검증하기 위한 판이다 ━━━━━━━━━━━━━━━━━━━━━━━━━

Isaac 파트 담당자가 만들 **본판이 따로 있다**(규격서
`src/dongmin/pipe_comm/ROS2_통신규격.md`). 이 파일은 받는 쪽(`pipe_comm`)을
실제 Isaac 과 붙여 보려고 먼저 만든 것이고, 아래 둘이 본판과 다르다:

  ⚠ **IMU 가 시뮬 센서가 아니다.** `IMUSensor` 를 붙이는 대신 세그먼트 프림의
     월드 변환에서 롤을 뽑는다. 물리적으로는 같은 값이지만 **본판은 반드시
     `IMUSensor` 를 쓸 것** — 규격서 §6.5 의 `lin_acc` 부호 함정(정립에서
     +9.81)은 이 경로로는 검증되지 않는다.
  ⚠ **깊이는 전방만 보낸다.** RGB 는 등록된 카메라를 전부 내보내지만
     (front/rear/torch — 웹 Camera 화면이 셋을 나란히 본다), 깊이를 3대분
     굽는 것은 검출이 전방만 쓰므로 낭비다.

━━ 자리 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    src/dongmin/isaac_bridge/ros_bridge.py   ← 이 파일 (Isaac 쪽, python 3.11)
    src/dongmin/pipe_comm/                   ← 규약·수신 노드 (ROS, python 3.10)

🔑 `pipe_comm` 과 **형제**로 둔다. 규약을 구현한 코드라 규약 옆이 맞고, 팀원은
   `src/dongmin` 만 받으면 양쪽이 다 따라온다(2026-08-08 `src/son` 에서 옮김).
🚨 `pipe_comm/` **안**에는 두지 않는다. 그쪽은 colcon 이 빌드하는 python 3.10
   ROS 패키지라, 3.11 전용인 이 파일이 들어가면 `find_packages()` 가 설치본에
   같이 복사하고 `from pipe_comm... import ros_bridge` 같은 오용을 부른다.
   이 디렉터리는 `package.xml`/`setup.py` 가 없어서 colcon 이 무시한다.

쓰는 법 — 시연 스크립트에서:

    sys.path.append(".../src/dongmin/isaac_bridge")            # 경로로 읽는다
    import ros_bridge
    bridge = ros_bridge.Bridge([r["name"] for r in robots])   # reset 뒤
    bridge.robot("elbow_v").attach_cameras(cams)              # annotator
    ...
    루프 안에서:
        rp = bridge.robot(name)
        rp.publish_state(state=..., s_mm=..., ...)      # 10Hz
        rp.publish_camera(only="front")                 # 10Hz, 한 대만
        rp.emit(contract.EV_STUCK, "방향 전환")          # 사건 때만
        cmd = rp.pop_mission()                          # 지령 확인
    bridge.spin()                                       # 매 스텝

접붙이는 자리 5곳의 상세는 `pipe_comm/ROS2_웹연동_이식가이드.md` 참고.
"""

import array
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

# ── 규약 단일 출처를 경로로 읽는다 (설치 불필요, 절대경로 금지) ──────
# 🔑 `pipe_comm` 은 **바로 옆(형제 디렉터리)** 이다:
#       src/dongmin/isaac_bridge/ros_bridge.py   ← 이 파일
#       src/dongmin/pipe_comm/pipe_comm/contract.py
#    그래서 워크스페이스 루트를 거치지 않는다 — 워크스페이스 이름이 바뀌거나
#    통째로 옮겨져도 두 디렉터리가 같이 움직이는 한 그대로 찾는다.
# 🚨 colcon 설치본(3.10)은 못 쓴다. Isaac 은 python 3.11 이라 ABI 가 다르다 —
#    그래서 **소스를 경로로** 읽는다. `isaacpjt/` 만 받은 사람은
#    PIPE_COMM_DIR 로 사본 자리를 가리키면 된다.
_PC = os.environ.get(
    "PIPE_COMM_DIR", str(Path(__file__).resolve().parents[1] / "pipe_comm"))
if _PC not in sys.path:
    sys.path.insert(0, _PC)

try:
    from pipe_comm import contract
    from pipe_comm import image_codec as codec
    from pipe_comm.contract import Topics
    CONTRACT_OK = True
except ImportError as _exc:                                  # pragma: no cover
    CONTRACT_OK = False
    _CONTRACT_ERR = _exc


def available():
    """rclpy 와 규약을 둘 다 쓸 수 있는가. 시연 스크립트가 먼저 물어본다."""
    if not CONTRACT_OK:
        print(f"[ROS] 규약을 못 읽었다 ({_CONTRACT_ERR}) — 경로 {_PC}")
        return False
    try:
        import rclpy                                          # noqa: F401
    except ImportError as exc:
        print(f"[ROS] rclpy 없음 ({exc}) — `isaac_ros` 를 먼저 실행할 것")
        return False
    return True


def _quat_x(deg):
    """X 축(관 축) 회전 쿼터니언. 롤을 실어 보낸다."""
    h = math.radians(deg) * 0.5
    return math.sin(h), 0.0, 0.0, math.cos(h)


def _mesh_z_shift(path):
    """이미 구워 둔 `.webmesh` 가 **어느 층 프레임**으로 구워졌는지. 모르면 None.

    앞 4바이트 길이 + 그만큼의 JSON 헤더만 읽는다(포맷은 usd_to_webmesh.py).
    5MB 를 통째로 파싱하지 않는다.
    """
    import json as _json
    try:
        with open(path, "rb") as f:
            n = int.from_bytes(f.read(4), "little")
            hdr = _json.loads(f.read(n).decode("utf-8"))
        z = hdr.get("z_shift_mm")
        return None if z is None else float(z)
    except Exception:
        return None


class RobotPub:
    """로봇 한 대의 발행자. 네임스페이스 하나를 갖는다."""

    def __init__(self, node, ns):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, JointState
        from std_msgs.msg import Bool, String, UInt8MultiArray

        self.ns = ns
        self.node = node
        t = Topics(ns)
        sq, cq = contract.sensor_qos(), contract.command_qos()

        self.p_rgb = node.create_publisher(CompressedImage, t.rgb, sq)
        self.p_depth = node.create_publisher(CompressedImage, t.depth, sq)
        self.p_info = node.create_publisher(CameraInfo, t.camera_info, sq)
        # 역할별 카메라 토픽. 🔑 깊이는 **front 만** 낸다 — 검출(dongyeon)이
        # 쓰는 건 전방뿐이고, 3대를 다 굽는 것은 시연에 필요 없는 비용이다.
        self.p_cam = {
            "front": (self.p_rgb, self.p_info, self.p_depth,
                      contract.FRAME_FRONT_CAM),
            "rear": (node.create_publisher(CompressedImage, t.rear_rgb, sq),
                     node.create_publisher(CameraInfo, t.rear_camera_info, sq),
                     None, contract.FRAME_REAR_CAM),
            "torch": (node.create_publisher(CompressedImage, t.torch_rgb, sq),
                      node.create_publisher(CameraInfo, t.torch_camera_info,
                                            sq),
                      None, contract.FRAME_TORCH_CAM),
        }
        self.p_odom = node.create_publisher(Odometry, t.odom, sq)
        self.p_imu = node.create_publisher(Imu, t.imu, sq)
        self.p_joint = node.create_publisher(JointState, t.joint_states, sq)
        self.p_moving = node.create_publisher(Bool, t.moving, cq)
        self.p_state = node.create_publisher(String, t.drive_state, cq)
        self.p_event = node.create_publisher(String, t.event, cq)
        # 코스 기하 — latched(TRANSIENT_LOCAL) 라 web_panel 이 늦게 떠도 받는다
        self.p_course = node.create_publisher(String, t.course,
                                              contract.latched_qos())
        # CAD 메시도 latched — Isaac PC 와 웹 PC 가 다를 때 파일 대신 이걸 탄다
        self.p_mesh = node.create_publisher(UInt8MultiArray, t.mesh,
                                            contract.latched_qos())

        # 지령은 큐에 쌓아 두고 시연 쪽이 꺼내 간다 — 콜백에서 로봇을 직접
        # 건드리면 물리 스텝 중간에 상태가 바뀌어 재현이 안 된다.
        self._missions = []
        node.create_subscription(String, t.mission, self._on_mission, cq)
        self._cmd_vel = None
        node.create_subscription(Twist, t.cmd_vel, self._on_cmd_vel, cq)

        self.cams = []            # [(role, annot_rgb, annot_depth, W, H, f_px)]
        self._moving_last = None
        self.n_img = 0
        self._warned_empty = set()   # 빈 버퍼 경고를 낸 역할

    # ── 지령 수신 ───────────────────────────────────────────────
    def _on_mission(self, msg):
        d = contract.parse(msg.data)
        if d is not None:
            self._missions.append(d)

    def _on_cmd_vel(self, msg):
        self._cmd_vel = float(msg.linear.x)

    def pop_mission(self):
        """받아 둔 지령을 하나 꺼낸다. 없으면 None."""
        return self._missions.pop(0) if self._missions else None

    def take_cmd_vel(self):
        """마지막 cmd_vel (m/s). 안 왔으면 None."""
        v, self._cmd_vel = self._cmd_vel, None
        return v

    # ── 카메라 ──────────────────────────────────────────────────
    def use_annotators(self, a_rgb, a_dep, w, h, f_px, role="front"):
        """**이미 붙어 있는** annotator 를 그대로 쓴다.

        🔑 `repair_demo.py` 는 카메라를 만들면서 annotator 까지 붙인다. 같은
           render product 에 또 붙이면 버퍼를 두 번 읽어 낭비이고, 무엇보다
           그쪽 검출 코드와 **다른 프레임을 볼 수 있다.** 발행하는 그림과
           판정하는 그림이 다르면 로그를 대조할 수 없다.

        role 은 `"front"` / `"rear"` / `"torch"` — 각각 다른 토픽으로 나간다.
        기본값이 front 라 예전 호출부는 그대로 둬도 동작이 안 바뀐다.
        """
        if role not in self.p_cam:
            raise ValueError(f"모르는 카메라 역할 {role!r} — {list(self.p_cam)}")
        self.cams.append((role, a_rgb, a_dep, w, h, f_px))
        return len(self.cams)

    def attach_cameras(self, cams, role="front"):
        """🚨 `world.reset()` **뒤에** 부를 것. annotator 는 런타임 자원이라
        reset 전에 붙이면 살아남지 않는다 (규격서 §8.2)."""
        import omni.replicator.core as rep

        for cam, w, h, f_px in cams:
            rp = cam.get_render_product_path()
            a_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            a_rgb.attach(rp)
            # 🔑 distance_to_image_plane 이 아니라 distance_to_camera.
            #    관 내부는 방사형이라 광축 투영이 아니라 실제 거리가 필요하다.
            a_dep = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
            a_dep.attach(rp)
            self.use_annotators(a_rgb, a_dep, w, h, f_px, role)
        return len(self.cams)

    def publish_camera(self, only=None):
        """등록된 카메라를 내보낸다. 10Hz 로 부를 것.

        only    역할 하나(`"front"`/`"rear"`/`"torch"`)만 내보낸다. None 이면
                전부(예전 동작). 🔑 시연이 **주행 방향·작업에 따라 카메라를
                갈아 끼우는** 용도다 — 전진이면 front, 후진이면 rear, 용접
                정렬~아크면 torch. 안 쓰는 카메라를 안 굽는 만큼 JPEG 인코딩
                비용이 그대로 빠진다(3대 → 1대).

        돌려주는 값은 전방 깊이의 유효 화소 비율이다(예전 호출부 호환).
        """
        from sensor_msgs.msg import CameraInfo, CompressedImage

        stamp = self.node.get_clock().now().to_msg()
        ratio = 0.0

        for role, a_rgb, a_dep, w, h, f_px in self.cams:
            if only is not None and role != only:
                continue
            p_rgb, p_info, p_depth, frame = self.p_cam[role]

            rgb = a_rgb.get_data()
            if rgb is not None and getattr(rgb, "size", 0):
                m = CompressedImage()
                m.header.stamp, m.header.frame_id = stamp, frame
                m.format = "jpeg"
                m.data = codec.rgb_to_jpeg(rgb)
                p_rgb.publish(m)
                self.n_img += 1
            elif role not in self._warned_empty:
                self._warned_empty.add(role)
                print(f"[ROS] {self.ns}: {role} rgb annotator 가 빈 버퍼다 — "
                      f"렌더가 도는지 확인할 것(headless 면 프레임이 안 나온다)")

            if p_depth is not None and a_dep is not None:
                d = a_dep.get_data()
                if d is not None and getattr(d, "size", 0):
                    m = CompressedImage()
                    m.header.stamp, m.header.frame_id = stamp, frame
                    m.format = "png"
                    m.data, ratio = codec.depth_to_png16(d)
                    p_depth.publish(m)

            ci = CameraInfo()
            ci.header.stamp, ci.header.frame_id = stamp, frame
            ci.width, ci.height = int(w), int(h)
            # 🔑 어안이라 K 는 핀홀 행렬이 아니다. K[0] 에 등거리 어안 f (r=f·θ).
            ci.k = [f_px, 0.0, w / 2.0, 0.0, f_px, h / 2.0, 0.0, 0.0, 1.0]
            ci.p = [f_px, 0.0, w / 2.0, 0.0, 0.0, f_px, h / 2.0, 0.0,
                    0.0, 0.0, 1.0, 0.0]
            ci.distortion_model = "equidistant"
            ci.d = [0.0, 0.0, 0.0, 0.0]
            p_info.publish(ci)
        return ratio

    # ── 상태 ────────────────────────────────────────────────────
    def publish_state(self, *, state, direction, speed_mps, s_mm, s_total_mm,
                      off_mm, lap, stuck, step, roll_deg=0.0, reason="",
                      art=None, wheel_idx=None, pos=None, cam=None):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Bool, String

        d = contract.drive_state(
            self.ns, state, direction=direction, speed_mps=speed_mps,
            s_mm=s_mm, s_total_mm=s_total_mm, off_mm=off_mm, lap=lap,
            stuck=stuck, step=step, reason=reason)
        # 🔑 월드 좌표(관 좌표계, m)를 추가 필드로 싣는다 — T 분기 맵에서
        #    웹이 가장 가까운 가지에 스냅해 그린다. s 는 외길 전제라 분기를
        #    구분 못 한다. 규약의 추가 필드라 스키마 변경이 아니다.
        if pos is not None:
            d["pos_m"] = [round(float(pos[k]), 4) for k in range(3)]
        # 🔑 지금 켜 둔 카메라 역할. 웹이 "왜 후방만 나오나" 를 묻지 않게 하는
        #    값이다 — 한 대만 발행하므로 나머지 둘은 정상적으로 비어 있다.
        if cam is not None:
            d["cam"] = str(cam)
        self.p_state.publish(String(data=contract.dumps(d)))

        # 🔑 moving 은 **전환될 때만** 낸다. 매번 내면 로그가 이것만으로 찬다.
        if d["moving"] != self._moving_last:
            self._moving_last = d["moving"]
            self.p_moving.publish(Bool(data=d["moving"]))

        stamp = self.node.get_clock().now().to_msg()
        qx, qy, qz, qw = _quat_x(roll_deg)

        m = Odometry()
        m.header.stamp, m.header.frame_id = stamp, contract.FRAME_PIPE
        m.child_frame_id = contract.FRAME_BASE
        # 🚨 x 에 **중심선 호길이** 를 싣는다 (규격서 §6.4). 직선거리를 쓰면
        #    곡관에서 위치가 뒤로 가는 것처럼 보인다.
        m.pose.pose.position.x = float(s_mm) * 0.001
        m.pose.pose.orientation.x = qx
        m.pose.pose.orientation.w = qw
        m.twist.twist.linear.x = float(direction) * float(speed_mps) \
            if d["moving"] else 0.0
        self.p_odom.publish(m)

        im = Imu()
        im.header.stamp, im.header.frame_id = stamp, contract.FRAME_BASE
        im.orientation.x, im.orientation.w = qx, qw
        r = math.radians(roll_deg)
        # 🚨 Isaac 의 lin_acc 는 고유가속도라 정지 시 **위로 +9.81** 이다.
        #    부호를 뒤집으면 롤이 180° 어긋나고 토치를 정반대로 돌린다.
        im.linear_acceleration.y = 9.81 * math.sin(r)
        im.linear_acceleration.z = 9.81 * math.cos(r)
        self.p_imu.publish(im)

        if art is not None and wheel_idx:
            js = JointState()
            js.header.stamp = stamp
            try:
                names = list(art.dof_names or [])
                vel = art.get_joint_velocities()
                pos = art.get_joint_positions()
                js.name = [names[k] for k in wheel_idx]
                js.velocity = [float(vel[k]) for k in wheel_idx]
                js.position = [float(pos[k]) for k in wheel_idx]
                self.p_joint.publish(js)
            except Exception:
                pass          # 관절 발행은 보조 신호다 — 실패해도 주행은 간다

    def publish_course(self, pts_m, *, ir_m, bend_r_m=0.0, z_shift_mm=None):
        """코스 중심선 표본을 **한 번** 발행한다 (기동 직후).
        pts_m = [[s,x,y,z], ...] (전부 m) — 규약은 contract.course() 참조.
        latched 라 재발행이 필요 없다. 웹 3D 맵이 이걸로 관을 그린다.

        🚨 로봇이 여러 대(층별 동시 주행)면 `z_shift_mm` 을 같이 준다 —
           `publish_mesh` 에 주는 것과 **같은 값**(`-Z_NET`). 층마다 월드
           프레임이 다르므로, 이게 없으면 관제 3D 맵이 두 대를 2.49m
           어긋난 자리에 그린다."""
        from std_msgs.msg import String
        d = contract.course(self.ns, pts_m, ir_m=ir_m, bend_r_m=bend_r_m,
                            z_shift_mm=z_shift_mm)
        self.p_course.publish(String(data=contract.dumps(d)))
        print(f"[ROS] {self.ns}: 코스 발행 — 표본 {len(d['pts'])}개, "
              f"총 {d['s_total_m'] * 1000:.0f}mm, 내반경 {ir_m * 1000:.0f}mm")

    def publish_mesh(self, webmesh, *, usd=None, z_shift_mm=None):
        """맵 CAD 메시(`.webmesh`)를 **한 번** latched 로 보낸다 (기동 직후).

        🔑 **Isaac PC 와 웹 PC 가 다른 자리에 있을 때를 위한 경로다.** 같은 PC
           라면 web_panel 이 파일을 직접 읽는 쪽이 낫지만, 갈라져 있으면 파일이
           안 보인다 — 그래서 이미 통하고 있는 DDS 로 넘긴다. 0.77MB 1회이고
           조각내기는 RTPS 가 알아서 한다(실측: latched 로 통과).

        🚨 **z 오프셋을 아는 것은 여기뿐이다.** 맵 배치(층별 z, scale)를 정하는
           것이 시연이므로 굽는 것도 시연이 한다. 받는 쪽이 파라미터로 다시
           말해 주는 구조는 층을 바꾸는 순간 조용히 어긋난다(floor2 250 /
           floor1 2740.23 → 2.49m 차이).

        webmesh     `.webmesh` 경로. 없거나 `usd` 보다 낡았으면 굽는다.
        usd         원본 맵 USD (굽기용). None 이면 굽지 않고 있는 것만 보낸다.
        z_shift_mm  맵 z 오프셋(mm). 시연의 `-Z_NET` 을 그대로 준다.
        """
        from pathlib import Path as _Path
        from std_msgs.msg import UInt8MultiArray

        out = _Path(webmesh)
        if usd is not None:
            usd = _Path(usd)
            # 🚨 낡음 판정에 **z 오프셋도 넣는다**(2026-08-10). 예전에는 mtime
            #    만 봐서, floor1 로 한 번 굽고 나면 `--course floor2` 로 돌려도
            #    **floor1 프레임 메시를 그대로 보냈다** — 웹 3D 맵에 건물이
            #    2.49m 어긋난 채 그려지고(로봇이 남의 층 배관 속을 달린다)
            #    에러는 하나도 안 난다. 실측 2026-08-10: floor2 로 띄웠는데
            #    받은 메시 헤더가 z=+2740.2(floor1) 였다.
            baked_z = _mesh_z_shift(out)
            stale = (not out.is_file()
                     or (usd.is_file()
                         and out.stat().st_mtime < usd.stat().st_mtime)
                     or (z_shift_mm is not None and baked_z is not None
                         and abs(baked_z - float(z_shift_mm)) > 0.01))
            if stale and usd.is_file():
                try:
                    import importlib.util
                    tool = (_Path(__file__).resolve().parents[1] / "pipe_comm"
                            / "tools" / "usd_to_webmesh.py")
                    spec = importlib.util.spec_from_file_location("_u2w", tool)
                    mod = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(mod)
                    print(f"[ROS] CAD 메시 굽는 중 — {usd.name} "
                          f"(z {z_shift_mm:+.1f}mm)…")
                    out = mod.convert(usd, float(z_shift_mm or 0.0))
                except Exception as exc:      # 메시는 항상 **덤**이다
                    print(f"[ROS] CAD 메시를 못 구웠다({exc}) — 웹은 코스 튜브만")
        if not out.is_file():
            print(f"[ROS] CAD 메시 없음({out}) — 웹 3D 맵은 코스 튜브만 그린다")
            return 0
        data = out.read_bytes()
        m = UInt8MultiArray()
        # 🔑 list() 로 넘기면 77만 개 int 를 만드느라 몇 초가 걸린다.
        #    array('B') 는 rclpy 가 그대로 받아 간다.
        m.data = array.array("B", data)
        self.p_mesh.publish(m)
        print(f"[ROS] {self.ns}: CAD 메시 발행 — {out.name} "
              f"{len(data) / 1e6:.2f} MB (latched, 1회)")
        return len(data)

    def emit(self, kind, detail="", s_mm=0.0, **extra):
        """1회성 사건. 그 순간에만 한 번 부른다. extra 는 규약 event 의
        추가 필드로 그대로 실린다(예: 용접 사건의 defect_s_mm/clock_deg)."""
        from std_msgs.msg import String
        self.p_event.publish(String(data=contract.dumps(
            contract.event(self.ns, kind, s_mm=s_mm, detail=detail, **extra))))


class Bridge:
    """rclpy 수명과 로봇별 발행자를 들고 있는다."""

    def __init__(self, names, node_name="isaac_bridge"):
        import rclpy
        from rclpy.node import Node

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.pubs = {n: RobotPub(self.node, n) for n in names}

        for level, msg in contract.check_env():
            print(f"[ROS 환경] {'🚨' if level == 'error' else '⚠'} {msg}")
        print(f"[ROS] 발행 시작 — 로봇 {len(names)}대 {list(names)}  "
              f"domain {os.environ.get('ROS_DOMAIN_ID', '(미설정)')}")
        for n in names:
            t = Topics(n)
            print(f"      {t.rgb}  {t.depth}  {t.drive_state}  {t.event}")

    def robot(self, ns):
        return self.pubs[ns]

    def spin(self):
        """🚨 매 스텝 부를 것. 안 부르면 지령을 아예 못 받는다.
        timeout 0 이 아니면 그만큼 **물리가 멈춘다**."""
        self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        total = sum(p.n_img for p in self.pubs.values())
        print(f"[ROS] 발행 종료 — 영상 {total}장")
        self.node.destroy_node()
        try:
            self._rclpy.shutdown()
        except Exception:
            pass
