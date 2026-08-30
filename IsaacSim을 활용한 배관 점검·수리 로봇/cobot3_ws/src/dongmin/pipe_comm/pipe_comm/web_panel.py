"""웹 관제 패널 — 브라우저에서 로봇을 **보고 조종한다** (FastAPI).

`web_view` 가 "카메라가 오는지" 보는 진단 도구라면, 이것은 시연용 관제
화면이다. 한 페이지에서:

    ▶ 시작 / ⏸ 정지 / ↩ 복귀 / ⛔ 비상정지     → `mission` 토픽 발행
    속도 슬라이더                               → `mission` SPEED
    3D 배관 맵 + 현재 위치 + 지나간 구간         ← `course` + `drive_state`
    전방 카메라                                  ← `rgb/compressed`
    결함 목록                                    ← `defect/report_json` (dongyeon)
    사건 로그                                    ← `event`

의존성: `pip install fastapi uvicorn` (ROS 노드 쪽 3.10 전용 — Isaac 쪽과
무관하다). `web_view` 와 달리 외부 패키지를 쓰는 이유는 **양방향**이 되면서
웹소켓·REST·JSON 검증을 손으로 다 짜는 비용이 재사용 이득을 넘어섰기 때문.
zero-dep 진단 경로가 필요하면 `web_view` 가 그대로 있다.

━━ 로봇 여러 대 — 층별 **동시** 주행 (2026-08-10) ━━━━━━━━━━━━━━━━━━

    ros2 run pipe_comm web_panel --ros-args -p ns:=floor1,floor2

`ns` 는 **쉼표 목록**이다. 목록의 순서가 곧 화면의 왼→오른쪽이다
(`floor1,floor2` → 왼쪽 1층 / 오른쪽 2층). 한 개만 주면 예전과 똑같이
한 칸짜리 화면이다.

🔑 **1층과 2층은 연결되어 있지 않다.** 각자 자기 층에서 따로 출발해 따로
   끝나는 별개의 임무다 — 그래서 상태·사건·결함·카메라·지령이 전부 **로봇마다
   따로** 간다. 이 노드는 각 네임스페이스를 독립적으로 구독·발행하고, 웹소켓
   메시지마다 `robot` 을 실어 어느 쪽 것인지 밝힌다.

🚨 **층이 다르면 월드 좌표 프레임도 다르다.** 시연은 활성 층의 수평망을 월드
   z=0 으로 올려놓고 좌표를 내므로(floor2 +250 / floor1 +2740.2), 두 대의
   `pos_m`·`course` 를 그대로 겹쳐 그리면 2.49m 어긋난다. 각 로봇의 z 오프셋을
   `course.z_shift_mm`(시연이 실으면) 또는 **받은 `.webmesh` 헤더**에서 읽어
   `hello` 로 알려 주고, 3D 맵이 그만큼 되밀어 한 좌표계에 겹친다.

🚨 결함 리포트(`defect/report_json`)는 dongyeon 검출 노드가 낸다. 로봇이 여러
   대면 **검출 노드도 대수만큼** 띄우고 `json_topic:=/floor2/defect/report_json`
   처럼 네임스페이스를 붙여야 한다. 이름 없는 옛 절대 토픽
   (`/defect/report_json`)도 계속 받되 **첫 번째 로봇 것으로 친다** — 두 대가
   같은 절대 토픽에 쏘면 구별할 방법이 없다.

━━ 페이지 파일 — 워크스페이스의 `web/` (한 파일 원칙을 버렸다) ━━━━━━

HTML/CSS/JS 는 이 패키지 안이 아니라 **`$COBOT3_WS/web/`** 에 있다
(index.html, panel.css, app.js, views/, three.js 벤더 사본). 3D 맵을 three.js
로 옮기면서 페이지가 파이썬 문자열로 들고 있기엔 너무 커졌고, 웹 소스는
ROS 파이썬 패키지와 수명·도구가 달라 따로 두는 편이 낫다. three.js 는
CDN 이 아니라 벤더 사본이다 — 현장망(오프라인)에서도 떠야 한다.

🔑 서빙 우선순위: **소스 트리**($COBOT3_WS/web) > 설치본
   (share/pipe_comm/web, setup.py 의 data_files 가 복사). 이 패키지는
   --symlink-install 로도 파일이 복사되어 .py 수정마다 재빌드해야 하는데,
   소스를 먼저 잡으면 HTML/CSS/JS 는 **재빌드 없이 새로고침**으로 반영된다.

━━ 3D 배관 맵 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

코스 기하는 하드코딩하지 않는다 — 시연(real_map_demo --ros)이 기동할 때
`course` 토픽(latched)으로 중심선 표본을 발행하고(단일 출처는 그쪽
CenterLine), 이 노드가 받아 웹소켓 {"type":"course"} 로 페이지에 넘긴다.
맵이 바뀌면 시연 쪽만 고치면 된다. 로봇 위치(pos_m)·결함(s, 시계각)은
그 좌표계(월드 m) 그대로 그린다.

CAD 메시: 🔑 **맵의 주인은 이 PC 다** (2026-08-11 로직 변경). 예전에는
Isaac(PC2)이 구워 `mesh` 토픽으로 넘겨 줬는데, 이제 이쪽도 맵 USD 를 갖고
있으므로 **여기서 굽는다.** 기동 때 `src/son/maps/restroom_final0807.usd`
(=`DEFAULT_MAP_USD`)를 `tools/usd_to_webmesh.py` 로 구워 `/mesh` 로 서빙하고,
페이지가 전체 맵(floor2+floor1+aisle)을 반투명 메시로 그린다.

🔑 **이미 구워진 `.webmesh` 가 있으면 그대로 쓴다** — 매번 굽지 않는다. 다시
   굽는 것은 파일이 없거나 · `.usd` 가 더 새롭거나 · 다른 z 프레임으로 구워져
   있을 때뿐이다(find_mesh). 굽는 데는 0.5초쯤 걸린다.
🚨 z_shift 는 시연과 **같아야 한다** — `-p z_shift_mm:=` 기본값 2740.2 는
   `real_map_demo_v1_3.py` 의 `-_Z1` 이다. 틀리면 에러 없이 건물만 어긋난다.

경로 우선순위: `-p mesh:=<파일>` 지정 > `src/son/maps/` 에서 굽기·재사용 >
`mesh` 토픽(Isaac 이 보내면 — 이제 폴백이다). 셋 다 없으면 코스 튜브만으로
동작한다 — 메시는 항상 **덤**이다.

🔑 메시에는 **건물 전체(두 층 다)** 가 들어 있다. 그래서 로봇이 두 대여도
   `/mesh` 는 하나만 준다 — 그 하나를 **기준 프레임**으로 삼고 나머지 로봇을
   거기에 맞춘다(`?robot=` 로 특정 로봇 것을 따로 볼 수 있다).

━━ 지령 경로 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

버튼 → POST /cmd {cmd, robot} → `contract.mission()` 검증 → 그 로봇의
`mission` 토픽 발행. `robot` 을 `all` 로 주면 전 로봇에 같이 보낸다(빼면
첫 번째 로봇). mission_cli 와 같은 원칙으로 **발행 뒤 구독자 수를 확인해**
0 이면 응답에 경고를 실어 화면에 띄운다 — RELIABLE 이라도 구독자가 없으면
지령은 소리 없이 사라지기 때문이다(latch 가 아니다).

실행:
  ros2 run pipe_comm web_panel --ros-args -p ns:=floor1,floor2 -p port:=8080
  → http://<서버IP>:8080   (web_view 와 포트가 겹치니 둘 중 하나만 띄울 것)

🚨 인증이 없다. 공인망에 열지 말 것 — 버튼이 있으니 web_view 보다 위험하다.
   보안그룹은 본인 IP /32 로만, 또는 SSH 터널을 쓸 것.
"""

import json
import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, UInt8MultiArray

from pipe_comm import contract
from pipe_comm.contract import Topics
from pipe_comm.drive_monitor import _quat_to_roll

# dongyeon 검출 노드의 출력. 네임스페이스 앞에 붙여 쓰고(`<ns>/defect/...`),
# 이름 없는 옛 절대 토픽도 첫 번째 로봇 것으로 같이 받는다.
DEFECT_REPORT_REL = "defect/report_json"
DEFECT_REPORT_TOPIC = "/" + DEFECT_REPORT_REL

# 맵 USD 는 **이 PC 가 갖고 있다** — `src/son/maps/`. 이 이름은 시연 쪽
# `real_map_demo_v1_3.py` 의 `_MAP` 과 **같아야 한다**: 둘이 갈라지면 웹의
# 3D 맵이 로봇과 다른 건물을 그린다(에러는 안 난다).
DEFAULT_MAP_USD = "restroom_final0807.usd"

# 웹소켓 바이너리 머리 **2 바이트** = [로봇 번호, 카메라 채널].
# 🔑 로봇이 늘면서 1 바이트로는 모자라졌다. 번호는 `hello` 가 알려 주는
#    `idx`(1부터, ns 목록 순서)이고 채널은 아래 CAM_CH 다.
#    web/app.js 의 CH·onFrame 과 **같이** 고칠 것.
# 🚨 v1_3 에서 **rear(후방, 채널 2)는 폐지**됐다 — floor1 전방 / floor2 토치
#    2대만 발행한다. 채널 번호는 재사용하지 않는다(2 는 비워 둔다) — 옛
#    브라우저와 섞여도 어긋나지 않게. 규약(contract.Topics.rear_rgb)은 되살릴
#    때를 위해 그대로 남아 있다.
CAM_CH = {"front": 1, "torch": 3}


def mesh_z_shift(data):
    """`.webmesh` 머리의 JSON 헤더에서 `z_shift_mm` 만 꺼낸다. 없으면 None.

    🔑 메시 전체를 파싱하지 않는다 — 앞 4바이트 길이 + 그만큼의 JSON 이
       전부다(포맷은 tools/usd_to_webmesh.py). 0.77MB 를 건드리지 않는다.
    """
    try:
        n = int.from_bytes(data[:4], "little")
        hdr = json.loads(bytes(data[4:4 + n]).decode("utf-8"))
        z = hdr.get("z_shift_mm")
        return None if z is None else float(z)
    except Exception:
        return None


class RobotStore:
    """로봇 **한 대**의 최신 상태. ROS 스레드가 쓰고 웹소켓 태스크들이 읽는다.

    사건·결함은 **덧붙이기만 하는 목록**이다 — 웹소켓마다 어디까지 보냈는지
    (인덱스)를 따로 들고 가므로, 늦게 접속한 브라우저도 지난 결함을 다 받는다.

    🔑 로봇마다 하나씩 만든다. 층이 다른 두 대는 임무도 좌표계도 별개라
       상태를 섞으면 안 된다 — 자물쇠도 따로 둬서 한 대의 10Hz 갱신이 다른
       대의 웹소켓 송신을 막지 않는다.
    """

    MAX_LOG = 300          # 사건 로그 상한. 넘치면 앞을 버린다

    def __init__(self, ns, label, idx):
        self.ns = ns
        self.label = label
        self.idx = idx               # 1부터. 웹소켓 바이너리 머리에 쓴다
        self.lock = threading.Lock()
        self.state = None            # drive_state dict + roll/odom 보강
        self.state_seq = 0
        self.course = None           # course dict — latched 라 마지막 것만
        self.course_seq = 0
        # 카메라 3대 — {역할: JPEG 바이트}, 역할별 시퀀스 번호.
        # 웹소켓마다 어디까지 보냈는지를 이 번호로 따라간다.
        self.cam = {r: None for r in CAM_CH}
        self.cam_seq = {r: 0 for r in CAM_CH}
        # 지나간 최대 호길이 — 지도는 이 구간까지만 그린다("촬영한 만큼").
        # 서버가 들고 있어야 브라우저를 새로고침해도 지도가 안 사라진다.
        self.max_s_mm = 0.0
        # 시연 판(run) 번호. Isaac GUI 에서 Stop→Play 로 **재시작**하면 올라간다.
        # 웹은 이 번호가 바뀌는 것만 보고 지나온 초록선·수리 마커를 지운다.
        self.run = 0
        self.last_step = -1
        self.events = []             # event dict 목록
        self.events_dropped = 0      # MAX_LOG 로 버린 수 (인덱스 보정용)
        self.defects = []            # report dict 목록 (갱신도 새 항목으로)
        self.defects_dropped = 0     # 판이 바뀌며 버린 수 (인덱스 보정용)
        # 마지막으로 drive_state 가 온 시각(monotonic). 시연이 죽었는지 판정한다.
        self.last_state_t = None
        # 🚨 **끊김.** 시연을 끄면 이 노드는 아무 말도 안 듣게 되는데, 그렇다고
        #    마지막 상태를 계속 들고 있으면 화면은 로봇이 관 속에 살아 있는
        #    것처럼 보인다(새로고침해도 그대로 — 사용자 보고 2026-08-10).
        #    끊긴 것으로 판정되면 그 판을 접고 이 깃발을 세운다.
        self.stale = False
        self.roll_deg = None
        self.odom_v = None
        # 🔑 Isaac 쪽에서 latched 로 넘어온 CAD 메시(.webmesh 바이트).
        #    **PC 가 갈릴 때의 주 경로다** — 파일은 저쪽 디스크에만 있다.
        #    이게 오면 로컬 파일보다 **먼저** 쓴다: 시연이 실제로 쓴 맵과
        #    z 오프셋으로 구워진 것이라 언제나 더 옳다.
        self.mesh = None
        # 이 로봇이 말하는 **좌표(pos_m·course)의** z 오프셋(mm). course 가
        # 실어 주면 그것을, 아니면 받은 메시 헤더에서 읽는다. 3D 맵이 층 사이
        # 2.49m 를 이걸로 되민다. 모르면 None — 화면은 "정렬 못 함" 으로 적는다.
        self.z_shift_mm = None
        # 🚨 **보내온 메시가 구워진 프레임**은 따로 센다. 둘은 다를 수 있다 —
        #    `.webmesh` 캐시가 지난 층 것이면(mtime 은 새것이라 안 다시 굽는다)
        #    좌표는 floor2 인데 건물은 floor1 프레임으로 온다. 실측
        #    2026-08-10: course z=+250 인데 메시 헤더 z=+2740.2 였고, 2층
        #    로봇이 1층 배관 속에 찍혔다. 기준 프레임은 **그림(메시)** 쪽이라
        #    이 값을 z_ref 로 쓰고, 로봇은 각자 z_shift_mm 으로 거기 맞춘다.
        self.mesh_z_mm = None

    def info(self):
        """`hello` 로 브라우저에 넘기는 명패."""
        return {"ns": self.ns, "label": self.label, "idx": self.idx,
                "z_shift_mm": self.z_shift_mm, "has_mesh": self.mesh is not None,
                "stale": self.stale, "run": self.run}

    def _drop_run(self):
        """이번 판의 것을 버린다. **자물쇠를 쥔 채** 부를 것.

        지우는 것: 마지막 상태 · 답파거리 · 결함 목록.
        남기는 것: 사건 로그(지난 판까지 포함해 읽는 기록이다) · 코스 · 메시.
        🔑 `run` 을 올리는 것이 화면에 전하는 유일한 신호다 — 열려 있는
           브라우저는 초록선과 마커를 지우고, 새로 붙는 브라우저는 애초에
           빈 화면을 받는다.
        """
        self.run += 1
        self.state = None
        self.state_seq += 1
        self.max_s_mm = 0.0
        self.last_step = -1
        # 🚨 결함도 같이 버린다. 예전에는 사건만 지워서, 재시작 뒤 새로고침하면
        #    **지난 판의 ✕ 마커와 Detect List 가 되살아났다**(실측).
        self.defects_dropped += len(self.defects)
        self.defects.clear()

    def mark_stale(self):
        """끊김으로 판정하고 판을 접는다. 실제로 접었으면 True."""
        with self.lock:
            if self.stale:
                return False
            self.stale = True
            self._drop_run()
            return True

    def set_state(self, d):
        with self.lock:
            self.last_state_t = time.monotonic()
            if self.stale:
                # 다시 살아났다 — 끊김 판정은 이미 판을 접어 뒀으므로
                # (`mark_stale` → `_drop_run`) 여기서는 깃발만 내린다.
                self.stale = False
            if self.roll_deg is not None:
                d["roll_deg"] = round(self.roll_deg, 1)
            if self.odom_v is not None:
                d["odom_v_mps"] = round(self.odom_v, 4)
            # 🔑 재시작은 **step 이 뒤로 가는 것**으로 드러난다 (시연이 Stop→Play
            #    에서 step 을 0 으로 되돌린다). 시연 쪽에 새 토픽·새 필드를
            #    요구하지 않고 이미 오는 값으로 알아채는 것이 요점이다.
            step = int(d.get("step") or 0)
            if step < self.last_step:
                # 지난 판의 사건은 **다음에 붙는 브라우저에 다시 보내지 않는다** —
                # 그러지 않으면 새로고침한 화면에 지난 판의 용접 마커가 되살아난다.
                # events_dropped 는 "앞에서 버린 수" 라 소켓 인덱스는 그대로 맞는다.
                self.events_dropped += len(self.events)
                self.events.clear()
                self._drop_run()          # 답파·결함·마지막 상태를 같이 버린다
            self.last_step = step
            d["run"] = self.run
            d["robot"] = self.ns          # 시연이 안 실어도 여기서 확정한다
            s = float(d.get("s_mm") or 0.0)
            if s > self.max_s_mm:
                self.max_s_mm = s
            d["max_s_mm"] = round(self.max_s_mm, 1)
            self.state = d
            self.state_seq += 1

    def set_course(self, d):
        with self.lock:
            self.course = d
            self.course_seq += 1
            z = d.get("z_shift_mm")
            if z is not None:
                # 🔑 시연이 직접 말해 준 값이 메시 헤더보다 우선이다 —
                #    좌표를 내는 쪽이 곧 프레임의 주인이다.
                self.z_shift_mm = float(z)

    def set_cam(self, role, data):
        with self.lock:
            self.cam[role] = data
            self.cam_seq[role] += 1

    def set_mesh(self, data):
        """받은 메시를 저장하고, 헤더의 z 오프셋을 잡는다.

        메시의 z 는 **그림의 프레임**이다. 로봇 좌표의 프레임(z_shift_mm)은
        course 가 말해 주는 쪽이 옳고, 아무도 안 말해 줬을 때만 이걸로 채운다.
        """
        with self.lock:
            self.mesh = data
            self.mesh_z_mm = mesh_z_shift(data)
            if self.z_shift_mm is None:
                self.z_shift_mm = self.mesh_z_mm

    def add_event(self, d):
        with self.lock:
            d.setdefault("robot", self.ns)
            self.events.append(d)
            if len(self.events) > self.MAX_LOG:
                cut = len(self.events) - self.MAX_LOG
                del self.events[:cut]
                self.events_dropped += cut

    def add_defect(self, d):
        with self.lock:
            self.defects.append(d)


class Hub:
    """로봇 전부 + 로봇에 속하지 않는 것(지령 이력·폴백 메시)을 들고 있다."""

    MAX_LOG = 300

    def __init__(self, specs):
        """specs — [(ns, label), ...]. **순서가 화면의 왼→오른쪽이다.**"""
        self.robots = [RobotStore(ns, label, i + 1)
                       for i, (ns, label) in enumerate(specs)]
        self.by_ns = {r.ns: r for r in self.robots}
        self.lock = threading.Lock()
        # 이 패널이 **보낸** 지령의 이력. 사건(event)과 달리 로봇이 아니라
        # 여기서 생기는 기록이라 따로 둔다 — "누가 뭘 눌렀나" 는 로봇 상태만
        # 봐서는 못 읽는다(구독자 0 이면 로봇에는 아예 안 갔을 수도 있다).
        self.cmds = []
        self.cmds_dropped = 0
        # 로컬 `src/son/maps/` 에서 굽거나 읽은 메시 — **이제 이쪽이 정본이다**
        self.file_mesh = None
        self.file_z_shift = None

    @property
    def primary(self):
        return self.robots[0]

    def mesh_ref(self):
        """(기준 로봇 ns|None, 메시 바이트|None, **그 메시가 구워진** z|None).

        🔑 메시에는 두 층이 다 들어 있으므로 **하나면 충분하다.** 먼저 온
           쪽(= ns 목록에서 앞선 쪽)을 기준 프레임으로 삼고, 다른 로봇의
           좌표는 3D 맵이 z 로 되밀어 여기에 겹친다.
        🚨 돌려주는 z 는 로봇이 말한 좌표 프레임이 아니라 **메시 헤더의 값**
           이다 — 기준은 그림 쪽이다(RobotStore.mesh_z_mm 주석 참고).
        """
        # 🔑 **로컬 파일이 먼저다** (2026-08-11 로직 변경). 맵 USD 를 이 PC 가
        #    갖고 있고 여기서 굽기 때문이다 — Isaac 이 `mesh` 토픽으로 보내는
        #    것은 이제 폴백이다(옛 시연·다른 PC 구성에서는 여전히 온다).
        if self.file_mesh is not None:
            return None, self.file_mesh, self.file_z_shift
        for r in self.robots:
            if r.mesh is not None:
                return r.ns, r.mesh, r.mesh_z_mm
        return None, None, None

    def roster(self):
        ref_ns, _, ref_z = self.mesh_ref()
        # 메시가 하나도 없으면 그릴 건물이 없다 — 그래도 로봇끼리는 서로
        # 맞춰야 하므로 **먼저 말한 로봇의 프레임**을 기준으로 삼는다.
        if ref_z is None:
            ref_z = next((r.z_shift_mm for r in self.robots
                          if r.z_shift_mm is not None), None)
        return {"robots": [r.info() for r in self.robots],
                "mesh_robot": ref_ns, "z_ref_mm": ref_z}

    def add_cmd(self, d):
        with self.lock:
            self.cmds.append(d)
            if len(self.cmds) > self.MAX_LOG:
                cut = len(self.cmds) - self.MAX_LOG
                del self.cmds[:cut]
                self.cmds_dropped += cut


def parse_specs(ns_param, label_param):
    """`-p ns:=floor1,floor2` → [(ns, label), ...].

    라벨은 `-p labels:=1층,2층` 로 덮어쓸 수 있고, 안 주면 규약의
    `contract.ns_label()` 이 정한다(floor1 → 1층).
    """
    names = [s.strip().strip("/") for s in str(ns_param).split(",")]
    names = [n for n in names if n]
    if not names:
        names = [contract.DEFAULT_NS]
    seen = []
    for n in names:                    # 같은 이름을 두 번 주면 두 칸이 겹친다
        if n not in seen:
            seen.append(n)
    given = ([s.strip() for s in str(label_param).split(",")]
             if str(label_param).strip() else [])

    def label(i, ns):
        return given[i] if i < len(given) and given[i] else contract.ns_label(ns)

    return [(ns, label(i, ns)) for i, ns in enumerate(seen)]


class PanelNode(Node):

    def __init__(self, hub):
        super().__init__("web_panel")
        self.declare_parameter("ns", contract.DEFAULT_NS)
        self.declare_parameter("labels", "")   # 쉼표 목록. 빈 값 = 규약 기본
        self.declare_parameter("port", 8080)
        self.declare_parameter("mesh", "")   # .webmesh 경로 (빈 값 = 자동 탐색)
        # 🔑 `.webmesh` 가 없거나 낡았으면 기동 때 **직접 굽는다**(0.5초쯤).
        #    map_usd 를 비우면 `src/son/maps/` 에서 고른다(DEFAULT_MAP_USD).
        #    🚨 z_shift 가 틀리면 건물이 로봇보다 위/아래로 통째로 어긋난다.
        #       기본값 2740.2 는 `real_map_demo_v1_3.py` 의 `-_Z1` 이다 —
        #       그쪽이 수평망을 월드 z=0 으로 올려놓고 좌표를 내므로 그림도
        #       같은 만큼 올려 구워야 겹친다. **저쪽을 고치면 여기도 고칠 것.**
        self.declare_parameter("map_usd", "")
        self.declare_parameter("z_shift_mm", 2740.2)
        # 🔑 **기본으로 켠다** (2026-08-11 로직 변경). 예전에는 Isaac(PC2)만
        #    굽고 `mesh` 토픽으로 넘겼는데, 이제 **이 PC 도 맵 USD 를 갖고
        #    있으므로 여기서 굽는다.** 이미 구워진 `.webmesh` 가 있으면 그것을
        #    그대로 쓴다(아래 find_mesh 의 낡음 판정 참고) — 매번 굽지 않는다.
        #    🔑 경합 걱정은 없어졌다: 양쪽이 **같은 맵 파일과 같은 z_shift** 를
        #       쓰므로 누가 구워도 같은 그림이 나온다.
        self.declare_parameter("bake_mesh", True)
        # 🚨 이 초 동안 `drive_state` 가 없으면 **시연이 죽은 것으로 보고 판을
        #    접는다**(마지막 상태·답파·결함을 버린다). 안 그러면 시연을 끄고
        #    새로고침해도 화면이 지난 판 그대로다 — 살아 있는 것처럼 보인다.
        #    상태는 10Hz 라 8초면 넉넉하다. 0 을 주면 이 판정을 끈다.
        self.declare_parameter("stale_s", 8.0)

        self.hub = hub
        self.pub_mission = {}
        sq, cq = contract.sensor_qos(), contract.command_qos()

        for r in hub.robots:
            t = Topics(r.ns)
            self.pub_mission[r.ns] = self.create_publisher(String, t.mission, cq)

            self.create_subscription(
                String, t.drive_state,
                lambda m, r=r: self._json(m, r.set_state), cq)
            self.create_subscription(
                String, t.event, lambda m, r=r: self._json(m, r.add_event), cq)
            # 코스 기하 — latched. 시연이 이 노드보다 먼저 떠 있어도 받는다.
            self.create_subscription(
                String, t.course, lambda m, r=r: self._on_course(m, r),
                contract.latched_qos())
            # CAD 메시 — 역시 latched. Isaac PC 가 따로 있을 때 이쪽으로 온다.
            self.create_subscription(
                UInt8MultiArray, t.mesh, lambda m, r=r: self._on_mesh(m, r),
                contract.latched_qos())
            # 카메라 — v1_3: floor1 은 전방(rgb), floor2 는 torch 슬롯을 쓴다
            # (rear 는 폐지, 위 CAM_CH 참고).
            self.create_subscription(
                CompressedImage, t.rgb,
                lambda m, r=r: r.set_cam("front", bytes(m.data)), sq)
            # 🔑 floor2 의 torch 슬롯은 **dongyeon opencv 오버레이**로만 채운다
            #    (contract.debug_rgb = `repair_robot/opencv_debug/compressed`
            #    — real_map_demo_v1_3.py `dy_pub["debug"]` 가 발행하는 실제
            #    이름. 이름과 무관하게 CompressedImage jpeg).
            #    floor2 는 drive_state.cam 이 torch 라(real_map_demo_v1_3.py:5183,
            #    cams 첫 역할) 이 칸이 화면에 뜬다. 🚨 Isaac 원본 torch 카메라
            #    (`torch/rgb/compressed`) 구독은 **일부러 뺐다** — 켜 두면 opencv
            #    안 된 원본이 같은 슬롯을 덮어써 오버레이가 안 보였다(실측
            #    2026-08-11). floor1 은 opencv_debug 를 발행 안 하니 기존
            #    front(rgb) 그대로다.
            self.create_subscription(
                CompressedImage, t.debug_rgb,
                lambda m, r=r: r.set_cam("torch", bytes(m.data)), sq)
            self.create_subscription(
                Odometry, t.odom,
                lambda m, r=r: setattr(r, "odom_v",
                                       float(m.twist.twist.linear.x)), sq)
            self.create_subscription(
                Imu, t.imu,
                lambda m, r=r: setattr(r, "roll_deg",
                                       _quat_to_roll(m.orientation)), sq)
            # 결함 리포트 — dongyeon 쪽은 기본 QoS(RELIABLE, depth 10)다.
            # 🔑 로봇이 여럿이면 검출 노드도 네임스페이스를 붙여 띄운다.
            self.create_subscription(
                String, f"/{r.ns}/{DEFECT_REPORT_REL}",
                lambda m, r=r: self._json(m, r.add_defect), 10)

        # 이름 없는 옛 절대 토픽 — **첫 번째 로봇 것으로 친다.** 두 대가 같은
        # 절대 토픽에 쏘면 구별할 방법이 없다(그래서 위처럼 갈라 띄울 것).
        self.create_subscription(
            String, DEFECT_REPORT_TOPIC,
            lambda m: self._json(m, hub.primary.add_defect), 10)

        self.stale_s = float(self.get_parameter("stale_s").value)
        if self.stale_s > 0:
            self.create_timer(1.0, self._check_stale)

        self.get_logger().info(
            "로봇 " + str(len(hub.robots)) + "대 구독 — "
            + " · ".join(f"{r.label}(/{r.ns})" for r in hub.robots)
            + (f" · 끊김 판정 {self.stale_s:.0f}초" if self.stale_s > 0
               else " · 끊김 판정 꺼짐"))
        contract.log_env(self.get_logger())

    def _check_stale(self):
        """상태가 끊긴 로봇의 판을 접는다 (1초마다).

        🔑 **끄는 것과 재시작하는 것을 같은 자리로 모은다.** 재시작은 step 이
           뒤로 가는 것으로 알아채지만(그때는 새 상태가 곧 온다), 그냥 끄면
           아무 신호도 안 온다 — 그 침묵을 여기서 신호로 바꾼다.
        """
        now = time.monotonic()
        for r in self.hub.robots:
            t = r.last_state_t
            if t is None or r.stale or (now - t) < self.stale_s:
                continue
            if r.mark_stale():
                self.get_logger().info(
                    f"[{r.label}] {now - t:.0f}초째 상태 없음 — 시연이 멈춘 "
                    f"것으로 보고 판을 접는다(답파·결함 초기화, 사건 로그는 "
                    f"남긴다). 다시 오면 새 판으로 센다")

    @staticmethod
    def _json(msg, sink):
        d = contract.parse(msg.data)
        if d is not None:
            sink(d)

    def _on_course(self, msg, r):
        d = contract.parse(msg.data)
        if d is not None and d.get("pts"):
            r.set_course(d)
            self.get_logger().info(
                f"[{r.label}] 코스 수신 — 표본 {len(d['pts'])}개, "
                f"총 {d.get('s_total_m', 0) * 1000:.0f}mm"
                + (f", z {d['z_shift_mm']:+.1f}mm" if "z_shift_mm" in d else ""))

    def _on_mesh(self, msg, r):
        """Isaac 쪽 CAD 메시. **로컬 파일보다 이게 옳다** — 시연이 실제로 쓴
        맵과 z 오프셋으로 구워진 것이다."""
        data = bytes(msg.data)
        if not data:
            return
        r.set_mesh(data)
        mz = r.mesh_z_mm
        self.get_logger().info(
            f"[{r.label}] CAD 메시 수신 — {len(data) / 1e6:.2f} MB "
            f"(z {mz:+.1f}mm). 이제부터 /mesh 는 이것을 준다"
            if mz is not None else
            f"[{r.label}] CAD 메시 수신 — {len(data) / 1e6:.2f} MB "
            f"(헤더에 z 가 없다 — 옛 포맷)")
        # 🚨 그림과 좌표가 다른 프레임이면 반드시 적는다. 3D 맵은 되밀어서
        #    맞추지만, 원인은 대개 **지난 층 것으로 캐시된 `.webmesh`** 라
        #    시연 쪽을 고쳐야 한다(ros_bridge.publish_mesh 가 z 로도 낡음을
        #    판정하도록 2026-08-10 고쳤다 — 옛 브리지면 여기 뜬다).
        if (mz is not None and r.z_shift_mm is not None
                and abs(mz - r.z_shift_mm) > 0.01):
            self.get_logger().warn(
                f"[{r.label}] 메시 프레임 z {mz:+.1f}mm 이 코스 프레임 "
                f"z {r.z_shift_mm:+.1f}mm 과 다르다 — 3D 맵이 "
                f"{(mz - r.z_shift_mm) / 1000:+.3f}m 되밀어 겹친다. "
                f"시연 쪽 `.webmesh` 캐시가 지난 층 것일 수 있다")

    # ── 지령 ────────────────────────────────────────────────────
    def send_mission(self, ns, cmd, mps=None, reason=""):
        """지령을 그 로봇에게 발행하고 (구독자 수) 를 돌려준다.
        검증은 contract 가 한다."""
        payload = contract.mission(cmd, mps=mps, reason=reason)
        pub = self.pub_mission[ns]
        pub.publish(String(data=contract.dumps(payload)))
        n = pub.get_subscription_count()
        self.get_logger().info(f"[{ns}] 지령 {cmd}"
                               + (f" mps={mps}" if mps is not None else "")
                               + f" → 구독자 {n}")
        return n


def _ws() -> Path:
    """워크스페이스 루트.

    🔑 **워크스페이스 이름을 가정하지 않는다.** 받는 사람이 어느 자리에 풀든
       돌아야 하므로 순서가 있다:
         ① `COBOT3_WS` 환경변수 — 명시가 항상 이긴다
         ② `__file__` 에서 위로 올라가며 `web/index.html` 을 가진 첫 디렉터리.
            소스 트리(`src/<이름>/pipe_comm/pipe_comm/…`)와 설치본
            (`install/pipe_comm/lib/python3.x/site-packages/pipe_comm/…`)
            **둘 다** 이 방식으로 잡힌다
         ③ 그래도 없으면 현재 작업 디렉터리 — 마지막 보루
    """
    env = os.environ.get("COBOT3_WS")
    if env:
        return Path(env)
    for parent in Path(__file__).resolve().parents:
        if (parent / "web" / "index.html").is_file():
            return parent
    return Path.cwd()


def bake_mesh(usd: Path, z_shift_mm: float, logger) -> Path | None:
    """맵 USD 를 `.webmesh` 로 굽는다. 실패하면 None — **메시는 항상 덤이다.**

    🔑 `tools/usd_to_webmesh.py` 를 **경로로 읽어** 그 안의 `convert()` 를
       부른다. tools/ 는 파이썬 패키지가 아니라 colcon 이 설치하지 않으므로
       import 로는 못 잡는다. 규칙(포맷·좌표계·법선 재계산)은 그 파일 한 곳에만
       둔다 — 여기에 베껴 적으면 반드시 갈라진다.

    🚨 이 노드는 python 3.10 이고 USD 바인딩(`pxr`)은 `usd-core` pip 패키지로
       들어와 있다(실측 0.26.8). 다른 PC 에는 없을 수 있으므로 **없어도 그냥
       진행한다** — 3D 맵은 코스 튜브만으로 동작한다.
    """
    import importlib.util

    tool = _ws() / "src/dongmin/pipe_comm/tools/usd_to_webmesh.py"
    if not tool.is_file():
        logger.warning(f"메시 변환기가 없다: {tool}")
        return None
    try:
        spec = importlib.util.spec_from_file_location("_usd_to_webmesh", tool)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        logger.info(f"CAD 메시 굽는 중 — {usd.name} (z {z_shift_mm:+.0f}mm)…")
        return mod.convert(usd, z_shift_mm)
    except (Exception, SystemExit) as exc:       # pxr 없음·USD 깨짐 등 전부
        # 🚨 SystemExit 도 반드시 잡는다 — tools/usd_to_webmesh.py 는 pxr 가
        #    없으면 **import 단계에서 `raise SystemExit`** 한다(그 파일 상단).
        #    SystemExit 은 Exception 이 아니라 BaseException 이라, 그냥
        #    `except Exception` 이면 여기를 빠져나가 노드를 통째로 죽인다.
        #    "메시는 항상 덤" 이라는 이 함수의 약속을 지키려면 이걸 삼켜서
        #    None 을 돌려주고, find_mesh 가 기존 `.webmesh` 로 폴백하게 둔다
        #    (usd-core 없는 관제 PC 에서 3D 맵이 안 뜨던 원인 — 2026-08-11).
        logger.warning(f"CAD 메시를 못 구웠다({exc.__class__.__name__}: {exc}) "
                       f"— 이미 구워진 .webmesh 가 있으면 그것을 쓰고, "
                       f"없으면 3D 맵은 코스 튜브만 그린다")
        return None


def find_mesh(explicit: str, logger, map_usd: str = "",
              z_shift_mm: float = 2740.2, bake: bool = True) -> bytes | None:
    """`.webmesh` 를 찾아 통째로 읽는다. 없으면 None — 페이지는 코스만 그린다.

    🔑 **이 PC 가 맵의 주인이다** (2026-08-11). 맵 USD 는 `src/son/maps/` 에
       있고, 여기서 `.webmesh` 로 구워 `/mesh` 로 서빙한다. Isaac(PC2)이
       `mesh` 토픽으로 보내 주기를 기다리지 않는다.
    🔑 **이미 구워져 있으면 그것을 쓴다** — 매번 굽지 않는다. 다시 굽는 것은
       셋 중 하나일 때뿐이다: 파일이 없다 · `.usd` 가 더 새롭다 · **다른
       z 프레임으로 구워져 있다**. 마지막 것을 빼먹으면 층을 바꿔도 옛 z 로
       구운 파일을 그대로 써서 에러 없이 건물만 어긋난다.
       🚨 낡은 메시는 없는 것보다 나쁘다 — 맵이 바뀐 줄 모르고 옛 건물 위에
          로봇을 그린다. 그래서 mtime 비교를 넣어 뒀다.

    map_usd     구울 원본. 비우면 `src/son/maps/` 에서 고른다(아래 규칙).
    z_shift_mm  맵 z 오프셋. `real_map_demo_v1_3.py` 의 `-_Z1` = 2740.2.
    bake        False 면 옛 동작(찾기만 한다).
    """
    maps = _ws() / "src/son/maps"
    if explicit:
        cands = [Path(explicit)]
    else:
        cands = sorted(maps.glob("*.webmesh"),
                       key=lambda p: p.stat().st_mtime, reverse=True)

    # 구울 원본을 정한다.
    # 🚨 **"가장 최근 파일" 로만 고르면 안 된다.** maps/ 에는 시연이 안 쓰는
    #    맵도 같이 있어서(reducer·tshape·straight290 …) 엉뚱한 건물을 구워도
    #    에러가 안 난다. 시연이 쓰는 이름을 먼저 찾고, 없을 때만 최근 것으로
    #    물러선다. 🔑 이름은 `real_map_demo_v1_3.py` 의 `_MAP` 과 같아야 한다.
    #    🚨 `*.usda` 는 일부러 안 본다 — maps/ 의 `.usda` 들은 부품·시험용이라
    #       mtime 이 더 새로워서 넣으면 그쪽이 뽑힌다.
    usd = Path(map_usd) if map_usd else None
    if bake and usd is None:
        named = maps / DEFAULT_MAP_USD
        if named.is_file():
            usd = named
        else:
            usds = sorted(maps.glob("*.usd"),
                          key=lambda p: p.stat().st_mtime, reverse=True)
            usd = usds[0] if usds else None
            if usd is not None:
                logger.warning(f"{DEFAULT_MAP_USD} 가 없다 — 가장 최근 "
                               f"{usd.name} 으로 굽는다. 시연이 쓰는 맵과 "
                               f"같은지 확인할 것")

    if bake and usd is not None and usd.is_file():
        out = usd.with_suffix(".webmesh")
        # 낡음 판정: 메시가 없거나 · 원본보다 오래됐거나 · **다른 층 프레임으로
        # 구워져 있다**. 마지막 것을 빼먹으면 층을 바꿔도 옛 z 로 구운 파일을
        # 그대로 쓴다(에러 없이 건물만 2.49m 어긋난다).
        baked_z = mesh_z_shift(out.read_bytes()) if out.is_file() else None
        stale = (not out.is_file()
                 or out.stat().st_mtime < usd.stat().st_mtime
                 or (baked_z is not None
                     and abs(baked_z - float(z_shift_mm)) > 0.01))
        if stale:
            if out.is_file():
                logger.info(f"CAD 메시가 낡았다 — {usd.name} 이 더 새롭다. 다시 굽는다")
            baked = bake_mesh(usd, z_shift_mm, logger)
            if baked is not None:
                cands = [baked] + [p for p in cands if p != baked]
        elif out not in cands:
            cands = [out] + cands

    for p in cands:
        if p.is_file():
            data = p.read_bytes()
            logger.info(f"CAD 메시 서빙: {p} ({len(data) / 1e6:.2f} MB)")
            return data
    logger.info("CAD 메시 없음 — 3D 맵은 코스 튜브만 그린다. "
                "tools/usd_to_webmesh.py 로 구울 것")
    return None


def find_web(logger) -> Path:
    """페이지 소스 디렉터리(`web/`). **소스 트리를 설치본보다 먼저** 잡는다.

    이 패키지는 --symlink-install 로도 파일이 복사되어 .py 수정마다 재빌드가
    필요한데(기록된 함정), 소스를 먼저 보면 HTML/CSS/JS 수정은 재빌드 없이
    **새로고침만으로** 반영된다. 설치본(share/pipe_comm/web)은 소스 트리 없이
    배포된 경우의 폴백이다 — setup.py 의 data_files 가 거기로 복사한다.
    """
    cands = [_ws() / "web"]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.append(Path(get_package_share_directory("pipe_comm")) / "web")
    except Exception:
        pass
    for i, p in enumerate(cands):
        if (p / "index.html").is_file():
            logger.info(f"웹 소스: {p}"
                        + (" (소스 트리 — 수정이 새로고침으로 반영된다)"
                           if i == 0 else " (설치본)"))
            return p
    raise SystemExit(f"web/index.html 이 없다 — 찾은 곳: "
                     f"{[str(c) for c in cands]}. 워크스페이스가 다른 자리면 "
                     f"COBOT3_WS 를 export 하고, 설치본을 쓸 거면 colcon "
                     f"build 를 다시 할 것")


def build_app(hub, node, web_dir=None):
    """FastAPI 앱. import 를 함수 안에 두어 fastapi 미설치 환경에서도
    모듈 자체는 읽히게 한다(오프라인 테스트가 Store 만 쓸 수 있게)."""
    import asyncio

    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse, JSONResponse, Response
    from fastapi.staticfiles import StaticFiles

    class NoCacheStatic(StaticFiles):
        """🚨 정적 파일에 **매번 재검증**을 시킨다 (`Cache-Control: no-cache`).

        기본 StaticFiles 는 `Last-Modified`/`ETag` 만 주고 `Cache-Control` 을
        안 준다. 그러면 브라우저가 **휴리스틱 캐싱**을 한다 — 조금 전에 고친
        파일일수록 "신선하다" 고 보고 서버에 묻지도 않는다. 그래서 `.js` 를
        고치고 새로고침해도 **화면이 그대로**였다(실측 2026-08-10: 3D Map 을
        세 칸으로 바꿨는데 옛 화면이 계속 떴다). 이 패널의 약속이
        "web/ 를 고치면 재빌드 없이 새로고침" 이므로 그 약속을 지키게 한다.

        no-cache 는 "캐시 쓰지 마라" 가 아니라 "쓰기 전에 물어봐라" 다 —
        안 바뀌었으면 304 라서 3D 맵(three.js 600KB)도 다시 안 받는다.
        """

        def file_response(self, *a, **kw):
            resp = super().file_response(*a, **kw)
            resp.headers["Cache-Control"] = "no-cache"
            return resp

    app = FastAPI(title="pipe_comm web_panel")
    app.mount("/static", NoCacheStatic(directory=str(web_dir)), name="static")

    ns_title = " · ".join(f"{r.label}(/{r.ns})" for r in hub.robots)

    def page():
        # 매 요청마다 읽는다 — 소스 서빙일 때 수정이 새로고침으로 반영된다.
        # 🚨 셸에도 no-cache 를 붙인다(정적 파일과 같은 이유). 이게 없으면
        #    **서버가 죽어 있어도 브라우저가 캐시로 페이지를 그려서**, 화면은
        #    멀쩡한데 웹소켓만 영영 "연결 대기…" 인 상태가 된다 — 서버가
        #    내려간 것인지 웹소켓만 막힌 것인지 구별이 안 된다.
        html = (web_dir / "index.html").read_text(encoding="utf-8")
        return HTMLResponse(html.replace("__NS__", ns_title),
                            headers={"Cache-Control": "no-cache"})

    # 🔑 페이지는 하나(셸)이고 갈라지는 것은 브라우저 쪽 라우터다. 서버는 어느
    #    주소로 들어와도 같은 셸을 준다 — 그래야 `/camera` 를 **직접 치거나
    #    새로고침해도** 404 가 아니다(웹소켓 하나를 페이지 전환 내내 공유하는
    #    구조라 서버 라우팅으로 가르면 전환마다 재접속이 된다).
    #    경로 목록은 web/app.js 의 ROUTES 와 맞출 것.
    for _p in ("/", "/home", "/camera", "/handling", "/map", "/detect",
               "/events"):
        app.get(_p)(page)

    @app.get("/robots")
    async def robots():
        """로봇 명패. 웹소켓의 `hello` 와 같은 내용 — 진단용으로 열어 둔다."""
        return hub.roster()

    @app.get("/mesh")
    async def mesh(robot: str = ""):
        # 🔑 **Isaac 쪽에서 온 것이 언제나 우선이다.** 시연이 실제로 쓴 맵과
        #    z 오프셋으로 구워진 것이라, 이쪽 디스크의 파일보다 옳다(층이
        #    바뀌면 로컬 파일은 2.49m 어긋난 채로 남아 있을 수 있다).
        #    브라우저가 페이지를 열 때 한 번 가져가므로, 늦게 도착해도
        #    새로고침하면 새 것을 받는다.
        # 🔑 메시는 건물 전체(두 층)를 담고 있어 로봇이 여럿이어도 하나면
        #    된다 — 기본은 기준 프레임 것. `?robot=floor1` 로 콕 집을 수 있다.
        if robot:
            r = hub.by_ns.get(robot.strip("/"))
            data = r.mesh if r is not None else None
        else:
            _, data, _ = hub.mesh_ref()
        if data is None:
            return JSONResponse({"error": "no mesh"}, status_code=404)
        return Response(content=data,
                        media_type="application/octet-stream")

    @app.post("/cmd")
    async def cmd(body: dict):
        import time as _t
        c = str(body.get("cmd", "")).upper()
        mps = body.get("mps")
        reason = str(body.get("reason", "web_panel"))
        # 대상: 빈 값 = 첫 번째 로봇(옛 클라이언트), `all`/`*` = 전부.
        want = str(body.get("robot", "")).strip().strip("/")
        if not want:
            targets = [hub.primary]
        elif want in ("all", "*"):
            targets = list(hub.robots)
        elif want in hub.by_ns:
            targets = [hub.by_ns[want]]
        else:
            return JSONResponse(
                {"ok": False, "error": f"모르는 로봇 {want!r} — "
                                       f"{[r.ns for r in hub.robots]}"},
                status_code=404)

        results, worst = [], None
        for r in targets:
            rec = {"stamp": round(_t.time(), 3), "cmd": c, "robot": r.ns,
                   "reason": reason}
            if mps is not None:
                rec["mps"] = mps
            try:
                n = node.send_mission(r.ns, c, mps=mps, reason=reason)
            except ValueError as exc:
                # 🔑 실패한 지령도 이력에 남긴다 — 왜 아무 일도 안 일어났는지가
                #    바로 이 줄에 있다(오타·미지원 지령).
                rec.update(ok=False, subscribers=0, error=str(exc))
                hub.add_cmd(rec)
                return JSONResponse({"ok": False, "error": str(exc)},
                                    status_code=400)
            # 구독자 0 = 지령이 아무 데도 안 갔다. 화면이 이 사실을 띄워야 한다.
            rec.update(ok=n > 0, subscribers=n)
            hub.add_cmd(rec)
            results.append({"robot": r.ns, "label": r.label,
                            "ok": n > 0, "subscribers": n})
            worst = n if worst is None else min(worst, n)

        dead = [x["label"] for x in results if not x["ok"]]
        return {"ok": not dead, "subscribers": worst or 0, "results": results,
                "warn": None if not dead else
                f"{'·'.join(dead)} 구독자 0 — 그쪽 시연이 떠 있는가?"}

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        # 🔑 **명패를 맨 먼저 보낸다.** 페이지는 이걸 받고서야 칸을 몇 개
        #    만들지 안다 — 시연이 아직 안 떠서 데이터가 하나도 없어도 두 칸이
        #    "수신 대기" 로 서 있어야 조작하는 사람이 안 헷갈린다.
        roster = hub.roster()
        await sock.send_text(json.dumps({"type": "hello", "data": roster},
                                        ensure_ascii=False))
        # 🚨 커서는 "이 소켓이 이미 본 **누적 개수**" 다. 새 소켓은 하나도 못
        #    봤지만, **버려진 것은 앞으로도 안 보낸다** — 그래서 0 이 아니라
        #    지금까지 버린 수에서 시작해야 한다. 0 으로 두면 `목록[0-버린수:]`
        #    가 음수 슬라이스가 되어, 지난 판이 끝난 뒤 새로 붙은 브라우저가
        #    이번 판의 앞부분을 통째로 놓친다(재시작 뒤에만 드러나는 함정).
        with hub.lock:
            cm_i = hub.cmds_dropped
        cur = {}
        for r in hub.robots:
            with r.lock:
                cur[r.ns] = {"state": -1, "course": -1,
                             "ev": r.events_dropped, "df": r.defects_dropped,
                             "cam": {c: -1 for c in CAM_CH}}
        try:
            while True:
                sent = False
                # 명패가 달라졌으면(메시·z 오프셋이 늦게 도착) 다시 보낸다 —
                # 3D 맵의 층 정렬이 여기에 달려 있다.
                now = hub.roster()
                if now != roster:
                    roster = now
                    await sock.send_text(json.dumps(
                        {"type": "hello", "data": roster}, ensure_ascii=False))
                    sent = True
                with hub.lock:
                    cmds = hub.cmds[cm_i - hub.cmds_dropped:]
                    cm_next = len(hub.cmds) + hub.cmds_dropped
                for m in cmds:
                    await sock.send_text(json.dumps(
                        {"type": "cmd", "robot": m.get("robot"), "data": m},
                        ensure_ascii=False))
                    sent = True
                cm_i = cm_next

                for r in hub.robots:
                    k = cur[r.ns]
                    with r.lock:
                        state, s_seq = r.state, r.state_seq
                        course, c_seq = r.course, r.course_seq
                        cams = {c: (r.cam[c], r.cam_seq[c]) for c in CAM_CH}
                        events = r.events[k["ev"] - r.events_dropped:]
                        ev_next = len(r.events) + r.events_dropped
                        defects = r.defects[k["df"] - r.defects_dropped:]
                        df_next = len(r.defects) + r.defects_dropped
                    # 코스를 상태보다 먼저 — 페이지가 코스 없이는 점을 못 찍는다
                    if course is not None and c_seq != k["course"]:
                        await sock.send_text(json.dumps(
                            {"type": "course", "robot": r.ns, "data": course},
                            ensure_ascii=False))
                        k["course"], sent = c_seq, True
                    if state is not None and s_seq != k["state"]:
                        await sock.send_text(json.dumps(
                            {"type": "state", "robot": r.ns, "data": state},
                            ensure_ascii=False))
                        k["state"], sent = s_seq, True
                    for e in events:
                        await sock.send_text(json.dumps(
                            {"type": "event", "robot": r.ns, "data": e},
                            ensure_ascii=False))
                        sent = True
                    k["ev"] = ev_next
                    for d in defects:
                        await sock.send_text(json.dumps(
                            {"type": "defect", "robot": r.ns, "data": d},
                            ensure_ascii=False))
                        sent = True
                    k["df"] = df_next
                    for role, ch in CAM_CH.items():
                        data, seq = cams[role]
                        if data is not None and seq != k["cam"][role]:
                            await sock.send_bytes(bytes([r.idx, ch]) + data)
                            k["cam"][role], sent = seq, True
                if not sent:
                    await asyncio.sleep(0.03)
        except (WebSocketDisconnect, ConnectionResetError):
            pass

    return app


def main(args=None):
    try:
        import uvicorn
    except ImportError:
        raise SystemExit("uvicorn/fastapi 가 없다 — pip install fastapi uvicorn")

    rclpy.init(args=args)
    # 파라미터를 읽으려면 노드가 먼저 있어야 하는데 노드는 hub 를 받아야 한다 —
    # 임시 노드 하나로 ns 만 먼저 읽는다(선언은 PanelNode 가 다시 한다).
    boot = Node("web_panel_boot")
    boot.declare_parameter("ns", contract.DEFAULT_NS)
    boot.declare_parameter("labels", "")
    specs = parse_specs(boot.get_parameter("ns").value,
                        boot.get_parameter("labels").value)
    boot.destroy_node()

    hub = Hub(specs)
    node = PanelNode(hub)
    port = int(node.get_parameter("port").value)
    file_mesh = find_mesh(
        str(node.get_parameter("mesh").value), node.get_logger(),
        map_usd=str(node.get_parameter("map_usd").value),
        z_shift_mm=float(node.get_parameter("z_shift_mm").value),
        bake=bool(node.get_parameter("bake_mesh").value))
    hub.file_mesh = file_mesh
    hub.file_z_shift = (mesh_z_shift(file_mesh) if file_mesh else None)
    web_dir = find_web(node.get_logger())

    # rclpy 는 뒷단 스레드에서 돌리고, uvicorn 이 주 스레드를 가진다.
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    node.get_logger().info(
        f"관제 패널 시작 — http://<서버IP>:{port}  "
        + " | ".join(f"{r.label} = /{r.ns}" for r in hub.robots)
        + "\n  🚨 지령 버튼이 있는 페이지다. 공인망에 열지 말 것 "
          "(본인 IP /32 또는 SSH 터널)")
    try:
        uvicorn.run(build_app(hub, node, web_dir),
                    host="0.0.0.0", port=port,
                    log_level="warning")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
