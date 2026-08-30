# `src/dongmin` — ROS 2 통신 규약 + 웹 관제 (김동민)

Isaac Sim 안에서 벌어지는 일을 **밖에서 보고 조종할 수 있게** 만드는 부분.
토픽 규약을 한 곳에 못박고, 그 규약대로 발행하는 브리지와 브라우저 관제 화면을 만들었다.

> 전체 개요와 실행 순서는 워크스페이스 루트의 [`README.md`](../../README.md) 를 볼 것.

---

## 📁 구성

```
dongmin/
├── isaac_bridge/
│   └── ros_bridge.py          Isaac(3.11) 쪽 발행자 — 시연 스크립트가 import 한다
└── pipe_comm/                 ROS 2 패키지 (ament_python, Python 3.10)
    ├── pipe_comm/
    │   ├── contract.py        📜 토픽 이름·JSON 스키마의 단일 출처
    │   ├── image_codec.py     ROS 영상 ↔ numpy (cv_bridge 를 쓰지 않는다)
    │   ├── web_panel.py       FastAPI 관제 서버 — 보고 + 조종
    │   ├── web_view.py        zero-dep 카메라 뷰어 (stdlib 웹소켓 직접 구현)
    │   ├── camera_monitor.py  카메라 수신 검증 — 해상도·밝기·유효 깊이 비율
    │   ├── drive_monitor.py   주행/사건 수신 — 로봇 여러 대를 한 노드에서
    │   └── mission_cli.py     임무 지령 CLI (START/STOP/RECALL/SPEED/ESTOP)
    ├── launch/monitor.launch.py
    ├── tools/usd_to_webmesh.py   맵 USD → 웹 3D 맵용 단일 바이너리
    ├── ROS2_통신규격.md          규격서
    └── ROS2_웹연동_이식가이드.md  시연 스크립트에 접붙이는 자리 5곳
```

웹 페이지 소스는 워크스페이스 최상위 [`web/`](../../web) 에 있다 —
ROS 파이썬 패키지와 수명·도구가 달라 분리했다.

---

## 🔑 핵심 설계 판단

### ① 규약을 파일 하나로 못박았다

**토픽 이름을 문자열로 직접 적지 않는다.**

```python
Topics("floor1").rgb        # '/floor1/rgb/compressed'
Topics("floor1").bogus      # AttributeError — 규약에 없는 토픽이다
```

오타는 **에러가 아니라 침묵**이다. 발행은 성공하고 아무도 못 받는다 —
진단이 제일 오래 걸리는 부류다. 실제로 팀 안에서 `/rgb`, `front/rgb`,
`/rgb/compressed` 세 갈래로 갈린 적이 있고, `pipe_comm` 은 그걸 합치려고 만든 패키지다.

### ② `contract.py` 는 순수 stdlib 다

`rclpy` 도 `numpy` 도 import 하지 않는다. 이유가 있다 —
**발행하는 쪽(Isaac, Python 3.11)과 받는 쪽(ROS, 3.10)이 같은 인터프리터를 못 쓴다.**
규약을 양쪽에 손으로 베껴 적으면 반드시 갈라지므로, 어느 파이썬에서나 읽히는
stdlib 만으로 짜서 **한 파일을 양쪽이 그대로 읽는다.**

### ③ `cv_bridge` 를 쓰지 않는다

같은 이유다. 3.10 용 `cv_bridge` 를 Isaac 의 3.11 이 못 읽는다.
어차피 하는 일은 `frombuffer` 와 `imencode` 둘뿐이라 `image_codec.py` 로 직접 짰다.

🚨 **깊이 압축 규약 — 16UC1 PNG, 단위 mm, 0 = 무효.**
JPEG 는 손실 압축이라 깊이값을 훼손하므로 절대 쓰지 않는다.
그리고 Isaac 의 `distance_to_camera` 는 빈 공간을 `inf`/`NaN`/최대거리 중 무엇으로도
돌려주므로, **uint16 캐스팅 전에 무효 화소를 0 으로** 만든다.

### ④ 로봇 여러 대 — 네임스페이스로 가른다

```bash
ros2 run pipe_comm web_panel --ros-args -p ns:=floor1 -p port:=8080
ros2 run pipe_comm web_panel --ros-args -p ns:=floor2 -p port:=8081
```

**1층과 2층은 연결되어 있지 않다.** 각자 자기 층에서 따로 출발해 따로 끝나는 별개의
임무라, 상태·사건·결함·카메라·지령이 전부 로봇마다 따로 간다.

🚨 **층이 다르면 월드 좌표 프레임도 다르다.** 시연은 활성 층의 수평망을 월드 z=0 으로
올려놓고 좌표를 내므로(floor2 +250 / floor1 +2740.2), 두 대의 위치를 그대로 겹쳐 그리면
**2.49m 어긋난다.** 각 로봇의 z 오프셋을 `course` 또는 받은 `.webmesh` 헤더에서 읽어
3D 맵이 그만큼 되민다.

### ⑤ CAD 메시를 DDS 로 보낸다

웹 3D 맵은 중심선 튜브만으로는 배관망이 어떤 건물 안에 있는지 안 보인다.
그래서 맵 USD 의 메시 전체를 `.webmesh` 단일 바이너리로 구워 **latched 로 1회** 보낸다.

```
[0:4]   uint32  JSON 헤더 길이 L
[4:4+L] utf-8   JSON {version, vtx_count, tri_count, bbox, parts:[...]}
        (4바이트 정렬 패딩)
[...]           정점·인덱스 이진 블록
```

glTF 를 안 쓴 이유는 로더(three.js 등)를 페이지에 들여와야 하는데 **파서가 JS 30줄**이라
포맷을 직접 정하는 쪽이 전체 비용이 싸기 때문이다.

🚨 **낡음 판정에 z 오프셋도 넣는다.** 예전에는 mtime 만 봐서, floor1 로 한 번 굽고 나면
`--course floor2` 로 돌려도 **floor1 프레임 메시를 그대로 보냈다** — 웹 3D 맵에 건물이
2.49m 어긋난 채 그려지고(로봇이 남의 층 배관 속을 달린다) **에러는 하나도 안 난다.**

### ⑥ `isaac_bridge` 를 `pipe_comm` **밖**에 둔 이유

`pipe_comm/` 은 colcon 이 빌드하는 Python 3.10 ROS 패키지다.
3.11 전용인 `ros_bridge.py` 가 그 안에 들어가면 `find_packages()` 가 설치본에 같이 복사하고
`from pipe_comm import ros_bridge` 같은 오용을 부른다.
**형제 디렉터리**로 두면 `package.xml` 이 없어 colcon 이 무시한다.

---

## 🚨 밟았던 함정

| 함정 | 증상 | 원인·수정 |
|---|---|---|
| annotator 부착 시점 | 영상이 아예 안 나옴 | annotator 는 런타임 자원이라 **`world.reset()` 뒤**에 붙여야 살아남는다 |
| 카메라 토픽 매핑 | 웹 카메라 칸이 빔 | 용접 카메라를 끄면서 토픽이 `torch/rgb`→`rgb` 로 바뀌었다 → **같은 영상을 두 역할로 등록**해 어느 쪽을 구독해도 받게 했다 |
| 메시 전송 | 발행에 몇 초씩 멈춤 | `list()` 로 넘기면 77만 개 int 를 만든다 → `array('B')` 는 rclpy 가 그대로 받아 간다 |
| `moving` 알림 | 로그가 이것만으로 참 | **전환될 때만** 낸다 |
| `spin` 타이밍 | 지령을 아예 못 받음 | 매 물리 스텝마다 `spin_once(timeout_sec=0.0)` — 0 이 아니면 **물리가 그만큼 멈춘다** |
| 워크스페이스 경로 | 다른 자리에 풀면 웹이 안 뜸 | 이름을 가정하지 않는다 → `COBOT3_WS` → `__file__` 에서 위로 올라가며 `web/index.html` 탐색 → cwd |

---

## ▶ 실행

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=143            # 🚨 규격서 값

ros2 run pipe_comm web_panel --ros-args -p ns:=floor1 -p port:=8080
ros2 run pipe_comm camera_monitor   # 카메라가 오는지부터 가린다
ros2 run pipe_comm drive_monitor    # 주행·사건 한 줄 로그
ros2 run pipe_comm mission_cli -- STOP --ns floor2
ros2 launch pipe_comm monitor.launch.py
```

시뮬레이션 쪽은 **`ROS_PUB=1`** 로 띄워야 발행이 켜진다(기본은 성능 때문에 꺼져 있다).

```bash
cd ../son && ROS_PUB=1 ./run_v13.sh both
```

## ✅ 검증

```bash
cd pipe_comm
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
#   test_contract.py     규약 스키마      11 통과
#   test_image_codec.py  영상·깊이 코덱    7 통과
```

> 🚨 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 이 필요하다. 시스템에 깔린 `anyio` 의
> pytest 플러그인이 자동으로 로드되면서 Humble 의 pytest 버전과 충돌한다
> (`ModuleNotFoundError: _pytest.scope`). 시험 코드의 문제가 아니다.
