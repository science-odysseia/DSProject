# pipe_comm — 배관 점검 로봇 ROS 2 통신

Isaac Sim 쪽(발행)과 ROS 2 노드 쪽(수신·지령)이 주고받는 **규약과 감시 도구**.

```
pipe_comm/
├─ contract.py        🔑 토픽 이름·JSON 스키마 **단일 출처**. 양쪽이 이것만 읽는다
├─ image_codec.py     영상 인코딩/디코딩 (cv_bridge 없이)
├─ camera_monitor.py  카메라가 오는지 검증하는 노드
├─ drive_monitor.py   주행/정지 알림을 보는 노드 (로봇 여러 대 동시)
├─ mission_cli.py     출발·정지·복귀 지령을 쏘는 CLI
├─ web_view.py        브라우저 카메라 뷰 (zero-dep 진단용, 웹소켓)
└─ web_panel.py       시연용 관제 패널 — 버튼·속도·지도·결함 목록 (FastAPI)
```

---

## 5분 요약

| | |
|---|---|
| 도메인 | **`ROS_DOMAIN_ID=143`** — 다르면 토픽이 아예 안 보인다 |
| 미들웨어 | `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (양쪽 같아야 한다) |
| 시연 구성 | **RTX 5080 MSI PC 한 대**에서 Isaac 과 ROS 노드를 같이 돌린다 |
| 메시지 | 경계는 **표준 메시지만**. 커스텀 `.msg` 없음 → 받는 쪽 빌드 불필요 |
| 구조체 | `std_msgs/String` 에 JSON. 스키마는 `contract.py` |
| 영상 QoS | **BEST_EFFORT** — RELIABLE 로 구독하면 프레임이 0 이다 |

---

## 🚨 PC 번호로 부르지 않는다

팀 관례는 `PC1 = 카메라·검출 쪽`, `PC2 = Isaac Sim` 인데 번호는 읽는 사람마다
뒤집힌다(이 규약을 처음 쓸 때 실제로 반대로 적었다). 그래서 문서·코드 전부
**역할 이름**만 쓴다 — **`Isaac 쪽`(3.11)** 과 **`ROS 노드 쪽`(3.10)**.

---

## 왜 커스텀 msg 를 안 쓰는가

**Isaac Sim 은 Python 3.11, 우리 ROS 2 Humble 은 3.10 이다.** 커스텀 msg 는
rosidl 이 만든 3.10 용 `.so` 에 묶여 있어 Isaac 쪽에서 못 쓴다. Isaac 5.1 의
내장 rclpy 는 `isaacsim.ros2.bridge/humble` 에 박제된 3.11 ABI 라 우리
워크스페이스를 그대로 얹을 수 없다.

표준 메시지는 그 번들에 **이미 들어 있으므로 빌드가 0 이다.** 구조가 필요한
값은 `std_msgs/String` 에 JSON 으로 싣는다. 부수 효과로 받는 쪽이 이 패키지를
빌드하지 않아도 `ros2 topic echo` 로 바로 볼 수 있다.

`pipe_msgs` 는 레포에 없고 앞으로도 만들지 않는다.

---

## 토픽 지도

로봇 한 대 = 네임스페이스 하나. `map_test_demo.py` 는 코스마다 로봇이 한
대씩이라 `/elbow_v` `/elbow_h` `/tee` 로 갈라지고, `repair_demo.py` /
`real_map_demo.py` 는 한 대라 `/robot` 하나다.

### Isaac → ROS 노드 (주는 데이터)

| 토픽 | 타입 | 내용 | 주기 |
|---|---|---|---|
| `rgb/compressed` | `CompressedImage` jpeg | 전방 카메라 | 10Hz |
| `depth/compressed` | `CompressedImage` **16UC1 PNG(mm)** | 전방 깊이 | 10Hz |
| `depth` | `Image` 32FC1(m) | 원본 정밀도. **선택** | 10Hz |
| `camera_info` | `CameraInfo` | 어안 f 를 `K[0]` 에 | 10Hz |
| `rear/rgb/compressed` | `CompressedImage` | 후방 카메라 | 10Hz |
| `torch/rgb/compressed` | `CompressedImage` | 토치 카메라(용접부 근접) | 10Hz |
| `odom` | `Odometry` | **`pose.position.x` = 중심선 호길이** | 10Hz |
| `imu` | `Imu` | 롤(결함 시계각 기준)·요 각속도 | 50Hz |
| `joint_states` | `JointState` | 휠 12 + 피스톤 6 | 10Hz |
| `moving` | `Bool` | 주행 중인가 — **주행/정지 알림** | 전환 시 |
| `drive_state` | `String` JSON | FSM 상태 전체 | 10Hz |
| `event` | `String` JSON | 1회성 사건 | 사건 시 |
| `course` | `String` JSON | 코스 중심선 표본 — **latched** | 기동 시 1회 |

### ROS 노드 → Isaac (받는 데이터)

| 토픽 | 타입 | 내용 |
|---|---|---|
| `cmd_vel` | `Twist` | `linear.x` 만 (m/s). 음수 = 후진 |
| `mission` | `String` JSON | `START` `STOP` `RECALL` `RETRY` `SPEED` `ESTOP` |
| `repair_target` | `String` JSON | 결함 위치(호길이 + 시계각) → 용접 정렬 |

### JSON 스키마

```jsonc
// drive_state — 10Hz
{"stamp":1.23,"robot":"elbow_v","step":2963,"state":"RUN","moving":true,
 "dir":1,"speed_mps":0.1,"s_mm":431.2,"s_total_mm":935.6,"off_mm":3.44,
 "lap":2,"stuck":0,"reason":""}

// event — 사건 때만. alert=true 면 사람이 바로 봐야 한다
{"stamp":1.23,"robot":"tee","event":"STUCK","s_mm":431.2,
 "detail":"방향 전환 → 후진","alert":true}

// course — 기동 시 1회, latched(TRANSIENT_LOCAL). 웹 3D 맵이 이걸로 관을
// 그린다. pts 는 [호길이 s, x, y, z] 이고 **전부 m**, s 오름차순(간격 불균일
// 허용). 좌표계는 drive_state 의 pos_m 과 같은 월드다.
{"stamp":1.23,"robot":"robot","ir_m":0.05,"bend_r_m":0.15,"s_total_m":2.5331,
 "pts":[[0.0,0.33,0.85,0.335],[0.02,0.33,0.85,0.315], ...]}

// mission
{"stamp":1.23,"cmd":"SPEED","reason":"곡관 진입","mps":0.05}

// repair_target — clock_deg 0°=천장, 180°=바닥(중력)
{"stamp":1.23,"id":"crack_0001","s_mm":1130.0,"clock_deg":180.0,
 "width_mm":2.2,"depth_mm":2.2,"confidence":0.87}
```

상태값 / 사건 / 지령의 **전체 목록은 `contract.py` 의 상수**다. 문자열을 손으로
적지 말 것 — 모르는 값을 주면 `ValueError` 로 바로 걸리게 해 뒀다.

---

## 규약이 정하는 것들 (틀리면 조용히 망가지는 자리)

| 항목 | 규약 | 안 지키면 |
|---|---|---|
| Depth 압축 | **16UC1 PNG, mm, 0=무효** | JPEG 는 손실이라 깊이가 훼손된다 |
| Depth 무효 | 캐스팅 **전에** 0 으로 | `inf`/`NaN` 이 쓰레기 정수가 되어 관 단절이 "가까운 벽"이 된다 |
| Depth 상한 | 65m 초과는 무효 | uint16 wrap — 먼 곳이 코앞으로 보인다 |
| RGB 채널 | cv2 에 넘길 때 BGR | R/B 가 바뀐 채로 YOLO 에 들어간다(검출률만 떨어짐) |
| RGB 알파 | RGBA 4채널을 3으로 자름 | Isaac annotator 는 RGBA 를 준다 |
| `odom.x` | **중심선 호길이** | 직선거리를 쓰면 곡관에서 위치가 뒤로 간다 |
| `K[0]` | 등거리 어안 f (r=f·θ) | 핀홀로 역산하면 입사각이 틀어진다 |
| 시계각 | 0°=천장, 180°=바닥 | 부호가 뒤집히면 **토치를 정반대로 돌린다** |
| 영상 QoS | BEST_EFFORT | 토픽은 보이는데 프레임이 0 |
| 지령 QoS | RELIABLE | 정지 지령을 놓친다 |

---

## 설치 — 받아서 바로 돌리기

**이 패키지는 `src/dongmin/` 만 받으면 된다.** 다른 팀원 코드에 의존하지
않고, 커스텀 인터페이스도 없어서 이 패키지 하나만 빌드하면 끝난다.

### 처음 한 번

```bash
# 0) 의존성 (한 번만)
sudo apt install -y python3-opencv python3-numpy

# 1) 받기 — 이미 워크스페이스가 있으면
cd ~/cobot3_ws && git pull

#    없으면 새로 만든다
mkdir -p ~/cobot3_ws/src && cd ~/cobot3_ws
git clone <레포주소> _repo && cp -r _repo/src/dongmin src/ && rm -rf _repo

# 2) 빌드
source /opt/ros/humble/setup.bash
colcon build --packages-select pipe_comm

# 3) 환경 — 🚨 이 두 줄을 `~/.bashrc` 에 넣어 둘 것
echo 'export ROS_DOMAIN_ID=143'                 >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc
```

### 매번

```bash
source /opt/ros/humble/setup.bash
source ~/cobot3_ws/install/setup.bash
```

🔑 **환경변수를 안 맞춰도 노드는 멀쩡히 뜬다.** 그래서 각 노드가 기동할 때
`ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` 을 스스로 점검해 **`[환경]` 으로 찍는다**
— 첫 줄에 빨간 글씨가 없으면 통과다.

---

## 실행

```bash
# 카메라가 오는지
ros2 run pipe_comm camera_monitor --ros-args -p ns:=elbow_v

# 주행/정지 알림 — 로봇 3대를 한 노드에서
ros2 run pipe_comm drive_monitor --ros-args -p ns:="[elbow_v,elbow_h,tee]"

# 둘 다 한 번에
ros2 launch pipe_comm monitor.launch.py ns:=all

# 지령
ros2 run pipe_comm mission_cli -- STOP --ns elbow_v
ros2 run pipe_comm mission_cli -- SPEED --mps 0.05 --ns all
```

로봇이 한 대인 시연(`repair_demo.py` / `real_map_demo.py`)이면 `ns` 를 생략한다
(기본 `/robot`).

### 관제 패널 (시연용)

```bash
pip install fastapi uvicorn websockets     # 🚨 web_panel 만 필요로 한다 (한 번만)
ros2 run pipe_comm web_panel --ros-args -p ns:=robot -p port:=8080
```

왼쪽 사이드바로 페이지를 오간다 — 주소도 같이 바뀐다(`<IP>:8080/camera`).

| 페이지 | 주소 | 내용 |
|---|---|---|
| Home | `/home` | 상태·진행률·결함 수·최근 사건·전방 카메라 요약 |
| Camera | `/camera` | 전방·후방·토치 3대 |
| Robot Handling | `/handling` | 시작·정지·복귀·끼임탈출·비상정지, 속도 |
| 3D Map | `/map` | 배관 맵 + 로봇 위치 + 결함/수리 마커 |
| Detect List | `/detect` | 결함 리포트 표 |
| Event Log | `/events` | 사건 로그 (경고만 보기) |

**페이지 소스는 워크스페이스의 `web/`** 에 있다 (HTML/CSS/JS + three.js 벤더
사본 — `web/README.md` 참조). web_panel 은 그 디렉터리를 설치본보다 먼저
잡으므로 **HTML/JS 수정은 재빌드 없이 새로고침**으로 반영된다.

🔑 라우팅은 브라우저가 하고 **웹소켓은 페이지 전환에도 안 끊긴다.** 대신
경로 목록이 `web/app.js` 의 `ROUTES` 와 `web_panel.py` 두 곳에 있으니 페이지를
추가하면 둘 다 고칠 것(한쪽만 고치면 새로고침에서만 404 가 난다).

**코스 기하는 하드코딩하지 않는다.** 시연이 기동할 때 `course` 토픽
(latched)으로 중심선 표본을 발행하고 패널이 그걸로 관을 그린다 — 단일 출처는
시연 쪽 `CenterLine` 이라 맵이 바뀌면 시연만 고치면 된다. 🚨 시연을 `--ros`
없이 띄우면 코스도 카메라도 안 온다 (페이지에 "코스 수신 대기" 로 뜬다).

3D 맵의 **CAD 메시**: `tools/usd_to_webmesh.py` 로 맵 USD 를 `.webmesh` 로
구워 두면(도구 docstring 에 실행법 — pxr 때문에 Isaac 파이썬 전용) web_panel
이 자동으로 찾아 `/mesh` 로 서빙하고, 페이지가 전체 맵(벽은 옅게, 배관은 층별
색)을 그린다. 파일이 없으면 코스 튜브만으로 그대로 돈다. 경로 지정은
`-p mesh:=<파일>`.

🚨 `web_view` 와 기본 포트(8080)가 같다 — 둘 중 하나만 띄우거나 `-p port:=` 로
가른다. 지령 버튼이 있으니 공인망에 열지 말 것 (본인 IP /32 또는 SSH 터널).

`mission_cli` 는 발행 뒤 **구독자 수를 확인하고 0 이면 0 이 아닌 종료코드**를
낸다 — RELIABLE 이라도 구독자가 없으면 지령은 그냥 사라지기 때문이다(latch 가
아니다). 스크립트에서 `&&` 로 이어 쓸 수 있다.

---

## 오프라인 검증

ROS 없이 돈다 (18항목).

```bash
cd src/dongmin/pipe_comm
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q
```

```
test_contract.py     토픽 이름·JSON 스키마·모르는 값 거부
test_image_codec.py  RGB 채널 순서, 알파 제거, 깊이 무효/오버플로 처리
```

🚨 **`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 을 붙이는 이유** — ROS 를 소싱한
셸에서는 ament 의 pytest 플러그인이 자동으로 로드되는데, Humble 이 기대하는
pytest 버전보다 새 pytest 가 깔려 있으면 훅 시그니처가 안 맞아 **수집 단계에서
통째로 죽는다**(`pytest_pycollect_makemodule(path, parent)` 오류). 시험 코드
문제가 아니다. 플러그인 자동로드만 끄면 어느 버전에서도 돈다.

🔑 규약이 깨지는 것은 **런타임에 침묵으로** 나타난다(토픽 오타 → 아무도 못
받음, 깊이 무효 처리 실패 → 판정이 조용히 뒤집힘). 그래서 여기서 미리 건다.

---

## 문제가 생기면 보는 순서

1. `ros2 topic list` 에 아무것도 없음 → `ROS_DOMAIN_ID` 143 인가, 양쪽
   `RMW_IMPLEMENTATION` 이 같은가
2. 토픽은 있는데 프레임 0 → 구독 QoS 가 BEST_EFFORT 인가, Isaac 이 **Play**
   상태인가
3. 영상이 검음 → 관 내부는 로봇 조명이 유일한 광원이다. 조명 intensity 확인
4. Depth 유효율이 낮음 → near clip 이 관벽(25mm)을 자르는가, 카메라가 본체에
   파묻혔는가
5. `drive_monitor` 가 **⛔ 두절** → Isaac 프로세스가 살아 있는가
6. `mission_cli` 가 **구독자 0** → Isaac 쪽 시연이 떠 있는가, `--ns` 가 맞는가

---

## 이전 패키지에서 바뀐 것

`src/dongmin` 의 `M0609` / `pipe_inspect` 를 지우고 이 패키지로 다시 만들었다
(2026-08-06). 옛 `pipe_inspect/camera_check.py` 는 `/rgb` `/depth` 를 **raw
Image** 로 받았는데, 같은 팀의 `dongyeon/pipe_vision_node` 는
`/rgb/compressed` `/depth/compressed` 를, `son/camera/rig.py` 는 `front/rgb`
`front/depth`(32FC1) 를 써서 **세 갈래로 갈려 있었다.** 어느 둘을 붙여도 안
맞는 상태였다. `contract.py` 하나로 합친 것이 이 패키지의 존재 이유다.
