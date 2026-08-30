# 배관 점검 로봇 — ROS 2 통신 규격 v1.0

**대상 독자: Isaac Sim 파트 담당자.** 이 문서 하나만 보고 발행자를 구현할 수
있도록 필드 단위까지 적었다. 받는 쪽(`pipe_comm`)은 이미 구현·검증돼 있다.

| | |
|---|---|
| 작성 | 2026-08-06 |
| 규약 단일 출처 | `src/dongmin/pipe_comm/pipe_comm/contract.py` |
| 받는 쪽 구현 | `src/dongmin/pipe_comm/` (colcon 빌드 통과, 자체시험 18/18) |
| 문서가 코드와 어긋나면 | **코드가 맞다.** `contract.py` 가 단일 출처다 |

---

## 0. 3줄 요약

1. **토픽만 쓴다. 커스텀 `.msg`/`.srv`/`.action` 을 만들지 않는다.** 경계는
   표준 메시지(`std_msgs`/`sensor_msgs`/`geometry_msgs`/`nav_msgs`)만.
2. 구조가 필요한 값은 **`std_msgs/String` 에 JSON**. 스키마는 §6.
3. `ROS_DOMAIN_ID=143`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, **영상 QoS 는
   BEST_EFFORT**. 셋 중 하나만 틀려도 **에러 없이 아무것도 안 온다.**

---

## 1. 시스템 구성

```
        Isaac Sim  (Python 3.11)                ROS 2 노드  (Python 3.10)
        ───────────────────────                 ────────────────────────
        map_test_demo.py                        pipe_comm/camera_monitor
        real_map_demo.py         ── DDS ──▶     pipe_comm/drive_monitor
        repair_demo.py           ◀── DDS ──     pipe_comm/mission_cli
          └ 로봇·카메라·물리                     dongyeon/pipe_vision_node (YOLO)
                                                son/condition·localization·driver
```

**🚨 PC 번호로 부르지 않는다.** 팀 관례가 `PC1 = 카메라·검출`,
`PC2 = Isaac Sim` 인데 이 규약 초안에서 정반대로 적었다가 걸렸다. 번호는 읽는
사람마다 뒤집히므로 이 문서는 **역할 이름**만 쓴다 — `Isaac 쪽` / `ROS 노드 쪽`.

**시연은 RTX 5080 MSI PC 한 대에서 양쪽을 같이 돌린다.** 같은 PC 안이라 DDS 가
루프백으로 통하고 멀티캐스트·NAT 문제가 없다. PC 를 갈라야 하면 §11 참조.

---

## 2. 환경 전제

### 2.1 Isaac 쪽 (Python 3.11)

**`source /opt/ros/humble/setup.bash` 를 하지 말 것.** 3.10 라이브러리가 앞에
잡혀 심볼이 충돌한다. Isaac Sim 5.1 은 3.11 용 rclpy 를 **내장**하고 있어서
따로 빌드할 게 없다.

```bash
isaac_ros        # LD_LIBRARY_PATH + PYTHONPATH 를 Isaac 번들로 잡는 별칭
export ROS_DOMAIN_ID=143
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
isaac_python map_test_demo.py
```

번들에 들어 있는 것 (실측 확인):

```
✅ rclpy   rclpy.action   sensor_msgs   geometry_msgs   nav_msgs
✅ std_msgs   std_srvs   diagnostic_msgs   vision_msgs   tf2_ros
❌ cv_bridge          ← 없다. numpy + cv2 로 직접 만든다 (§7 코드 참조)
❌ 우리 커스텀 인터페이스  ← 넣을 수 없다 (§3)
```

### 2.2 ROS 노드 쪽 (Python 3.10)

```bash
ros_set                       # /opt/ros/humble + cobot3_ws/install
export ROS_DOMAIN_ID=143
```

### 2.3 세 값이 양쪽에서 같아야 한다

| 값 | 값 | 틀리면 |
|---|---|---|
| `ROS_DOMAIN_ID` | **143** | `ros2 topic list` 에 아무것도 안 보인다 |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | 서로 발견하지 못한다 |
| 영상 QoS | `BEST_EFFORT` | **토픽은 보이는데 프레임이 0** |

셋 다 실패 시 **에러가 안 난다.** 조용히 아무 일도 안 일어난다.

---

## 3. 무엇을 쓰고 무엇을 안 쓰는가

| 수단 | 쓰는가 | 이유 |
|---|---|---|
| **토픽** | ✅ **전부** | 주력. 아래 §5 |
| 커스텀 `.msg` | ❌ **금지** | §3.1 |
| 커스텀 `.srv` / `.action` | ❌ **금지** | 같은 이유 |
| 표준 서비스 (`std_srvs`) | △ **선택 (2단계)** | §3.2 |
| 액션 | ❌ 안 씀 | §3.3 |
| TF (`tf2_ros`) | ❌ 1단계에서는 안 씀 | §3.4 |
| ROS 파라미터 | ✅ 노드 설정에만 | 통신 데이터로는 안 쓴다 |

### 3.1 🚨 커스텀 인터페이스를 만들면 안 되는 이유

커스텀 `.msg` 는 `rosidl` 이 만든 **Python 3.10 용 `.so`** 에 묶여 있다.
경계에 쓰면 Isaac 쪽(3.11)에서도 그 인터페이스를 빌드해야 하는데, Isaac 5.1
내장 rclpy 는 `isaacsim.ros2.bridge/humble` 에 **박제된 3.11 ABI** 라 우리
워크스페이스를 그대로 얹을 수 없다.

표준 메시지는 그 번들에 **이미 들어 있으므로 빌드가 0 이다.**

부수 효과가 둘 더 있다:
- **받는 쪽이 `pipe_comm` 을 빌드하지 않아도 된다.** `ros2 topic echo` 로 바로
  보이고, 팀원이 자기 노드를 붙일 때 우리 패키지에 의존하지 않는다.
- `ros2 bag record` 로 받은 것을 그대로 재생할 수 있다.

> `pipe_msgs` 는 레포에 없고 앞으로도 만들지 않는다. `son/condition/node.py`
> 와 `son/driver/node.py` 에 이미 JSON 폴백이 들어 있다.

### 3.2 서비스는 2단계 선택 사항

`std_srvs` 는 번들에 있으므로 **표준 서비스는 쓸 수 있다.** 다만 1단계에서는
안 쓴다 — Isaac 쪽 서비스 서버는 시뮬 루프 안에서 `spin_once()` 로 처리해야
하고, 콜백이 길면 **물리가 그동안 멈춘다.**

토픽 지령의 약점은 하나다: RELIABLE 이라도 **구독자가 없으면 그냥 사라지고,
보낸 쪽은 도착 여부를 모른다**(latch 가 아니다). 그래서 `mission_cli` 는 발행
뒤 구독자 수를 확인하고 0 이면 0 이 아닌 종료코드를 낸다 — 1단계는 이걸로
충분하다.

접수 확인이 꼭 필요해지면 표준 srv 로만 두 개를 붙인다:

| 서비스 | 타입 | 용도 |
|---|---|---|
| `~/estop` | `std_srvs/SetBool` | `data=true` 비상정지 / `false` 해제. 응답으로 접수 확인 |
| `~/status` | `std_srvs/Trigger` | 응답 `message` 에 `drive_state` JSON 을 그대로 담는다 |

`Trigger` 의 응답이 `success(bool) + message(string)` 이라 JSON 을 실을 수
있다. 문자열 인자를 받는 표준 srv 는 없으므로 **임무 지령은 서비스로 못
만든다** — 그래서 지령은 토픽이다.

### 3.3 액션을 안 쓰는 이유

`rclpy.action` 은 rclpy 안에 있으므로 **기술적으로는 가능하다**(별도 패키지가
아니다). 문제는 **쓸 만한 표준 액션 타입이 없다**는 것이다. 용접·복귀처럼
오래 걸리는 작업에 맞는 액션은 커스텀 `.action` 이 필요한데 §3.1 에 막힌다.

대신 **사건(`event`) 토픽으로 진행을 알린다** — `WELD_BEGIN` → `WELD_DONE` 이
액션의 feedback/result 역할을 한다.

### 3.4 TF 를 1단계에서 안 쓰는 이유

관 안에서 로봇의 절대 자세는 **모른다**(도면 매칭은 관리자 쪽 몫이다. 설계
v3 5.3 대비 변경점). 우리가 아는 것은 **관 중심선을 따라 잰 호길이 `s`** 하나
뿐이라 TF 트리로 표현할 게 사실상 없다. `odom` 하나로 충분하다.

---

## 4. 네임스페이스 규칙

**로봇 한 대 = 네임스페이스 하나.** 토픽 이름 앞에 붙인다.

| 시연 | 로봇 | 네임스페이스 |
|---|---|---|
| `map_test_demo.py` | 3대 (코스마다 하나) | `/elbow_v` `/elbow_h` `/tee` |
| `real_map_demo.py` | 1대 | `/robot` |
| `repair_demo.py` | 1대 | `/robot` |

```
/elbow_v/rgb/compressed
/elbow_v/drive_state
/tee/cmd_vel
```

🚨 **로봇이 여러 대인데 네임스페이스를 안 나누면 세 대의 상태가 한 토픽에
섞여 들어온다.** 받는 쪽은 어느 로봇 것인지 구분할 방법이 없다(JSON 의
`robot` 필드로 알 수는 있지만 QoS·주기가 뒤엉킨다).

---

## 5. 토픽 전체 목록

### 5.1 Isaac → ROS 노드 (발행)

| # | 토픽 (ns 상대) | 타입 | 주기 | QoS | 필수 |
|---|---|---|---|---|---|
| 1 | `rgb/compressed` | `sensor_msgs/CompressedImage` | 10Hz | BEST_EFFORT | ✅ |
| 2 | `depth/compressed` | `sensor_msgs/CompressedImage` | 10Hz | BEST_EFFORT | ✅ |
| 3 | `camera_info` | `sensor_msgs/CameraInfo` | 10Hz | BEST_EFFORT | ✅ |
| 4 | `rear/rgb/compressed` | `sensor_msgs/CompressedImage` | 10Hz | BEST_EFFORT | ○ |
| 5 | `rear/camera_info` | `sensor_msgs/CameraInfo` | 10Hz | BEST_EFFORT | ○ |
| 6 | `depth` | `sensor_msgs/Image` (32FC1) | 10Hz | BEST_EFFORT | ○ |
| 7 | `odom` | `nav_msgs/Odometry` | 10Hz | BEST_EFFORT | ✅ |
| 8 | `imu` | `sensor_msgs/Imu` | 50Hz | BEST_EFFORT | ✅ |
| 9 | `joint_states` | `sensor_msgs/JointState` | 10Hz | BEST_EFFORT | ○ |
| 10 | `moving` | `std_msgs/Bool` | 전환 시 | RELIABLE | ✅ |
| 11 | `drive_state` | `std_msgs/String` (JSON) | 10Hz | RELIABLE | ✅ |
| 12 | `event` | `std_msgs/String` (JSON) | 사건 시 | RELIABLE | ✅ |
| 12a | `course` | `std_msgs/String` (JSON) | **기동 1회** | latched | ○ |
| 12b | `mesh` | `std_msgs/UInt8MultiArray` | **기동 1회** | latched | ○ |

✅ = 1단계 필수 / ○ = 선택

🔑 `course`·`mesh` 는 **맵 기하**다. 상태가 아니라 **선언**이라 latched
(TRANSIENT_LOCAL) 로 기동 때 한 번만 낸다 — 받는 쪽이 늦게 떠도 마지막 값을
받는다. 🚨 latched 는 **양쪽이 같이 써야** 한다(구독만 TRANSIENT_LOCAL 이고
발행이 VOLATILE 이면 못 받는다).

`mesh` 는 맵 CAD 를 브라우저용으로 구운 `.webmesh` 바이트다(0.77MB 실측).
**Isaac PC 와 웹 PC 가 다를 때를 위한 경로**다 — 같은 PC 면 파일을 직접 읽으면
되지만 갈라지면 파일이 안 보인다. 굽는 것도 Isaac 이 한다: 맵 z 오프셋을 아는
것이 거기뿐이라(floor2 +250 / floor1 +2740.23), 받는 쪽이 파라미터로 다시
말해 주는 구조는 **층을 바꾸는 순간 2.49m 어긋난다.**

### 5.2 ROS 노드 → Isaac (구독)

| # | 토픽 (ns 상대) | 타입 | QoS | 필수 |
|---|---|---|---|---|
| 13 | `cmd_vel` | `geometry_msgs/Twist` | RELIABLE | ○ |
| 14 | `mission` | `std_msgs/String` (JSON) | RELIABLE | ✅ |
| 15 | `repair_target` | `std_msgs/String` (JSON) | RELIABLE | ○ |

> `cmd_vel` 은 **HANDOFF.md 가 "아직 안 이어진 것" 으로 남겨 둔 부분**이다.
> 지금 시연 스크립트는 자체적으로 속도를 준다. 1단계는 `mission` 만 받아도
> 되고(START/STOP/RECALL), `cmd_vel` 은 주행 제어를 ROS 쪽으로 넘길 때 붙인다.

### 5.3 QoS 프로파일 — 정확한 값

```python
# 영상·IMU 등 흘려보내는 신호
QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
           history=HistoryPolicy.KEEP_LAST, depth=5)

# 지령·상태·사건 — 놓치면 안 되는 것
QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
           history=HistoryPolicy.KEEP_LAST, depth=10)
```

`contract.sensor_qos()` / `contract.command_qos()` 를 쓰면 이 실수를 할 수 없다.

🚨 **발행자가 BEST_EFFORT 인데 구독자가 RELIABLE 이면 호환되지 않아 연결
자체가 안 맺어진다.** 토픽 목록에는 보이고 프레임만 0 이다. 반대(발행
RELIABLE / 구독 BEST_EFFORT)는 된다.

---

## 6. 메시지 필드 규약

**이 절이 이 문서의 핵심이다.** 타입만 맞고 필드 의미가 다르면 에러 없이
결과만 틀린다.

### 6.1 `rgb/compressed` — `sensor_msgs/CompressedImage`

| 필드 | 값 |
|---|---|
| `header.stamp` | 그 프레임을 읽은 시각 |
| `header.frame_id` | `"front_camera"` (후방은 `"rear_camera"`) |
| `format` | `"jpeg"` |
| `data` | JPEG 바이트 |

🚨 **Isaac annotator 의 `rgb` 는 RGBA(HxWx4) 다.** 알파를 안 떼고 cv2 에
넘기면 채널 수가 안 맞는다.
🚨 **cv2 는 BGR 순서를 기대한다.** 안 뒤집으면 R/B 가 바뀐 채로 YOLO 에
들어간다 — 에러가 안 나고 **검출률만 떨어져** 원인을 못 찾는다.

### 6.2 `depth/compressed` — `sensor_msgs/CompressedImage`

| 필드 | 값 |
|---|---|
| `header.frame_id` | `"front_camera"` |
| `format` | `"png"` |
| `data` | **16UC1 PNG. 단위 mm. 0 = 무효** |

**JPEG 는 절대 쓰지 않는다** — 손실 압축이 깊이값을 훼손한다.

🚨 **무효 픽셀은 uint16 캐스팅 *전에* 0 으로 눌러야 한다.** Isaac 의
`distance_to_camera` 는 빈 공간을 `inf` / `NaN` / 최대거리 중 무엇으로도
돌려준다(무엇인지는 `camera/depth_probe.py` 가 가린다 — 실측값은
`empty_is_zero`). 그냥 캐스팅하면 쓰레기 정수가 되어 **관 단절이 "가까운 벽"
으로 둔갑하고 판정이 조용히 뒤집힌다.**

🚨 **65.535m 를 넘으면 wrap 된다** — 먼 곳이 코앞으로 보인다. 상한 초과도
0(무효)으로 누른다. 관 안에서는 안 닿지만 **관 단절로 시야가 뻥 뚫리면 닿는다.**

### 6.3 `camera_info` — `sensor_msgs/CameraInfo`

| 필드 | 값 |
|---|---|
| `width` / `height` | 영상 해상도 |
| `k` | `[f, 0, cx, 0, f, cy, 0, 0, 1]` |
| `p` | `[f, 0, cx, 0, 0, f, cy, 0, 0, 0, 1, 0]` |
| `distortion_model` | `"equidistant"` |
| `d` | `[0, 0, 0, 0]` |

🔑 **어안이라 `K` 는 엄밀한 핀홀 행렬이 아니다.** `K[0]` 에 **등거리 어안의
f (r = f·θ)** 를 싣는 규약이다. 받는 쪽이 이 값으로 입사각을 역산한다.

```python
F_PX = (width / 2.0) / math.radians(HFOV_deg / 2.0)   # HFOV 140° 기준
```

`fisheyePolynomial` 은 Isaac Sim 5.0 부터 폐기됐다. `set_opencv_fisheye_properties()`
를 쓰고, 실패하면 핀홀로 떨어지므로 그 경우 `distortion_model` 을 `"plumb_bob"`
으로 바꾸고 알려 줄 것.

### 6.4 `odom` — `nav_msgs/Odometry`

| 필드 | 값 |
|---|---|
| `header.frame_id` | `"pipe"` |
| `child_frame_id` | `"base_link"` |
| **`pose.pose.position.x`** | **관 중심선을 따라 잰 호길이 `s` (m)** |
| `pose.pose.position.y/z` | 0 (안 쓴다) |
| `pose.pose.orientation` | 롤만 의미 있음 (§6.5 와 같은 값) |
| `twist.twist.linear.x` | 진행 속도 (m/s). **후진이면 음수** |

🚨 **`position.x` 는 직선거리가 아니라 호길이다.** 직선거리를 쓰면 곡관에서
**위치가 뒤로 가는 것처럼 보인다.** `map_test_demo.py` 의 `Centerline.nearest()`
가 돌려주는 `s` 가 바로 이 값이다.

### 6.5 `imu` — `sensor_msgs/Imu`

| 필드 | 값 |
|---|---|
| `header.frame_id` | `"base_link"` |
| `orientation` | 롤을 담은 쿼터니언 (X 축 회전) |
| `linear_acceleration` | Isaac IMU 의 `lin_acc` 그대로 |
| `angular_velocity` | Isaac IMU 의 `ang_vel` 그대로 |

🚨🚨 **이 시스템에서 가장 조용히 망가지는 자리다.**

롤이 곧 **결함의 시계각 기준**이고, 그걸로 **토치 링 J1 을 어디로 돌릴지**
정한다. 부호가 뒤집히면 **정반대를 용접한다. 에러는 안 난다.**

**Isaac 의 `lin_acc` 는 고유가속도 규약이라 정지 상태에서 위로 +9.81 을 읽는다
— 중력 벡터가 아니다.** 롤 θ 로 굴리면 `lin_acc = (0, g·sinθ, g·cosθ)` 이므로:

```python
roll_deg = math.degrees(math.atan2(gy, gz))     # ✅ 맞다
roll_deg = math.degrees(math.atan2(gy, -gz))    # ❌ 정립에서 180° 를 보고한다
```

2026-08-04 실측 (월드 고정 조인트로 정지시켜 측정):

| 적용 롤 | `lin_acc` | `atan2(gy,-gz)` 옛 | `atan2(gy,gz)` 지금 |
|---|---|---|---|
| 0° | (0, 0.000, +9.810) | +180.00° | **0.00°** |
| +30° | (0, +4.905, +8.496) | +150.00° | **+30.00°** |
| −45° | (0, −6.937, +6.937) | −135.00° | **−45.00°** |
| +90° | (0, +9.810, 0.000) | +90.00° | **+90.00°** |

기준 세그먼트는 **후방으로 통일**한다(설계 5.1). 로봇이 어떻게 투입되었든
중력은 변하지 않으므로 정찰기와 수리기가 같은 롤 기준을 공유한다.

🔑 **IMU 는 `world.reset()` *전에* 만들어야 갱신된다.** reset 뒤에 붙이면
`physics_step` 이 0 에 머물고 `lin_acc` 가 `[0,0,0]` 이라 **롤이 항상 0 으로
나간다**(2026-08-04 실측: reset 뒤 부착 0 스텝 / reset 앞 부착 52 스텝).

### 6.6 `joint_states` — `sensor_msgs/JointState`

| 필드 | 값 |
|---|---|
| `name` | 조인트 이름 (휠 12 + 피스톤 6) |
| `position` | 회전각(rad) / 피스톤 스트로크(m) |
| `velocity` | 각속도(rad/s) |

휠 각속도 × 반경 = 접지 속도. 개별 값의 편차로 슬립을 본다(설계 5.2).
피스톤 스트로크는 관경 변화·이물 감지의 보조 신호다.

### 6.7 `moving` — `std_msgs/Bool`

**주행/정지 알림.** `true` = 주행 중.

🔑 **전환될 때만 발행한다**(+ 10초쯤마다 재확인용 1회). 매 스텝 발행하면
로그가 이것만으로 찬다. 판정 기준은 `drive_state.moving` 과 **반드시 같아야
한다** — `contract.MOVING_STATES` 한 곳에서 정한다.

---

## 7. JSON 스키마 (`std_msgs/String`)

### 7.1 `drive_state` — 10Hz

```jsonc
{
  "stamp": 1786010899.551,   // float, 초
  "robot": "elbow_v",        // 네임스페이스와 같은 값
  "step": 2963,              // 물리 스텝 수
  "state": "RUN",            // §7.4 목록 중 하나
  "moving": true,
  "dir": -1,                 // +1 전진 / -1 후진
  "speed_mps": 0.1,          // 지령 속도. 항상 양수 (방향은 dir 로)
  "s_mm": 431.2,             // 중심선 호길이
  "s_total_mm": 935.6,       // 코스 전체 길이
  "off_mm": 3.44,            // 중심선 이탈. 50mm(관 내반경) 넘으면 이탈
  "lap": 2,                  // 왕복 횟수
  "stuck": 1,                // 누적 끼임 횟수
  "reason": ""               // 정지·이탈 사유 (사람이 읽는 문자열)
}
```

### 7.2 `event` — 사건이 일어난 그 순간에만 1회

```jsonc
{
  "stamp": 1786010899.551,
  "robot": "elbow_v",
  "event": "STUCK",              // §7.5 목록 중 하나
  "s_mm": 431.2,
  "detail": "방향 전환 → 후진",   // 한글 가능 (ensure_ascii=False)
  "alert": true                  // 사람이 바로 봐야 하는가
}
```

🔑 **상태 스트림으로 사건을 유추하지 말 것.** 10Hz 표본 사이에 일어난 전환은
통째로 사라진다 — 끼였다가 즉시 빠져나온 경우가 그렇다. 그래서 사건은
RELIABLE 로 따로 보낸다.

### 7.3 `mission` (수신) / `repair_target` (수신)

```jsonc
// mission
{"stamp":1786010899.551,"cmd":"SPEED","reason":"곡관 진입","mps":0.05}

// repair_target — clock_deg 0°=천장, 180°=바닥(중력 방향)
{"stamp":1786010899.551,"id":"crack_0001","s_mm":1130.0,"clock_deg":180.0,
 "width_mm":2.2,"depth_mm":2.2,"confidence":0.87}
```

### 7.4 `state` 값 목록

| 값 | 뜻 | `moving` |
|---|---|---|
| `SETTLE` | 투입 직후 피스톤이 관벽을 잡는 과도 구간 | false |
| `RUN` | 주행 중 | **true** |
| `HOLD` | 지령으로 정지(멀쩡함). 재개 가능 | false |
| `STUCK` | 끼임 감지 — 방향 전환 재시도 중 | **true** |
| `INSPECT` | 결함 앞 정지, 카메라 판정 중 | false |
| `REPAIR` | 용접 중 | false |
| `RETURN` | 복귀 주행 | **true** |
| `DONE` | 임무 완료 | false |
| `DEAD` | 코스 이탈·복구 불가. 그 로봇만 멈춘다 | false |

### 7.5 `event` 값 목록

| 값 | 언제 | `alert` |
|---|---|---|
| `START` | 안착 끝 → 주행 시작 | |
| `ARRIVE` | 코스 끝 도달 | |
| `HOME` | 출발점 복귀 | |
| `STUCK` | 끼임 → 방향 전환 | **⚠** |
| `OFF_COURSE` | 관 밖으로 이탈 (복구 안 함) | **⚠** |
| `DISCONNECT` | 관 단절 감지 | **⚠** |
| `BRANCH` | T 분기 감지 | |
| `DEFECT` | 결함 발견 | |
| `WELD_BEGIN` / `WELD_DONE` | 용접 시작/완료 | |
| `ESTOP` | 비상 정지 | **⚠** |
| `DONE` | 임무 완료 | |

### 7.6 `mission.cmd` 값 목록

| 값 | 뜻 |
|---|---|
| `START` | 주행 시작 / `HOLD` 에서 재개 |
| `STOP` | 정지(`HOLD`). 재개 가능 |
| `RECALL` | 복귀 — 방향을 뒤집어 출발점으로 |
| `FORWARD` | 복귀 취소 — 다시 **전진**하며 점검한다 (`RECALL` 의 짝) |
| `RETRY` | 끼임 탈출 재시도 (방향 전환) |
| `SPEED` | 주행 속도 변경. `mps` 를 같이 준다 |
| `ESTOP` | 비상 정지. **재개 불가** — 사람이 풀어야 한다 |

🔑 `START` 는 **방향을 안 바꾼다** — 복귀 중에 `START` 를 눌러도 계속 뒤로
간다. 앞으로 돌려세우려면 `FORWARD` 다(그래서 지령을 따로 뒀다).

🔑 **문자열을 손으로 적지 말고 `contract.py` 의 상수를 쓸 것.** 모르는 값을
주면 `ValueError` 로 즉시 걸리게 해 뒀다. 손으로 적으면 오타가 **침묵**이다.

---

## 8. Isaac 쪽 구현 — 최소 코드

### 8.1 규약 파일 가져오기 (설치 불필요)

`contract.py` 는 **순수 stdlib** 라 3.11 에서 그대로 import 된다.

🚨 **절대경로를 박지 말 것.** 워크스페이스 위치는 PC 마다 다르다.

```python
import os
import sys
from pathlib import Path

# 예: <ws>/src/son/map_test_demo.py 에서 부를 때
WS = Path(__file__).resolve().parents[2]          # src/ 의 부모 = 워크스페이스
sys.path.insert(0, str(WS / "src" / "dongmin" / "pipe_comm"))
# 레이아웃이 다르면 환경변수로:  PIPE_COMM_DIR=/경로/pipe_comm
if "PIPE_COMM_DIR" in os.environ:
    sys.path.insert(0, os.environ["PIPE_COMM_DIR"])

from pipe_comm import contract
from pipe_comm.contract import Topics
from pipe_comm import image_codec as codec       # numpy + cv2 만 필요
```

토픽 이름을 문자열로 적지 말 것:

```python
t = Topics("elbow_v")
t.rgb          # '/elbow_v/rgb/compressed'
t.drive_state  # '/elbow_v/drive_state'
t.mission      # '/elbow_v/mission'
```

### 8.2 카메라 annotator 붙이기

🚨 **`world.reset()` *뒤에* 붙여야 한다.** annotator 와 render product 는
런타임 자원이라 reset 전에 붙이면 살아남지 않는다.

```python
import omni.replicator.core as rep

rp = cam.get_render_product_path()
rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_annot.attach(rp)
depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
depth_annot.attach(rp)
```

🔑 **`distance_to_image_plane` 이 아니라 `distance_to_camera` 다.** 관 내부는
방사형이라 광축 투영 거리가 아니라 실제 거리가 필요하다.

### 8.3 발행

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Bool, String

rclpy.init()
node = Node("isaac_bridge")
t = Topics("elbow_v")
sq, cq = contract.sensor_qos(), contract.command_qos()

pub_rgb   = node.create_publisher(CompressedImage, t.rgb, sq)
pub_depth = node.create_publisher(CompressedImage, t.depth, sq)
pub_state = node.create_publisher(String, t.drive_state, cq)
pub_event = node.create_publisher(String, t.event, cq)
pub_moving = node.create_publisher(Bool, t.moving, cq)

# ── 시뮬 루프 안에서, 10Hz 로 ──────────────────────────────
stamp = node.get_clock().now().to_msg()

rgb = rgb_annot.get_data()
if rgb is not None and getattr(rgb, "size", 0):
    m = CompressedImage()
    m.header.stamp, m.header.frame_id = stamp, contract.FRAME_FRONT_CAM
    m.format = "jpeg"
    m.data = codec.rgb_to_jpeg(rgb)          # RGBA 처리·BGR 뒤집기 포함
    pub_rgb.publish(m)

d = depth_annot.get_data()
if d is not None and getattr(d, "size", 0):
    m = CompressedImage()
    m.header.stamp, m.header.frame_id = stamp, contract.FRAME_FRONT_CAM
    m.format = "png"
    m.data, valid_ratio = codec.depth_to_png16(d)   # 무효·오버플로 처리 포함
    pub_depth.publish(m)
    if valid_ratio < 0.05:
        print("[경고] depth 유효 픽셀 5% 미만 — near clip / 카메라 매몰 확인")

pub_state.publish(String(data=contract.dumps(contract.drive_state(
    "elbow_v", contract.STATE_RUN, direction=r["dir"],
    speed_mps=TARGET_SPEED_MPS, s_mm=s_now * 1000, s_total_mm=cl.total * 1000,
    off_mm=off_now * 1000, lap=r["lap"], stuck=r["stuck"], step=step))))

# 사건은 그 순간에만 1회
pub_event.publish(String(data=contract.dumps(contract.event(
    "elbow_v", contract.EV_STUCK, s_mm=s_now * 1000, detail="방향 전환 → 후진"))))

# 지령을 받으려면 매 스텝 (blocking 0 으로)
rclpy.spin_once(node, timeout_sec=0.0)
```

🚨 **`rclpy.spin_once(timeout_sec=0.0)` 을 시뮬 루프에서 매 스텝 부를 것.**
안 부르면 `mission` 을 아예 못 받는다. 0 이 아닌 timeout 을 주면 그만큼
**물리가 멈춘다.**

### 8.4 `map_test_demo.py` 의 어느 값을 쓰는가

이미 FSM 에 다 있다. 새로 계산할 게 없다.

| JSON 필드 | 소스 (`map_test_demo.py`) |
|---|---|
| `state` | `r["state"]` — `SETTLE`/`RUN` (이탈 시 `DEAD`) |
| `dir` | `r["dir"]` |
| `s_mm` | `s_now * 1000` (`r["cl"].nearest(p1)` 의 첫 값) |
| `s_total_mm` | `r["cl"].total * 1000` |
| `off_mm` | `off_now * 1000` |
| `lap` / `stuck` | `r["lap"]` / `r["stuck"]` |
| `step` | `step` |
| 사건 `STUCK` | 끼임 분기 (`map_test_demo.py:744-752`) |
| 사건 `OFF_COURSE` | 이탈 분기 (`map_test_demo.py:733-738`) |
| 사건 `ARRIVE`/`HOME` | 방향 전환 분기 (`map_test_demo.py:755-766`) |

카메라는 프림까지는 이미 만들어져 있고(`map_test_demo.py:611-650`),
**annotator 만 안 붙어 있다.**

---

## 9. 조용히 망가지는 자리 모음

| 항목 | 규약 | 안 지키면 |
|---|---|---|
| `ROS_DOMAIN_ID` | 143 | 토픽이 하나도 안 보인다 |
| 영상 QoS | BEST_EFFORT | 토픽은 보이는데 프레임 0 |
| Depth 압축 | 16UC1 PNG, mm | JPEG 는 깊이를 훼손 |
| Depth 무효 | 캐스팅 **전에** 0 | 관 단절이 "가까운 벽"이 된다 |
| Depth 상한 | 65m 초과 무효 | uint16 wrap — 먼 곳이 코앞으로 |
| RGB 채널 | cv2 에 BGR | R/B 바뀐 채 YOLO 로 (검출률만 하락) |
| RGB 알파 | RGBA → 3채널 | annotator 는 RGBA 를 준다 |
| `odom.x` | **호길이** | 곡관에서 위치가 뒤로 간다 |
| `K[0]` | 어안 f (r=f·θ) | 입사각 역산이 틀어진다 |
| **IMU 롤 부호** | `atan2(gy, gz)` | **토치를 정반대로 돌린다** |
| IMU 생성 시점 | `reset()` **전** | 롤이 항상 0 |
| annotator 부착 | `reset()` **후** | 영상이 안 나온다 |
| `spin_once` | 매 스텝, timeout 0 | 지령을 못 받는다 / 물리가 멈춘다 |
| 네임스페이스 | 로봇마다 하나 | 세 대 상태가 섞인다 |

---

## 10. 검증 절차

### 10.1 받는 쪽 도구 (이미 구현돼 있다)

```bash
ros_set && export ROS_DOMAIN_ID=143

# 토픽이 보이는가
ros2 topic list | grep elbow_v

# 카메라가 오는가 — 해상도·평균밝기·Depth 유효율을 1초마다 찍는다
ros2 run pipe_comm camera_monitor --ros-args -p ns:=elbow_v

# 주행/정지 알림 — 로봇 3대를 한 노드에서
ros2 run pipe_comm drive_monitor --ros-args -p ns:="[elbow_v,elbow_h,tee]"

# 둘 다 한 번에
ros2 launch pipe_comm monitor.launch.py ns:=all

# 지령 쏘기 (구독자 0 이면 종료코드 1)
ros2 run pipe_comm mission_cli -- STOP --ns elbow_v
```

`drive_monitor` 출력 예 (실제 확인된 형식):

```
[INFO] [drive_monitor]:   elbow_v    주행  RUN   s  431mm ←  46%  이탈 3.4mm  왕복 2  끼임 1
[WARN] [drive_monitor]: [elbow_v] ▶ STUCK  s=431mm  방향 전환 → 후진
```

### 10.2 순서

1. Isaac 을 띄우고 `ros2 topic list` 에 `/elbow_v/...` 가 보이는지
2. `camera_monitor` 로 **프레임 Hz 가 0 이 아닌지** — 0 이면 §9 표
3. 평균 밝기가 5/255 넘는지 — 관 내부는 로봇 조명이 유일한 광원이다
4. Depth 유효율이 20% 넘는지 — 낮으면 near clip 이 관벽(25mm)을 자르는 것
5. `drive_monitor` 로 s 가 늘어나는지, 끼임/이탈 사건이 찍히는지
6. `mission_cli -- STOP` 이 실제로 로봇을 세우는지

---

## 11. PC 를 갈라야 할 때

시연은 한 PC 라 필요 없지만, 나눠야 하면:

1. 양쪽 `ROS_DOMAIN_ID=143`
2. 양쪽 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
3. **같은 랜이면 멀티캐스트로 자동 발견된다** — 별도 설정 불필요
4. **EC2 ↔ 외부 PC 는 화이트리스트로 못 푼다.** 상대가 사설망 NAT 뒤라 EC2
   에서 그 주소로 가는 경로가 없다 — VPN(Tailscale) 이나 Fast DDS Discovery
   Server 가 필요하다
5. WAN 을 건널 때 대역폭 (640×360, 10Hz 거친 어림):

   | 토픽 | 대역 | WAN |
   |---|---|---|
   | `rgb/compressed` JPEG | ≈ 2 Mbps | ✅ |
   | `depth/compressed` PNG16 | ≈ 10 Mbps | △ |
   | `depth` 32FC1 (1280×720) | ≈ **300 Mbps** | ❌ |

   → `depth` 원본(32FC1)은 선택 항목이다. WAN 에서는 끄고 압축만 쓴다.

---

## 12. 체크리스트 (Isaac 쪽 구현자용)

- [ ] `isaac_ros` 로 3.11 rclpy 경로를 잡았다 (`source /opt/ros/humble/...` 안 함)
- [ ] `ROS_DOMAIN_ID=143`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- [ ] `contract.py` 를 `sys.path` 로 읽어 토픽 이름을 상수로 쓴다
- [ ] 로봇마다 네임스페이스를 나눴다
- [ ] IMU 를 `world.reset()` **전에** 만들었다
- [ ] annotator 를 `world.reset()` **후에** 붙였다 (`distance_to_camera`)
- [ ] 영상은 `sensor_qos()`, 상태·사건·지령은 `command_qos()`
- [ ] RGB 는 `codec.rgb_to_jpeg()`, Depth 는 `codec.depth_to_png16()` 을 쓴다
- [ ] `odom.pose.position.x` 에 **호길이**를 넣었다
- [ ] IMU 롤을 `atan2(gy, gz)` 로 구했다 (정립 상태에서 0° 가 나오는지 확인)
- [ ] `CameraInfo.k[0]` 에 어안 f 를 넣고 `distortion_model="equidistant"`
- [ ] 사건은 그 순간 1회만 발행한다
- [ ] `moving` 은 전환될 때만 발행한다
- [ ] 시뮬 루프에서 `rclpy.spin_once(timeout_sec=0.0)` 을 매 스텝 부른다
- [ ] `camera_monitor` / `drive_monitor` 로 끝까지 확인했다

---

## 13. 참고 파일

| 파일 | 내용 |
|---|---|
| `src/dongmin/pipe_comm/pipe_comm/contract.py` | **규약 단일 출처.** 토픽·스키마·QoS |
| `src/dongmin/pipe_comm/pipe_comm/image_codec.py` | 영상 인코딩/디코딩 (cv_bridge 없이) |
| `src/dongmin/pipe_comm/README.md` | 받는 쪽 사용법 |
| `src/son/camera/rig.py` | 1세대 카메라 발행 구현 — annotator 패턴 참고 |
| `src/son/robot/state_bridge.py` | 1세대 상태 발행 구현 — IMU 롤 유도 참고 |
| `src/son/HANDOFF.md` | Isaac 함정 모음 |
| `docs/SETUP_EC2.md` | 환경 구축·검증 이력. **`.gitignore` 의 `docs/` 때문에 git 으로 안 따라간다** — 필요하면 따로 받을 것 |

🚨 `rig.py` 와 `state_bridge.py` 는 **1세대(6륜) 로봇 전용**이다. DOF 이름
(`joint_wheel_*`/`joint_arm_*`/`joint_waist`)과 링크 이름(`body_front`/
`body_rear`)이 v2 로봇(`FrontBody`/`RearBody`, `DriveJoints` 스코프)과 달라
**그대로 쓰면 매칭이 0 개가 되고 에러 없이 빈 값만 나간다.** 패턴만 참고하고
v2 구조에 맞게 다시 고를 것.
