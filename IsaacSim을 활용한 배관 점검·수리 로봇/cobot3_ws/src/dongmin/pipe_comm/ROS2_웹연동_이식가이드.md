# 웹 관제 + ROS 2 통신을 새 주행 코드에 붙이기

**받는 사람**: 전체 로봇 주행 코드를 쓰는 사람
**주는 것**: Isaac 시연 → ROS 2 → 브라우저 관제 화면을 잇는 **접붙임 지점 5곳**

지금 `src/dongyeon/integration_test/real_map_demo.py` 에 붙어 있는 것을 그대로
옮기면 된다. 이 문서만 보고 따라 할 수 있게 적었다.

> 규약(토픽 이름·JSON 필드) 자체의 상세는 옆의 **`ROS2_통신규격.md`** 가 정본이다.
> 이 문서는 "**어디에 무슨 줄을 넣나**" 만 다룬다.

---

## 0. 5분 요약

```
 [Isaac Sim · python 3.11]              [ROS 2 · python 3.10]        [브라우저]
 ┌───────────────────────┐   토픽    ┌──────────────────┐   WS    ┌──────────┐
 │ 당신의 주행 코드       │ ────────▶ │ pipe_comm/       │ ──────▶ │ 관제 화면 │
 │   + ros_bridge.py     │           │   web_panel      │         │  :8080   │
 │       (발행자)         │ ◀──────── │  (FastAPI)       │ ◀────── │  버튼    │
 └───────────────────────┘  mission  └──────────────────┘   POST  └──────────┘
```

- **당신이 건드릴 파일은 당신 주행 코드 하나뿐이다.** `ros_bridge.py`,
  `contract.py`, `web_panel.py`, `web/` 는 **그대로 쓴다**(고칠 일 없음).
- 붙이는 자리는 **5곳**: 기동 / 매 스텝 / 10Hz / 상태 전이 / 종료.
- 실제로 추가되는 줄은 **50줄 남짓**이다.

### 파일 셋만 알면 된다

| 파일 | 무엇 | 당신이 하는 일 |
|---|---|---|
| `src/dongmin/pipe_comm/pipe_comm/contract.py` | **규약 단일 출처.** 토픽 이름·JSON 스키마·상태/사건/지령 상수 | `import` 만. 문자열을 손으로 적지 말 것 |
| `src/dongmin/isaac_bridge/ros_bridge.py` | Isaac 쪽 **발행자**(python 3.11). 위 규약대로 토픽을 낸다 | `import` 해서 5곳에서 호출 |
| `src/dongmin/pipe_comm/pipe_comm/web_panel.py` | 받아서 웹으로 뿌리는 **관제 서버** | 그냥 실행만 |

🚨 **토픽 이름을 문자열로 직접 적지 말 것.** 오타는 에러가 아니라 **침묵**이다 —
발행은 성공하고 아무도 못 받는다. 이 판에서 진단이 제일 오래 걸리는 부류다.
`contract.Topics` 에 없는 이름을 물으면 `AttributeError` 로 즉시 걸리게 해 뒀다.

---

## 1. 접붙임 ① — 기동 (world.reset() **뒤**)

```python
# 파일 맨 위 (SimulationApp 만들기 전에 argv 만 읽는다)
ROS = "--ros" in sys.argv
ROS_NS = os.environ.get("ROS_NS", "robot")      # 토픽 앞에 붙는 이름
```

```python
# ── 카메라·annotator 를 다 만들고 world.reset() 을 지난 자리 ──
bridge = None
if ROS:
    # ros_bridge.py 가 있는 곳. 워크스페이스 루트에서 src/dongmin/isaac_bridge
    sys.path.append(str(WS / "src" / "dongmin" / "isaac_bridge"))
    import ros_bridge
    if not ros_bridge.available():
        raise SystemExit("[중단] --ros 인데 rclpy/규약을 못 쓴다 — `isaac_ros` 먼저")
    from pipe_comm import contract

    bridge = ros_bridge.Bridge([ROS_NS])
    rp = bridge.robot(ROS_NS)

    # (1) 카메라 — **이미 붙어 있는 annotator 를 그대로 넘긴다**
    #     역할은 "front" / "rear" / "torch" 셋뿐. 없으면 안 넘기면 된다.
    for name, ann_rgb, ann_depth in rigs:          # ← 당신 코드의 카메라 목록
        role = {"front_camera": "front", "back_camera": "rear",
                "torch_camera": "torch"}.get(name)
        if role:
            rp.use_annotators(ann_rgb, ann_depth, CAM_W, CAM_H, F_PX, role)

    # (2) 코스 기하 — latched 로 **한 번만**. 웹 3D 맵이 이걸로 관을 그린다.
    #     [[호길이 s, x, y, z], ...] 전부 m. s 오름차순, 간격 불균일 허용.
    pts = [[float(s), *map(float, p)] for s, p in 중심선표본]   # 20mm 간격이면 충분
    rp.publish_course(pts, ir_m=PIPE_IR, bend_r_m=BEND_R)

    # (3) CAD 메시 — 맵 USD 를 브라우저용으로 굽고 latched 로 넘긴다.
    #     🚨 **웹 PC 가 다른 자리에 있어서** 파일로는 못 넘긴다. 0.77MB 1회.
    #     z 오프셋은 활성 층 수평망을 월드 z=0 으로 올리는 값(= -Z_NET):
    #     floor2 +250 / floor1 +2740.23. **이걸 아는 것은 여기뿐이다** —
    #     받는 쪽에 파라미터로 맡기면 층 바꾸는 순간 2.49m 어긋난다.
    rp.publish_mesh(Path(MAP_USD).with_suffix(".webmesh"),
                    usd=MAP_USD, z_shift_mm=-Z_NET)   # 없거나 낡았으면 굽는다

    ROS_EVERY = max(1, int(PHYSICS_HZ / 10))       # 10Hz 로 낼 주기(스텝 수)
    ros_prev_state = None
```

🚨 **`world.reset()` 뒤라야 한다.** annotator 는 런타임 자원이라 reset 전에
붙이면 살아남지 않는다(규격서 §8.2). `use_annotators()` 는 **이미 붙어 있는
것을 재사용**한다 — 같은 render product 에 또 붙이면 버퍼를 두 번 읽어 낭비이고,
무엇보다 당신의 검출 코드와 **다른 프레임을 볼 수 있다.** 발행하는 그림과
판정하는 그림이 다르면 로그를 대조할 수 없다.

🔑 **코스를 안 내면 웹 3D 맵이 "코스 수신 대기" 로 멈춘다.** 맵 기하의 단일
출처는 당신 코드의 중심선이다 — 웹에는 좌표가 하나도 없다.

🔑 메시(`publish_mesh`)는 **덤이다.** 안 내면 3D 맵이 관 튜브만 그린다(건물이
안 보일 뿐 나머지는 다 돈다). 굽는 데 0.5초, 발행 1ms 이고 낡았을 때만 다시
굽는다. `pxr`(USD)이 없으면 경고만 찍고 넘어간다.

---

## 2. 접붙임 ② — 매 스텝 `bridge.spin()`

```python
while True:
    world.step(render=...)
    ...
    if bridge:
        bridge.spin()          # 🚨 매 스텝. 안 부르면 지령을 아예 못 받는다
```

`spin_once(timeout_sec=0.0)` 라 물리를 안 멈춘다. **timeout 을 0 이 아닌 값으로
주면 그만큼 물리가 멈춘다** — 0 을 유지할 것.

---

## 3. 접붙임 ③ — 10Hz 상태·영상

```python
    if bridge:
        if step % ROS_EVERY == 0:
            rp = bridge.robot(ROS_NS)

            # (1) 지령 먼저 꺼낸다 — §5 참고. 그래야 방금 누른 버튼이
            #     아래 publish_state 에 바로 반영된다.
            while (cmd := rp.pop_mission()) is not None:
                apply_mission(cmd)

            # (2) 상태 — 아래 인자 이름은 규약이 정한 것이다
            rp.publish_state(
                state=STATE_MAP.get(내부상태, contract.STATE_RUN),
                direction=-1 if 후진중 else 1,
                speed_mps=목표속도,           # 항상 양수. 방향은 direction 으로
                s_mm=s * 1000,                # 🚨 중심선 **호길이** (§단위 규약)
                s_total_mm=코스길이 * 1000,
                off_mm=중심선이탈 * 1000,
                lap=0, stuck=끼임횟수, step=step,
                roll_deg=롤,                  # 0=천장, 180=바닥
                reason=내부상태이름,           # 사람이 읽는 한 줄
                art=art, wheel_idx=wheel_idx, # 관절 상태(선택)
                pos=월드좌표_m,               # 🔑 3D 맵이 이걸로 점을 찍는다
                cam=현재카메라역할)            # "front"/"rear"/"torch"

            # (3) 영상 — **지금 쓰는 한 대만** 굽는다
            rp.publish_camera(only=현재카메라역할)
        bridge.spin()
```

### 이 블록을 **어디에 두는가**가 함정이다

🚨 **"임무 끝(DONE)" 검사보다 앞에 둘 것.** 뒤에 두면 시퀀스가 끝나는 순간
발행도 지령 수신도 통째로 멈춘다 — 화면이 얼어붙고 버튼도 안 먹는다(실측 교훈).

### 카메라 분기 — 한 번에 한 대

```python
def active_camera(state):
    if state in ("RETURN", "RECOVER"):        return "rear"    # 후진
    if state in ("ALIGN", "EXTEND", "ARC"):   return "torch"   # 용접
    return "front"                                             # 전진
```

- 셋 다 굽는 것은 낭비다(JPEG 인코딩 3배). `only=` 로 한 대만 낸다.
- 안 켠 카메라 토픽은 **정상적으로 조용하다.** 웹은 `cam` 필드를 보고
  "지금 후방 차례" 라고 표시한다 — `cam` 을 안 실으면 웹이 전방으로 가정한다.
- 깊이(`depth/compressed`)는 **front 일 때만** 나간다(검출이 전방만 쓴다).

---

## 4. 접붙임 ④ — 상태가 바뀌는 순간 사건 하나

```python
    if bridge and 상태 != ros_prev_state:
        ev = {"INSPECT": contract.EV_DEFECT,
              "ARC":     contract.EV_WELD_BEGIN,
              "REPOSITION": contract.EV_WELD_DONE,
              "RECOVER": contract.EV_STUCK,
              "JUNCTION": contract.EV_BRANCH,
              "DISCONNECTED": contract.EV_DISCONNECT}.get(상태)
        if 상태 == "CRUISE" and ros_prev_state == "SETTLE":
            ev = contract.EV_START
        if ev:
            # 용접 사건에는 결함 좌표를 같이 싣는다 → 3D 맵이 관 벽면에
            # 노란 수리 스티커를 붙인다 (추가 필드라 스키마 변경 아님)
            extra = {"defect_s_mm": 결함_s * 1000, "clock_deg": 시계각}
            bridge.robot(ROS_NS).emit(ev, f"{ros_prev_state} → {상태}",
                                      s * 1000, **extra)
        ros_prev_state = 상태
```

🚨 **상태 스트림으로 사건을 유추하지 말 것.** 10Hz 샘플 사이에 일어난 전환은
통째로 사라진다(끼임 후 즉시 빠져나온 경우가 그렇다). 그래서 사건은 별도 토픽이다.

---

## 5. 접붙임 ⑤ — 지령(버튼) 받기

웹 버튼 → `POST /cmd` → `mission` 토픽 → `pop_mission()` 으로 들어온다.
**받는 것과 거는 것을 분리한다**: 콜백에서 로봇을 직접 건드리면 물리 스텝
중간에 상태가 바뀌어 재현이 안 된다. `ros_bridge` 가 큐에 쌓아 두므로 당신은
꺼내서 **플래그만 세우고**, FSM 앞에서 실제로 건다.

### 지령 7종

| 지령 | 시연에서의 동작 |
|---|---|
| `START` | 정지 해제 (방향은 **안 바꾼다**) |
| `STOP` | 주행 상태로 돌아오는 순간 정지(HOLD) |
| `RECALL` | 방향을 뒤집어 출발점으로 (RETURN 진입) |
| `FORWARD` | 복귀 취소 — 다시 전진 점검 (`RECALL` 의 짝) |
| `RETRY` | 끼임 탈출 재시도 (후진 후 재진입) |
| `SPEED` | `mps` 로 주행 속도 변경 |
| `ESTOP` | 어느 상태에서든 **그 자리에서 즉시** 언다. 재개 불가 |

### 🚨 지령을 **거절하지 말고 미룬다**

용접 시퀀스(정렬·전개·아크·냉각) 중에 세우면 **재개 경로가 없다** — 토치가
뻗고 아크가 붙은 상태로 대기하게 된다. 그래서 `STOP`/`RECALL`/`FORWARD`/`RETRY`
는 받아서 **플래그만 세워 두고**, FSM 이 주행 상태로 돌아오는 순간 건다.
조작하는 사람 입장에서 "거절" 보다 "지연" 이 예측 가능하다. `ESTOP` 만 예외로
즉시 건다(비상정지가 미뤄지면 비상정지가 아니다).

```python
HOLD_STATES = ("SETTLE", "CRUISE", "RESUME", "RETURN", "JUNCTION")  # 주행 상태

def apply_mission(cmd):                     # ③ 에서 꺼낸 것
    global hold, estop, recall, forward
    c, why = cmd.get("cmd"), cmd.get("reason", "")

    # ESTOP 해제는 START + reason=ESTOP_RELEASE 로만. 실수로 ▶시작을 눌러
    # 비상정지가 풀리면 안 되므로 규약에 지령을 새로 만들지 않고 이렇게 한다.
    if estop and c == contract.CMD_START and "ESTOP_RELEASE" in why.upper():
        estop, hold = False, True           # 풀어도 바로 안 간다
    elif estop:
        return                              # 그 밖의 지령은 무시
    elif c == contract.CMD_START:   hold = False
    elif c == contract.CMD_STOP:    hold = True
    elif c == contract.CMD_RECALL:  recall, forward, hold = True, False, False
    elif c == contract.CMD_FORWARD: forward, recall, hold = True, False, False
    elif c == contract.CMD_SPEED:
        v = min(max(float(cmd["mps"]), 0.005), SPEED_MAX)
        # 🚨 목표 속도와 **휠 각속도를 같이** 갈 것. 하나만 바꾸면 화면
        #    숫자만 변하고 로봇은 그대로 간다(조용한 거짓말).
        TARGET_SPEED_MPS, SPIN_DEG_S = v, math.degrees(v / WHEEL_R)
    elif c == contract.CMD_ESTOP:
        estop, hold = True, True
        drive(0.0); 아크_끄기()

# ── FSM 바로 앞에서 실제로 건다 ──────────────────────────
if bridge and estop:
    drive(0.0)
    continue                                # FSM 통째로 건너뜀
if bridge and 상태 in HOLD_STATES:
    if recall:  recall = False;  상태 = "RETURN";  continue
    if forward: forward = False; 상태 = "CRUISE";  continue
    if hold:    drive(0.0);                        continue
```

🔑 **서 있는 동안 상태를 그렇게 실을 것.** `CRUISE` 를 그대로 내보내면 화면에는
"주행 중" 인데 실제로는 멈춰 있다.

```python
held = estop or (hold and 상태 in HOLD_STATES)
state = (contract.STATE_DEAD if estop else
         contract.STATE_HOLD if held else STATE_MAP.get(상태))
reason = f"HOLD({상태})" if held else 상태      # 뭘 하다 멈췄는지가 여기 남는다
```

🔑 **`FORWARD` 를 적용할 때 결함 검사 플래그를 되돌릴 것.** 시연에서는
`inspected`/`rechecked` 가 남아 있으면 재검(RECHECK)을 건너뛰고 바로 용접으로
가는데, **용접봉 잔량 검사가 그 재검 안에** 있다. 소진 때문에 복귀했던 경우
검사 없이 그대로 용접해 버린다.

---

## 6. 접붙임 ⑥ — 종료

```python
if bridge is not None:
    print(f"[ROS] 발행 종료 — 영상 {sum(p.n_img for p in bridge.pubs.values())}장")
    bridge.node.destroy_node()
    bridge.shutdown()            # 다른 rclpy 노드를 같이 쓰면 여기서 하지 말 것
```

⚠ 한 프로세스에서 **다른 rclpy 노드**(예: 활성카메라 발행)를 같이 쓰면
`rclpy.shutdown()` 은 **한 번만** 불러야 한다. 시연은 그래서 `destroy_node()` 만
하고 shutdown 은 마지막 한 곳에 몰아 뒀다.

---

## 7. 단위·좌표 규약 — 여기만 틀려도 화면이 조용히 거짓말한다

| 값 | 규약 | 틀리면 |
|---|---|---|
| `s_mm` | 관 **중심선을 따라 잰** 진행거리(mm) | 직선거리를 쓰면 곡관에서 위치가 **뒤로 간다** |
| `pos_m` | 월드 좌표 `[x,y,z]` (m). `course` 와 **같은 좌표계** | 3D 맵의 로봇 점이 관 밖에 뜬다 |
| `speed_mps` | 항상 **양수**. 방향은 `direction`(+1/-1) | 후진이 음수 속도로 나가면 화면이 뒤집힌다 |
| `roll_deg` / `clock_deg` | **0° = 천장, 180° = 바닥**(중력 방향) | 토치를 정반대로 돌린다. 에러는 안 난다 |
| `step` | 단조증가. **재시작하면 0** 으로 되돌릴 것 | 웹이 재시작을 못 알아채 지나온 초록선이 안 지워진다 |
| `cam` | `"front"`/`"rear"`/`"torch"` | 웹이 전방으로 가정해 "왜 후방이 안 나오나" 가 된다 |
| 각도 | 전부 **도(degree)**, 길이는 상태=mm / 코스=m | — |

### 상태값 (`STATES`) — 당신 FSM 을 여기에 **좁혀서** 매핑한다

`SETTLE` `RUN` `HOLD` `STUCK` `INSPECT` `REPAIR` `RETURN` `DONE` `DEAD`

```python
STATE_MAP = {"SETTLE": contract.STATE_SETTLE,
             "CRUISE": contract.STATE_RUN, "RESUME": contract.STATE_RUN,
             "INSPECT": contract.STATE_INSPECT,
             "ALIGN": contract.STATE_REPAIR, "ARC": contract.STATE_REPAIR,
             "RECOVER": contract.STATE_STUCK,
             "RETURN": contract.STATE_RETURN, "DONE": contract.STATE_DONE}
```

규약에 없는 값을 주면 `ValueError` 로 **즉시** 걸린다(조용히 안 넘어간다).

### 사건값 (`EVENTS`)

`START` `ARRIVE` `HOME` `STUCK` `OFF_COURSE` `DISCONNECT` `BRANCH` `DEFECT`
`WELD_BEGIN` `WELD_DONE` `ESTOP` `DONE`

---

## 8. 실행 절차

### 터미널 A — 받는 쪽 (python 3.10)

```bash
ros_set
colcon build --packages-select pipe_comm     # .py 를 고쳤을 때만
ros2 run pipe_comm web_panel --ros-args -p ns:=robot -p port:=8080
```

→ 브라우저 `http://<서버IP>:8080` (터널이면 `ssh -L 8080:localhost:8080 …`)

### 터미널 B — 보내는 쪽 (Isaac, python 3.11)

```bash
isaac_ros                       # 🚨 ros_set 을 같이 하면 안 된다
PYTHONUNBUFFERED=1 isaac_python 당신_주행코드.py --ros
```

### 확인

```bash
ros2 topic list | grep robot            # 토픽이 보이나
ros2 topic echo /robot/drive_state --once
ros2 run pipe_comm mission_cli -- STOP  # 지령이 먹나 (구독자 0 이면 종료코드 1)
ros2 run pipe_comm drive_monitor        # 터미널로 상태 보기
```

🚨 **인증이 없다.** 공인망에 열지 말 것 — 버튼이 있는 페이지다. 보안그룹은
본인 IP /32 로만, 또는 SSH 터널.

---

## 9. 이미 데인 함정 (전부 실측)

| 증상 | 원인 | 조치 |
|---|---|---|
| 지령이 안 먹는다 | `bridge.spin()` 을 매 스텝 안 부름 | 매 스텝 호출 (timeout 0) |
| 임무 끝나면 화면이 얼어붙음 | 발행 블록이 `DONE` 검사 **뒤**에 있었다 | 발행 블록을 앞으로 |
| 발행은 되는데 아무도 못 받음 | 토픽 이름 오타 / 네임스페이스 불일치 | `contract.Topics` 만 쓸 것 |
| 버튼을 눌렀는데 무반응 | `mission` 은 RELIABLE 이지만 **latch 가 아니다** — 구독자 0 이면 사라진다 | 화면이 구독자 수를 띄운다. 0 이면 시연이 안 떠 있는 것 |
| 카메라 프레임 0 | `--headless` 는 `world.step(render=False)` — 렌더가 안 돈다 | 화면/스트리밍으로 띄울 것 |
| rclpy 없음 → **세그폴트** | `isaac_ros` 없이 `--ros` 를 줬다 | `isaac_ros` 먼저. 검사를 `SimulationApp` **앞**으로 옮기면 근본 해결 |
| 3D 맵이 "코스 수신 대기" | `publish_course()` 를 안 불렀다 | 기동 때 1회 (latched 라 재발행 불필요) |
| 재시작해도 초록선이 남음 | `step` 을 0 으로 안 되돌렸다 | 재시작 시 `step = 0` |
| 곡관에서 위치가 뒤로 감 | `s_mm` 에 직선거리를 실었다 | 중심선 호길이로 |
| GPU 물리에서 로봇이 관을 뚫음 | `contactOffset` 이 GPU 허용치보다 작음 | 물/파티클 모드는 엔진 기본값 |

---

## 10. 이식 체크리스트

- [ ] `--ros` 플래그와 `ROS_NS` 를 읽는다
- [ ] `world.reset()` **뒤**에 `Bridge` 생성 + `use_annotators()`
- [ ] `publish_course()` 를 기동 때 **1회**
- [ ] 매 스텝 `bridge.spin()`
- [ ] 10Hz `publish_state(... pos=..., cam=...)` + `publish_camera(only=...)`
- [ ] 발행 블록이 **DONE 검사보다 앞**에 있다
- [ ] 상태 전이마다 `emit()`
- [ ] `pop_mission()` → 플래그 → FSM 앞에서 적용 (ESTOP 만 즉시)
- [ ] 서 있을 때 `STATE_HOLD` / `STATE_DEAD` 로 싣는다
- [ ] 재시작 시 `step = 0`
- [ ] 종료 시 `destroy_node()`

### 통과 기준

1. `ros2 topic echo /robot/drive_state` 에 10Hz 로 JSON 이 흐른다
2. 브라우저 3D 맵에 관과 **빨간 점**이 뜨고, 점이 관을 따라 움직인다
3. Camera 화면에 영상이 뜨고, 후진하면 **후방 카메라로 바뀐다**
4. ▶ 시작 / ⏸ 정지 버튼이 실제로 로봇을 세우고 다시 보낸다
5. Isaac 을 재시작하면 지나온 초록선이 **0 부터 다시** 그려진다

---

## 11. 물어볼 곳

- 규약 상세(필드 단위까지) — `ROS2_통신규격.md` (13절)
- 웹 화면 구조·페이지 추가 — `web/README.md`
- 지금 붙어 있는 실물 — `src/dongyeon/integration_test/real_map_demo.py`
  의 `if ROS:` 블록(기동)과 주 루프의 `if bridge:` 블록
- 남은 일 목록 — `src/dongmin/TODO.md`
