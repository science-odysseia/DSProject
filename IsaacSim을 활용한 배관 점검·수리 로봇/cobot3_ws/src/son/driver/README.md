# driver — 주행 제어

관 상태 판정을 받아 **전진 속도 하나**를 낸다.

## 조향이 없다

관절은 수동이고 6륜이 관벽에 9N으로 압착되어 횡방향은 기계가 잡는다.
설계 문서도 `cmd_vel` 은 `linear.x` 만 쓴다고 못박고 있다. 그래서 여기 있는 것은
조향 PID 가 아니라 **속도 법칙**이다.

```
v = V_MAX x 전방개방도 x 곡관감속 x 상태게이트
```

| 인자 | 근거 |
|---|---|
| 전방개방도 | 개구부 면적이 작을수록 감속 |
| 곡관감속 | 개구부가 화면 중앙에서 벗어난 각. **관절 엔코더보다 빠른 선행 지표** — 엔코더는 이미 꺾인 뒤에야 반응한다 |
| 상태게이트 | 판정이 내리는 통행 허가 (full / slow / stop) |

## 파일

| 파일 | 역할 | 실행 환경 |
|---|---|---|
| `control.py` | FSM + 속도 법칙 (ROS 비의존) | python3 |
| `node.py` | ROS 노드 | rclpy |
| `config/driver.yaml` | 파라미터 전부 | — |

## FSM

```
IDLE ──start──> CRUISE ⇄ SLOW
                  │  │
      통행 불가 ───┘  └─── 끼임 ──> RECOVER ──> CRUISE
        ↓                              │ 3회 초과
       HOLD ──통행 회복──> CRUISE       ↓
                                     RETURN ──> DONE
                     30m 도달 / 복귀 지시 ──┘
```

## 끼임 판정

**바퀴는 도는데 실제로 안 나가는 것**이 끼임이다. 설계 문서 8.2 도 그렇게 적어
놓았으나 실제 이동을 무엇으로 재는지는 없었다. 휠 엔코더만으로는 6륜이 동시에
미끄러지면 못 잡는다.

`condition/odometry.py` 의 시각 오도메트리와 비교한다.

```
slip = |시각 속도| / |휠 속도|
slip < 0.30 이 1.5초 지속  →  끼임 확정  →  후진 후 재시도 (최대 3회)
```

## 구독·발행

구독 — `condition`, `wheel_speed`, `visual_speed`, `start`, `recall`
발행 — `cmd_vel` (linear.x 만), `drive_state`

이미지는 구독하지 않는다. 설계 원칙상 이미지는 인지 노드 밖으로 나가지 않고
여기로는 스칼라만 들어온다.

## 검증

`test_code/driver/` — FSM 19항목 전수 통과, 시각 오도메트리 정확도 ±0.25mm.
ROS 왕복(`condition/node.py` → `driver/node.py`)도 실제로 확인했다.

## 실행

```bash
python3 node.py --ros-args --params-file config/driver.yaml
python3 node.py --ros-args -r __ns:=/scout
```
