# `src/son` — Isaac Sim 시뮬레이션 본체 (손영빈)

배관 안에서 **로봇이 실제로 움직이고, 보고, 판단하고, 용접하는 부분** 전부.
씬 구축 · 물리 · 임무 FSM · 용접 FSM · 자율주행 판단 모듈이 여기에 있다.

> 전체 개요와 실행 순서는 워크스페이스 루트의 [`README.md`](../../README.md) 를 볼 것.

---

## ★ 메인 실행 파일

```
real_map_demo_v1_3.py      5,677행 — 이 프로젝트의 시연 본판
run_v13.sh                 실행 스크립트 (검증 레시피·rclpy 경로 내장)
```

```bash
./run_v13.sh floor1     # 아래층 — T 분기 정찰 (닫힌 루프 순회)
./run_v13.sh floor2     # 윗층   — 결함 2곳 용접 수리 → 단절 감지 → 복귀
./run_v13.sh both       # 두 층 동시 (기본값)
```

**읽는 방법: 위에서 아래로가 곧 실행 순서다.** 이 파일은 함수 정의와 모듈 레벨
실행문이 섞여 있고, **모듈 레벨 실행문의 순서 자체가 설계 제약**이다.

```
기동 인자·레시피 → SimulationApp → 씬 구축 → world.reset() → DOF 매핑
   → 센서·브리지 배선 → [무한 물리 루프] → 결과 요약 → close()
```

---

## 📁 디렉터리

| 디렉터리 | 내용 | 실행 환경 |
|---|---|---|
| `robot/` | 로봇 자산 `pipe_robot_v11_weld.usda` — 12륜 + 다리 12 + 롤·굽힘 관절 + 용접 링/토치 | 자산 |
| `maps/` | 배관망 CAD `restroom_final0807.usd` + 웹 3D 맵용 `.webmesh` | 자산 |
| `camera/` | 카메라 하우징 STL + 어안 규격 `config/camera.yaml` | 자산 |
| `condition/` | **관 상태 판정** `detector.py` + **시각 오도메트리** `odometry.py` + ROS 노드 | 순수 / rclpy |
| `driver/` | **주행 제어** `control.py` (속도 법칙 · 끼임 · FSM) + ROS 노드 | 순수 / rclpy |
| `localization/` | **추측 항법** `deadreckon.py` — 도면 없이 지나온 길을 기록 + ROS 노드 | 순수 / rclpy |
| `welder/` | **용접 3겹 검증** `weld.py` · **복귀 감사** `audit.py` · 아크 스파크 `spark_fx.py` | 순수 / Isaac |
| `defect/` | OpenCV 결함 검출 ROS 노드 (학습 모델 불필요) | rclpy |
| `test_code/` | 오프라인 검증 — 강제 입력·합성 장면으로 로직만 시험 | python3 |
| `tools/` | `detect_view.py` — 검출 창 뷰어 (별도 프로세스여야 한다) | python3 |
| `pyver.py` | 인터프리터 가드 — 틀린 파이썬이면 즉시 중단 | 공용 |

`floor1_tee_tape.txt` 는 floor1 의 **T 통과 관절 궤적 녹화본**이다(되감기 재생용 자산).

---

## 🔑 핵심 설계 판단

### 씬 구축 순서가 곧 규칙이다

```
① 조명 → ② 맵 1회 참조 + 콜라이더 정책 → ③ 코스 중심선 → ④ 건물 페인트
→ ⑤ 결함·비드 프림 → ⑥ 로봇 참조·조립·배치 → ⑦ 드라이브 값 덮어쓰기
→ ⑧ contactOffset → ⑨ world.reset() → ⑩ DOF 매핑 → ⑪ 다리 초기 신장
→ ⑫ 카메라 → ⑬ 스파크 → ⑭ ROS → ⑮ 판단 모듈
```

**이 순서를 바꾸면 조용히 망가진다.** 값은 맞는데 그 값이 엔진에 도달하지 않는 부류라
USD 파일을 읽어도 안 보이고 기하 검사도 통과한다.

### 이 코드를 지탱하는 불변식 — 고칠 때 먼저 볼 것

1. **usda·usd 는 읽기 전용이다.** 다른 값이 필요하면 로드 직후 코드가 덮어쓰고,
   그 덮어쓰기는 **`world.reset()` 앞**이어야 PhysX 로 넘어간다.
2. **조인트는 이름이 아니라 구조로 찾는다**(`discover`). 이름으로 고르면 자산 교체 시
   조용히 0개가 된다. 개수는 항상 탐색 결과와 대조하고 틀리면 중단한다.
3. **닫힌 루프에서 전역 최근접은 금지.** 진행거리 `s` 는 직전 값을 힌트로 ±0.35m 창
   안에서만 찾는다 — 전역 argmin 은 T 근처의 로봇을 코스 끝점에 붙여
   "도달"을 오판했다(실측 s 599 → 4088 점프).
4. **다리는 힘으로 몰고, 접촉은 위치·속도로 읽는다.** 위치 목표는 자기 참조라 벽이
   없으면 한계까지 래칫된다. 힘으로 접촉을 판정하면 12개 중 10개를 오판한다 —
   목표에 도달한 다리는 벽에 닿았든 허공이든 힘이 0 에 가깝기 때문이다.
5. **속도 관문은 곱하지 않고 `min` 으로 고른다.**
6. **`set_joint_positions()` 뒤에는 드라이브 타깃을 다시 써야 한다** — PhysX 런타임
   타깃이 지워지는데 USD 속성값은 멀쩡히 남아 파일만 봐서는 안 보인다.
7. **게이트 사슬의 순서가 곧 우선순위다.** 용접·테이프가 주행을 막고, 탈출 국면이
   이탈 심판을 면제하고, 원호가 이탈 한계를 넓힌다.
8. **판단과 규약은 남의 모듈** — `contract.py`, `finders.py` 를 이 파일이 고치지 않는다.

### 좌표·단위 규약

| 이름 | 뜻 | 주의 |
|---|---|---|
| `s` | 중심선 진행거리(m) | 창 추적. 관 밖에서는 0/끝점에 클램프 |
| `off` | 중심선까지 거리(m) | 관벽이 50mm 이므로 **한 자릿수 mm 여야 정상** |
| 시계각 | 0° = +Z(위), 반시계 | 다리 0/120/240°, 결함 60/300° |
| "오른쪽" | **월드 기준** 진행방향 × 중력반대 | 🚨 몸 프레임이 아니다. 수직 라이저를 지나면 몸 기준 오른쪽이 월드 왼쪽이 된다 |
| 관절 강성 단위 | **라디안당** | 자산이 도(度)로 적혀 있어도 실효는 rad (실측 확인) |
| 화면 방위 | `φ = atan2(dy, dx)`, dy = 화면 **아래** | USD 카메라는 로컬 −Z 를 보고 +Y 가 화면 위 |

---

## 🚨 밟았던 함정 (전부 실측)

| 함정 | 증상 | 원인·수정 |
|---|---|---|
| 물리 스텝 | NaN 발산 | 1/60 이면 피스톤이 한 스텝에 관벽까지를 뛰어넘는다 → **1/240** |
| 맵 스케일 | 건물이 2.5km 로 들어옴 | 맵 usd 는 `metersPerUnit 0.001`, 스테이지는 1.0 → 참조 Xform 에 `scale 0.001` |
| 건물 셸 | 관벽이 사라지고 로봇 추락 | 이 맵은 건물과 관 외피가 융합돼 있다 — 셸(`PartBody`)도 콜라이더를 줘야 한다 |
| 배관 콜라이더 | 관 속이 꽉 찬 덩어리 | 반드시 `approximation="none"` (기본 convexHull 이면 못 들어간다) |
| 휠 콜라이더 | 주행 2.5mm 에서 사망 | 이 조합에서는 자산 원본(convexHull)이 맞다 — 실린더로 바꾸면 죽는다 |
| 차동 속도 | 이탈 1.4mm 로 잘 정렬된 채 **전진만 못 함** | 속도 드라이브는 지령보다 빠른 바퀴를 잡아채므로 **어떤 바퀴도 기준보다 느리게 주면 안 된다.** 비율은 유지하되 최솟값이 1.0 이 되게 통째로 올린다 |
| 코너 감속 | 정적으로 밀다 쐐기 | 곡관 감속이 45→20mm/s 로 깎아 관성이 죽었다 → **코너는 감속 면제** |
| 다리 감속 판정 | 정상 구간에서도 상시 발동 | 편차(spread)는 정상에서도 8~18mm — **평균의 기준선 이탈**을 쓴다(리듀서 4.2~5.8 vs 그 외 0.1~1.4, 40배) |
| 용접 정렬 | 수렴이 원리적으로 불가 | 중심선 `s` 가 13mm 로 양자화 → 축오차를 **결함 접선에 투영**해 연속값으로 |
| 토치 신장 | 용접봉이 관을 뚫음 | 종료 판정이 신장량이었다 → **팁의 실제 반경 48mm** 로 |
| 맵 저장 | 원본이 3.4KB 껍데기로 날아감 | 🚨 **Isaac GUI 에서 맵 usd 에 Ctrl+S 하지 말 것** |
| headless | 카메라 프레임 0 | 토픽은 보이는데 프레임이 안 온다 — 영상 시험은 반드시 GUI |

---

## ▶ 자율주행 ROS 노드 (선택 — 판단을 별도 프로세스로 뺄 때)

기본 시연은 이 노드들을 **띄우지 않는다.** `real_map_demo_v1_3.py` 가 같은 판단 모듈
(`detector.py` · `odometry.py` · `control.py`)을 한 프로세스 안에서 직접 부르기 때문이다.
아래는 **Isaac(발행) ↔ ROS 노드(판단) 를 두 프로세스로 갈라** 돌릴 때의 구성이다.

```
Isaac 영상 ──▶ condition/node.py ──/condition──▶ driver/node.py ──/cmd_vel──▶ Isaac
                     └────────────/visual_speed─────┘
Isaac 상태 ──▶ localization/node.py ──▶ /pose · /path · /defect_marks
```

이 노드들은 **ament 패키지가 아니라 단독 스크립트**다 — `colcon build` 대상이 아니므로
`ros2 run` 이 아니라 `python3` 로 직접 띄운다(각 노드가 자기 `config/*.yaml` 을 읽는다).

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=143

cd condition    && python3 node.py --ros-args --params-file config/pipe_condition.yaml
cd driver       && python3 node.py --ros-args --params-file config/driver.yaml
cd localization && python3 node.py --ros-args --params-file config/localization.yaml
cd defect       && python3 node.py --ros-args --params-file config/defect_detect.yaml

# 로봇이 여러 대면 네임스페이스를 씌운다
python3 node.py --ros-args -r __ns:=/floor2 --params-file config/driver.yaml

# Isaac 연결 직후 1회 — 빈 공간의 Depth 반환 방식을 확인해 임계값 근거를 잡는다
cd condition && python3 depth_probe_ros.py --topic /floor2/depth --frames 60
```

🚨 **이 노드들은 시스템 python3(3.10) 전용이다.** Isaac 내장 3.11 로 띄우면
파일 맨 위의 `pyver.py` 가드가 즉시 중단시킨다.

---

## ✅ 검증 (Isaac 불필요, 시스템 python3)

판단 로직을 ROS 노드 **밖**(순수 Python)에 둔 이유가 이것이다 — 노드 안에 있으면
`rclpy` 없이 검증할 방법이 없다.

```bash
python3 test_code/driver/test_control.py            # 속도 법칙·FSM      19/19 통과
python3 test_code/driver/test_odometry.py           # 시각 오도메트리    오차 ±0.25mm
python3 test_code/welder/test_weld.py               # 용접 3겹 검증      10/10 통과
python3 test_code/welder/test_audit.py              # 복귀 감사          12/12 통과
python3 test_code/localization/test_deadreckon.py   # 추측 항법           5/5  통과

python3 test_code/condition/make_scenes.py          # 합성 장면 생성 (용량 때문에 제외됨)
python3 test_code/condition/test_detector.py        # 관 상태 판정
```

### 검증이 잡아낸 것 (대표 사례)

- **용접 3겹**: "파여 있는데 검출기가 놓침" 시나리오에서 Depth 프로파일이 +1.16mm 를
  잡아 FAILED 로 뒤집었다. 검출기 하나였으면 **성공으로 기록됐을** 경우다.
- **복귀 감사**: 1차 검증만 보면 3건 중 2건 성공(66.7%)이던 것이 감사 후 1건(33.3%)으로
  정정됐다.
- **추측 항법**: 슬립 구간에서 휠만 쓰면 +744mm(21.1%) 오차, 시각 오도메트리를 더하면
  +74mm(2.1%) — **오차 90% 감소.**
- **조용한 전멸 버그**: Depth 단위(mm/m)를 `nanmax` 로 판별했는데 `nanmax` 는 `NaN` 만
  무시하고 **무한대는 그대로 돌려준다.** 단절 화소가 하나만 들어와도 프레임 전체가
  1/1000 로 눌려 **무슨 일이 있어도 NORMAL** 이 나왔다 — 에러도 경고도 없이.
  유한값만으로 판별하도록 고쳤고, 이후 "판정 로직은 반드시 오프라인 프레임으로
  회귀 시험" 이 규칙이 됐다.
