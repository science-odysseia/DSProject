# `src/dongyeon` — 결함 검출 · 리포트 · 수리 판정 (김동연)

"이게 결함인가" 를 화면에서 가려내고, 그 결과를 **임무 기록으로 남기고**,
용접봉 잔량까지 따져 **수리할 수 있는가**를 판정하는 부분.

> 전체 개요와 실행 순서는 워크스페이스 루트의 [`README.md`](../../README.md) 를 볼 것.

---

## 📁 구성

```
dongyeon/
├── detect/
│   └── finders.py             🔍 구멍·비드 검출 (순수 OpenCV) — 시연이 직접 로드한다
└── pipe_inspect_demo/         ROS 2 패키지 (ament_python, Python 3.10)
    ├── pipe_inspect_demo/
    │   ├── view_active_cam_node.py   활성 카메라 + 판정 오버레이 실시간 뷰어
    │   ├── repair_decision.py        용접봉 소요량·수리 가능 판정
    │   ├── repair_target_registry.py 결함 ID별 수리 목표·상태 관리
    │   ├── defect_report_store.py    임무 단위 결함 기록 적재 (JSONL + 요약 JSON)
    │   ├── pipe_report_node.py       결함 JSON 구독 → 디스크 보존
    │   ├── pipe_coordinator_node.py  잔량 + 요청 → 최종 수리 판정 발행
    │   └── repair_target_test_node.py 목표 요청·결과 흐름 시험용 노드
    ├── test/                  오프라인 시험
    └── PIPE_INSPECTION_PROTOTYPE_V1.md
```

---

## 🔍 `detect/finders.py` — 이 프로젝트의 결함 검출기

**시뮬레이션 본체가 이 파일을 경로로 직접 읽어 매 프레임 호출한다.**
`src/son/real_map_demo_v1_3.py` 가 `find_wall_hole` / `find_weld_bead` 를 부른다.
`src/son` 쪽에 흩어져 복붙으로 갈라져 있던 검출 함수를 하나로 합치고 개선한 판이다.

### 왜 순수 OpenCV 인가 — YOLO 를 쓰지 않는다

이 저장소에서 **YOLO Seg 는 한 번도 실행된 적이 없다.** 학습 가중치가 실린 적이 없고
`ultralytics` 도 설치돼 있지 않다. 반면 구멍·비드 검출은 **학습·GPU·torch 로딩 없이**
`cv2` 만으로 끝난다 — 상시로 가볍게 돌려야 하는 요구에 이쪽이 맞다.
그래서 제출본에서 YOLO 경로(노드 4개·시험 2개·백업 폴더)를 **전부 제외**했다.

### 밟았던 함정 3가지 (전부 실측)

**① 전방 개구부(관 저 끝)를 반드시 걸러야 한다**

"화면 중앙 덩어리 제외" 는 저 끝이 화면 중앙에 있다는 전제라, **관이 꺾이면 저 끝이
화면 옆으로 밀려 통째로 "구멍" 으로 잡힌다**(실전 맵 실측 8,182px 오검).

→ **Depth 로 가른다.** 벽면 구멍은 테두리가 근접 벽(≈0.1m), 관 저 끝은 먼 벽(0.3~0.5m).
관이 꺾여도 성립한다. Depth 가 없으면 중앙 덩어리 제외로 물러난다.

**② 전역 이진화는 굽은 관에서 성립하지 않는다**

조명이 한쪽으로 몰리면 화면 절반이 통째로 어두워 결함이 배경과 이어져 버려진다.
→ 예상 위치가 있으면 **그 둘레 창만 잘라 창의 분포로** 임계를 잡는다.
자기충족이 아니다 — 창 안에 어두운 덩어리가 실제로 없으면 아무것도 안 잡히고,
**수리가 되면 정확히 그렇게 된다.**

**③ 물결 그림자가 구멍으로 잡힌다**

관에 물이 차 있으면 물결 그림자가 어둡게 잡힌다.
→ **파랑 화소를 후보에서 뺀다**(`B−R > 18` 이고 `B ≥ G`). 물결 그림자는 어두워도
파란 기가 돌고(물 재질 diffuse B 0.85 > R 0.15), **진짜 구멍은 무채색 검정**이다.
`find_weld_bead` 의 "색으로 가른다" 와 같은 원리다.

---

## 🧮 `repair_decision.py` — 용접봉 잔량으로 수리 가능 여부를 판정한다

결함을 찾는 것과 **고칠 수 있는가**는 다른 문제다. 용접봉은 유한하다.

| 상수 | 값 | 뜻 |
|---|---|---|
| `ROD_DIAMETER_MM` | 2.0 | 용접봉 지름 |
| `VOLUME_MARGIN_FACTOR` | 1.2 | 체적 여유 |
| `HOLE_VOLUME_FACTOR` | 1.3 | 관통 구멍은 크랙보다 더 든다 |
| `RESERVE_ROD_LENGTH_MM` | 10.0 | 예비분 — 0 까지 쓰지 않는다 |
| `MISSION_LOAD_MARGIN_FACTOR` | 1.15 | 임무 전체 계획의 여유 |
| `ROD_COIL_CAPACITY_MM` | 1880.0 | 코일 1개 용량 |

결함 치수 → 소요 체적 → **소요 봉 길이**로 환산하고, 남은 길이와 비교해
`수리 가능 / 불가` 를 낸다. 임무 단위로는 결함 목록 전체를 놓고
**"이번 임무에 다 고칠 수 있는가"** 를 계획한다.

## 🗂 `defect_report_store.py` — 기록은 원본과 요약을 나눈다

- **JSONL 이벤트 원본** — 들어온 그대로, 시간순. 나중에 되짚을 수 있어야 한다.
- **결함 ID별 요약 JSON** — 같은 결함이 여러 번 보고되면 최신 상태로 접는다.

실행마다 폴더가 갈린다(임무 단위). 저장 위치는 **워크스페이스 이름을 가정하지 않고**
`PIPE_REPORT_OUTPUT` → `__file__` 기준 워크스페이스 루트 → 현재 디렉터리 순으로 찾는다.

## 👁 `view_active_cam_node.py` — 판정을 원본 위에 겹쳐 그린다

시뮬레이션이 발행하는 것은 **이미 렌더된 이미지가 아니라 좌표·판정 근거 JSON** 이다.
이 노드가 그 좌표를 **지금 들어오는 raw 프레임 위에 직접** 그린다.

그래서 판정이 나는 "순간의 정지 이미지" 가 아니라, 로봇이 결함에 다가가는 동안
**원이 실시간으로 따라온다.**

```
구독  /{ns}/repair_robot/active_cam/rgb/compressed   CompressedImage
      /{ns}/repair_robot/active_cam/which            String  (지금 활성 카메라)
      /{ns}/repair_robot/opencv_judgement/json       String  (판정 좌표·근거)
```

`m` 키로 원본 ↔ 엣지검출(Canny) 순환, `q`/`Esc` 로 종료.

> **표시 전용이다.** 결함을 실제로 등록·확정하는 시점은 시뮬레이션 쪽의
> INSPECT / RECHECK / VERIFY 세 번뿐이고, 이 오버레이가 그 판정을 바꾸지 않는다.

---

## ▶ 실행

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=143

ros2 run pipe_inspect_demo view_active_cam
ros2 run pipe_inspect_demo pipe_report
ros2 run pipe_inspect_demo pipe_coordinator
ros2 run pipe_inspect_demo repair_target_test
```

`finders.py` 는 노드가 아니다 — 시뮬레이션 본체가 직접 로드하므로 따로 띄우지 않는다.

## ✅ 검증

```bash
cd pipe_inspect_demo
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
#   test_repair_decision.py        용접봉 소요량·판정
#   test_repair_target_registry.py 목표 등록·상태 전이
#   test_defect_report_store.py    기록 적재·요약 접기
```

> 🚨 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 이 필요하다. 시스템에 깔린 `anyio` 의 pytest
> 플러그인이 Humble 의 pytest 버전과 충돌한다(`ModuleNotFoundError: _pytest.scope`).
