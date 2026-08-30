# defect/ — OpenCV 결함 검출 → 관 상대좌표 저장 (2026-08-06 신설)

`pipe_inspect_demo`(dongyeon, YOLOv8-Seg 기반)와는 **별도의, 독립적으로 도는
노드**다. `pipe_vision_node.py`는 건드리지 않았다 — 백업은
`../pipe_inspect_demo/archive/yolo_vision_pipeline_backup_20260806/`.

## 왜 별도 노드인가

- YOLO Seg 는 이 리포에서 한 번도 실행된 적이 없다(가중치 `.gitignore` 대상,
  `ultralytics` 미설치 — `repair_demo.py` 도 같은 이유로 OpenCV 로 갔다).
- `repair_demo.py`의 `find_wall_hole`/`find_weld_bead`는 **순수 `cv2`**라
  학습·GPU·torch 로딩이 없다. PC B 에서 가볍게 상시로 돌리는 게 목적이라,
  기존 YOLO 노드를 고치기보다 옆에 독립적으로 세우는 쪽을 택했다(무거워지면
  안 된다는 요구 때문에라도 하나의 노드에 두 검출 경로를 섞지 않았다).

## 연결점 — `pipe_inspect_demo` 대신 `localization`에 붙였다

처음엔 `pipe_inspect_demo`의 `RepairTargetRegistry`(결함 ID별 목표·상태
관리)에 연결하려 했으나, 실측해보니 그 경로가 스키마·메시지 타입 모두
끊겨 있었다(원인은 백업 폴더 README 참고 — `/odom`·`/imu` 를 아무도
발행하지 않는 타입으로 구독, `repair_target` 딕셔너리 키 불일치 등).
그쪽을 고치려면 "YOLO 노드는 건드리지 않는다"는 조건과 충돌한다.

대신 **`integration_test` 안에 이미 동작·검증된 저장 경로**를 그대로 썼다:

```
defect/node.py (신규)                    localization/node.py (기존, 무수정)
  OpenCV 로 구멍·비드 검출                  DeadReckoner 가 arc_mm(호 위치) +
  연속 프레임으로 확정                       roll_deg(원주 각도) 를 자기 상태로 추적
    │                                            ▲
    └── ~/mark_defect JSON ────────────────────┘
        {defect_id, kind, size_mm, confidence}
                                                 │
                                            ~/defect_marks 로 재발행
                                            (test_code/localization/
                                             test_deadreckon.py 로 검증된 경로)
```

`localization/node.py`는 한 줄도 안 고쳤다 — `~/mark_defect` 토픽 스키마에
그대로 맞춰 발행만 한다.

## 의도적으로 뺀 것

**수리로봇이 저장된 좌표로 실제 이동하는 로직은 여기 없다.** 지금 범위는
"검출 → 관 상대좌표 저장 연결"까지다. `~/defect_marks`(arc_mm/roll_deg 목록)
가 이미 나오고 있으니, 다음 단계는 `driver/control.py`의 FSM에 목표 arc_mm
까지 가서 멈추는 상태를 추가하는 것이다(현재 `DriveController`는 전진속도
게이팅만 하고 목표 지점 추종이 없다).

## 알려진 근사 — 나중에 정밀화 필요

- **원주 각도(clock 위치)는 근사치다.** 결함이 시야에 들어온 순간의 로봇
  절대 롤(`imu_roll`)을 그대로 결함의 시계 위치로 쓴다. 화면 중앙 근처에서
  검출됐다면 오차가 작지만, 화면 가장자리에서 처음 잡힌 결함은 실제 위치와
  몇 도 어긋날 수 있다. 정밀도가 필요해지면 `repair_demo.py`의
  `defect_pixel`/어안 역투영 아이디어를 가져와 화면 내 방위각을 더해야 한다.
- `size_mm`은 등가반경(px) × depth 로 만든 대략치다 — `measure_mask_size_3d`
  (`pipe_inspect_demo`)급 skeleton 기반 정밀 측정이 아니다. 일부러 뺐다
  (연산량 문제, 목적은 좌표 저장이지 정밀 치수 측정이 아니라서).

## 실행 / 시험

**2026-08-06 갱신 — 카메라 스트리밍 연결됨.** 처음 만들었을 때는
`repair_demo.py`가 카메라 영상을 ROS 로 아예 안 내보내서 이 노드가 프레임을
못 받았다(진단 내용은 대화 기록 참고). `repair_demo.py`에 `--ros-cameras`
플래그를 추가해 해결했다 — 기본은 꺼져 있고(기존 검증된 실행 경로를 안
바꾸려는 것), 줬을 때만 `front/rgb`·`front/depth`·`front/camera_info`
(+`rear/*`)를 raw `Image`(rgb8/32FC1) + BEST_EFFORT 로 상시 발행한다.
**GUI 필수** — headless 는 렌더가 안 일어나 빈 프레임이다.

```bash
# PC1 (Isaac Sim, GUI 필수)
DISPLAY=:1 isaac_python repair_demo.py --ros-cameras --hold

# PC2 — 결함 검출 + 좌표 저장
python3 defect/node.py --ros-args --params-file defect/config/defect_detect.yaml
python3 localization/node.py --ros-args --params-file localization/config/localization.yaml

# 확인
ros2 topic hz front/rgb
ros2 topic echo defect_marks

# 오프라인 검출 로직만 시험 (ROS·Isaac 불필요)
python3 ../test_code/defect/test_opencv_hole_detector.py
```
