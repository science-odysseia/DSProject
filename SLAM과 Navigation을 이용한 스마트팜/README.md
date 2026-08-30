# SLAM과 Navigation을 이용한 스마트팜 순찰 로봇

TurtleBot4(Create3 + OAK-D)와 ROS2 Nav2 스택을 이용해 스마트팜 내부를 자율 순찰하고,
사람이 감지되면 추적·경고 후 복귀(도킹)까지 수행하는 로봇 시스템입니다.

## 주요 기능

- **자율 순찰(Patrol)**: AMCL 위치 추정을 기반으로 미리 정의된 웨이포인트를 순서대로 순회합니다.
- **사람 탐지(YOLO Detection)**: OAK-D 카메라의 RGB 영상을 YOLOv8n으로 분석해 사람을 탐지하고, 탐지 좌표를 토픽으로 발행합니다.
- **추적 모드(Tracking)**: 순찰 중 사람이 감지되면 순찰을 즉시 중단하고, 깊이(depth) 영상으로 거리 정보를 계산해 대상에게 접근합니다. 일정 거리 이내에서는 정지 후 추종(follow) 상태로 전환됩니다.
- **탐색(Search) 및 복귀(Gohome)**: 추적 중 대상을 놓치면 제자리에서 한 바퀴 회전하며 재탐색하고, 끝내 찾지 못하면 가장 가까운 경유지를 거쳐 도킹 스테이션으로 복귀합니다.
- **경고음(Warning Beep)**: 사람이 감지되는 동안 TurtleBot4(Create3) 스피커로 경고음을 반복 재생합니다.
- **CCTV 사람 탐지 및 리포트(Report)**: 웹캠 기반 CCTV 노드가 사람을 탐지하면 자동으로 리포트용 launch를 실행해 별도 기록/알림 흐름을 시작합니다.
- **예약 순찰(Reserve)**: Tkinter GUI로 반복 순찰 간격(시/분)을 입력하면, 해당 주기마다 순찰(`patrol.launch.py`)을 자동 실행합니다.

## 시스템 구성

| 구분 | 노드 (실행 파일) | 역할 |
|---|---|---|
| 순찰/추적 | `patrol.py` | AMCL 기반 순찰, 사람 감지 시 추적 전환, 탐색, 복귀·도킹까지 담당하는 메인 로직 |
| 사람 탐지 | `yolo_detection.py` | 로봇 카메라 영상에서 YOLOv8n으로 사람을 탐지해 좌표 발행 |
| 경고음 | `beep_infinite.py` | 경고 상태 토픽을 구독해 Create3 스피커로 경고음 재생/정지 |
| CCTV 탐지 | `cctv_camera_publisher.py` | 별도 웹캠으로 사람을 탐지해 이미지·플래그 토픽 발행 |
| 리포트 트리거 | `initiating_report.py` | CCTV 탐지 플래그를 감지하면 `report.launch.py`를 자동 실행 |
| 리포트 로직 | `report.py` | 리포트 launch에 포함되는 순찰/추적 로직 (patrol과 유사 구성) |
| 예약 UI | `time_pub.py` | Tkinter GUI에서 순찰 반복 간격(HH:MM)을 입력받아 발행 |
| 예약 스케줄러 | `time_sub.py` | 반복 간격을 구독해 다음 실행 시각을 계산하고, 주기마다 순찰 launch 실행 |

## 폴더 구조

```
SLAM과 Navigation을 이용한 스마트팜/
├── README.md
├── imgs/
│   └── videos/
│       └── 그린가드_1분영상.mp4        # 프로젝트 소개 영상
└── pressedfinal/                      # ROS2 패키지 (ament_python)
    ├── package.xml
    ├── setup.py / setup.cfg
    ├── launch/
    │   ├── patrol.launch.py           # beep_infinite + yolo_detection + patrol
    │   ├── report.launch.py           # beep_infinite + yolo_detection + report
    │   └── start_report.launch.py     # cctv_camera_publisher + initiating_report
    ├── pressedfinal/                  # 노드 소스 코드
    │   ├── patrol.py
    │   ├── yolo_detection.py
    │   ├── beep_infinite.py
    │   ├── cctv_camera_publisher.py
    │   ├── initiating_report.py
    │   ├── report.py
    │   ├── time_pub.py
    │   └── time_sub.py
    └── test/                          # ament 코드 스타일 테스트
```

## 실행 방법

ROS2(Humble 이상) + TurtleBot4 Nav2/SLAM 스택이 구동 중인 환경을 전제로 합니다.

```bash
# 워크스페이스 빌드
colcon build --packages-select pressedfinal
source install/setup.bash

# 순찰 + 사람 탐지 + 경고음
ros2 launch pressedfinal patrol.launch.py

# CCTV 사람 탐지 → 자동 리포트 실행 대기
ros2 launch pressedfinal start_report.launch.py

# 예약 순찰 GUI (반복 간격 설정 → 자동 patrol 실행)
ros2 run pressedfinal time_pub
ros2 run pressedfinal time_sub
```

## 주요 토픽 (네임스페이스: `/robot3`)

- `amcl_pose`: 로봇 현재 위치 (Nav2 AMCL)
- `detected_bottom`: YOLO가 탐지한 사람의 바닥 접점 픽셀 좌표 (`-1, -1`이면 미탐지)
- `warning`: 사람 감지 여부 경고 상태 (Bool)
- `cmd_audio`: Create3 스피커 재생용 음계 메시지
- `/detection/cctv/human`, `/detection/cctv/human/flag`: CCTV 웹캠 사람 탐지 결과
- `/reserve`, `/reserve_time`: 예약 순찰 간격 및 다음 실행 시각
