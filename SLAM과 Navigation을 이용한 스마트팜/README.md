# Green Guard

SLAM과 Navigation을 이용한 스마트팜 AMR 순찰·보안 시스템

## 프로젝트 개요

- **목표**: 고정형 CCTV의 감시 사각지대와 반복 순찰에 따른 인력 부담을 보완하기 위해, TurtleBot4 기반 AMR이 스마트팜을 자율 순찰하며 침입자·이상 작물을 감지하고 이력을 관리하는 End-to-End 보안·관리 시스템 구현
- **주요 기능**: 고정 카메라 감시, AMR 자율 순찰, YOLO 기반 AI 감지, 감지 이벤트 DB 저장, Web UI 모니터링
- **사용 장비**: TurtleBot4(Create3), OAK-D 카메라, USB 웹캠(CCTV), RPLIDAR
- **개발 환경**: Ubuntu 22.04, ROS2 Humble, Nav2
- **주요 기술 스택**: ROS2, Nav2, AMCL(SLAM 기반 위치추정), YOLOv8, OpenCV, Flask, SQLite3
- **기간**: 2026.06.23 ~ 2026.07.14

## 시연 영상

https://github.com/user-attachments/assets/07690226-8a90-406d-bcf1-c45918c75e2c

## 상세 설명

### 문제정의

- 고정형 CCTV의 설치 위치 중심 감시로 사각지대 발생
- 반복 순찰에 따른 인력·시간 부담
- 병든 토마토 등 작물 이상 상태의 조기 발견 한계
- 감지 시점의 위치·시간·이미지 이력 관리 어려움
- ROS2 상태 확인에 대한 사용자 접근성 부족

### 해결방안

- 고정 카메라로 1차 감시 구역을 확인하고, TurtleBot4 AMR이 waypoint 기반으로 이동 순찰하여 고정 카메라의 사각지대를 보완
- YOLO 기반 이미지 판별로 침입자와 이상 토마토를 감지
- 감지 결과(이미지·시간·위치)를 SQLite3 DB에 저장하고 Flask 기반 Web UI로 로봇 상태·감지 로그를 조회
- 고정 카메라 감지 → AMR 이동 → AI 판별 → DB 저장 → Web UI 조회로 이어지는 End-to-End 구조로 연결

### 주요기능 (AMR/순찰 파트)

- **자율 순찰(Patrol)**: AMCL 위치 추정을 기반으로 미리 정의된 waypoint를 순서대로 순회
- **사람 탐지(YOLO Detection)**: OAK-D 카메라의 RGB 영상을 YOLOv8n으로 분석해 사람을 탐지하고 좌표를 토픽으로 발행
- **추적 모드(Tracking)**: 순찰 중 사람이 감지되면 순찰을 즉시 중단하고, depth 영상으로 거리 정보를 계산해 대상에게 접근. 일정 거리 이내에서는 정지 후 추종(follow) 상태로 전환
- **탐색(Search) 및 복귀(Gohome)**: 추적 중 대상을 놓치면 제자리에서 한 바퀴 회전하며 재탐색하고, 끝내 찾지 못하면 가장 가까운 경유지를 거쳐 도킹 스테이션으로 복귀
- **경고음(Warning Beep)**: 사람이 감지되는 동안 Create3 스피커로 경고음 반복 재생
- **CCTV 사람 탐지 및 리포트(Report)**: 웹캠 기반 CCTV 노드가 사람을 탐지하면 자동으로 리포트용 launch를 실행해 별도 기록/알림 흐름 시작
- **예약 순찰(Reserve)**: Tkinter GUI로 반복 순찰 간격(시/분)을 입력하면 해당 주기마다 순찰(`patrol.launch.py`)을 자동 실행

## 시스템 구성 (`pressedfinal` ROS2 패키지)

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

### 폴더 구조

```
SLAM과 Navigation을 이용한 스마트팜/
├── README.md
├── imgs/
│   └── videos/
│       └── 그린가드_1분영상.mp4        # 시연 영상
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

### 실행 방법

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

### 주요 토픽 (네임스페이스: `/robot3`)

- `amcl_pose`: 로봇 현재 위치 (Nav2 AMCL)
- `detected_bottom`: YOLO가 탐지한 사람의 바닥 접점 픽셀 좌표 (`-1, -1`이면 미탐지)
- `warning`: 사람 감지 여부 경고 상태 (Bool)
- `cmd_audio`: Create3 스피커 재생용 음계 메시지
- `/detection/cctv/human`, `/detection/cctv/human/flag`: CCTV 웹캠 사람 탐지 결과
- `/reserve`, `/reserve_time`: 예약 순찰 간격 및 다음 실행 시각

## 프로젝트 기여자 (C-3조)

- 방현식 (문서 / 시스템 설계도)
- 김찬혁 (시스템 모니터 / Web UI·DB)
- 황재문 (AI Detection / 객체 탐지 검증)
- 유상우 (AMR 제어 / 노드 구성)
- 멘토: Andy Kim

## 참고자료

- https://github.com/turtlebot/turtlebot4
- https://docs.nav2.org/
- https://github.com/ultralytics/ultralytics
