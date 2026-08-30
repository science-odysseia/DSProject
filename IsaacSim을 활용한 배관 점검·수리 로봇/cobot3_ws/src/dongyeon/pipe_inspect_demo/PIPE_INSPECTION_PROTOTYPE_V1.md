# 파이프 결함 검출 1차 프로토타입 구성

## 1. 목적

이 프로토타입은 Isaac Sim 파이프 환경의 RGB·Depth 카메라 영상을 ROS 2로 전송하고, YOLOv8 Instance Segmentation으로 크랙을 실시간 검출하여 입구 기준 위치와 Depth를 JSON으로 발행한다. 현재 검출 클래스는 `crack` 하나이며 실제 로봇 대신 키보드로 움직이는 시뮬레이션 카메라를 사용한다.

## 2. 전체 실행 흐름

```text
Isaac Sim 파이프 환경
    ↓
RGB JPEG + Depth 16비트 PNG + CameraInfo + Odom + IMU 발행
    ↓
pipe_vision에서 RGB·Depth 시간 동기화
    ↓
YOLOv8 Seg 추론 및 마스크별 Depth 계산
    ↓
연속 검출 확인 및 동일 결함 병합
    ↓
입구 기준 위치·시계 방향을 포함한 JSON 발행
    ↓
pipe_report에서 이벤트 원본·결함별 최신 요약 저장
    ↓
yolo_viewer에서 마스크·클래스·신뢰도 실시간 표시
```

## 3. Isaac Sim 발행 측

기준 경로는 다음과 같다.

```text
rokey_cobot/isaacpjt/dongyeon/pipe_inspect_test/
```

### 3.1 필수 모듈

| 파일 | 역할 | 주요 기능 |
|---|---|---|
| `robot_pipe_integration_test.py` | 최신 통합 standalone 진입점 | 파이프·벨로우즈 로봇·카메라·조명·ROS 2 Bridge 구성, 바퀴 주행과 결함 정지 및 정렬 실행 |
| `pipe_inspection_scene.py` | 파이프 검사 환경 구성 | STL의 USD 변환과 세 파이프 직렬 배치 및 환경 조명 구성 |
| `robot_ros_sensor_publisher.py` | 로봇 센서 ROS 발행 | 640×640 RGB·Depth·CameraInfo와 파이프 입구 기준 Odom·IMU 발행, 정지·재출발 상태 처리 |
| `robot_camera_alignment.py` | 결함 중심 카메라 정렬 | 검출 중심으로 Pan/Tilt 조정, 정렬 JSON과 수리 목표 Pose를 결함마다 한 번 발행 |
| `pipe_frame_transform.py` | 파이프 상대좌표 변환 | 월드 카메라 위치·방향을 `pipe_entry` 기준 위치·쿼터니언으로 변환 |
| `camera_keyboard_controller.py` | 카메라 수동 시험 | 독립 카메라 환경에서 위치·방향 조작과 초기화 수행 |
| `crack_detector.py` | 카메라 영상 변환 | Isaac Sim RGBA 프레임을 RGB `uint8` 배열로 변환 |

### 3.2 필수 환경 파일

```text
usd 폴더/
├── pipe.stl
├── pipe_from_stl.usd
├── pipe_crack.STL
├── pipe_crack_from_stl.usd
├── inner_pipe_crack1.stl
└── inner_pipe_crack1_from_stl.usd
```

| 파일 | 역할 |
|---|---|
| `pipe.stl` | 정상 파이프 CAD 원본 |
| `pipe_from_stl.usd` | 정상 파이프 변환 결과 |
| `pipe_crack.STL` | 첫 번째 크랙 파이프 CAD 원본 |
| `pipe_crack_from_stl.usd` | 첫 번째 크랙 파이프 변환 결과 |
| `inner_pipe_crack1.stl` | 두 번째 크랙 파이프 CAD 원본 |
| `inner_pipe_crack1_from_stl.usd` | 두 번째 크랙 파이프 변환 결과 |

USD 파일이 없거나 STL보다 오래된 경우 `pipe_inspection_scene.py`가 Asset Converter로 다시 생성한다. 파이프는 +Y 방향으로 정상 파이프, 첫 번째 크랙 파이프, 두 번째 크랙 파이프 순서로 연결한다.

### 3.3 카메라와 조명 설정

| 항목 | 현재 값 |
|---|---|
| 해상도 | 640×640 |
| 센서 주기 | 20Hz 카메라 생성, ROS 영상 발행 5Hz |
| 초기 시선 | +Y 방향 |
| RGB 압축 | JPEG, 품질 85 |
| Depth 압축 | 16비트 PNG, 단위 mm |
| Depth 복원 단위 | m |
| 전·후방 조명 | 각각 원뿔각 120° |
| 파이프 설계 내경 | 0.100m |
| 파이프 설계 외경 | 0.108m |

영상 구독자가 없으면 RGB·Depth 복사와 압축을 생략하여 Isaac Sim 부하를 줄인다. CameraInfo·Odom·IMU는 영상과 같은 시각을 기준으로 발행한다.

## 4. ROS 2 검출 측

패키지 경로는 다음과 같다.

```text
rokey_cobot/src/dongyeon/pipe_inspect_demo/
```

### 4.1 패키지 필수 파일

| 파일 | 역할 |
|---|---|
| `package.xml` | ROS 2 의존성과 패키지 정보 정의 |
| `setup.py` | Python 패키지, 모델 파일, console script 설치 |
| `setup.cfg` | ROS 2 Python 실행 파일 설치 위치 설정 |
| `resource/pipe_inspect_demo` | ament package index 등록 파일 |
| `resource/yolov8n_seg_best.pt` | 기본 YOLOv8 Seg 학습 모델 |

### 4.2 실행 모듈

| 파일 | 역할 | 주요 기능 |
|---|---|---|
| `pipe_vision_node.py` | 결함 검출 메인 노드 | 압축 RGB·Depth 동기화, YOLO 추론 호출, 센서 상태 반영, JSON·검출 상태·디버그 영상 발행 |
| `yolo_seg_detector.py` | YOLOv8 Seg 추론 | Ultralytics 모델 로딩, 마스크·박스·클래스·신뢰도 추출, 마스크 영역 Depth 중앙값 계산, 디버그 영상 작성 |
| `ros_image_utils.py` | ROS 영상 변환 | JPEG RGB 디코딩, 16UC1 PNG Depth를 m 단위로 복원, 디버그 RGB를 JPEG로 압축 |
| `defect_position_tracker.py` | 결함 위치 및 ID 관리 | Odom 이동 추적, IMU Roll 계산, 입구 기준 축 위치·시계 방향 계산, 연속 검출 확인, 중복 결함 병합 |
| `pipe_report_node.py` | 검사 결과 저장 노드 | 결함 JSON 구독, 실행별 임무 폴더 생성, 원본 이벤트와 결함별 요약 저장 |
| `defect_report_store.py` | 파일 저장 로직 | JSONL 안전 추가, 최신·최고 신뢰도 보고 집계, 요약 JSON 원자적 교체 |
| `repair_decision.py` | 수리 가능 여부 판정 | 결함 종류 분류, 용접 체적·용접봉 소요량 계산, 예비 10mm를 반영한 수리·보고 판정 |
| `pipe_coordinator_node.py` | 수리 직전 최종 판단 | 결함 보고와 수리로봇의 실제 용접봉 잔량 보관, 수리 요청 시 최종 판정 JSON 발행 |
| `yolo_debug_viewer.py` | 실시간 시각화 | 압축 디버그 영상 구독, 마스크·박스 결과 표시, FPS·해상도·프레임 번호 표시 |
| `__init__.py` | Python package 선언 | `pipe_inspect_demo` 모듈 import 지원 |

### 4.3 ROS 실행 명령

| 명령 | 진입 함수 |
|---|---|
| `ros2 run pipe_inspect_demo pipe_vision` | `pipe_vision_node.main` |
| `ros2 run pipe_inspect_demo pipe_report` | `pipe_report_node.main` |
| `ros2 run pipe_inspect_demo pipe_coordinator` | `pipe_coordinator_node.main` |
| `ros2 run pipe_inspect_demo yolo_viewer` | `yolo_debug_viewer.main` |

## 5. ROS 토픽

### 5.1 Isaac Sim 발행

| 토픽 | 메시지 | 내용 |
|---|---|---|
| `/rgb/compressed` | `sensor_msgs/CompressedImage` | JPEG 압축 RGB 영상 |
| `/depth/compressed` | `sensor_msgs/CompressedImage` | mm 단위 16UC1 PNG Depth |
| `/camera_info` | `sensor_msgs/CameraInfo` | 640×640 카메라 내부 파라미터 |
| `/odom` | `nav_msgs/Odometry` | 파이프 입구 기준 카메라 상대 위치와 회전 |
| `/imu` | `sensor_msgs/Imu` | 카메라 월드 기준 자세 |

### 5.2 검출 노드 발행

| 토픽 | 메시지 | 내용 |
|---|---|---|
| `/defect/report_json` | `std_msgs/String` | 확정 또는 유효하게 갱신된 결함 JSON |
| `/defect/detected` | `std_msgs/Bool` | 현재 프레임의 유효 결함 검출 여부 |
| `/defect/debug_image/compressed` | `sensor_msgs/CompressedImage` | 마스크·박스·클래스·신뢰도가 표시된 JPEG 영상 |

영상 토픽은 Sensor Data QoS를 사용한다. Isaac 발행 측의 영상 QoS 큐는 1이며 최신 프레임 중심의 실시간 처리를 목적으로 한다.

## 6. 결함 검출과 등록 기준

| 항목 | 현재 값 |
|---|---|
| YOLO 추론 최소 신뢰도 | 0.05 |
| 결함 등록 최소 신뢰도 | 0.8 |
| 신규 결함 확정 | 서로 다른 연속 3프레임 |
| 연속 프레임 최대 간격 | 0.5초 |
| 동일 결함 축 방향 허용 거리 | 0.10m |
| 동일 결함 원주 방향 허용 각도 | 45° |
| 기존 결함 갱신 | 저장 신뢰도보다 0.02 이상 상승 |

같은 프레임에서 여러 마스크가 검출돼도 연속 검출 횟수는 한 번만 증가한다. 확정 전 후보는 JSON으로 발행하지 않으며, 확정 후에는 `new`, 의미 있는 신뢰도 상승 시에는 `updated` 상태로 발행한다.

## 7. 위치 계산

현재 축 방향은 파이프의 +Y축을 기준으로 한다.

```text
입구 기준 결함 축 위치 = 카메라의 입구 기준 Y 변위 + 결함 Depth
```

카메라 내부 파라미터와 마스크 중심 픽셀로 카메라 기준 X·Y 위치를 계산하고, IMU Roll을 반영하여 결함의 원주 방향을 0~360°와 1~12시 방향으로 기록한다. `travel_distance_from_entry_m`는 Odom 위치 변화량을 누적한 값이며 실제 로봇 단계에서는 바퀴 엔코더 기반 거리로 교체할 예정이다.

## 8. JSON 주요 구조

현재 `pipe_vision`이 발행하는 JSON 스키마 버전은 `1.5`다. RGB·Depth·CameraInfo·Odom·IMU를 같은 촬영 시점으로 동기화하고, 입구 기준 상대 quaternion으로 카메라 광선을 회전하여 수리 목표 좌표를 계산한다. Seg 마스크는 Skeleton 중심선으로 축소하며 3차원 최장 경로를 길이로, 중심선에서 경계까지의 국부 거리 중앙값을 폭으로 측정한다. 같은 결함의 여러 관측은 길이 상위값과 폭 중앙값으로 융합한다.

```text
schema_version
event
timestamp
frame_id
defect_id
class
confidence
registration
robot
├── travel_distance_from_entry_m
├── displacement_from_entry_m
├── position_relative_to_entry_m
├── orientation_xyzw
├── roll_deg
├── pitch_deg
├── yaw_deg
└── orientation_source
observation_pose
├── frame_id
├── camera_position_xyz_m
├── camera_orientation_xyzw
├── roll_deg
├── pitch_deg
└── yaw_deg
defect
├── camera_depth_m
├── axial_position_from_entry_m
├── camera_position_m
├── pipe_position_from_entry_m
├── clock_angle_deg
└── clock_hour
repair_target
├── frame_id
├── position_xyz_m
├── axial_position_m
├── clock_angle_deg
├── clock_hour
├── wall_radius_m
├── approach_direction_xyz
└── pose_transform_valid
measurement
├── length_mm
├── width_mm
├── physical_size_valid
├── method
├── valid_depth_points
├── valid_depth_ratio
├── skeleton_points
├── skeleton_components
├── longest_component_points
├── touches_image_border
├── observation_count
├── length_range_mm
├── width_range_mm
└── quality
segmentation
├── center_pixel
├── area_px
└── bbox_xyxy
sensor_status
├── rgb
├── depth
├── camera_info
├── odom
├── imu
└── capture_synchronized
```

치수 측정은 최소 20개의 유효 Depth 점과 50% 이상의 마스크 Depth 유효 비율을 요구한다. 화면 가장자리에 잘린 마스크는 치수 융합에서 제외한다. 동일 결함의 유효 관측이 4회 이하면 최대 길이, 5회 이상이면 길이 상위 90백분위를 대표 길이로 사용하며 대표 폭은 중앙값으로 계산한다. 신뢰도 갱신과 치수 갱신은 별도로 판단한다. `repair_decision.py`는 설계문서의 용접봉 기준을 구현했지만 아직 ROS JSON 발행·저장 흐름에는 연결하지 않았다. 크랙 홈 깊이나 구멍 지름·관 두께가 없으면 카메라 거리를 대신 사용하지 않고 추가 측정을 요구한다.

## 9. 통합 테스트 실행 방법

### 9.1 최초 빌드 또는 ROS 코드 변경 후 재빌드

```bash
cd ~/cobot3_ws
colcon build --packages-select pipe_inspect_demo
source ~/cobot3_ws/install/setup.bash
```

각 ROS 터미널은 실행 전에 `source ~/cobot3_ws/install/setup.bash`를 적용한다. Isaac Sim 터미널은 Isaac Sim에 포함된 Python과 ROS 2 Bridge를 사용하므로 `/opt/ros/humble/setup.bash`를 source하지 않는다.

### 9.2 터미널 1: Isaac Sim 로봇·카메라·파이프 통합 환경

```bash
cd ~/cobot3_ws/rokey_cobot/isaacpjt/dongyeon/pipe_inspect_test
isaac_python robot_pipe_integration_test.py
```

이 standalone은 파이프와 벨로우즈 로봇을 불러오고 RGB·Depth·CameraInfo·Odom·IMU를 발행한다. 로봇은 결함이 3프레임 연속 검출되면 정지하며 카메라가 결함 중심으로 정렬된다. Isaac Sim의 Stop 후 Play를 누르면 로봇 자세, 이동 거리, 결함 추적 상태가 초기화된다.

### 9.3 터미널 2: YOLO Seg 결함 검출

```bash
source ~/cobot3_ws/install/setup.bash
ros2 run pipe_inspect_demo pipe_vision
```

다른 모델을 임시로 시험할 때는 설치된 기본 모델을 교체하지 않고 절대경로를 전달한다.

```bash
ros2 run pipe_inspect_demo pipe_vision --ros-args -p model_path:=/절대경로/best.pt
```

### 9.4 터미널 3: 실시간 검출 영상

```bash
source ~/cobot3_ws/install/setup.bash
ros2 run pipe_inspect_demo yolo_viewer
```

### 9.5 터미널 4: 결함 보고 저장

```bash
source ~/cobot3_ws/install/setup.bash
ros2 run pipe_inspect_demo pipe_report
```

결과는 실행할 때마다 다음 구조의 새 폴더에 저장된다.

```text
rokey_cobot/src/dongyeon/pipe_inspect_demo/output/mission_날짜_시간_밀리초/
├── defect_events.jsonl
└── defect_summary.json
```

`defect_events.jsonl`은 수신한 모든 `new`, `updated`, `measurement_updated` 보고를 순서대로 보존한다. `defect_summary.json`은 결함 ID별 최신 보고와 최고 신뢰도 보고, 임무 상태, 이벤트·결함 수를 유지하며 `Ctrl+C` 종료 시 임무를 `completed`로 기록한다. 출력 위치는 `output_root` ROS parameter로 변경할 수 있다.

`defect_summary.json`의 `repair_plan`은 결함별 용접봉 소요량 합계, 15% 여유를 더한 권장 초기 장착량, 코일 한계 1,880mm 초과 여부를 기록한다. 치수가 부족한 결함이 있으면 `planning_complete=false`와 해당 ID를 기록하고 권장 장착량을 확정하지 않는다.

```bash
ros2 run pipe_inspect_demo pipe_report --ros-args -p output_root:=/절대경로/output
```

### 9.6 터미널 5: 수리 직전 최종 판정

```bash
source ~/cobot3_ws/install/setup.bash
ros2 run pipe_inspect_demo pipe_coordinator
```

수리로봇은 `/repair_robot/welding_rod_status`에 `remaining_length_mm`와 `sensor_valid`를 포함한 JSON을 발행한다. 수리 대상은 `/repair/request_json`에 `defect_id`로 요청하며 결과는 `/repair/decision_json`으로 발행된다. 실제 잔량은 coordinator가 임의 차감하지 않고 수리로봇의 다음 상태 메시지로 갱신한다.

### 9.7 결함 정지 후 재출발

카메라 정렬과 결과 확인을 마친 뒤 다음 명령으로 로봇을 다시 출발시킨다.

```bash
source ~/cobot3_ws/install/setup.bash
ros2 topic pub --once /inspection/resume std_msgs/msg/Bool '{data: true}'
```

### 9.8 결과 토픽 확인

일반 검출·치수 융합 JSON은 다음 명령으로 확인한다.

```bash
source ~/cobot3_ws/install/setup.bash
ros2 topic echo /defect/report_json --field data --full-length
```

카메라 정렬 완료 후 한 번 발행되는 최종 JSON은 다음 명령으로 확인한다.

```bash
ros2 topic echo /defect/aligned_report_json --field data --full-length
```

수리로봇에 전달되는 파이프 입구 상대 목표 Pose는 다음 명령으로 확인한다.

```bash
ros2 topic echo /repair/target_pose --full-length
```

최종 정렬 JSON의 스키마는 `1.7`이다. `frame_id=pipe_entry`와 `observation_pose`, `defect_pose`, `repair_target.navigation_goal_xyz_m`, `repair_target.orientation_xyzw`를 확인한다. 내벽 좌표가 유효하면 `repair_target.wall_radius_m`이 설계 반지름 `0.05m` 부근이고 `pose_transform_valid=true`, `pose_transform_reason=valid`가 된다.

### 9.9 토픽 연결 상태 확인

영상이나 JSON이 나오지 않으면 다음 명령으로 발행 주기와 연결 수를 확인한다.

```bash
ros2 topic hz /rgb/compressed
ros2 topic hz /depth/compressed
ros2 topic info /rgb/compressed --verbose
ros2 topic info /defect/report_json --verbose
```

권장 실행 순서는 `robot_pipe_integration_test.py` → `pipe_vision` → `yolo_viewer` → `pipe_report` → `pipe_coordinator`다. 저장과 화면 확인이 필요 없으면 `yolo_viewer`, `pipe_report`, `pipe_coordinator`는 생략할 수 있다.

## 10. 모델 학습과 교체

학습 도구와 데이터셋은 Isaac 작업 폴더에 유지한다.

```text
train_yolo_seg.py
yolo_seg_augmentation.py
split_yolo_seg_dataset.py
train_yolo_seg_colab.ipynb
simul_dataset_v3/
```

추가학습 실행 예시는 다음과 같다.

```bash
cd ~/cobot3_ws/rokey_cobot/isaacpjt/dongyeon/pipe_inspect_test
python3 train_yolo_seg.py --name pipe_crack_seg_v3 --no-augmentation
```

검증된 `best.pt`를 다음 기본 모델 위치에 복사한 후 패키지를 다시 빌드한다.

```text
rokey_cobot/src/dongyeon/pipe_inspect_demo/resource/yolov8n_seg_best.pt
```

```bash
cd ~/cobot3_ws
colcon build --packages-select pipe_inspect_demo
source ~/cobot3_ws/install/setup.bash
```

기본 모델 교체 전에는 `model_path` ROS parameter로 직접 시험할 수 있다.

```bash
ros2 run pipe_inspect_demo pipe_vision --ros-args -p model_path:=/절대경로/best.pt
```

## 11. Archive와 백업

과거 진단·환경·출력 파일은 다음 위치에 보관한다.

```text
rokey_cobot/isaacpjt/dongyeon/pipe_inspect_test/archive/
rokey_cobot/src/dongyeon/pipe_inspect_demo/archive/
```

1차 프로토타입 실행 필수 파일의 외부 백업은 다음 위치에 있다.

```text
/home/rokey/백업폴더/isaacpjt_inspection_backup
/home/rokey/백업폴더/pipe_inspection_backup
```

## 12. 현재 제한 사항

- 실제 RealSense가 아니라 Isaac Sim RGB·Depth 카메라를 사용한다.
- 실제 바퀴 엔코더 대신 카메라 Odom을 이동 신호로 사용한다.
- 파이프가 +Y 직선이라는 가정으로 축 위치를 계산한다.
- 결함 클래스는 `crack` 하나만 지원한다.
- 크랙 길이·폭 정확도는 YOLO 마스크 경계 품질과 Depth 해상도에 영향을 받으므로 CAD 정답 치수와 반복 비교가 필요하다.
- 보고 저장 노드가 비정상 종료되면 요약의 상태가 `running`으로 남지만, 종료 전까지 받은 JSONL 이벤트와 최신 요약은 보존된다.
- 설계문서의 중복 기준 50mm·30° 대신 프로토타입 안정화를 위한 100mm·45°를 사용한다.
- 수리 가능 여부, 용접봉 예산, 관리자 보고, 임무 FSM은 아직 구현하지 않았다.
