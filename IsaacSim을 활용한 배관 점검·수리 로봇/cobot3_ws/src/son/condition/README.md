# pipe_condition — 관 상태 판정 (주행 안전 정지)

전방 Depth 로 관 상태를 판정해 주행 제어에 즉시 알린다. 결함(크랙·구멍) 검출은
YOLO 가 별도 담당하며 이 모듈은 **주행 안전 판정 전용**이다.

| 상태 | 의미 | 대응 |
|---|---|---|
| `NORMAL` | 정상 직관 또는 곡관 | 계속 주행 |
| `MISALIGNMENT` | 조인트 축 어긋남 | 오프셋 등급별 |
| `DISCONNECTED` | 관 개방·단절 | 즉시 정지·복귀 |
| `UNDETERMINED` | 판정 불가 | 즉시 정지·복귀 |

## 파일

| 파일 | 역할 | 실행 환경 |
|---|---|---|
| `detector.py` | 판정 로직 (ROS 비의존) | 일반 python3 |
| `pipe_condition_node.py` | ROS 2 노드 | rclpy |
| `depth_probe.py` | Depth 반환 방식 확인 진단 | **Isaac Sim 연결 필요** |
| `config/pipe_condition.yaml` | 임계값 전부 | — |

시험 장면 합성·전수 시험·모의 발행기는 전부 **`../test_code/pipe_condition/`** 에 있다.

## 실행

```bash
# 오프라인 시험
cd ../test_code/pipe_condition
python3 make_scenes.py --png && python3 test_detector.py

# ROS 연동 시험 (터미널 2개)
python3 ../../pipe_condition/pipe_condition_node.py
python3 mock_publisher.py --dwell 12
```

Isaac Sim 연결 후 **가장 먼저**:
```bash
python3 depth_probe.py --topic /scout/front/depth
```

## 판별 원리

**Disconnected 와 Misalignment 의 구분은 Depth 반사 유무 하나로 한다.**

- 어긋난 조인트 → 먼 관 끝면이 초승달로 노출 → **전 영역 Depth 유효**
- 관이 끊김 → 그 너머는 빈 공간 → **반사 없음, Depth 무효**

판정 순서
```
1. Depth 무효 비율 > 임계        → DISCONNECTED
2. 개구부 원형도 < 임계           → UNDETERMINED
   개구부 윤곽 거칠기 > 임계      → UNDETERMINED
3. 원 피팅 중심 이탈 → 오프셋(mm)
     ≤1mm      NORMAL
     1~3mm     MISALIGNMENT, 정상 속도
     3~8mm     MISALIGNMENT, 저속 + 우선 보고
     >8mm      통행 불가, 정지·복귀
```

곡관 예외 — 관절 엔코더가 임계 이상이면 위를 전부 무시하고 `NORMAL`.
곡관에서는 시야에 벽이 들어와 원형도가 정상적으로 떨어지기 때문.

## 검증 결과 (2026-08-03)

**합성 장면 11종 전수 통과. 오프셋 실측 오차 최대 0.10mm.**
ROS 파이프라인(`mock_publisher` → 토픽 → 노드 → `/condition`)도 실제로 확인했다.

장면 구성·임계값 분리도·수치는 **`../test_code/README.md`** 참조.

## 사양서에서 바뀐 것

### 1. 원형도만으로는 UNDETERMINED 를 못 가른다 — 지표 추가

원형도 `4πA/P²` 는 둘레에 좌우돼 **완만한 요철에 둔감하다.** 약하게 불규칙한
개구부(`undetermined_mild`)가 0.896 으로 정상군(0.887~0.902) 안에 들어와
**임계값을 어떻게 잡아도 못 가른다.**

윤곽 반경의 상대 표준편차(`roughness`)는 같은 요철을 훨씬 민감하게 잡는다.
원형도는 심한 경우용 보조로 남기고 이 지표를 실제 판별자로 쓴다.

**단 간극이 0.0017 로 얇다.** 합성 데이터 기준이므로 실기에서 재조정 필요.

### 2. 오프셋 역투영 거리를 조인트 평면으로 바꿈

픽셀→mm 환산에 화면 중앙 깊이를 쓰면 **오프셋이 1.5배 과대평가된다**
(2mm→3.04, 5mm→7.60, 12mm→18.49). 중앙 깊이는 먼 관 안쪽까지의 거리이지
개구부가 놓인 조인트 평면까지의 거리가 아니기 때문이다.

개구부 테두리 바깥 띠의 깊이 중앙값을 쓰도록 고쳤다. 오차 0.10mm 로 떨어졌다.

## 미검증 사항

- **`depth_probe.py` 는 실행 못 함.** Isaac Sim 이 필요하다. 이 확인 전에는
  `invalid_mode` 가 맞는지 알 수 없고, 틀리면 **DISCONNECTED 가 통째로 작동
  하지 않는다**
- 검증은 **전부 합성 장면 기준**이다. 실제 강관 내벽의 반사·조명 불균일·
  RealSense 최소 측정 거리 문제는 반영되어 있지 않다
- 근접 벽면(25mm)이 무효로 나오면 무효율이 상시 높아져 **DISCONNECTED 오탐**이
  발생한다. `depth_probe.py` 가 이것도 같이 본다
- `pipe_msgs/PipeCondition` 이 아직 없어 **JSON 문자열로 발행**한다.
  인터페이스가 생기면 노드의 `USE_MSG` 분기가 자동으로 그쪽을 쓴다

## 구현 주의

- **이미지 QoS 는 BEST_EFFORT.** Isaac Sim 이 기본 BEST_EFFORT 라 구독자가
  RELIABLE 이면 토픽이 아예 수신되지 않는다
- **headless 에서는 카메라 프레임이 발행되지 않는다.** `ros2 topic list` 에
  보여도 프레임 0 이다. 카메라 검증은 GUI 로 할 것
- 이 노드의 발행은 **조율 노드를 거치지 않고 주행 제어로 직행**한다.
  정지 판단 지연을 막기 위한 설계상의 유일한 예외다
- 이 모듈이 실패해도 **전방 서스펜션 스트로크 상한 도달 시 즉시 정지**하는
  별도 안전장치가 있어야 한다 (이 모듈 밖, 미구현)

## 남은 작업

- Isaac Sim 연결 후 `depth_probe.py` 실행 → `invalid_mode` 확정
- 실기 정상 주행 데이터로 `invalid_ratio_max`, `roughness_max` 재조정
- `pipe_msgs/PipeCondition` 인터페이스 생성 (팀 공동)
- 서스펜션 상한 정지 안전장치 (별도 모듈)
