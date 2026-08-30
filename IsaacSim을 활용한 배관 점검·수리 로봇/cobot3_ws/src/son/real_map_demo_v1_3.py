#!/usr/bin/env python3
"""[Isaac 3.11] **v1_3 통합 완성본** — `restroom_final0807` 화장실 배관망.

v1_2(검증 본체)를 승계한 통합판이다 (2026-08-11 사용자 확정 설계):
  ✅ 로봇 = **pipe_robot_v11_weld 단일** (v10 동일 규격 실측 확인 + 용접 모듈.
     토치 스트로크 35mm·용접부 질량 15g 은 코드 덮어쓰기 — CAD 재요청 불가)
  ✅ 맵 = **원본 restroom_final0807 단일**, 건물 1회 고정(floor1=바닥 z0,
     floor2=+2490.2mm 윗층) — 모드 3종 `--course floor1 | floor2 | both`
  ✅ 검증 레시피(t50 T 정복·t91 되감기·floor2 완주)를 **기본값으로 내장**
  ⬜ 용접 계층(동연 ActiveCamBridge+finders, 로봇별 네임스페이스) — 이식 중
  ⬜ 웹 연결(동민 ros_bridge, 활성 카메라 1대만 발행) — 이식 중

━━ 맵 실측 (restroom_final0807.usd, map mm, upAxis Z, metersPerUnit 0.001) ━━
  진입   샤워 배수구 사각 거름망 (330,850) — **구 맵과 같은 진입점**
  floor2 z_net −250. (330,850)→(680,850)→(680,1400)→(1200,1400)→(1200,600)
         →(1500,600) 막다른 끝. 총 2,533mm.
         🔑 **구 맵의 ø90 병목이 없어졌다** — 실측 내반경 49.1mm 전 구간 균일
            (구 맵은 s 960~1660 이 45mm 라 우회로 없는 병목이었다).
  floor1 z_net −2740.2. **닫힌 루프다**(0807 개정판의 핵심 — 사용자 확인:
         *"한바퀴 순회전 하도록 했어"*):
             진입 (330,850) →+X→ ★T (730,850)★
             ├ −Y 팔 → (730,100) →+X→ (1300,100) →+Y→ 합류(1300,750)
             └ +Y 팔 → (730,1400) →−X← (1300,1400) ←−Y← 합류(1300,750)
         두 팔이 x=1300 에서 만나 **한 바퀴가 닫힌다.**
         🎯 **임무 규칙(오른손 법칙)이 그대로 성립한다**: +X 로 진행하며 T 를
            만나면 오른쪽 = travel×up = (+X)×(+Z) = **−Y 팔**로 나가고, 루프를
            한 바퀴 돌아 **+Y 팔에서 T 로 복귀**해 진입 라이저로 나온다.
         🔑 T 실측(730,850): 정면(+X) 50mm 벽, −Y 731mm·+Y 531mm 열림 =
            **정면 T**(양옆 개구) — 오늘 tee_go 로 검증한 바로 그 장면이다.

🚨 맵 usd 는 `metersPerUnit 0.001` 인데 스테이지는 1.0(m) 이다 — 참조 Xform 에
   scale 0.001 을 명시한다. 안 하면 건물이 2.5km 로 들어온다.
🚨 맵 usd 를 Isaac GUI 에서 **저장(Ctrl+S)하지 말 것** — 형상 없는 껍데기가
   원본을 덮어쓴다(기록된 사고).
🚨 활성 층 배관만 콜라이더를 준다 — 다른 층·건물 셸은 렌더만. 삼각형을 줄이고
   무엇보다 다른 층 관을 잘못 잡는 것을 막는다.

실행:
  isaac_python real_map_demo_v1_3.py --headless                # both(기본)
  DISPLAY=:1 isaac_python real_map_demo_v1_3.py --glass --hold
  isaac_python real_map_demo_v1_3.py --course floor1 --headless
옵션·노브는 v1_2 와 같다 (SPEED_MPS·FAIL_S·GUI_EVERY·CTL_DEBUG…).
"""

import json
import math
import os
import struct
import sys
import time
from pathlib import Path

# 기동 시간 계측 — "실행하고 나서 왜 한참 기다리나" 를 구간별로 가른다.
_T0 = time.time()


def tick(what):
    print(f"[기동 {time.time() - _T0:6.1f}s] {what}")
    sys.stdout.flush()

HEADLESS = "--headless" in sys.argv
HOLD = "--hold" in sys.argv
# (v1_3: 배관은 **항상** 유리다 — `--glass` 는 v1_2 잔재라 폐지.
#  받아도 무시되지 않게 인자 목록에서도 뺀다.)
# 🔁 코스를 **반대 방향으로** 통과시킨다 (분기 복귀 시험).
REVERSE = "--reverse" in sys.argv
# 🚨 **카메라는 기본으로 안 만든다** (2026-08-07). 이 연습장은 영상을 한 번도
#    읽지 않는다 — `Camera` 를 만들고 설정만 하고 `get_rgba` 조차 안 부른다.
#    그런데 로봇당 2대 × 640×360 렌더 프로덕트 + 리그 조명 2개가 GUI 에서
#    매 프레임 값을 치른다. 🔑 어제 벨로우즈로 돌릴 때 가벼웠던 이유가
#    이것이다 — 그 자산엔 `FrontBody`/`RearBody` 가 없어 **카메라가 0대로
#    조용히 건너뛰어졌다.** 자산을 바꾸니 6대가 생겼다.
#    카메라 시야를 봐야 할 때만 `--cam` 으로 켠다.
# 🎯 v1_3: **카메라는 기본 ON** (2026-08-11) — 용접 카메라와 웹 영상 발행이
#    이 빌드의 본체다. v1_2 는 영상을 안 써서 기본 OFF 였고, 그 기본값 탓에
#    관찰용 런에서 카메라가 한 대도 안 생겨 "카메라가 다 죽은" 것처럼
#    보였다(실측). 순수 주행 시험만 할 때 `--nocam` 으로 끈다.
CAMERAS = "--nocam" not in sys.argv
# 🎯 **검출 창** (`--detect`) — 전방 카메라 프레임에 센터링·관경 링·결함
#    검출을 그려 OpenCV 창으로 띄운다(2026-08-11 사용자 요청: YOLO 처럼
#    화면으로 직접 보고 싶다). 판정은 동연 `detect/finders.py`(= 네 원본
#    검출 함수의 모듈판)를 그대로 쓴다 — 화면용으로 새로 만들지 않는다.
DETECT_WIN = "--detect" in sys.argv
# 🎥 **관 밖 관찰 카메라** (`--obs`, 2026-08-11 사용자: 영상 촬영용).
#    로봇에 붙지 않는 **고정 카메라** 2대 — 단절 끝과 곡관을 밖에서 본다.
#    `--glass2` 와 함께 쓸 것(배관·건물이 비쳐야 관 안이 보인다).
# 뷰포트 카메라 좌표를 3초마다 로그에 찍는다(VIEW_LOG=0 으로 끔).
VIEW_LOG = os.environ.get("VIEW_LOG", "1") == "1"
# 결함 테두리가 이보다 가까워야 **진짜 벽면 구멍**이다(관 저 끝은 0.3~0.5m).
DETECT_NEAR_M = float(os.environ.get("DETECT_NEAR_M", 0.22))
# 단절 판정 문턱 — 무효 화소 비율 / 전방 거리(설계 문서 8.2·detector 규약)
DETECT_INV_MAX = float(os.environ.get("DETECT_INV_MAX", 0.02))
DETECT_STOP_M = float(os.environ.get("DETECT_STOP_M", 0.10))
# 기본은 **all** — 코스 3개에 로봇 3대를 동시에 굴린다.
COURSE = "all"
if "--course" in sys.argv:
    COURSE = sys.argv[sys.argv.index("--course") + 1]
STEPS = 9000
if "--steps" in sys.argv:
    STEPS = int(sys.argv[sys.argv.index("--steps") + 1])
# 🚨 이 파일은 argparse 를 안 쓴다(repair_demo 와 같은 방식). 그래서 오타난
#    플래그가 **조용히 무시된다** — `--know-defect` 를 줬는데 아무 일도 안
#    일어난 사고가 실제로 있었다. 모르는 `--` 인자는 여기서 잘라낸다.
_KNOWN = {"--headless", "--hold", "--course",
          "--steps", "--reverse", "--cam", "--nocam", "--glass2", "--detect"}
_bad = [a for i, a in enumerate(sys.argv[1:], 1)
        if a.startswith("--") and a not in _KNOWN]
if _bad:
    raise SystemExit(f"[중단] 모르는 인자 {_bad} — 쓸 수 있는 것: "
                     f"{sorted(_KNOWN)}")

# ── 🎯 v1_3 검증 레시피 내장 (2026-08-11) ───────────────────────────
# v1_2 는 이 값들을 매 실행 env 로 줘야 했다(잊으면 검증 안 된 조합으로 돈다).
# v1_3 은 **마지막까지 검증된 floor1/floor2 레시피를 기본값으로 굽는다** —
# t50 나가는 T 정복 + t91 되감기 최심 + floor2 완주 조합 그대로.
# env 를 주면 여전히 이긴다(setdefault) — 실험은 가능하되 기본이 정답.
for _k, _v in {
        "NAV": "blueprint",          # 정답지 주행 (자율은 NAV=vision)
        # T 레시피는 **floor1 전용** — 코스별 키로만 준다(위 회귀 참조)
        "BP_TEE_BEND": "70", "BP_TEE_ARCS_floor1": "1,6",
        "BP_TEE_V": "0.45", "BP_CORNER_F": "0.45",
        "BP_TEE_CALM_R": "0.22", "BP_TEE_SWING": "0.12",
        "BP_SCHED_ESC": "0", "BP_SCHED_REAR": "0",
        "BP_SCHED_LAG2": "0.05", "BP_SCHED_LAG": "0.02",
        "STUCK_ARC_S": "25", "BP_BEND_MF": "15", "BP_ROLL_MF": "8",
        "WHEEL_MAXFORCE": "0.25",
        # 되감기 테이프 — t88 녹화본을 저장소 자산으로 승격 (5609행)
        "BP_TAPE": str(Path(__file__).resolve().parent
                       / "floor1_tee_tape.txt"),
        }.items():
    os.environ.setdefault(_k, _v)

from isaacsim import SimulationApp                        # noqa: E402

simulation_app = SimulationApp({"headless": HEADLESS})
tick("SimulationApp 기동 완료")

import numpy as np                                        # noqa: E402
from isaacsim.core.api import World                       # noqa: E402
from isaacsim.core.prims import SingleArticulation        # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import (Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux,  # noqa: E402
                 UsdPhysics, UsdShade, Vt)

MM = 0.001
PHYSICS_HZ = float(os.environ.get("PHYSICS_HZ", 240))
PHYSICS_DT = 1.0 / PHYSICS_HZ
SON = Path(__file__).resolve().parent
# 🚨 이 usda 는 **씬 파일**이다 — `/World` 밑에 PhysicsScene·Looks 까지 들어
#    있다. 통째로 참조하면 물리 씬이 둘이 된다 → **`/World/Robot` 서브트리만**
#    참조한다(AddReference 의 두 번째 인자).
# 🚨 앞 세대(벨로우즈 12륜)와 **구조가 다르다.** 옮겨 쓸 때 걸린 것들:
#      관 축   로컬 **X** (벨로우즈는 Z) → 배치 회전이 필요 없다
#      휠      반경 **8mm** (벨로우즈 10mm), 중심 반경 40mm, 12개
#      링크    RearBody/FrontBody + <바디>_<A|B><0..2>_{Arm,Wheel}
#      용접기  **내장** (WeldRing·WeldTorch + RingRotate·TorchExtend 조인트)
#      질량    2.04kg (벨로우즈 0.91kg)
# 🔑 전장 148mm(충돌체 136mm) — 진입 라이저(185mm)에 통째로 들어간다.
#    굽힘축 간격 ±30mm · 다리 한계 −15~+35mm · 롤 자유(드라이브 15/2/2) ·
#    굽힘 ±95°(25/2.5) · 다리 강성 3000/maxF 60 · 휠 드라이브 0.11.
#    WeldPad×3 · LensBoss · CamF/CamR 은 **충돌체 없음**(시각 전용)이라
#    물리 길이에 안 들어간다.
# 🎯 **v1_3 기준 로봇 = `pipe_robot_v11_weld` 단일** (2026-08-11 사용자 확정).
#    v10 과 주행부 완전 동일(조인트 28개 수치 차이 0건 — usda 전수 대조 실측)
#    + 용접 모듈(WeldRing 66g·RingRotate ±자유 / WeldTorch 20g·TorchExtend).
#    자산과 다른 두 가지는 로드 후 코드로 덮어쓴다(CAD 재요청 불가 확정):
#      ① TorchExtend 상한 5mm → **35mm**(설계 J2 — 관벽 반경 42~50 도달)
#      ② 질량 WeldRing 66→10g / WeldTorch 20→5g (+86g → **+15g**, 질량비
#         유지로 물리 안정 — 완전 0 은 관절 질량비 폭주로 금지)
ROBOT_USDA = os.environ.get(
    "ROBOT_USD", str(SON / "robot" / "pipe_robot_v11_weld.usda"))
ROBOT_PRIM_IN_USDA = os.environ.get("ROBOT_PRIM", "")
MAPS = SON / "maps"
ASSET = Path(ROBOT_USDA).stem

# 🔑 **자산별 설정값** — 로직은 하나, 숫자만 다르다.
#    두 로봇의 다리·휠 규격이 달라 같은 숫자를 쓰면 뜻이 달라진다:
#      벨로우즈  휠 r10 · 다리 예압 9N  · 휠 중심 33.25+스트로크 · 질량 0.91kg
#      용접기v2  휠 r8  · 다리 예압 18N · 휠 중심 40.0+스트로크  · 질량 2.04kg
#    🚨 **다리 개수가 다르다** (실측): 벨로우즈는 다리 6 · 다리당 휠 2,
#       용접기 v2 는 **다리 12** · 다리당 휠 1 이다. 그래서 예압을 다리 하나
#       기준으로 같게 두면 총 수직항력이 두 배가 된다.
#    🔑 **견인/마찰 비를 맞춘 값**이라 주행 거동이 대등하다:
#      벨로우즈 견인 12×0.05/0.010 = 60N  / 마찰 0.4× 6×9 =  21.6N → 2.8배
#      용접기v2 견인 12×0.08/0.008 = 120N / 마찰 0.4×12×9 =  43.2N → 2.8배
#    🚨 자산 기본 maxForce 60N 을 그대로 쓰면 안 된다. 자산이 60 인 것은
#       한계(6mm)에서 3000×0.006 = 18N 으로 **어차피 잘리기** 때문인데,
#       스트로크를 35mm 로 늘리는 순간 그 상한이 진짜로 걸린다 →
#       12다리×18N 이면 마찰 86.4N 대 견인 120N 으로 여유가 1.39배뿐이다.
TUNING = {
    # 🎯 **2026-08-08 CAD 수정본 2종 — 전장 126mm** (카메라 부착 시 140mm =
    #    벨로우즈 v1 과 같은 길이라 **샤워실 배수구 진입이 가능해진다**).
    #    실측 제원(둘 다): 휠 r8·중심반경 40mm, 다리 축 X 한계 −4~+6mm
    #    (강성 3000·maxF 60), 중앙 D6 4개, 콜라이더 Cylinder+Cone.
    #    → 용접기 v2 와 **같은 계열**이라 설정값을 그대로 물려받는다.
    #    다른 점은 **다리·휠 개수와 질량**뿐이고, 견인/마찰 비는 둘 다 2.8배로
    #    같아서 예압 9N 이 그대로 성립한다:
    #      126   : 다리 12(앞6뒤6) · 2,040g · 견인 12×0.08/0.008 = 120N
    #                                        마찰 0.4×12×9 = 43.2N → 2.8배
    #      short : 다리  9(앞3뒤6) · 1,830g · 견인  9×0.08/0.008 =  90N
    #                                        마찰 0.4× 9×9 = 32.4N → 2.8배
    #    ⚠ short 는 **앞 세그먼트가 3다리**라 중심 유지의 최소 구성이다
    #      (3점 지지). 접합부에서 한 다리가 개구에 빠지면 남는 것이 2개뿐이라
    #      앞이 흔들릴 수 있다 — 비교 시험의 핵심 관전점.
    # 🎯 **v10** (2026-08-10 CAD 수정본 — `CAD_요청_로봇_v10.md` 반영 확인).
    #    구조·이름은 v9 그대로(DiscR─RollR+BendR─Body─RollF+BendF─DiscF),
    #    치수만 맵에 맞췄다. 반입 시 자산 실측(usda 파싱):
    #      전장     148mm(LensBoss 포함) / 충돌체 136mm  ← 라이저 185 여유 49mm
    #      굽힘축   ±30mm(중앙 강체 60mm — R150 처짐 7.7→3.0mm)
    #      다리     한계 −15~+35mm **자산 내장** (v9 는 −4~+3.7 을 런타임 덮어씀)
    #      관절     롤 자유(15/2/2)·굽힘 ±95(25/2.5)·다리 3000/60·휠 0.11
    #               — v9 와 완전 동일(견적서 "관절 불변" 요청 이행 확인)
    #      신규     WeldPad×3·LensBoss·CamF/CamR — 충돌체 없음(시각 전용)
    "pipe_robot_v10": dict(
        wheel_r=0.008, wheel_maxf=0.11,      # 자산값 (v9 와 동일)
        # 🔑 스트로크 = 자산 한계 그대로 **35mm** — v9 에서 35 가 위험했던
        #    관통(33mm 돌출)은 LEG_BORE_CAP(관경 추적 상한, DN100 에서 6~12mm
        #    자동 제한)이 막는 것을 실측 확인했고, DN150(벽 67mm) 도달에는
        #    40+35+8=83mm 가 필요하다(견적서 ② 근거). 문제가 보이면 첫 대조는
        #    PISTON_STROKE=0.015 (v9 채택값 — 이탈 5.8mm 역대 최저 기록).
        piston_stroke=0.035, piston_init=0.001, piston_maxf=9.0,
        piston_retract=0.015,                # 자산 한계와 동일(덮어써도 무해)
        center_delta=0.003,
        bel_stiff=25.0, bel_maxf=2.5,        # 자산값 유지 — 올리면 악화(v9 실측)
        wheel_center_mm=40.0,                # 다리 밑동 y=0.04 유지 확인
        seg_rear="DiscR", seg_front="DiscF", seg_mid="Body",
        cam_front="DiscF", cam_rear="DiscR",
        steer_mode="rollbend",
        roll_joints=("RollF", "RollR"), bend_joints=("BendF", "BendR"),
        drum="SphF",
        # 🔑 렌즈 위치 = 자산 LensBoss 앞면(디스크 원점 +11+3 = 14mm).
        #    디스크 앞면(+8)보다 6mm 돌출 — 견적서 ④ "+10mm 이내" 충족.
        #    보스는 충돌체가 없어 물리에는 안 걸린다.
        cam_x=0.014,
        steer=None),
    # 🎯 **v11_weld** (2026-08-11, v1_3 기준) — v10 완전 승계 + 용접 모듈.
    #    주행 설정값은 v10 사전을 그대로 복사한다(조인트 동일 실측 근거).
    "pipe_robot_v11_weld": dict(
        wheel_r=0.008, wheel_maxf=0.11,
        # 🎯 스트로크 35 → **50mm** (2026-08-11): 거름망 하우징 사각 벽
        piston_stroke=0.035, piston_init=0.001, piston_maxf=9.0,
        piston_retract=0.015,
        center_delta=0.003,
        bel_stiff=25.0, bel_maxf=2.5,
        wheel_center_mm=40.0,
        seg_rear="DiscR", seg_front="DiscF", seg_mid="Body",
        cam_front="DiscF", cam_rear="DiscR",
        steer_mode="rollbend",
        roll_joints=("RollF", "RollR"), bend_joints=("BendF", "BendR"),
        drum="SphF",
        cam_x=0.014,
        steer=None,
        # 용접 모듈 (Body 에 링, 링에 토치 — repair_demo FSM 이 몬다)
        weld_ring_joint="RingRotate", weld_torch_joint="TorchExtend",
        weld_ring_link="WeldRing", weld_torch_link="WeldTorch",
        # 🚨 **행정 한계는 자산 기하가 정한다** (2026-08-11 실측, 사용자
        #    스크린샷: 용접봉이 관을 뚫었다). 토치 팁(Tip 메시)이 **정지
        #    상태에서 이미 축반경 41mm** 에 있다 — 설계문서의 "J2 35mm,
        #    반경 20→48" 은 이 자산과 다르다(자산 원본 5mm 가 오히려 근사).
        #      관벽 50 − 팁 41 = **가용 9mm**  → 하드 한계 10mm(1mm 여유),
        #      실제 정지는 아래 EXTEND 가 팁 반경 48mm(간극 2mm)로 잰다.
        torch_tip_r0=0.041, torch_stroke=0.010),
}
# (v1_2 의 구세대 프리셋 — pipe_robot_v9 · welder_126 · welder_short ·
#  robot_from_bot_welder_art_v2 · robot_v2_12wheel — 은 v1_3 대청소로 삭제.
#  필요하면 real_map_demo_v1_2.py 에 그대로 남아 있다.)
if ASSET not in TUNING:
    raise SystemExit(
        f"[중단] 모르는 로봇 자산 {ASSET!r} — TUNING 에 설정값을 먼저 적을 것 "
        f"(있는 것: {list(TUNING)})")
TUNE = TUNING[ASSET]

# 🔑 용접기는 **로봇 usda 안에 이미 있다** — `welder/torch_spec.py` 로 따로
#    조립하지 않는다(2026-08-06 사용자 지시). 그래서 parts_meta 도 안 읽는다.

PIPE_IR = 0.050
WHEEL_R = float(os.environ.get("WHEEL_RAD", TUNE["wheel_r"]))
SEG_GAP = 0.076

# 주행 속도 — **0.10 m/s.** repair_demo 설계값(0.05)의 2배다.
# 🔑 여기는 검출을 안 하므로 "카메라 10Hz 로 결함을 놓치지 않는" 제약이 없다.
#    수직곡관은 0.05 로는 토크를 올려도 못 올라간다(아래 실측표).
TARGET_SPEED_MPS = float(os.environ.get("SPEED_MPS", 0.10))
SPIN_DEG_S = math.degrees(TARGET_SPEED_MPS / WHEEL_R)
# 감지 밴드 — 한 스텝 이동이 이보다 크면 바퀴가 관벽을 뛰어넘는다.
# 물이 없으므로 CPU 물리다 → GPU 하한(0.02) 을 깔 이유가 없다.
# 🚨 이 로봇은 2.04kg 으로 앞 세대(0.98kg)의 두 배다. 감지 밴드가 좁으면
#    관벽을 뛰어넘어 빠져나간다 — 필요하면 CONTACT_OFF 로 넓힌다.
CONTACT_OFFSET = float(os.environ.get(
    "CONTACT_OFF", max(0.0005, 1.2 * TARGET_SPEED_MPS / PHYSICS_HZ)))
REST_OFFSET = 0.0

# ── 휠 토크 한계 — **수직곡관을 오르려면 올려야 한다** (2026-08-06 실측) ──
# v2 usda 기본값은 0.014175 N·m/휠 → 12개 합산 견인력 17.0N, 로봇 중량 9.56N.
# 여유가 1.8배뿐이라 마찰·벨로우즈 저항을 빼면 수직 구간에서 남는 게 없다.
#
#   토크        속도        수직곡관 결과
#   0.014175    0.05        끼임 90mm
#   0.014175    0.10        끼임 90mm      ← 속도만 올리면 소용없다
#   0.0425      0.05        **발산**(로봇이 8m 밖으로) ← 중간값에 불안정 구간
#   0.14        0.05        끼임 135mm     ← 더 가지만 못 넘는다
#   0.14        0.10        **끝까지 296mm ✅**
#
# 🔑 **usda 는 안 고친다.** 로봇 설계값(v2 원본)을 그대로 두고 이 연습장에서만
#    덮어쓴다 — 되돌리기 쉽고, 다른 시연이 조용히 영향받지 않는다.
# 🚨 0.0425 에서 솔버가 터진 적이 있다. 이 값을 손으로 바꿀 때는 중간값에서
#    발산할 수 있다는 것을 알고 볼 것.  WHEEL_MAXFORCE 로 조절.
# 🔑 **0.05 N·m — 수직·수평 곡관을 둘 다 넘는 값**(2026-08-06 실측).
#    원본 0.014175 는 수직에서 90mm 에 멈추고, 0.14~0.28 은 수평에서 막힌다.
#      토크        속도   수직곡관   수평곡관
#      0.014175    0.05   끼임 90    ✅
#      0.014175    0.10   끼임 90    ✅
#      0.03        0.10   ✅         (미시험)
#      0.05        0.10   ✅         ✅      ← 채택
#      0.14        0.10   ✅         ✅
#      0.28        0.10   ✅         ❌ 190mm
#      용접기 v2 는 자산 기본값 0.08 N·m 를 그대로 쓴다 — 휠이 r8 이라
#      12개 합산 견인력 120N 으로 벨로우즈(60N)와 견인/마찰 비가 같다.
WHEEL_MAXFORCE = float(os.environ.get("WHEEL_MAXFORCE", TUNE["wheel_maxf"]))

# ── 휠 드라이브 댐핑 — **수직관에서 굴러 내려가지 않으려면 필요하다** ──────
# 🚨 자산값 0.012 N·m·s/rad 로는 제동이 사실상 없다(2026-08-07 실측). 속도
#    드라이브의 토크는 `댐핑 × (지령 − 실제)` 라, 자중 20N 을 버티는 데 필요한
#    휠당 0.0133 N·m 를 내려면 **ω = 1.1 rad/s(64°/s)** 까지 굴러야 하고,
#    상한 0.08 N·m 에 닿으려면 380°/s(= 3 m/s) 가 필요하다.
#    → `--reverse` 로 수직 가지에서 출발시켰더니 안착 1.5초 만에 **635mm 굴러
#      내려갔다**(실속도/지령 212%). 0 을 주면 그대로 자산값을 쓴다.
WHEEL_DAMP = float(os.environ.get("WHEEL_DAMP", 0.0))

# 🎯 **중앙 관절 강성 1.0 N·m/도** (usda 원본 0.0698 은 너무 무르다).
#    분기 구멍 위에서 앞 세그먼트가 관절째 11~19° 꺾여 내려앉는 것을 막는다.
#    2.0 은 오히려 나쁘고 4.0 은 발산한다(실측).
# 🚨 용접기 v2 의 중앙 관절은 **D6(`PhysicsJoint`)** 라 revolute 가 아니다 —
#    `rotY`·`rotZ` 두 축에 각각 드라이브가 붙어 있고(자산 강성 0.2 / maxForce
#    1 / 한계 ±20°), 4개(J0·J1·J3·J_front)가 직렬이다.
# 🚨 **이 값의 실효 단위는 도(度)당이 아니라 라디안당이다** (2026-08-07 실측).
#    강성 2.0 에 오차 13.5° 면 "N·m/도" 라면 27 N·m 라 상한 24 에 걸려야 하는데
#    관절이 지령의 **27% 밖에** 안 따라왔다. 라디안당으로 보면 2.0×0.236 =
#    0.47 N·m 뿐이라 앞뒤가 맞는다. 조향에 필요한 실측 저항은 **약 3 N·m**
#    (다리 6개×9N 을 벽을 따라 미끄러뜨리는 힘 + 스프링 압축).
#      강성 2 / 상한 24  → 관절합 −16° (지령 −54°)  ❌ 이탈 80mm
#      강성 20 / 상한 12 → 관절합 −58°              이탈 최대 32.8mm
#      강성 40 / 상한 12 → 관절합 −61°              이탈 최대 27.6mm
#      강성 **60** / 상한 **15** → 관절합 −58°      이탈 최대 21.6mm ← 채택
BELLOWS_STIFF = float(os.environ.get("BELLOWS_STIFF", TUNE["bel_stiff"]))
BELLOWS_MAXF = float(os.environ.get("BELLOWS_MAXF", TUNE["bel_maxf"]))

# 🔑 **다리는 반경 방향 직동 슬라이더**다. 도달 반경을 늘리는 데 기구를 바꿀
#    필요가 없다 — 스트로크 상한만 올리면 된다.
#      10mm → 휠 중심 40.0mm (DN100 r50 에 딱)
#      35mm → 휠 중심 68.2mm (DN150 r75 에 필요한 65mm 확보)
# 🚨 예압은 목표를 스트로크 밖에 두는 힘 드라이브라, 스트로크를 늘리면 목표도
#    같이 밀어야 9N 이 유지된다(목표 = 스트로크 × 2).
PISTON_STROKE = float(os.environ.get("PISTON_STROKE", TUNE["piston_stroke"]))
# 🚨 **다리를 벽에 닿은 자세로 시작한다.** 정지에서 9N 으로 튀어나가면 한 스텝
#    이동이 감지폭(0.5mm)을 넘어 벽을 뚫는다. 스트로크 10mm 면 뚫어도 관 안에
#    남지만 35mm 면 관 밖까지 나가 통째로 발산했다(실측).
#    용접기 v2 는 다리가 다 들어간 자세에서 휠 중심 반경이 이미 40.0mm 라
#    관벽(50 − 휠반경 8 = 42mm)까지 2mm 뿐이다 → 초기 신장 2mm.
PISTON_INIT = float(os.environ.get("PISTON_INIT", TUNE["piston_init"]))
# 🚨 **예압은 maxForce 로 잘린다.** 목표(Δ)를 키워도 상한이 9N 이면 힘은 9N 그대로다.
#    설계 예압이 9N(피스톤 6개 → 수직항력 54N)이라 기본은 9. 더 밀어야 하면
#    이 값을 같이 올려야 한다.
PISTON_MAXF = float(os.environ.get("PISTON_MAXF", TUNE["piston_maxf"]))
# 🚨 **수축 쪽 한계는 따로다.** `PISTON_STROKE` 는 뻗는 쪽(상한)만 바꾸고
#    수축 한계는 자산값(용접기 v2 = −4mm)이 그대로였다. 접합부에서 앞
#    세그먼트가 13mm 밀리면 가까운 쪽은 13mm 를 접어야 하는데 −4mm 에서
#    스토퍼에 박혀 물린다(실측: 앞 다리 3개가 −4.0 에 바닥). 0 이면 자산값.
# 🎯 **−15mm 로 늘리니 T 분기를 정방향으로 통과한다**(끼임 0회, 2026-08-07).
#    −25mm 도 결과가 소수점까지 같으니 −15 면 충분하다. **자산 설계값으로
#    넘길 것** — usda 는 안 고치고 여기서만 덮어쓴다.
PISTON_RETRACT = float(os.environ.get(
    "PISTON_RETRACT", TUNE.get("piston_retract", 0.0)))

# ── 중심 유지 — 다리 신장을 서로 같게 맞춘다 ────────────────────────
# 🚨 9N **상수력**만으로는 어느 쪽으로 치우쳐도 힘이 같아 되돌리는 힘이 없다
#    (실측: 관경 변화 구간에서 평균 이탈 16.9mm). 이 로봇은 실린더가 밖으로
#    팽창하며 벽을 밀어 가는 구조라, **세 다리 신장이 같아야 중앙에 선다.**
#    → 세그먼트마다 평균 신장 + Δ 를 모두의 목표로 준다. 덜 뻗은 다리가 더
#      세게 밀어 몸체가 중앙으로 돌아온다. Δ×강성(3000) = 예압 9N.
CENTER_ON = os.environ.get("CENTER", "1") == "1"
CENTER_DELTA = float(os.environ.get("CENTER_DELTA", TUNE["center_delta"]))

# ── 다리별 접촉 판정 — **분기 개구 위의 다리를 따로 다룬다** ──────────
# 🔑 예압 목표를 스트로크 **밖**에 두고 있으므로 검출이 공짜다:
#      벽에 닿은 다리 → 스트로크 안에서 멈춘다
#      허공을 짚는 다리 → **상한까지 뻗는다**
#    상한 근처면 "벽 없음"이다.
# 🚨 벽 없는 다리를 평균에 넣으면 **그 값이 평균을 끌어올려 나머지 다리가
#    몸을 구멍 쪽으로 민다**(기록된 진단). → 평균에서 빼고, 그 다리만 접는다.
# 🚨 **위치로는 못 가른다** (2026-08-07 실측). `CENTER` 가 켜지면 다리 목표가
#    스트로크 밖의 예압값이 아니라 **평균+Δ** 로 바뀌므로, 벽 없는 다리도
#    상한(35mm)까지 안 가고 평균 근처에 멈춘다(실측 최대 신장 11mm). 위치
#    기준 판정기는 **한 번도 안 걸렸다**(뜬다리 0개).
#    → **관절이 실제로 내는 힘**으로 가른다. 벽을 밀면 힘이 실리고, 허공을
#      짚으면 0 에 가깝다. 임계는 예압 9N 의 20% 로 둔다.
# 🚨 **기본은 꺼 둔다 — 이 판정기는 아직 틀린다.** `CENTER` 가 위치 목표를
#    주는 구조에서는 힘도 접촉 지표가 못 된다: 목표에 **도달한** 다리는 벽에
#    닿았든 허공이든 힘이 0 에 가깝다. 실측에서 12개 중 **10개를 '벽 없음'
#    으로 오판**했다(실제로는 전부 −9.6~+0.2mm 로 눌려 있었다).
#    제대로 하려면 예압을 **다리별 힘 지령**으로 주고(위치 목표를 버리고)
#    거기서 위치로 접촉을 읽어야 한다 — 구조 변경이라 별도 작업.
# 🔑 **위치로 판정한다 — 스트로크 한계에 붙은 다리는 확실히 벽이 없다.**
#    `center_legs` 가 **평균+Δ** 를 목표로 주므로, 벽이 없으면
#    뻗음 → 평균 상승 → 목표 상승 → 더 뻗음 으로 **한계까지 래칫**된다
#    (실측: `tee_in` 접합부에서 35.0mm = 도달 75mm = **관 밖 20mm**).
#    한계에 붙었다는 것 자체가 "아무것도 안 막았다"는 뜻이다.
# 🚨 힘으로 판정하는 것은 **틀렸다**(실측 12개 중 10개 오판) — 목표에 도달한
#    다리는 벽에 닿았든 허공이든 힘이 0 에 가깝다. 위치가 맞는 지표다.
LEG_FREE_ON = os.environ.get("LEG_FREE", "1") == "1"
LEG_FREE_MARGIN = float(os.environ.get("LEG_FREE_MARGIN", 0.004))
LEG_FREE_CLEAR = float(os.environ.get("LEG_FREE_CLEAR", 0.010))
# 🚨 한계에 **닿아야** 판정하면 이미 늦다(최대신장 35.0mm = 관 밖 20mm 가
#    그대로 찍혔다). 같은 세그먼트 다리들의 **중앙값**을 관벽 추정치로 삼아,
#    거기서 이만큼 넘게 튀어나간 다리는 그 전에 벽이 없다고 본다.
#    중앙값은 6개 중 2개까지 튀어도 안 흔들린다.
LEG_FREE_GAP = float(os.environ.get("LEG_FREE_GAP", 0.008))
LEG_FREE_GAP_CLEAR = float(os.environ.get("LEG_FREE_GAP_CLEAR", 0.004))
# 벽 없는 다리의 목표 신장 — 접어서 개구 턱에 안 걸리게 한다
LEG_FOLD = float(os.environ.get("LEG_FOLD", 0.0))

# ── 🎯 다리를 **위치가 아니라 힘으로** 몬다 (2026-08-07 신설) ──────────
# 🚨 위치 목표(`center_legs` 의 평균+Δ)는 **자기 참조**라 벽이 없으면
#    뻗음 → 평균 상승 → 목표 상승 → 더 뻗음 으로 **한계까지 래칫**된다
#    (실측: 각진 T 양방향 모두 35.0mm = 도달 75mm = 관 밖 20mm).
# 🔑 힘으로 몰면 그 결합이 사라진다 — 허공을 짚는 다리가 뻗어도 **다른 다리의
#    목표를 끌어올리지 않는다.** 그리고 위치가 곧 접촉 정보가 된다(같은 힘을
#    줬는데 끝까지 나가면 벽이 없는 것).
#    · 기본 예압  F0 = 9N 바깥쪽 (설계값)
#    · 중심 유지  덜 뻗은 다리(=벽이 가까운 쪽)를 더 세게 민다: +Kc·(평균−q)
#    · 벽 없는 다리는 **안쪽으로** 당겨 접는다(개구 턱에 안 걸리게)
# 🚨 드라이브는 **순수 댐퍼**로 바꾼다(강성 0) — 안 그러면 위치 드라이브와
#    힘 지령이 서로 싸운다.
LEG_FORCE = os.environ.get("LEG_FORCE", "1") == "1"
LEG_KC = float(os.environ.get("LEG_KC", 300.0))     # N/m, 중심 복원
LEG_FOLD_N = float(os.environ.get("LEG_FOLD_N", 6.0))   # N, 뜬 다리 접는 힘
LEG_DAMP = float(os.environ.get("LEG_DAMP", 100.0))     # N·s/m
# 🔑 **힘 모드에서만 쓸 수 있는 판정 — 속도.** 같은 힘을 주면 벽에 닿은
#    다리는 **서 있고**, 허공을 짚는 다리는 **계속 나간다**(종단속도
#    F0/damping = 9/100 = 90mm/s). 위치로는 DN150 확장과 구별이 안 되지만
#    (둘 다 많이 뻗는다) 속도로는 갈린다 — 확장 구간에서는 벽에 닿는 순간 선다.
#    기준 반경은 **서 있는 다리들**에서 잡는다(그것이 곧 관벽이다).
LEG_FREE_V = float(os.environ.get("LEG_FREE_V", 0.02))    # m/s, 계속 나가면 뜬 것
LEG_STOP_V = float(os.environ.get("LEG_STOP_V", 0.005))   # m/s, 이하면 서 있다
LEG_REF_GAP = float(os.environ.get("LEG_REF_GAP", 0.006))  # m, 기준보다 이만큼 밖

# ── 속도 유지 조절기 (휠 실측 각속도 되먹임) ────────────────────────
GOV_ON = os.environ.get("GOV", "1") == "1"
GOV_KI = float(os.environ.get("GOV_KI", 2.0))
GOV_MIN, GOV_MAX = 0.5, 1.5

# ── 곡관 감속 — 관절 엔코더로 스스로 정한다 ─────────────────────────
AUTO_SPEED = os.environ.get("AUTO_SPEED", "1") == "1"
CURVE_MIN = float(os.environ.get("CURVE_MIN", 0.45))
BEND_REF = float(os.environ.get("BEND_REF", 14.0))
ACC_MPS2, DEC_MPS2 = 0.08, 0.40

# ── 다리 실린더를 **판단층에 올린다** (2026-08-07 신설) ─────────────
# 🚨 지금까지 속도를 정하는 데 다리 값이 **한 군데도** 안 들어갔다. 판단은
#    중앙 관절 각도, 되먹임은 휠 각속도뿐이었다. 그런데 **분기에서는 관절이
#    거의 안 꺾인다**(R150 에서 2~3°) — 곡관 감속이 사실상 안 걸린다.
#    다리가 그 구멍을 메운다. 설계에도 "서스펜션 스트로크는 관경 변화 감지
#    보조"로 적혀 있던 항목이다.
#      · 다리 신장 **편차**가 커진다 = 벽이 사라졌다(분기) / 단면이 안 둥글다
#      · 다리 평균이 **기준선에서 벗어난다** = 관경이 변한다(리듀서)
# 🔑 곡관 감속과 **같은 층**이므로 곱하지 않고 **더 느린 쪽을 택한다**(min).
#    곱하면 둘 다 걸릴 때 0.45×0.45=0.20 으로 과도하게 죽는다.
LEG_SLOW_ON = os.environ.get("LEG_SLOW", "1") == "1"
LEG_SPREAD_REF = float(os.environ.get("LEG_SPREAD_REF", 0.008))   # 8mm
LEG_DEV_REF = float(os.environ.get("LEG_DEV_REF", 0.005))         # 5mm (실측 근거)
LEG_MIN = float(os.environ.get("LEG_MIN", 0.45))

# ── 조향 — 중앙 관절로 **중심선을 따라간다** (2026-08-07 신설) ────────
# 🔑 T 분기가 실패한 원인은 CAD 가 아니라 **아무도 꺾으라고 안 했기 때문**이다.
#    곡관은 벽이 로봇을 꺾어 주지만(최대 꺾임 46~61° 실측) 분기는 직진로가
#    열려 있어 안 꺾인다(6.9°). 로봇은 본관을 그대로 지나가고 중심선만 가지로
#    올라가니 이탈이 벌어진다.
# 🚨 **이것은 도면을 아는 주행이다.** 연습장이 중심선을 이미 갖고 있어서 쓰는
#    것이고, 자율 인지(카메라로 분기를 찾아 오른손 법칙으로 고르는 것)가
#    아니다. 실기에서는 `condition/` 의 개구 검출이 이 목표각을 줘야 한다.
STEER_ON = os.environ.get("STEER", "1") == "1"
STEER_KP = float(os.environ.get("STEER_KP", 0.9))
# 🚨 목표각을 한 번에 던지면 발산한다(벨로우즈에서 실측: 링크가 6.7g 인데
#    45° 를 즉시 주면 3.1 N·m 가 걸려 NaN). 초당 각도로 제한한다.
STEER_RATE = float(os.environ.get("STEER_RATE", 45.0))
# 관절 하나당 목표각 상한 — 자산 한계 ±20° 안쪽으로 둔다.
STEER_MAX = float(os.environ.get("STEER_MAX", 17.0))
# 직진 장면에서 잔류 굽힘을 펴는 속도(°/s) — steer_vision 의 완화 경로 전용
RELAX_RATE = float(os.environ.get("RELAX_RATE", 20.0))
# 앞 세그먼트 위치에서 **추가로** 얼마나 더 내다볼 것인가.
# 🎯 **0.05 → 0.12** (2026-08-07 실측). 50mm 만 보면 원호에 들어가고 나서야
#    꺾기 시작해 뒤처지고, 만회하려면 **80°** 가 필요해진다 — 그게 관절 한계
#    (D6 4개 × ±20° = 80°)와 같아서 못 따라잡는다.
#    ⚠ 한계를 넓히는 쪽(`STEER_MAX` 17→19.5°)은 **효과가 없었다**(끼임 2회).
#    늦게 꺾는 것이 원인이지 못 꺾는 것이 원인이 아니다.
#      LOOK 0.05  → 끼임 3회 (s=340~350)
#      LOOK 0.12  → **코스 끝 도달 · 끼임 0회** · 이탈 4.8/10.8mm  ← 채택
STEER_LOOK = float(os.environ.get("STEER_LOOK", 0.12))
# 🎯 v9 굽힘 상한 — 관절 한계는 ±95° 지만 곡관 실측 최적이 **37°** 였고
#    50° 는 실패했다(+111mm). 상한을 45° 로 둔다.
STEER_MAX_V9 = float(os.environ.get("STEER_MAX_V9", 45.0))
# 정답지 조향의 관절당 굽힘 상한(도) — 관절 한계 ±95 안쪽
BP_BEND_MAX = float(os.environ.get("BP_BEND_MAX", 60.0))
# 🎯 **굽힘 중 롤 동결 문턱(도)** — 카메라 조향 결함 ⑤(굽힘 중 롤 회전 =
#    요동)의 정답지판 (2026-08-10). T 원호에서 자유 롤 디스크가 벽 접촉
#    토크로 돌고 `_aim()` 이 도는 디스크를 뒤쫓아 **적분 폭주**(지령 +392°,
#    누적 꺾임 593°, 사출 3연발)했다. 평면 회전은 몸이 펴진 순간에만 한다.
# ⚠ 기본 999 = **꺼짐** (2026-08-10). 8° 로 켜면 floor1 T 폭주는 멎지만
#    **틀린 평면에 갇힌다**(H런: 진입 곡관의 수직 평면인 채 T 도달, 끼임
#    37회). floor2 의 곡관1→2 평면 전환(s=510)도 굽힘이 8° 밑으로 안
#    내려가는 구간이라 켜면 회귀 위험. 실험용 노브로만 남긴다.
BP_ROLL_LOCK_AT = float(os.environ.get("BP_ROLL_LOCK_AT", 999.0))
# 🎯 **원호 구간 출구 조준** (2026-08-10 H런 사슬로 확정). 롤 동결만으로는
#    **틀린 평면에 갇힌다** — 진입 곡관의 굽힘이 8° 밑으로 안 내려간 채 T 에
#    도달하면 수직 평면에 동결된 채 수평 T 벽만 민다(s=638 끼임 37회 실측).
#    원호 구간(+직전 PRE)에서는 리딩 목표 접선을 look 지점이 아니라 **원호
#    출구 접선**으로 준다: 절대 목표라 오차가 자연 수렴(폭주 원리적 소멸),
#    직선 구간에서 미리 평면을 돌려놓고 원호에서는 유지만 한다.
BP_ARC_PRE = float(os.environ.get("BP_ARC_PRE", 0.10))
BP_ARC_KAPPA = float(os.environ.get("BP_ARC_KAPPA", 2.0))  # 1/m, 원호 판정
# ⚠ 기본 0 = **꺼짐** (2026-08-10 J런 실측). 절대 평면 목표조차 디스크 방향
#    피드백(_bd)을 참조하는 순간 직선 선조준에서 감긴다(누적 987°, 사출
#    641mm) — 자유 롤 디스크의 방위 제어는 추종 구조 자체가 문제다.
#    floor1 T 재도전 시 실험용으로만 켤 것.
BP_ARC_AIM = os.environ.get("BP_ARC_AIM", "0") == "1"
# 🎯 **스케줄 조향** (2026-08-10 저녁 — v9 T 통과 레시피의 정답지판).
#    10 트라이얼(A~J)의 결론: 자유 롤 디스크의 방위는 **추종 제어로 못 잡는다**
#    — 절대 목표조차 디스크 방향 피드백(_bd)을 참조하는 순간 감긴다(J: 987°).
#    v9 조원 스크립트(run_v9_final)가 T 를 전 방향 통과한 구조를 그대로 옮긴다:
#      ① 원호를 중심선에서 **미리** 뽑아 두고(아는 지형 = 스케줄로 못박기)
#      ② 진입 전 직선(PRE)에서만 롤을 조준 — 지령을 **실측 관절각에 앵커**
#         (`목표 = 실측 + 오차`). v9 의 `base = _roll_actual()` 이식.
#         적분기(지령 += 오차)가 아니므로 관절이 벽에 막혀도 지령이 폭주할
#         수 없다 — A~J 폭주의 공통 원인이 이 적분 구조였다.
#      ③ 원호 안에서는 롤 **동결** + 고정 굽힘(v9 ROLL_LOCK_AT=3° 의 확장판)
#      ④ 원호 끝 + LAG 까지 유지 후 해제 (v9 LAG 48mm → 몸이 긴 만큼 100mm)
#    BP_SCHED=0 이면 기존 추종 조향(steer_bp_rollbend)으로 돈다.
BP_SCHED = os.environ.get("BP_SCHED", "1") == "1"
# 원호 안 고정 굽힘 상한(도) — v9 곡관 실측 최적 37°, 50° 는 실패. 원호의
# 실제 꺾임각이 이보다 작으면 그 각을 쓴다.
BP_SCHED_BEND = float(os.environ.get("BP_SCHED_BEND", 40.0))
BP_SCHED_LAG = float(os.environ.get("BP_SCHED_LAG", 0.10))   # 원호 뒤 유지(m)
BP_SCHED_PRE = float(os.environ.get("BP_SCHED_PRE", 0.10))   # 진입 전 조준(m)
# 롤 조준의 스텝당 보정 클램프(도) — v9 ROLL_STEP_MAX 이식. 앵커 구조라
# 폭주는 없지만, 한 스텝의 덜컥임(lurch)을 막는다.
BP_SCHED_STEP = float(os.environ.get("BP_SCHED_STEP", 20.0))
BP_REAR_SIGN = float(os.environ.get("BP_REAR_SIGN", 1.0))
ROLL_SIGN = float(os.environ.get("ROLL_SIGN", 1.0))   # 롤 오차 부호 (실측)
# 굽힘이 이보다 크면 롤 조준을 멈춘다(도) — 조원 스크립트 값 3.0 이식
ROLL_LOCK_AT_V9 = float(os.environ.get("ROLL_LOCK_AT_V9", 3.0))
ROLL_CAL = (float(os.environ["ROLL_CAL"])
            if os.environ.get("ROLL_CAL") is not None else None)
# 🎯 **곡관이 아닐 때의 굽힘 상한** (2026-08-09 GUI 실측으로 신설).
# 🚨 컨트롤러의 조향 지령(최대 40°)은 **welder 기준**이다 — 그쪽은 관절 4개에
#    나눠 10°씩 쓴다. v9 는 **관절 하나에 40° 를 다 준다**(4배 공격적).
#    곡관에서는 그게 맞지만(R150 에 37° 필요) 직선에서는 머리가 벽을 파고든다.
#    실측(GUI): 수직 곡관을 내려온 뒤 수평 구간에서 지령 굽힘이 −40° 로 계속
#    붙은 채 머리가 박혔다 — 접힌 몸 → 비스듬한 시야 → 큰 입사각 → 최대 조향
#    → 더 접힘 의 악순환.
# 🔑 오늘 검출기를 고쳐 **곡관 판정이 정확해졌으므로**(덤프 55장 100%) 그
#    판정에 상한을 걸 수 있다: 곡관이면 크게, 아니면 작게.


# 출발 롤(°) — 다리 클럭 위치를 개구에 대해 돌려 본다. 다리가 120° 간격이라
# 0~120° 가 전부다. 🚨 실기에서는 로봇이 롤을 제어하지 못한다 — 이 값에
# 통과가 좌우되면 그 자체가 **설계가 불안정하다는 뜻**이다.
# 🎯 **항법 모드** (2026-08-07 사용자 지시: *"연습장에서 도면 의존부터 걷어내"*)
#   onboard   : **로봇 신호만** 쓴다 — 관절 엔코더 · 다리 실린더 · 휠 각속도.
#               중심선(도면)은 **채점에만** 쓰고 판단에는 안 쓴다.
#   blueprint : 예전 방식 — 중심선을 따라간다. **도면을 아는 주행**이므로
#               성능 수치를 자율주행 근거로 쓰면 안 된다. 비교용으로만 남긴다.
# 🚨 남아 있는 도면 의존(둘 다 **심판** 용도): 끼임·이탈 판정(`cl.nearest`),
#    출발/종료 지점(START_S·END_S). 실기로 옮길 때 이 둘도 걷어내야 한다.
#   vision    : **설계대로** — 카메라 → `condition/detector` + `condition/
#               odometry` → `driver/control.DriveController` 가 속도를 정한다.
#               연습장은 심판만 본다. 🚨 카메라를 켜므로 GUI 가 느려진다.
NAV = os.environ.get("NAV", "onboard")
# 개구 쪽으로 몰아 주는 조향 세기(총 꺾임, 도) — 로봇 신호만으로 정한다
STEER_ONBOARD_DEG = float(os.environ.get("STEER_ONBOARD_DEG", 40.0))

ROLL_DEG = float(os.environ.get("ROLL_DEG", 0.0))
# 🎯 바퀴 차동 속도 (2026-08-08 신설) — 굽힘에서 바깥 바퀴를 빠르게.
DIFF_ON = os.environ.get("DIFF", "1") == "1"
# 🎯 **기본 ON** (2026-08-09 확정). 경위를 순서대로 남긴다 — 한때 기본 OFF 였고
#    그 이유가 아직 유효한 것으로 오해하기 쉬워서다:
#    ① 1차 구현은 **해로웠다**. 켜니 floor2 가 s 969 → **355** 로 후퇴했다
#       (진입 곡관에서 이탈 1.4mm 로 잘 정렬된 채 **전진만 못 함** = 안쪽
#       바퀴가 브레이크처럼 끈 정황). 그래서 실험용으로 0 을 기본에 뒀다.
#    ② 원인은 부호가 아니라 **감속**이었다 — 속도 드라이브는 지령보다 빨리
#       도는 바퀴를 잡아채므로, 안쪽에 0.5배를 주는 순간 그것이 곧 제동이다.
#       → 비율은 유지하되 **최솟값이 1.0** 이 되도록 통째로 올리는 것으로
#         고쳤다(아래 `drive()`) + 곡률 반경 바닥 0.10m.
#    ③ 수정판 실측: floor2 s **1595** (종전 최고 1061 · 차동 OFF 969).
#       floor1 무해. → 실험용으로 남아 있던 기본 0 을 1 로 되돌린다.
#    끄려면 `DIFF=0`. DIFF_SIGN 으로 부호를, DIFF_GAIN 으로 세기를 바꾼다.
DIFF_SIGN = float(os.environ.get("DIFF_SIGN", 1.0))
DIFF_GAIN = float(os.environ.get("DIFF_GAIN", 1.0))

# 안착 궤적을 0.25초마다 찍는다 (수직관에서 흘러내리는지 진단).
SETTLE_TRACE = os.environ.get("SETTLE_TRACE", "0") == "1"

ROBOT = "/World/Robot"

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
stage = world.stage
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.Xform.Define(stage, "/World")

light = UsdLux.SphereLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3e6)
light.CreateRadiusAttr(0.05)
UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(-0.2, -0.4, 0.6))


def rot(deg, axis):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(*axis), deg))
    return m


def trans(x, y, z):
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(x, y, z))
    return m


def scale(s):
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(s, s, s))
    return m


def load_stl(path):
    data = Path(path).read_bytes()
    n = struct.unpack("<I", data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = a[:, 12:48].copy().view("<f4").reshape(n * 3, 3).astype(np.float64)
    pts, inv = np.unique(np.round(tri, 5), axis=0, return_inverse=True)
    return pts * MM, inv.reshape(n, 3)


def make_mesh(path, stl, color=None, xform=None):
    pts, idx = load_stl(stl)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * len(idx))
    mesh.CreateFaceVertexIndicesAttr(idx.reshape(-1).tolist())
    mesh.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
    mesh.CreateSubdivisionSchemeAttr("none")
    if color:
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if xform is not None:
        UsdGeom.Xformable(mesh).AddTransformOp().Set(xform)
    return mesh

# ── 실전 맵 코스 (restroom_final0807) ───────────────────────────────
# 🔑 **중심선은 채점 전용이다.** 주행(조향·감속·분기 선택)은 카메라 depth 만
#    보는 자율 스택이 한다 — 아래 좌표는 이탈량·진행도를 재고 로봇을 출발
#    지점에 놓는 데만 쓴다. 코너는 직선의 교점이고 필렛 R150(LR)이 끼워진다.
# 🔑 실측 근거는 파일 머리말 참조(맵 메시 31개 bbox + 12방위 광선 내반경).
_MAP = MAPS / "restroom_final0807.usd"
_BEND_R = 150.0                    # LR 곡관 굽힘 반경 (map mm)


def _fillet(corners, R=_BEND_R, n=18):
    """코너 목록(교점) → R 필렛이 끼워진 중심선 점열(map mm).

    각 코너에서 앞뒤 직선을 R·tan(θ/2) 만큼 잘라내고 원호로 잇는다.
    직선 구간은 20mm 간격으로 촘촘히 깐다(이탈 계산이 점열 최근접이라
    성기면 곡선 구간에서 오차가 생긴다).
    """
    P = [np.array(c, dtype=np.float64) for c in corners]
    out = [P[0]]
    for i in range(1, len(P) - 1):
        a, b, c = P[i - 1], P[i], P[i + 1]
        u = a - b
        v = c - b
        lu, lv = np.linalg.norm(u), np.linalg.norm(v)
        if lu < 1e-9 or lv < 1e-9:
            continue
        u, v = u / lu, v / lv
        cosang = float(np.clip(np.dot(u, v), -1.0, 1.0))
        ang = math.acos(cosang)                  # 두 직선 사이 각
        if ang > math.pi - 1e-6:                 # 일직선 — 필렛 없음
            continue
        t = R / math.tan(ang / 2.0)              # 코너에서 잘라낼 길이
        t = min(t, 0.49 * lu, 0.49 * lv)
        p_in, p_out = b + u * t, b + v * t
        # 직선 (이전 점 → 필렛 시작) — 기점을 먼저 떠 두고 보간한다.
        # 🚨 `out[-1]` 을 루프 안에서 다시 읽으면 방금 넣은 점이 기점이 되어
        #    구간이 기하급수로 짧아진다(첫 판의 실수).
        p0 = out[-1].copy()
        seg = float(np.linalg.norm(p_in - p0))
        nstep = max(int(seg / 20.0), 1)
        for k in range(1, nstep + 1):
            out.append(p0 + (p_in - p0) * (k / nstep))
        # 원호 — 중심은 두 접선의 안쪽 이등분선 위
        w = u + v
        w = w / max(np.linalg.norm(w), 1e-12)
        d_c = R / math.sin(ang / 2.0)
        ctr = b + w * d_c
        r0, r1 = p_in - ctr, p_out - ctr
        nrm = np.cross(r0, r1)
        ln = np.linalg.norm(nrm)
        if ln < 1e-12:
            continue
        nrm = nrm / ln
        sweep = math.acos(float(np.clip(
            np.dot(r0, r1) / (np.linalg.norm(r0) * np.linalg.norm(r1)),
            -1.0, 1.0)))
        for k in range(1, n + 1):
            th = sweep * k / n
            ct, st = math.cos(th), math.sin(th)
            out.append(ctr + r0 * ct + np.cross(nrm, r0) * st)
    # 마지막 직선
    seg = np.linalg.norm(P[-1] - out[-1])
    nstep = max(int(seg / 20.0), 1)
    tail = out[-1].copy()
    for k in range(1, nstep + 1):
        out.append(tail + (P[-1] - tail) * (k / nstep))
    return np.array(out)


# 🎯 **floor1 = 닫힌 루프.** 임무 = 진입 → T 에서 **오른쪽(−Y) 팔**로 나가
#    한 바퀴 돌아 **+Y 팔로 T 에 복귀** → 진입 라이저로 나온다.
#    오른쪽 판정: 진행 +X, 상방 +Z → travel×up = (+X)×(+Z) = −Y. 실측 T
#    (730,850)에서 −Y 731mm·+Y 531mm 가 열려 있고 정면(+X)은 50mm 벽이다.
_Z1 = -2740.2
# 🎯 **v1_3 대청소 (2026-08-11 사용자 확정): 맵은 원본 단일.**
#    v1_2 의 실험 맵 3종(MAP_V2 = v3 스위프 T 경량본 / MAP_S290 = 직선 290
#    연장판 / BP_TEE_PLUG 마개)은 전부 삭제 — 마지막까지 실제 주행에 쓴
#    조합은 **원본 restroom_final0807 + 스위치 전부 OFF** 였다(t50 나가는 턴
#    정복·t91 되감기 최심 기록 전부 이 조합). 실험 맵이 필요하면 v1_2 로.
_F1_CORNERS = [
    (330.0, 850.0, -2405.2),       # 라이저 상단
    (330.0, 850.0, _Z1),           # 라이저 → 수평망
    (730.0, 850.0, _Z1),           # ★T 분기★
    (730.0, 100.0, _Z1),           # 오른팔(−Y)로 나감
    (1300.0, 100.0, _Z1),          # 루프 아래변 → +X
    (1300.0, 1400.0, _Z1),         # 우변 +Y (합류부 통과)
    (730.0, 1400.0, _Z1),          # 위변 −X
    (730.0, 850.0, _Z1),           # ★T 로 복귀★ (한 바퀴 닫힘)
    # 🎯 **역재생 귀가** (2026-08-10 밤 — t52~74 실측 결론: 왼팔→본관 전진
    #    턴은 T 의 X/Y 비대칭(본관 관통=포켓 캡 막힘 vs 팔 관통=뚫림) 탓에
    #    "성공 기동의 역순"이 아니다 — 세워줄 벽이 없어 20여 런 전부 정체.
    #    진짜 역순은 **오른팔에서 후진으로 본관 진입**: 성공한 나가는 턴의
    #    시간 역재생이라 포켓 캡이 다시 받침벽이 되고 검증 안 된 물리가
    #    없다. → 코스는 크로스바 직진으로 오른팔에 250mm 들어가 끝나고,
    #    복귀는 BACK(후진)이 나가는 가지 s 로 스냅해 원길을 되밟는다.
    # 🎯 되감기 최종 설계 (t89 해부 — 크로스바 횡단은 양 옆벽(본관 입구+
    #    포켓 입구)이 동시에 빈 구간이라 좌초. 대신 **루프를 코스 후진으로
    #    되밟아** 오른팔에서 접합부로 후진 접근(전 구간 벽 있음, floor2
    #    검증 부류) → s∈[400,1000] 에서 테이프 되감기 → 포켓 벽 받침으로
    #    T 역통과 → 본관 → 라이저 → 탈출. 벽 없는 기동 전무.)
]
_Z2 = -250.0
_F2_CORNERS = [
    (330.0, 850.0, 85.0),
    (330.0, 850.0, _Z2),
    (680.0, 850.0, _Z2),
    (680.0, 1400.0, _Z2),
    (1200.0, 1400.0, _Z2),
    (1200.0, 600.0, _Z2),
    (1500.0, 600.0, _Z2),
]

# ── 🎯 v1_3 건물 고정 배치 (2026-08-11 사용자 확정) ──────────────────
# **건물은 세 모드 모두 딱 한 번, 같은 위치에 얹는다** — floor1(아래층)
# 수평망이 월드 z=0(바닥), floor2 는 같은 건물 안 **+2490.2mm** 위.
# v1_2 는 "층에 맞춰 건물을 옮기는" 방식이라 두 층 동시가 불가능했다
# (같은 맵 2회 참조 = 건물 셸 겹침 → 콜라이더 관통, 기록된 발산 부류).
# v1_3 은 뒤집는다: 건물 고정 + 로봇이 자기 층 높이로 간다. 코스 중심선은
# 각 층의 원본 z 를 그대로 갖고 있으므로 같은 변환 하나로 다 맞는다.
_BLD_XF = scale(MM) * trans(0.0, 0.0, -_Z1 * MM)
COURSES = {
    "floor1": (_fillet(_F1_CORNERS),
               "화장실 floor1 — **닫힌 루프**. T 에서 오른팔로 나가 한 바퀴"),
    "floor2": (_fillet(_F2_CORNERS),
               "화장실 floor2 — 막다른 끝(단절)까지 2.5m [윗층 +2490mm]"),
}
# 모드 3종: floor1 / floor2 / both(2기 동시). all·쉼표 목록은 both 로 수렴.
if COURSE in ("all", "both", "floor1,floor2", "floor2,floor1"):
    COURSE = "both"
    RUN_NAMES = ["floor1", "floor2"]
else:
    RUN_NAMES = [COURSE]
    if COURSE not in COURSES:
        raise SystemExit(f"[중단] --course 는 floor1 | floor2 | both "
                         f"(받은 값: {COURSE!r})")
if not _MAP.is_file():
    raise SystemExit(
        f"[중단] 맵 USD 가 없다: {_MAP}\n"
        f"        isaac_python tools/step_to_usd.py "
        f"~/Downloads/{_MAP.stem}.stp 로 먼저 구울 것")


def xf_pts(mat, pts_mm):
    """원본 좌표(mm) 배열 → 월드(m). 코스 변환을 점에도 똑같이 먹인다."""
    return np.array([list(mat.Transform(Gf.Vec3d(*map(float, p))))
                     for p in pts_mm])


class Centerline:
    """코스 중심선 — 진행거리 s 와 중심선까지의 거리를 준다.

    🚨 **닫힌 루프에서는 전역 최근접이 성립하지 않는다** (2026-08-08 실전 맵
       실측). floor1 은 T 에서 나가 한 바퀴 돌아 **같은 T 로 돌아오는** 루프라
       중심선의 끝점과 초반 접근 구간이 같은 자리에 있다. 전역 argmin 은
       T 근처의 로봇을 **끝점(s=4088)** 에 붙여 버렸고, 데모는 그것을 "코스 끝
       도달" 로 읽어 출발 직후 복귀 지시를 내렸다(실측: s 599 → 4088 점프).
    → **창(window) 안에서만 찾는다.** 직전 s 를 힌트로 받아 ±`win` 안의
      구간만 본다. 로봇은 한 스텝에 1mm 도 못 가므로 창을 벗어날 수 없고,
      그래서 루프가 닫혀 있어도 s 가 단조롭게 이어진다.
    """

    def __init__(self, pts):
        self.p = pts
        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(d)])
        self.total = float(self.s[-1])

    def nearest(self, q, hint=None, win=0.35):
        if hint is None:
            i = int(np.argmin(np.linalg.norm(self.p - q, axis=1)))
        else:
            lo = int(np.searchsorted(self.s, hint - win))
            hi = int(np.searchsorted(self.s, hint + win))
            lo, hi = max(lo, 0), min(max(hi, lo + 2), len(self.p))
            i = lo + int(np.argmin(
                np.linalg.norm(self.p[lo:hi] - q, axis=1)))
        return float(self.s[i]), float(np.linalg.norm(self.p[i] - q)), i

    def tangent(self, i):
        j = min(i + 1, len(self.p) - 1)
        k = max(j - 1, 0)
        v = self.p[j] - self.p[k]
        return v / max(np.linalg.norm(v), 1e-12)


# ── 물리 재질 — 물이 없으므로 **배수(건식)** 고정 ────────────────────
FRICTION_STATIC, FRICTION_DYNAMIC = 0.40, 0.35
_pm = UsdPhysics.MaterialAPI.Apply(
    UsdShade.Material.Define(stage, "/World/PipePhysMat").GetPrim())
_pm.CreateStaticFrictionAttr(FRICTION_STATIC)
_pm.CreateDynamicFrictionAttr(FRICTION_DYNAMIC)
_pm.CreateRestitutionAttr(0.0)

# 🎯 v1_3: **배관은 항상 유리** (2026-08-11 사용자 확정 — "배관만 유리
#    유지"). --glass 플래그와 무관하게 전 층 배관에 적용.
# 🎯 **굴절 제거** (같은 날 사용자: "굴절 때문에 관찰이 어렵다") —
#    ior=1.0 이면 광선이 꺾이지 않고 직진 투과라 로봇이 왜곡 없이 보인다.
# 🎯 **층별 배관 재질** (2026-08-11 사용자 지시: *"floor2 만 배관 색깔을
#    입힌다 — 그래야 용접 카메라가 잘 보인다"*). 유리는 투명해서 밖에서
#    로봇을 보기엔 좋지만, 용접 카메라처럼 **관 안에서 보는 시점**에서는
#    벽 너머가 다 비쳐 결함·용접봉이 묻힌다. floor2 는 실제 배수관처럼
#    불투명 PVC, floor1(T 자 관찰용)은 유리 유지.
# 🎯 floor2 **유리 모드** (`--glass2`) — v1_2 에서 `--glass` 로 돌렸을 때의
#    그 유리 그대로(회백 0.55/0.58/0.60 · 투명도 0.25 · 거칠기 0.4).
#    기본은 PVC(불투명)다 — 용접 카메라는 관 **안**에서 보므로 벽이 비치면
#    결함이 묻힌다. 관 **밖**에서 용접 장면을 통째로 볼 때 이 모드를 쓴다.
GLASS2 = "--glass2" in sys.argv
PVC_FLOORS = (set() if GLASS2 else
              {x for x in os.environ.get("PVC_FLOORS", "floor2").split(",")
               if x})
# 성능 노브 — 유리(투과)는 RTX 에서 비싸다. PIPE_GLASS=0 이면 불투명 회색.
PIPE_GLASS = os.environ.get("PIPE_GLASS", "1") == "1"
# 진단용 — 유리 재질 바인딩 자체를 뺀다(물리 재질 덮어쓰기 혐의 검증)
GLASS_BIND = os.environ.get("GLASS_BIND", "1") == "1"
_gl = UsdShade.Material.Define(stage, "/World/Glass")
_gs = UsdShade.Shader.Define(stage, "/World/Glass/Shader")
_gs.CreateIdAttr("UsdPreviewSurface")
# 🎯 **색 있는 무광 반투명** (2026-08-11 사용자 지시) — 안이 비쳐 로봇을
#    볼 수 있으면서, 색이 있어 벽·바닥과 구분되고, **광택 없음**(거칠기 높음)
#    이라 반사가 화면을 어지럽히지 않는다. 굴절도 없다(ior 1.0).
_gc = [float(x) for x in os.environ.get("GLASS_COLOR",
                                        "0.92,0.94,0.95").split(",")]
_gs.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(*_gc))
_gs.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(
    float(os.environ.get("PIPE_OPACITY", 0.06)) if PIPE_GLASS else 1.0)
_gs.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05)  # 맑게
_gs.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
_gs.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.0)
_gl.CreateSurfaceOutput().ConnectToSource(_gs.ConnectableAPI(), "surface")

# PVC 배수관 — 회색 경질 PVC(KS 배수관). 불투명·약한 광택.
_pvc = UsdShade.Material.Define(stage, "/World/PipePVC")
_ps = UsdShade.Shader.Define(stage, "/World/PipePVC/Shader")
_ps.CreateIdAttr("UsdPreviewSurface")
# 🚨 회색 PVC(0.68,0.69,0.66)는 **벽·바닥과 톤이 같아 오히려 안 보였다**
#    (2026-08-11 사용자). 무지개 구분색 때가 더 잘 보였다는 지적대로
#    **짙은 회색**(사용자 지정) — 밝은 벽·바닥과 명도가 확실히 갈린다.
#    PIPE_COLOR="r,g,b" 로 즉시 교체 가능.
_pc = [float(x) for x in os.environ.get("PIPE_COLOR",
                                        "0.30,0.34,0.36").split(",")]
_ps.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(*_pc))
_ps.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
_ps.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
_pvc.CreateSurfaceOutput().ConnectToSource(_ps.ConnectableAPI(), "surface")


# floor2 유리 모드 전용 재질 — v1_2 값 그대로. floor1 유리는 그 뒤 "거의
#    완전 투명"(0.06)으로 따로 조정했으므로 값을 공유하지 않는다.
_gl2 = UsdShade.Material.Define(stage, "/World/Glass2")
_g2s = UsdShade.Shader.Define(stage, "/World/Glass2/Shader")
_g2s.CreateIdAttr("UsdPreviewSurface")
_g2s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
    Gf.Vec3f(*[float(x) for x in os.environ.get(
        "GLASS2_COLOR", "0.55,0.58,0.60").split(",")]))
_g2s.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(
    float(os.environ.get("GLASS2_OPACITY", 0.10)))
_g2s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
# 🚨 투명도 0.25(v1_2 값)는 **배수구에서 안 통한다** — 그 자리엔 라이저
#    관벽·거름망 하우징·바닥 슬래브가 겹쳐 25% 가 세 겹이면 거의 불투명이
#    된다(사용자: "저 부분만 로봇이 안 보인다"). 0.10 이면 겹쳐도 비친다.
# 굴절은 뺀다(2026-08-11 사용자: 굴절 때문에 관찰이 어렵다). v1_2 그대로가
# 좋으면 GLASS2_IOR=1.5.
_g2s.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(
    float(os.environ.get("GLASS2_IOR", 1.0)))
_gl2.CreateSurfaceOutput().ConnectToSource(_g2s.ConnectableAPI(), "surface")


def pipe_mat(path_or_name):
    """그 층 배관에 쓸 재질 — PVC_FLOORS 에 든 층은 PVC, 나머지는 유리."""
    _f2 = ("/floor2/" in str(path_or_name)
           or str(path_or_name) == "floor2")
    for _f in PVC_FLOORS:
        if _f and (f"/{_f}/" in str(path_or_name)
                   or str(path_or_name) == _f):
            return _pvc
    return _gl2 if (GLASS2 and _f2) else _gl

# ── 배관 적재 — **건물 1회 참조** (v1_3) ────────────────────────────
print("=" * 78)
paths = {}
print(f"[코스] 건물 1회 적재(floor1=바닥) — 로봇 투입 {', '.join(RUN_NAMES)}")
root_path = "/World/Pipe_map"
root = stage.DefinePrim(root_path, "Xform")
root.GetReferences().AddReference(str(_MAP))
UsdGeom.Xformable(root).AddTransformOp().Set(_BLD_XF)
# 🚨 맵 내장 PhysicsScene 은 끈다 — 물리 씬 중복은 기록된 발산 사고.
for _ps in [c for c in Usd.PrimRange(root)
            if c.IsA(UsdPhysics.Scene)]:
    _ps.SetActive(False)
# 🚨 인스턴스 프로토타입이면 콜라이더 부여가 안 먹는다 — 먼저 푼다.
while True:      # 풀면 자식이 새로 드러나므로 더 나올 게 없을 때까지 돈다
    inst = [p for p in stage.Traverse()
            if p.IsInstance() and str(p.GetPath()).startswith(root_path)]
    if not inst:
        break
    for p in inst:
        p.SetInstanceable(False)
meshes = [p for p in stage.Traverse()
          if p.IsA(UsdGeom.Mesh) and str(p.GetPath()).startswith(root_path)]
# 건물 셸 3개(비배관) — 아래 페인트 단계가 색을 칠한다. 그 외 = 전부 배관.
_BLDG_TAILS = ("/floor2/PartBody/Mesh", "/floor1/tn__PartBody1_YE/Mesh",
               "/aisle/PartBody/Mesh")
n_col = n_skip = 0
for p in meshes:
    _pp0 = str(p.GetPath())
    _is_bldg = any(_pp0.endswith(_b) for _b in _BLDG_TAILS)
    # 🚨 `Sweep` 은 관 **속살(내면)만** 따로 들어 있는 면체(surface body)라
    #    PartBody 의 내면과 완전히 겹친다. 둘 다 콜라이더로 주면 같은 자리에
    #    접촉이 두 번 생기고, 렌더도 z-fighting 이 난다.
    if "Sweep" in _pp0:
        UsdGeom.Imageable(p).MakeInvisible()
        n_skip += 1
        continue
    # 배관 조각(비건물): **유리(항상, 굴절 없음)** — 2026-08-11 사용자 확정.
    # 건물 셸은 유리 대신 아래 페인트 단계가 색을 입힌다.
    UsdGeom.Mesh(p).CreateDoubleSidedAttr(True)   # 단면 메시면 밖에서 투명
    if not _is_bldg and GLASS_BIND:
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            pipe_mat(_pp0),
            bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    # 🚨 콜라이더·물리재질은 **활성 층만** — v1_2 규약 그대로. ⚠ 건물 셸도
    #    포함해야 한다: 이 맵은 PartBody(건물+관 외피 융합)가 **실제 관벽
    #    콜라이더**다. 셸을 빼자 관벽이 사라져 로봇 2기가 추락했다
    #    (paint1 실측: 362스텝 이탈 9338mm 전멸 — 기록된 함정).
    if not any(f"/{_nm}/" in _pp0 for _nm in RUN_NAMES):
        n_skip += 1
        continue
    UsdPhysics.CollisionAPI.Apply(p)
    # 🚨 배관은 반드시 approximation="none". convexHull 이면 관 속이 꽉 찬다.
    UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("none")
    UsdShade.MaterialBindingAPI.Apply(p).Bind(
        UsdShade.Material.Get(stage, "/World/PipePhysMat"),
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics")
    n_col += 1

# 🔑 **맵은 손대지 않는다.** 구 real_map_demo 는 맵을 고쳐 쓰는 코드가
#    셋 있었다 — ① `tools/fix_map.py`(좁은 중복 사본을 참 반경으로 투영,
#    ø90 가지 확장) ② `refine_near()`(결함 주변 메시 세분) ③ 결함 관통
#    개구 절단. 셋 다 **구 맵의 결함과 수리 시연을 위한 것**이고, 실측
#    결과 새 맵에는 그 결함이 없다(내반경 49.0~49.4 균일·R150 확인,
#    구 ø90 자리 49.3). 이 데모는 원본 형상을 그대로 굴린다.
print(f"  건물 메시 {len(meshes)} (콜라이더 {n_col} / 제외 {n_skip})")
for name in RUN_NAMES:
    cl_mm, desc = COURSES[name]
    if REVERSE and name == COURSE:
        cl_mm = cl_mm[::-1]          # 반대 방향 통과 시험
    cl = Centerline(xf_pts(_BLD_XF, cl_mm))
    paths[name] = cl
    print(f"  {name:8s} 중심선 {cl.total * 1000:.0f}mm  입구 "
          f"({cl.p[0][0] * 1000:+.0f}, {cl.p[0][1] * 1000:+.0f}, "
          f"{cl.p[0][2] * 1000:+.0f})mm → 출구 ({cl.p[-1][0] * 1000:+.0f}, "
          f"{cl.p[-1][1] * 1000:+.0f}, {cl.p[-1][2] * 1000:+.0f})mm")
    print(f"           {desc}")

# ── 🎨 건물 페인트 2차 — **재질 서브셋** (2026-08-11 사용자 사진 회신 반영) ──
# 1차 실패 원인 실측 2건:
#   ① displayColor 는 CAD 자산에 저작된 재질 바인딩이 우선이라 **안 보였다**
#      → 면 그룹(GeomSubset)마다 재질을 직접 바인딩한다.
#   ② 보이는 관 외피(수평망 전체)는 건물 셸에 **융합**돼 있어 셸이 불투명이면
#      관도 불투명 → **중심선 반경 80mm 안의 면만 유리 서브셋**으로 갈라낸다
#      ("배관 위치 = 로봇 순회 범위" — 사용자 정의 그대로).
# 사진 확인 구조: 샤워기 헤드·샤워 칸막이·세면대(볼+수납장)·거울·바닥 배수구.
# 이번 라운드: 바닥/천장/벽 = 현실색, 실내 돌출물 = 군집 구분색(사진 1회 더
# 받아 군집→물체 확정 후 최종 현실색).


def _paint_mat(nm9, color, rough=0.6, metallic=0.0):
    """불투명 페인트 재질 하나 (UsdPreviewSurface)."""
    _m = UsdShade.Material.Define(stage, f"/World/PaintMats/{nm9}")
    _s = UsdShade.Shader.Define(stage, f"/World/PaintMats/{nm9}/S")
    _s.CreateIdAttr("UsdPreviewSurface")
    _s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*color))
    _s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    _s.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    _m.CreateSurfaceOutput().ConnectToSource(_s.ConnectableAPI(), "surface")
    return _m


# 🎨 **실제 화장실 재질색** (2026-08-11 사용자: "화장실이 무지개 색이지는
#    않잖아"). 사진으로 확인한 구성: 샤워헤드·샤워 칸막이·세면대(볼+수납장)
#    ·거울·바닥 배수구. 군집 원색은 정체 확인용 임시였고 이제 확정색으로 간다.
_MAT_WALL = _paint_mat("wall", (0.88, 0.87, 0.84), rough=0.35)    # 타일 벽
_MAT_FLOOR = _paint_mat("floor", (0.46, 0.47, 0.49), rough=0.55)  # 짙은 타일
_MAT_CEIL = _paint_mat("ceil", (0.95, 0.95, 0.93), rough=0.85)    # 도장 천장
_MAT_CERAMIC = _paint_mat("ceramic", (0.96, 0.96, 0.94), rough=0.15)   # 도기
_MAT_WOOD = _paint_mat("wood", (0.60, 0.45, 0.32), rough=0.55)         # 수납장
_MAT_PANEL = _paint_mat("panel", (0.80, 0.86, 0.88), rough=0.25)       # 칸막이
_MIRROR_CHEAP = os.environ.get("MIRROR_CHEAP", "0") == "1"
_MAT_MIRROR = _paint_mat("mirror", (0.92, 0.94, 0.96),
                         rough=0.35 if _MIRROR_CHEAP else 0.04,
                         metallic=0.0 if _MIRROR_CHEAP else 1.0)  # 거울
_MAT_CHROME = _paint_mat("chrome", (0.78, 0.80, 0.82), rough=0.12,
                         metallic=1.0)                                  # 크롬
_MAT_FIX = _paint_mat("fixture", (0.90, 0.90, 0.88), rough=0.35)  # 미분류 집기


def _subset_bind(mesh9, nm9, faces9, mat9):
    """면 그룹을 GeomSubset 으로 갈라 재질을 바인딩한다."""
    if not len(faces9):
        return
    _ss = UsdGeom.Subset.CreateGeomSubset(
        mesh9, nm9, UsdGeom.Tokens.face,
        Vt.IntArray([int(f) for f in faces9]))
    _ss.GetFamilyNameAttr().Set("materialBind")
    UsdShade.MaterialBindingAPI.Apply(_ss.GetPrim()).Bind(
        _gl if mat9 is None else mat9)


# 층별 배관 중심선 — **월드 m** (paths[].p). 🚨 셸 메시 점을 로컬로 읽으면
# 프림 자체 변환 때문에 좌표계가 어긋난다 → 면 중심도 월드로 변환해 비교.
for p in meshes:
    _pp0 = str(p.GetPath())
    if not any(_pp0.endswith(_b) for _b in _BLDG_TAILS):
        continue
    _who = "floor2" if "/floor2/" in _pp0 else (
        "floor1" if "/floor1/" in _pp0 else "aisle")
    _bm = UsdGeom.Mesh(p)
    _bp = np.array(_bm.GetPointsAttr().Get(), dtype=np.float64)
    _M9 = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()), dtype=np.float64)
    _bp = _bp @ _M9[:3, :3] + _M9[3, :3]          # → 월드 m
    _bc = np.array(_bm.GetFaceVertexCountsAttr().Get())
    _bi = np.array(_bm.GetFaceVertexIndicesAttr().Get())
    _bo = np.concatenate([[0], np.cumsum(_bc)])
    _nf0 = len(_bc)
    _ctr = np.zeros((_nf0, 3))
    _nrm = np.zeros((_nf0, 3))
    for _f in range(_nf0):
        _vs = _bp[_bi[_bo[_f]:_bo[_f + 1]]]
        _ctr[_f] = _vs.mean(0)
        _n0 = np.cross(_vs[1] - _vs[0], _vs[2] - _vs[0])
        _l0 = np.linalg.norm(_n0)
        _nrm[_f] = _n0 / _l0 if _l0 > 1e-12 else 0.0
    # ① 배관 면 — 그 층 중심선까지 80mm 이내 (관 외반경 ~57 + 여유)
    _is_pipe = np.zeros(_nf0, dtype=bool)
    if _who in paths:
        _cl9 = paths[_who].p
        for _f in range(_nf0):
            if np.linalg.norm(_cl9 - _ctr[_f], axis=1).min() < 0.080:
                _is_pipe[_f] = True
    # ② 바닥/천장/외벽/실내 집기
    _zlo, _zhi = _bp[:, 2].min(), _bp[:, 2].max()
    _is_fl = (np.abs(_nrm[:, 2]) > 0.7) & (_ctr[:, 2] < _zlo + 0.060) \
        & ~_is_pipe
    _is_ce = (np.abs(_nrm[:, 2]) > 0.7) & (_ctr[:, 2] > _zhi - 0.060) \
        & ~_is_pipe
    _blo, _bhi = _bp[:, :2].min(0), _bp[:, :2].max(0)
    _edge = ((np.abs(_ctr[:, 0] - _blo[0]) < 0.060)
             | (np.abs(_ctr[:, 0] - _bhi[0]) < 0.060)
             | (np.abs(_ctr[:, 1] - _blo[1]) < 0.060)
             | (np.abs(_ctr[:, 1] - _bhi[1]) < 0.060)) & ~_is_pipe
    _cand = np.flatnonzero(~(_is_fl | _is_ce | _edge | _is_pipe))
    # ③ **벽 걷어내기 → 집기 분리** (2단계).
    #    ⓐ 동일 평면(법선+거리) 그룹으로 묶는다. 방 벽은 **경계에 있는 큰
    #       평면**, 집기는 그 밖이거나 작은 평면이다. 이것 없이 연결관계만
    #       쓰면 집기가 실내 벽을 통해 전부 한 덩어리가 된다(실측: 448면).
    #    ⓑ 남은 면만 정점 공유로 묶어 물체 단위를 얻는다.
    _room = (_ctr[_is_fl][:, :2] if _is_fl.any() else _ctr[_cand][:, :2])
    _rlo, _rhi = _room.min(0), _room.max(0)
    _pl = {}
    for _f in _cand:
        _n9 = _nrm[_f]
        _k9 = (round(float(_n9[0]), 1), round(float(_n9[1]), 1),
               round(float(_n9[2]), 1),
               round(float(np.dot(_n9, _ctr[_f])), 2))
        _pl.setdefault(_k9, []).append(_f)
    _fi, _wall_extra = [], []
    for _k9, _fs9 in _pl.items():
        _w9 = _ctr[_fs9]
        _lo9, _hi9 = _w9.min(0), _w9.max(0)
        _ext = float(max(_hi9[0] - _lo9[0], _hi9[1] - _lo9[1],
                         _hi9[2] - _lo9[2]))
        _c9 = (_lo9 + _hi9) * 0.5
        _dbnd = float(min(abs(_c9[0] - _rlo[0]), abs(_c9[0] - _rhi[0]),
                          abs(_c9[1] - _rlo[1]), abs(_c9[1] - _rhi[1])))
        # 방 경계(±250mm) 위의 **큰** 평면 = 벽. 그 밖이거나 작으면 집기.
        if _dbnd < 0.25 and _ext > 1.2:
            _wall_extra.extend(_fs9)
        else:
            _fi.extend(_fs9)
    _fi = np.array(sorted(_fi), dtype=np.int64)
    _vkey = {}
    _vid = np.zeros(len(_bp), dtype=np.int64)
    for _i9, _q9 in enumerate(np.round(_bp, 5)):
        _vid[_i9] = _vkey.setdefault(tuple(_q9), len(_vkey))
    _par = list(range(len(_vkey)))

    def _find9(a):
        while _par[a] != a:
            _par[a] = _par[_par[a]]
            a = _par[a]
        return a

    for _f in _fi:
        _vs9 = _vid[_bi[_bo[_f]:_bo[_f + 1]]]
        _r0 = _find9(int(_vs9[0]))
        for _v9 in _vs9[1:]:
            _r1 = _find9(int(_v9))
            if _r0 != _r1:
                _par[_r1] = _r0
    _grp = {}
    for _f in _fi:
        _grp.setdefault(_find9(int(_vid[_bi[_bo[_f]]])), []).append(_f)
    # ④ 형상으로 정체 판정 → 재질 (기준: 그 방 바닥 높이 위 h)
    _z_fl = float(_ctr[_is_fl][:, 2].mean()) if _is_fl.any() else _zlo
    _bind, _tally = [], {}
    for _gi, (_rt, _fs) in enumerate(sorted(
            _grp.items(), key=lambda kv: -len(kv[1]))):
        _w9 = _ctr[_fs]
        _lo9, _hi9 = _w9.min(0), _w9.max(0)
        _sz = _hi9 - _lo9
        _thin = min(_sz[0], _sz[1])
        _h_lo, _h_hi = _lo9[2] - _z_fl, _hi9[2] - _z_fl
        # (2026-08-11 사용자 지시로 **무지개 직후 첫 판정 규칙**으로 원복.
        #  이후에 넣었던 문턱 보정·"부스내벽→타일" 은 걷어냈다.)
        if _thin < 0.06 and _sz[2] > 0.6 and _h_lo < 0.35:
            _nm9, _mt9 = "샤워칸막이", _MAT_PANEL
        elif _thin < 0.06 and _h_lo > 0.8:
            _nm9, _mt9 = "거울", _MAT_MIRROR
        elif max(_sz) < 0.22 and _h_lo > 1.2:
            _nm9, _mt9 = "샤워헤드", _MAT_CHROME
        elif _h_hi < 0.85:
            _nm9, _mt9 = "세면대수납장", _MAT_WOOD
        elif _h_lo < 1.05 and _sz[2] < 0.35:
            _nm9, _mt9 = "세면대볼", _MAT_CERAMIC
        else:
            _nm9, _mt9 = "집기", _MAT_FIX
        _tally[_nm9] = _tally.get(_nm9, 0) + 1
        _bind.append((f"paint_{_gi}_{_nm9}", _fs, _mt9))
        if len(_fs) >= 4:
            print(f"   · {_nm9:8s} {len(_fs):4d}면  크기("
                  f"{_sz[0] * 1000:.0f},{_sz[1] * 1000:.0f},"
                  f"{_sz[2] * 1000:.0f})mm  바닥위 "
                  f"{_h_lo * 1000:.0f}~{_h_hi * 1000:.0f}mm")
    # ⑤ 바인딩 (기본 = 벽)
    # 🎯 `--glass2` 는 **그 층 건물까지 유리**로 만든다 (2026-08-11 사용자:
    #    로봇이 샤워 거름망 **밖으로 나오는 장면**을 찍고 싶다 — 바닥·하우징이
    #    불투명이면 탈출 순간이 통째로 가린다). 집기·바닥·천장 서브셋도
    #    같은 재질로 덮어 층 전체가 비쳐 보이게 한다.
    if GLASS2 and _who == "floor2":
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            _gl2, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        print(f"[페인트] {_who}: --glass2 — 건물 셸도 유리(탈출 촬영용)")
        continue
    UsdShade.MaterialBindingAPI.Apply(p).Bind(_MAT_WALL)
    _subset_bind(_bm, "paint_pipes", np.flatnonzero(_is_pipe),
                 pipe_mat(_who))          # 층별 유리/PVC
    _subset_bind(_bm, "paint_floor", np.flatnonzero(_is_fl), _MAT_FLOOR)
    _subset_bind(_bm, "paint_ceil", np.flatnonzero(_is_ce), _MAT_CEIL)
    for _nm9, _fs, _mt9 in _bind:
        _subset_bind(_bm, _nm9, _fs, _mt9)
    print(f"[페인트] {_who}: 유리(배관) {int(_is_pipe.sum())} 바닥 "
          f"{int(_is_fl.sum())} 천장 {int(_is_ce.sum())} 벽 "
          f"{len(_wall_extra)}+기본 / 집기 {len(_fi)}면 → 물체 "
          f"{len(_grp)}개 " + str(_tally))

# ── 🔧 결함·비드 프림 (v1_3 용접 시연 — 설계 확정 "프림 가시성 전환") ──
# 결함(표시)과 비드(숨김)를 같은 자리에 미리 두고 용접 성공 시 visibility 만
# 바꾼다. 콜라이더 없음(시각 전용) — 주행 물리에 영향 0.
# 위치: WELD_SITES_<코스> = "s_mm@시계각;..." (시계각 0=+Z 위, 90=진행방향 왼쪽)
_WELD_SITES = {}
for _nm in RUN_NAMES:
    # 🎯 결함 배치 (2026-08-11 사용자 지시): **floor1 없음 / floor2 2곳.**
    #    floor1 은 T 분기 임무(정찰)에 집중하고, 수리 시연은 floor2 가 맡는다.
    # 🎯 시계각은 **고정**(랜덤 배제)이고, **바퀴를 피하는 각도**다.
    #    자산 실측: 다리는 전부 시계 **0/120/240°** (앞뒤 줄 오프셋 없음).
    #    → 각 다리에서 가장 멀리 떨어진 자리는 **60 / 180 / 300°** (60° 여유).
    #    15°·135°·255° 는 다리에서 15° 뿐이라 바퀴에 바짝 붙어 가렸다(실측).
    #    서로 반대편이라 영상에서 구분되는 **60°** 와 **300°** 를 쓴다.
    _spec = os.environ.get(f"WELD_SITES_{_nm}",
                           {"floor1": "", "floor2": "1000@60;2000@300"}.get(
                               _nm, ""))
    _cl9 = paths[_nm]
    _sites = []
    for _tok in [t for t in _spec.split(";") if t.strip()]:
        _sv, _cv = _tok.split("@")
        _s9, _c9 = float(_sv) / 1000.0, math.radians(float(_cv))
        _i9 = int(np.argmin(np.abs(_cl9.s - _s9)))
        _p9 = _cl9.p[_i9]
        _t9 = _cl9.tangent(_i9)
        _b1 = np.array([0.0, 0.0, 1.0]) - _t9 * float(_t9[2])
        _n9 = float(np.linalg.norm(_b1))
        if _n9 < 1e-6:
            print(f"[경고] {_nm} 결함 s={_sv}: 수직 구간이라 시계각 기준 없음 "
                  f"— 생략")
            continue
        _b1 /= _n9
        _b2 = np.cross(_t9, _b1)
        _dir = _b1 * math.cos(_c9) + _b2 * math.sin(_c9)
        _rotm = Gf.Rotation(Gf.Vec3d(0, 0, 1),
                            Gf.Vec3d(*[float(v) for v in _dir]))
        _base9 = f"/World/Weld_{_nm}_{len(_sites)}"
        _dp9 = UsdGeom.Cylinder.Define(stage, _base9 + "/defect")
        _dp9.CreateRadiusAttr(0.007)
        _dp9.CreateHeightAttr(0.0012)
        _dp9.CreateAxisAttr("Z")
        _dp9.CreateDisplayColorAttr([Gf.Vec3f(0.45, 0.05, 0.05)])
        _ctr_d = _p9 + _dir * (PIPE_IR - 0.0006)
        UsdGeom.Xformable(_dp9).AddTransformOp().Set(
            Gf.Matrix4d().SetRotate(_rotm)
            * trans(float(_ctr_d[0]), float(_ctr_d[1]), float(_ctr_d[2])))
        _bp9 = UsdGeom.Cylinder.Define(stage, _base9 + "/bead")
        _bp9.CreateRadiusAttr(0.008)
        _bp9.CreateHeightAttr(0.0025)
        _bp9.CreateAxisAttr("Z")
        _bp9.CreateDisplayColorAttr([Gf.Vec3f(0.76, 0.73, 0.66)])
        _ctr_b = _p9 + _dir * (PIPE_IR - 0.0012)
        UsdGeom.Xformable(_bp9).AddTransformOp().Set(
            Gf.Matrix4d().SetRotate(_rotm)
            * trans(float(_ctr_b[0]), float(_ctr_b[1]), float(_ctr_b[2])))
        UsdGeom.Imageable(_bp9.GetPrim()).MakeInvisible()
        _sites.append(dict(s=_s9, clock=math.degrees(_c9), dir=_dir,
                           pos=_p9.copy(), tan=_t9.copy(),
                           prim_d=_dp9.GetPrim(), prim_b=_bp9.GetPrim(),
                           done=False))
    _WELD_SITES[_nm] = _sites
    if _sites:
        print(f"[준비] 🔧 {_nm} 결함 {len(_sites)}곳 — "
              + ", ".join(f"s={w['s']*1000:.0f}@{w['clock']:.0f}°"
                          for w in _sites))

tick("배관 적재 완료")
print(f"[항법] {NAV} — "
      + ("조향이 **중심선(도면)** 을 따라간다. 자율 근거로 쓰지 말 것"
         if NAV == "blueprint" else
         "조향·감속이 **로봇 신호만** 쓴다(관절·다리·휠). "
         "중심선은 채점 전용"))
print(f"[준비] 관 상태 **배수(건식)** — 마찰 {FRICTION_STATIC}/"
      f"{FRICTION_DYNAMIC}. 이 연습장에는 물이 없다"
      + "   배관 표시 반투명(유리·굴절 없음)")

# ── 로봇 3대 — **코스마다 한 대씩 동시에 굴린다** (2026-08-06 사용자 지시) ──
# 🔑 관을 하나씩 갈아 끼우며 세 번 돌리면 세 배 걸린다. 셋을 한 씬에 같이
#    올리고 각자 제 코스를 돌게 둔다. 물리는 어차피 한 스텝에 전부 푼다.
#    (실전 맵은 층 겹침 때문에 **한 번에 한 대**다 — 위 RUN_NAMES 가드 참조)
# 🎯 **임무는 편도 + 복귀 1회다** (2026-08-09 확정 — 왕복 반복 폐지). 끝에
#    닿으면 되돌아오고, 배수구 밖으로 나오면 끝이다. 끼이면 방향을 뒤집어
#    빠져나온다(임무 규칙 8 "끼임 → 후진 재시도").



CM = SON / "camera" / "meshes"
_res = os.environ.get("CAM_RES", "640x360").lower().split("x")
CAM_W, CAM_H, CAM_HFOV = int(_res[0]), int(_res[1]), 140.0
F_PX = (CAM_W / 2.0) / math.radians(CAM_HFOV / 2.0)
# 용접 카메라 화각 — **여기 한 곳에서만** 정한다(생성부·조준부 공용).
WELD_CAM_HFOV = float(os.environ.get("WELD_CAM_HFOV", 95.0))
# 🔑 **이 로봇은 관 축이 로컬 X 다** — 카메라도 ±X 를 본다(앞 세대는 ∓Z 였다).
#    실측: FrontBody 원점 x=+62mm, 로봇 앞끝 x=+94mm, 뒤끝 −94mm.
# 🚨 카메라를 본체 안에 박아 두면 안 된다(기록된 사고 — 관벽 화소 0). 앞끝
#    보다 앞으로 내고, 하우징은 센서 뒤 5mm 에 둔다.
# 🚨 부모 링크 이름이 자산마다 다르다 — welder 는 Front/RearBody, v9 는
#    DiscF/DiscR. 상수로 박으면 자산을 바꾸는 순간 **카메라가 0대로 조용히
#    건너뛰어진다**(벨로우즈에서 실제로 그랬던 기록).
# 🚨 **카메라 오프셋도 자산별이다** (2026-08-09 사용자 관찰 → 프로브 실측).
#    +35mm 는 welder 기준(몸통 원점 +62, 코끝 +94 → 카메라가 코보다 3mm 앞).
#    v9 는 앞 디스크가 로봇 맨 앞이라 +35mm 를 물려받으면 카메라·하우징이
#    **로봇 앞 허공에 떠서** 따라다닌다(프로브: 간격 35.0mm 고정 = 부착은
#    정상, 위치가 틀린 것). 곡관에서는 그 떠 있는 카메라가 벽에 제일 먼저
#    박혀 **관경 4mm 장님**의 한 원인이 된다.
# 🎯 **후방 카메라 폐지** (2026-08-11 사용자: "렌더링 진짜 빡세다").
#    로봇당 카메라는 전방 + 용접 둘뿐이다. 후진(복귀)에도 전방 영상으로
#    충분하다 — 전방 개구 중심은 **관 축**을 알려주므로 진행 방향과 무관
#    하다(발표 문서 구현사항 1 의 근거와 같다).
CAM_SPECS = [
    # 이름, 부모 링크, 링크 로컬 x, 전방 여부
    ("front_camera", TUNE.get("cam_front", "FrontBody"),
     TUNE.get("cam_x", 0.035), True),
]

# 출발점 — 입구에서 안쪽으로 120mm. 🚨 안착 중에 로봇이 뒤로 밀린다(피스톤
# 6개가 동시에 벽을 밀며 자세를 잡는 과도 구간, 토치를 달면 74mm). 입구에
# 너무 붙여 세우면 밀린 뒤 후방 휠이 관 밖 자유공간에 떠서 못 나간다.
# 🚨 **실전 맵 floor1 의 진입부는 로봇보다 짧다** (2026-08-08 실측 — 이 코스의
#    가장 중요한 제약이다. 구간을 실제로 재 보고 알았다):
#        라이저 직선   s   0 ~ 185mm   (거름망 아래 수직)
#        진입 곡관R150 s 185 ~ 421mm
#        곡관→T 직선   s 421 ~ **521mm**  ← 100mm 뿐이다
#        T 분기 원호   s 521mm ~
#    **로봇 전장 188mm 는 어느 직선 구간에도 통째로 안 들어간다**(라이저 185,
#    진입 직선 100). 구 데모의 벨로우즈는 140mm 라 라이저에 들어갔다.
# 🚨 첫 판의 0.46 은 **s=520 = T 원호 시작점 그 자리**였다 — 로봇이 접근할
#    거리 없이 분기 한복판에서 깨어나 판정이 요동쳤다(관경 74mm·이탈 48mm).
# 🚨 **배치 코드는 로봇을 직선으로 눕힌다** — 곡관에 걸친 s 를 주면 뒤쪽
#    절반이 관 벽을 파고든 채 시작한다. 0.42 로 두었더니 뒤끝이 진입 곡관
#    원호 한복판(x=324)에 박혀 뜬다리 7개·요동으로 못 나갔다(실측).
#    연습장 코스는 전부 직선에서 출발해 이 함정이 안 드러났다.
# 🔑 **온전히 직선에 들어가는 창을 계산해서 그 가운데를 쓴다** (실측):
#        진입 직선   x 480(곡관 끝) ~ 682(T 개구 시작) = **202mm**
#        로봇        188mm (앞끝 = 전방바디+32, 뒤끝 = 전방바디−156)
#        가능한 전방바디 x 636~650 → **START_S 0.577 ~ 0.591 (여유 14mm)**
#    ⚠ 여유가 14mm 뿐이다 — 이 맵의 진입부는 이 로봇에게 원리적으로 빠듯하다
#      (연습장 T 는 접근 직선이 245mm 였다). CAD 여유를 늘릴 수 있다면
#      진입 곡관과 T 사이를 300mm 이상으로 하는 것이 실물에서도 안전하다.
# 🎯 **위 제약은 188mm 판 이야기다 — welder_126 으로 풀렸다** (2026-08-08 실측,
#    2026-08-09 기본값 반영). 전장 126mm 는 라이저(s 0~185mm) 한가운데에
#    통째로 들어간다: 전방바디 기준 앞끝 = s+24 / 뒤끝 = s−102 라 **여유
#    59mm**. 두 층 모두 이탈 3mm대로 안착했다.
# 🎯 **기본 출발점 = 배수구(샤워실 사각 거름망) 안** — 임무 그대로다.
#    진입 라이저 → 진입 곡관 → T → 루프 → 복귀 → 배수구 밖(EXIT_S).
#    ⚠ **T 분기만 반복 시험할 때는 `START_S=0.585`** 를 준다 — 진입 라이저와
#      곡관을 매번 지나지 않아 시도 회전이 빠르다(구 기본값이 그 값이었다).
START_S = float(os.environ.get("START_S", 0.132))
# 코스 **끝**에서도 여유를 둔다 — 안 그러면 로봇이 관 끝 밖으로 나간다.
# 🎯 0.16 → 0.10 (2026-08-07 사용자: *"양끝을 좀 더 이동하게"*). 관을 늘리지
#    않고 기존 관의 반환점을 끝쪽으로 60mm 당겼다. s 는 전방 세그먼트 기준
#    이라 0.10 이면 전방 휠이 관 끝 ~55mm 안쪽에서 돌아선다(사출 여유 유지).
END_S = float(os.environ.get("END_S", 0.10))
# 🎯 **임무 종점 = 배수구 밖** (2026-08-08 사용자 확정). 중심선 s 가 이 값
#    아래로 내려오면 진입 라이저를 빠져나온 것으로 보고 임무를 끝낸다.
#    (s 는 채점 정보다 — 주행 판단이 아니라 **언제 끝났는지**를 정하는 데만
#     쓴다. 코스 끝 판정(`recall`)이 이미 같은 규약이다.)
EXIT_S = float(os.environ.get("EXIT_S", 0.02))
# 🎯 **완전 탈출** (2026-08-10 사용자 지시: *"완전히 튀어나와서 몸 전체가
#    배수구 관 밖으로 나올 수 있게"*). 중심선 s 는 0 에 클램프되어 관 밖을
#    못 세므로, s<EXIT_S 도달 후에는 **월드 변위**로 이만큼 더 후진한다.
# 🚨 **관구(중심선 끝) 위에 거름망 하우징이 더 있다** (2026-08-10 사용자
#    제보 → 맵 버텍스 실측으로 확정. 배관과 거름망은 **서로 다른 규격**이
#    맞물린 구조라 치수를 단정하지 말 것 — 실측이 기준):
#      z 0(관 끝, r50~55) → z+85 까지 벽 r50~55 지속(짚을 수 있음)
#      z+85 환형 시트 r50→92 → z+100 샤워 바닥면 림 r92
#    완주 거리: 래치(s=20) + 보어 85 + 시트 15 + 디스크 끝 14 + 여유 ≈ 150mm.
EXIT_EXTRA = float(os.environ.get("EXIT_EXTRA", 0.15))
# 🚨 관구 위 공동은 **짚을 벽이 없다** (실측 정정 2단계: 하우징 내벽 r92 >
#    다리 도달 83mm. z=0 의 r50~55 는 벽이 아니라 바닥판 구멍 테두리였다 —
#    "보어가 이어진다"던 1차 해석은 다리 신장 35 허공 실측으로 반증).
#    → **문턱 두 개를 분리한다** (2026-08-10 스윕 실측):
#      감속+증강 = 일찍(−20mm) / 접기 = 늦게(−3mm).
#      +10 에서 접으면 다리가 +8 에서 사각 벽(65mm)을 먼저 물고(2mm 차),
#      −20 에서 접으면 접는 순간 접지를 잃어 접기선에서 왕복(s=144↔164).
#      −3 은 s-기준 시절 최고(s=62 도달)와 같은 선 — 그때 없던 저속·증강이
#      이번엔 이미 걸려 있다. 거름망 구멍 r50, 접힌 휠 r33 만 통과.
EXIT_BORE_H = float(os.environ.get("EXIT_BORE_H", -0.003))
EXIT_SLOW_H = float(os.environ.get("EXIT_SLOW_H", -0.02))
# 🎯 **림 맨틀링** (2026-08-10): 거름망 판 윗면(h≈100)을 넘은 다리는 다시
#    편다 — 바퀴가 림 모서리를 잡고 돌아 몸을 끌어올린다(맨홀 탈출 동작).
#    전부 접힌 채 +33mm 에서 "밀 곳 없음" 정지 실측 → 관 밖 추진력은
#    림을 잡은 바퀴에서 나온다.
#    🚨 펴는 선 = **하우징 전체(사각 130 + 판 구멍)를 완전히 벗어난 뒤**
#    (2026-08-10 사용자 확정: *"DN100 보다 넓은 그곳을 탈출했을 때 펴라"*).
#    바닥면(h100)에서 펴면 편 다리가 판·림을 또 문다(GUI 2회 실측) —
#    바닥 +20mm 여유. 하우징 실측: 사각 한 변 ~130(대각 r92)·높이 85 +
#    판 15(구멍 Ø100) = 바닥면 h100.
EXIT_RIM_H = float(os.environ.get("EXIT_RIM_H", 0.120))
# 🎯 **하우징 박스 상단**(2026-08-11 실측): 관구 위 사각 박스(한 변 130 =
#    벽 65 / 대각 92) 높이 85, 그 위에 판 15(구멍 Ø100). 판 밑면에 편 다리가
#    걸리므로 **판 10mm 전(=75mm)** 에서 접는다.
EXIT_BOX_H = float(os.environ.get("EXIT_BOX_H", 0.075))
# 🚨 끼임 탈출은 임무 방향 **반대로 이만큼만** 물러나는 동작이다 — 물러난 뒤
#    임무 방향으로 복원한다. 복원이 없으면 복귀 중 끼임 한 번에 코스 끝
#    판정까지 **한 바퀴를 더 돈다**(2026-08-10 GUI 런 실측: 복귀 라이저
#    끼임 s≈200 → 전진 그대로 → 2바퀴째 s=727 에서 사용자가 창을 닫음).
UNJAM_M = float(os.environ.get("UNJAM_M", 0.05))
# 🎯 탈출 중 **아직 관 안인 다리의 예압 증강**(N). 관구 밖 세그먼트를 접고
#    나면 남은 다리 3개 마찰(0.4×9N×3=10.8N)이 자중 13.3N 을 못 이겨 수직
#    라이저에서 미끄러진다(실측: s=62 끼임 7회 반복). 30N 이면 36N > 13.3N.
#    ⚠ 30N 은 과했다 — 라이저에서 굽힘 자기쐐기(아래 EXIT_STRAIGHT_S)를
#    벽에 더 세게 박아 s=123 끼임 16회. 15N 도 마찰 54N > 자중 13.3N.
EXIT_PUSH_N = float(os.environ.get("EXIT_PUSH_N", 15.0))
# 🎯 복귀 관구 접근(s < 이 값)은 **라이저 직선** — 굽힘을 무조건 편다.
#    굽힘각을 접선↔몸축으로 재는 자기참조 탓에, 곡관에서 얻은 굽힘(F−48°)이
#    직선에서도 자기 자신을 지탱하며 관을 가로질러 버티는 **자기쐐기**가
#    실측됐다(2026-08-10 회귀 런: s=123 끼임 16회, 이탈 1~10mm 정중앙).
# 🚨 **원호 시작(164mm)보다 커야 한다** (2026-08-11 실측). 0.16 은 4mm
#    모자라 원호#0 경계(s=164)가 창 밖에 남았고, 라이저를 다 올라온 로봇이
#    거기서 "원호 진입" 판정을 받아 굽힘 −40° 를 물었다 → 직선 관에서
#    자기쐐기 + 오른손 검사 부호 반전 진동으로 s=164↔224 무한 왕복
#    (탈출 실패의 직접 원인. 다리·마찰·스트로크는 전부 정상이었다).
#    라이저 전체(관구~원호 시작 164 + 여유)를 덮는 0.20 으로.
EXIT_STRAIGHT_S = float(os.environ.get("EXIT_STRAIGHT_S", 0.20))
# 🎯 탈출 마지막 구간 속도 상한(m/s). 마지막 3다리 구간의 정지 원인은
#    접지 부족이 아니라 **휠 감쇠가 토크 예산을 먹는 것** (2026-08-10 실측:
#    45mm/s → 감쇠 0.067/0.11N·m, 78mm/s → 0.117 = 예산 초과·견인 0).
#    20mm/s 면 휠당 견인 10N × 3륜 = 30N > 자중 13.3N.
EXIT_V = float(os.environ.get("EXIT_V", 0.02))
# 🎯 **탈출 강제 상승** (2026-08-11 사용자 지시: *"수직곡관에서 한 30mm만
#    강제로 더 올라가게 만들면 되는 거 아니냐"*). 라이저 탈출은 제어로
#    풀려던 시도가 반복 실패했고(다리·마찰·스트로크 전부 정상인데 정체),
#    blueprint 는 어차피 **아는 지형을 못박는** 모드다 — 정체하면 그냥
#    올려 준다. 0 이면 끔.
EXIT_ASSIST_M = float(os.environ.get("EXIT_ASSIST_M", 0.030))
EXIT_ASSIST_S = float(os.environ.get("EXIT_ASSIST_S", 1.2))   # 이만큼 정체하면
EXIT_ASSIST_T = float(os.environ.get("EXIT_ASSIST_T", 0.6))   # 이 시간에 걸쳐
EXIT_ASSIST_MOUTH_S = float(os.environ.get("EXIT_ASSIST_MOUTH_S", 0.02))
EXIT_ASSIST_V = float(os.environ.get("EXIT_ASSIST_V", 0.040))  # 연속 상승 속도

robots = []



def discover(root):
    """로봇 서브트리에서 **역할별 조인트**를 찾는다 — 이름이 아니라 구조로.

    🚨 두 자산의 이름 규약이 완전히 다르다(`seg0_piston_0` ↔ `RearBody_A0`).
       이름으로 고르면 자산을 갈아끼우는 순간 **조용히 0개**가 되어 아무것도
       안 하고, 로그만 정상으로 보인다(실제로 그렇게 돌고 있었다). 구조로 고른다:

         세그먼트 바디 = prismatic 조인트를 3개 이상 매단 body0
         다리(서스펜션) = body0 가 세그먼트 바디인 prismatic
         휠 구동        = body1 이름에 'wheel' 이 든 revolute
         중앙 관절      = 위 둘도 용접기도 아니면서 **회전 드라이브**가 있는 것

    🔑 용접기 조인트(`RingRotate`·`TorchExtend`)는 body 이름으로 걸러 낸다 —
       벨로우즈 자산에는 아예 없고, 용접기 v2 에서 중앙 관절로 오인되면
       주행 중에 토치가 돌아간다.
    """
    def b(j, k):
        t = j.GetRelationship(f"physics:body{k}").GetTargets()
        return t[0] if t else None

    joints = [p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.Joint)]
    pris = [j for j in joints if j.IsA(UsdPhysics.PrismaticJoint)]
    cnt = {}
    for j in pris:
        p0 = b(j, 0)
        if p0:
            cnt[p0] = cnt.get(p0, 0) + 1
    seg_bodies = [p for p, c in cnt.items() if c >= 3]

    legs, wheels, bend, weld = [], [], [], []
    for j in joints:
        p0, p1 = b(j, 0), b(j, 1)
        n0 = p0.name if p0 else ""
        n1 = p1.name if p1 else ""
        if any(k in (n0 + n1).lower() for k in ("weld", "ring", "torch")):
            weld.append(j)
            continue
        if j.IsA(UsdPhysics.PrismaticJoint) and p0 in seg_bodies:
            legs.append(j)
        elif "wheel" in n1.lower() and j.IsA(UsdPhysics.RevoluteJoint):
            wheels.append(j)
        elif any(a.startswith("drive:rot") or a.startswith("drive:angular")
                 for a in [x.GetName() for x in j.GetAttributes()]):
            bend.append(j)
    # 다리 → 세그먼트 묶음 (조인트 이름 → 세그먼트 번호)
    order = sorted({str(b(j, 0)) for j in legs})
    leg_seg = {j.GetName(): order.index(str(b(j, 0))) for j in legs}
    return dict(seg_bodies=seg_bodies, legs=legs, wheels=wheels,
                bend=bend, weld=weld, leg_seg=leg_seg)


def wlocal(prim):
    """프림 자기 로컬 변환의 이동 성분 (로봇 루트 기준 — 중간 Xform 은 항등)."""
    t = UsdGeom.Xformable(prim).GetLocalTransformation().ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])])


def seg_body_prims(root, jd):
    """세그먼트 바디 두 개를 (뒤, 앞) 순서로 준다.

    이름 규약이 자산마다 다르므로(`seg0_body`/`seg1_body` ↔ `RearBody`/
    `FrontBody`) 'front' 또는 '1' 이 든 쪽을 앞으로 본다.
    """
    prims = [stage.GetPrimAtPath(p) for p in jd["seg_bodies"]]
    # 🎯 **자산이 이름을 알려 주면 그것을 쓴다** (2026-08-09). 이름 휴리스틱
    #    ('front' 가 들었나 / '1' 이 들었나)은 세그먼트가 2개일 때만 통한다 —
    #    v9 는 **3개**(DiscR / Body / DiscF)라 셋 다 키가 0 이 되어 순서가
    #    사전 순에 좌우된다(= 앞뒤가 뒤집힐 수 있다).
    _fr, _rr = TUNE.get("seg_front"), TUNE.get("seg_rear")
    if _fr and _rr:
        _by = {p.GetName(): p for p in prims}
        if _fr in _by and _rr in _by:
            return _by[_rr], _by[_fr]
        raise SystemExit(f"[중단] 세그먼트 이름 {_rr}/{_fr} 을 못 찾았다 — "
                         f"자산에 있는 것: {sorted(_by)}")
    prims.sort(key=lambda p: (2 if "front" in p.GetName().lower() else 0)
               + (1 if "1" in p.GetName() else 0))
    return prims[0], prims[-1]


def cyl_to_hull(cprim, n=16):
    """실린더 프리미티브 콜라이더 → **convexHull 메시 콜라이더**로 교체.

    🚨 이 로봇은 휠 12개와 본체 2개의 충돌체가 `UsdGeom.Cylinder` 다. PhysX 5.4
       는 실린더를 **convex-core** 로 다루는데, 관 안에서 narrow phase 가
       `contactConvexCoreConvex` 에서 세그폴트를 냈고(코어 덤프), 안 죽는
       조건에서도 로봇이 관을 뚫고 나갔다. 우리 벨로우즈 로봇이 멀쩡했던 이유가
       휠이 **메시 convexHull** 이었기 때문이다(기록된 대조).
    → 같은 치수의 n각기둥 메시를 만들어 콜라이더를 옮기고, 원래 실린더는
      충돌만 끈다(형상·재질은 그대로 두어 보이는 것은 안 바뀐다).
    🚨 **`Cone` 도 같은 부류다** — PhysX 는 실린더·원뿔·캡슐을 전부 convex-core
       로 다룬다. 용접기 v2 는 토치 팁이 `Cone` 콜라이더라 이것 하나만 남겨도
       같은 세그폴트가 난다. 두 타입을 같이 받는다.
    🚨 물리 재질(`material:binding:physics`)을 **같이 옮겨야 한다** — 안 옮기면
       타이어 마찰이 조용히 씬 기본값으로 떨어진다.
    """
    is_cone = cprim.IsA(UsdGeom.Cone)
    g = UsdGeom.Cone(cprim) if is_cone else UsdGeom.Cylinder(cprim)
    r = float(g.GetRadiusAttr().Get())
    h = float(g.GetHeightAttr().Get())
    ax = str(g.GetAxisAttr().Get() or "Z")

    def put(s, c, d):
        return {"X": (s, c, d), "Y": (c, s, d)}.get(ax, (c, d, s))

    pts, idx = [], []
    if is_cone:
        for k in range(n):                          # 밑면
            a = 2.0 * math.pi * k / n
            pts.append(put(-h / 2.0, r * math.cos(a), r * math.sin(a)))
        pts.append(put(+h / 2.0, 0.0, 0.0))         # 꼭짓점
        for k in range(n):
            idx.append([k, (k + 1) % n, n])
        for k in range(1, n - 1):
            idx.append([0, k + 1, k])
    else:
        for s in (-h / 2.0, +h / 2.0):
            for k in range(n):
                a = 2.0 * math.pi * k / n
                pts.append(put(s, r * math.cos(a), r * math.sin(a)))
        for k in range(n):
            k2 = (k + 1) % n
            idx += [[k, k2, n + k2], [k, n + k2, n + k]]
        for k in range(1, n - 1):                  # 양 끝 뚜껑
            idx += [[0, k, k + 1], [n, n + k + 1, n + k]]
    m = UsdGeom.Mesh.Define(stage, str(cprim.GetPath()) + "_hull")
    m.CreatePointsAttr([Gf.Vec3f(*q) for q in pts])
    m.CreateFaceVertexCountsAttr([3] * len(idx))
    m.CreateFaceVertexIndicesAttr([i for f in idx for i in f])
    m.CreateSubdivisionSchemeAttr("none")
    mp = m.GetPrim()
    UsdPhysics.CollisionAPI.Apply(mp)
    UsdPhysics.MeshCollisionAPI.Apply(mp).CreateApproximationAttr("convexHull")
    UsdGeom.Imageable(mp).MakeInvisible()           # 충돌 전용
    # 물리 재질을 그대로 옮긴다(안 옮기면 타이어 마찰이 씬 기본값이 된다)
    _src = cprim.GetRelationship("material:binding:physics")
    if _src and _src.GetTargets():
        mp.CreateRelationship("material:binding:physics").SetTargets(
            _src.GetTargets())
    # 원래 프리미티브에서 접촉 오프셋도 물려받는다(자산이 저작한 값)
    _po = PhysxSchema.PhysxCollisionAPI.Get(stage, cprim.GetPath())
    if _po:
        _pn = PhysxSchema.PhysxCollisionAPI.Apply(mp)
        for _get, _set in ((_po.GetContactOffsetAttr, "CreateContactOffsetAttr"),
                           (_po.GetRestOffsetAttr, "CreateRestOffsetAttr")):
            _v = _get().Get() if _get() else None
            if _v is not None:
                getattr(_pn, _set)(_v)
    UsdPhysics.CollisionAPI.Apply(cprim).CreateCollisionEnabledAttr(False)
    return mp


def build_robot(name, path_cl):
    """코스 하나에 로봇 한 대. 반환 = FSM 상태를 담을 dict."""
    ref = f"/World/Ref_{name}"
    prim = stage.DefinePrim(ref, "Xform")
    # 🚨 용접기 v2 는 **씬 파일**이다 — `/World` 밑에 PhysicsScene 과 Looks 가
    #    같이 들어 있다. `/World/Robot` 서브트리만 참조하면 머티리얼 바인딩이
    #    스코프 밖이라 전부 끊긴다(**타이어 물리 재질까지** 끊겨 마찰이 바뀐다).
    #    → **파일을 통째로** 참조하고 중복 PhysicsScene 만 꺼 버린다.
    if ROBOT_PRIM_IN_USDA:
        prim.GetReferences().AddReference(ROBOT_USDA,
                                         Sdf.Path(ROBOT_PRIM_IN_USDA))
    else:
        prim.GetReferences().AddReference(ROBOT_USDA)
    _dead = [c for c in Usd.PrimRange(prim) if c.IsA(UsdPhysics.Scene)]
    for c in _dead:
        c.SetActive(False)
    # 🎥 자산 내장 Camera 프림(CamF/CamR)은 끈다 (2026-08-11 사용자 지시).
    #    CAD 작업자가 v10 견적서 ④ 렌즈 위치를 표시하며 넣은 프림이 v11 에
    #    승계된 것 — 렌더 프로덕트가 없어 RTX 비용은 0이지만 GUI 카메라
    #    목록을 오염한다(2기면 4대). 실사용 카메라는 <코스>_*_camera 뿐.
    _cam0 = [c for c in Usd.PrimRange(prim) if c.GetTypeName() == "Camera"]
    for c in _cam0:
        c.SetActive(False)
    if _cam0:
        print(f"  {name}: 자산 내장 카메라 {len(_cam0)}대 비활성 "
              f"({', '.join(c.GetName() for c in _cam0)} — CAD 표시용)")

    # 아티큘레이션 루트를 **찾는다** — 참조 프림 자신일 수도, 그 밑일 수도 있다
    root = next((c for c in Usd.PrimRange(prim)
                 if c.HasAPI(UsdPhysics.ArticulationRootAPI)), prim)
    rp = str(root.GetPath())

    # 🚨 **자체 충돌을 끈다.** 이 로봇의 휠은 실린더 프리미티브라 PhysX 가
    #    convex-core 로 다루는데, 관 안에서 암·휠이 서로 닿는 순간
    #    `contactConvexCoreConvex` 에서 **세그폴트**가 났다(코어 덤프 2회).
    #    관벽과의 접촉은 그대로 살아 있다 — 꺼지는 것은 로봇 자기들끼리다.
    PhysxSchema.PhysxArticulationAPI.Apply(root) \
        .CreateEnabledSelfCollisionsAttr(False)

    jd = discover(root)
    if not (jd["legs"] and jd["wheels"] and jd["bend"]):
        raise SystemExit(
            f"[중단] {name}: 조인트를 못 찾았다 — 다리 {len(jd['legs'])} / "
            f"휠 {len(jd['wheels'])} / 중앙 {len(jd['bend'])}. "
            f"자산 구조가 바뀌었는지 볼 것 ({ASSET})")
    for _j in jd["legs"]:
        UsdPhysics.PrismaticJoint(_j).GetUpperLimitAttr().Set(PISTON_STROKE)
        if PISTON_RETRACT > 0:
            UsdPhysics.PrismaticJoint(_j).GetLowerLimitAttr().Set(
                -PISTON_RETRACT)
        _d = UsdPhysics.DriveAPI.Get(_j, "linear")
        if _d and LEG_FORCE:
            # 순수 댐퍼 — 예압·중심 유지는 매 스텝 **힘 지령**으로 준다
            _d.GetStiffnessAttr().Set(0.0)
            _d.GetDampingAttr().Set(LEG_DAMP)
            _d.GetTargetPositionAttr().Set(0.0)
            _d.GetMaxForceAttr().Set(max(PISTON_MAXF * 8.0, 60.0))
        elif _d:
            # 🚨 예압은 **목표를 스트로크 밖에** 두는 힘 드라이브라, 스트로크를
            #    늘리면 목표도 같이 밀어야 같은 힘이 유지된다.
            _d.GetTargetPositionAttr().Set(PISTON_STROKE * 2.0)
            _d.GetMaxForceAttr().Set(PISTON_MAXF)
    _nb = 0
    for _j in jd["bend"]:
        # 🚨 중앙 관절이 revolute(`angular`)일 수도 D6(`rotY`·`rotZ`)일 수도
        #    있다 — 붙어 있는 드라이브를 **전부** 덮는다.
        for _tok in ("angular", "rotX", "rotY", "rotZ"):
            _d = UsdPhysics.DriveAPI.Get(_j, _tok)
            if _d and _d.GetStiffnessAttr():
                _d.GetStiffnessAttr().Set(BELLOWS_STIFF)
                _d.GetMaxForceAttr().Set(BELLOWS_MAXF)
                _nb += 1
    print(f"  {name}: 다리 {len(jd['legs'])} · 휠 {len(jd['wheels'])} · "
          f"중앙 관절 {len(jd['bend'])}(드라이브 {_nb}축) · "
          f"용접기 {len(jd['weld'])} — 구조로 찾음")
    print(f"  {name}: 관절 강성 {BELLOWS_STIFF} N·m/도 (상한 {BELLOWS_MAXF}), "
          f"다리 스트로크 {PISTON_STROKE * 1000:.0f}mm 예압 {PISTON_MAXF:.0f}N "
          f"→ 휠 중심 도달 "
          f"{TUNE['wheel_center_mm'] + PISTON_STROKE * 1000:.1f}mm")

    _nh = 0
    for _c in Usd.PrimRange(prim):
        if _c.GetTypeName() in ("Cylinder", "Cone") \
                and _c.HasAPI(UsdPhysics.CollisionAPI):
            cyl_to_hull(_c)
            _nh += 1
    if _nh:
        print(f"  {name}: 실린더·원뿔 콜라이더 {_nh}개 → convexHull 메시로 교체")

    _s0 = START_S
    i0 = int(np.argmin(np.abs(path_cl.s - _s0)))
    p0 = path_cl.p[i0]
    tan = path_cl.tangent(i0)
    # 배치는 각 링크 로컬 변환에 굽는다(부모 Xform·root 에 걸면 PhysX 가 발산).
    # 🔑 **출발 자세를 접선에서 만든다 — 수직 구간에서도 출발할 수 있다.**
    # 🚨 **전방 축을 자산에서 잰다.** 벨로우즈는 로컬 −Z 가 앞이고 용접기 v2 는
    #    +X 다. `rot(-90,(0,1,0))` 을 상수로 박아 두면 자산을 갈아끼우는 순간
    #    로봇이 관을 가로질러 놓인다 → 두 세그먼트 바디의 로컬 위치 차이를
    #    그대로 전방 벡터로 쓴다(이름 규약과 무관).
    _rear, _front = seg_body_prims(root, jd)
    _fw = np.array(
        wlocal(_front) - wlocal(_rear), dtype=np.float64)
    # 🔑 두 세그먼트 중심 사이 거리 = 굽힘이 걸리는 **호의 길이**. 차동 속도의
    #    곡률 반경 R = 이 길이 / 총 굽힘각 계산에 쓴다(로봇 제원, 도면 무관).
    _seg_span = float(np.linalg.norm(_fw))
    _fw = _fw / max(_seg_span, 1e-12)
    _to_x = Gf.Matrix4d(1.0)
    _to_x.SetRotate(Gf.Rotation(Gf.Vec3d(*[float(v) for v in _fw]),
                                Gf.Vec3d(1, 0, 0)))
    _into = Gf.Matrix4d(1.0)
    _into.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0),
                                Gf.Vec3d(*[float(v) for v in tan])))
    # 🚨 **출발 롤을 명시한다.** 접선이 +X 와 정반대면 `Gf.Rotation(from,to)` 가
    #    **임의의 수직축**을 고르므로 다리 클럭 위치가 실행마다 우연히 정해진다.
    #    다리는 120° 간격이라 개구와의 정렬이 통과 여부를 가를 수 있다 →
    #    `ROLL_DEG` 로 못박고 진단 때 쓸어 본다.
    place = _to_x * rot(ROLL_DEG, (1, 0, 0)) * _into \
        * trans(float(p0[0]), float(p0[1]), float(p0[2]))
    # 🚨 **직계 자식만 돌면 안 된다** — 용접기 v2 는 중앙 링크가
    #    `Robot/Bellows/Link0..3` 로 한 단 더 들어가 있다. 직계만 옮기면 중앙
    #    4링크가 원점에 남아 아티큘레이션이 통째로 찢어진다.
    #    중간 Xform(`Bellows`)이 항등이라 손자에도 같은 식이 그대로 성립한다 —
    #    항등이 아니면 경고를 찍는다(추측으로 넘어가지 않는다).
    _nl = 0
    for child in Usd.PrimRange(root):
        if not child.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        par = child.GetParent()
        while par.IsValid() and par != root:
            _pm2 = UsdGeom.Xformable(par).GetLocalTransformation()
            if float(np.abs(np.array(_pm2) - np.eye(4)).max()) > 1e-9:
                print(f"[경고] {name}: 중간 Xform {par.GetName()} 이 항등이 "
                      f"아니다 — 배치가 틀어질 수 있다")
            par = par.GetParent()
        xf = UsdGeom.Xformable(child)
        local = xf.GetLocalTransformation()
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local * place)
        _nl += 1
    print(f"  {name}: 링크 {_nl}개 배치 — 전방 축 로컬 "
          f"({_fw[0]:+.2f}, {_fw[1]:+.2f}, {_fw[2]:+.2f})")

    # ── 카메라 하우징·조명 (센서 프림은 reset 뒤에) ────────────────
    for nm, seg, x, fwd in (CAM_SPECS if CAMERAS else []):
        if not stage.GetPrimAtPath(f"{rp}/{seg}").IsValid():
            continue
        # 🎥 GUI 카메라 목록에서 층이 바로 보이게 **코스 접두사**를 붙인다
        #    (2026-08-11 사용자 지시: floor1_/floor2_ 로 구분).
        nm = f"{name}_{nm}"
        base = f"{rp}/{seg}/{nm}_rig"
        UsdGeom.Xform.Define(stage, base)
        hp, hi = load_stl(CM / "camera_housing.stl")
        hm = UsdGeom.Mesh.Define(stage, f"{base}/housing")
        hm.CreatePointsAttr([Gf.Vec3f(*q) for q in hp])
        hm.CreateFaceVertexCountsAttr([3] * len(hi))
        hm.CreateFaceVertexIndicesAttr(hi.reshape(-1).tolist())
        hm.CreateExtentAttr([Gf.Vec3f(*hp.min(0)), Gf.Vec3f(*hp.max(0))])
        hm.CreateSubdivisionSchemeAttr("none")
        # son 하우징 STL 은 광축이 +X — 전방은 그대로, 후방만 Rz(180) 으로 돌린다.
        # 🚨 센서와 같은 자리에 두면 렌즈·조명을 가둔다 → 광축 뒤 5mm.
        hm_x = x + (-0.005 if fwd else 0.005)
        UsdGeom.Xformable(hm).AddTransformOp().Set(
            (Gf.Matrix4d(1.0) if fwd else rot(180, (0, 0, 1)))
            * trans(hm_x, 0, 0))
        # 관 내부는 로봇 조명이 유일한 광원. 센서보다 4mm 앞, 광축 둘레 대칭.
        for k in range(2):
            lg = UsdLux.SphereLight.Define(stage, f"{base}/light_{k}")
            lg.CreateIntensityAttr(4.0e5)
            lg.CreateRadiusAttr(0.002)
            UsdGeom.Xformable(lg).AddTranslateOp().Set(Gf.Vec3d(
                x + (0.004 if fwd else -0.004),
                0.012 * (1 if k == 0 else -1), 0.0))

    return {"name": name, "path": rp, "jd": jd, "fw": _fw,
            "cl": path_cl, "i0": i0,
            "state": "SETTLE", "t": 0, "dir": +1, "lap": 0, "stuck": 0,
            # 🔑 창 기반 s 추적의 씨앗 — 출발 지점을 알고 시작한다(닫힌 루프).
            "s_hint": float(path_cl.s[i0]),
            "seg_span": _seg_span,
            "s_last": 0.0, "mark": 0, "dead": False, "art": None,
            "wheel": [], "seg1": None, "best": 0.0, "wheel_rad": 0.0}


for _nm in RUN_NAMES:
    robots.append(build_robot(_nm, paths[_nm]))
print(f"[준비] 로봇 {len(robots)}대 = {', '.join(r['name'] for r in robots)} "
      f"— 코스마다 한 대씩 동시에 굴린다")

# ── 휠 토크 한계 덮어쓰기 ───────────────────────────────────────────
# 🚨 **world.reset() 앞이어야 한다.** 시뮬이 시작된 뒤 USD 드라이브 속성을 쓰면
#    PhysX 로 안 넘어간다(기록된 함정).
_n_wf = 0
# 🚨 구동 조인트와 서스펜션은 **이름이 같다**(`RearBody_A0` 이 양쪽에 있다).
#    그래서 `discover()` 가 구조로 골라 둔 것을 그대로 쓴다.
for _r in robots:
    for _p in _r["jd"]["wheels"]:
        _d = UsdPhysics.DriveAPI.Get(_p, "angular")
        if _d:
            _d.CreateMaxForceAttr(WHEEL_MAXFORCE)
            if WHEEL_DAMP > 0:
                _d.CreateDampingAttr(WHEEL_DAMP)
            _n_wf += 1
print(f"[준비] 휠 토크 한계 {WHEEL_MAXFORCE:.4f} N·m × {_n_wf}개 "
      f"(휠 반경 {WHEEL_R * 1000:.0f}mm) → 로봇 1대당 합산 견인력 "
      f"{12 * WHEEL_MAXFORCE / WHEEL_R:.1f}N, 다리 "
      f"{len(robots[0]['jd']['legs'])}개 마찰 "
      f"{FRICTION_STATIC * len(robots[0]['jd']['legs']) * PISTON_MAXF:.1f}N "
      f"(여유 "
      f"{12 * WHEEL_MAXFORCE / WHEEL_R / (FRICTION_STATIC * len(robots[0]['jd']['legs']) * PISTON_MAXF):.1f}배)")

# ── 롤 드라이브 토크 한계 덮어쓰기 (2026-08-10 스케줄 조향) ─────────
# 🚨 v10 자산의 롤 드라이브 maxForce 는 2 N·m 인데 조향 저항 실측이 ~2.5 N·m
#    (다리를 벽에 끌며 도는 마찰 — v9 조원 스크립트 주석과 같은 수치)다.
#    직선(저마찰)에서는 조준이 되지만 **T 벽 접촉 토크는 못 버텨** 진입 순간
#    드럼이 되돌아갔다(t4 실측: dot +1.00 → 1초 뒤 -0.94). v9 조원도 같은
#    이유로 ROLL_MF 2→5 로 올렸다. world.reset() 앞, 런타임 지령은 위치만.
BP_ROLL_MF = float(os.environ.get("BP_ROLL_MF", 5.0))
# 🎯 굽힘 토크도 덮어쓴다 (2026-08-10 밤 — 사용자: *"v11 속성값은 절대적인
#    게 아니다, 힘이 필요하면 뚫어라"*). 자산 저작값 2.5 N·m 는 허공에서는
#    지령을 따라가지만 **벽에 눌린 몸을 꺾어 돌리는 부하에서는 부족**하다 —
#    welder 실측: 상한 15 로 올리고서야 관절이 벽을 이겼다(강성 스윕 기록).
BP_BEND_MF = float(os.environ.get("BP_BEND_MF", 0.0))   # 0 = 자산값
_n_rf = _n_bf = 0
if TUNE.get("steer_mode") == "rollbend":
    _rn3 = set(TUNE.get("roll_joints", ()))
    _bn3 = set(TUNE.get("bend_joints", ()))
    for _r in robots:
        for _p in Usd.PrimRange(stage.GetPrimAtPath(_r["path"])):
            _d = None
            if BP_ROLL_MF > 0 and _p.GetName() in _rn3:
                _d = UsdPhysics.DriveAPI.Get(_p, "angular")
                if _d:
                    _d.CreateMaxForceAttr(BP_ROLL_MF)
                    _n_rf += 1
            elif BP_BEND_MF > 0 and _p.GetName() in _bn3:
                _d = UsdPhysics.DriveAPI.Get(_p, "angular")
                if _d:
                    _d.CreateMaxForceAttr(BP_BEND_MF)
                    _n_bf += 1
    if _n_rf or _n_bf:
        print(f"[준비] 조향 토크 덮어씀 — 롤 {BP_ROLL_MF:.1f} N·m × {_n_rf}개"
              + (f", 굽힘 {BP_BEND_MF:.1f} N·m × {_n_bf}개" if _n_bf else
                 " (굽힘은 자산값)"))

# ── 용접 모듈 덮어쓰기 (v11_weld, world.reset() 앞) ─────────────────
# 🎯 2026-08-11 사용자 확정 — CAD 재요청 불가, 자산과 다른 값은 코드가 정본:
#    ① TorchExtend 상한 5mm(자산) → 35mm(설계 J2: 반경 20→48mm, 관벽 도달)
#    ② 용접부 질량 86g → 15g (주행 특성을 v10 과 사실상 동일하게 유지.
#       0 으로는 안 내린다 — 몸통 200g 대비 질량비가 벌어지면 강성 4000
#       드라이브가 붙은 관절이 떨거나 발산한다)
if TUNE.get("weld_torch_joint"):
    _n_weld = 0
    for _r in robots:
        for _p in Usd.PrimRange(stage.GetPrimAtPath(_r["path"])):
            _nm = _p.GetName()
            if _nm == TUNE["weld_torch_joint"]:
                _pj = UsdPhysics.PrismaticJoint(_p)
                _pj.CreateUpperLimitAttr(float(TUNE.get("torch_stroke",
                                                        0.035)))
                _n_weld += 1
            elif _nm == TUNE.get("weld_ring_link"):
                UsdPhysics.MassAPI.Apply(_p).CreateMassAttr(0.010)
                _n_weld += 1
            elif _nm == TUNE.get("weld_torch_link"):
                UsdPhysics.MassAPI.Apply(_p).CreateMassAttr(0.005)
                _n_weld += 1
    print(f"[준비] 용접 모듈 덮어씀 — 토치 스트로크 "
          f"{TUNE.get('torch_stroke', 0.035) * 1000:.0f}mm·질량 15g "
          f"(프림 {_n_weld}개)")

# ── 감지 밴드 (contactOffset) ───────────────────────────────────────
# 🚨 **반드시 여기서 돌린다** — 배관·로봇·토치가 전부 스테이지에 올라온 뒤이고
#    world.reset() 의 cook 보다는 앞이다. 배관 직후에 돌리면 로봇 usda 가 아직
#    AddReference 전이라 휠 콜라이더에 안 닿는다(기록된 사고).
# 🚨 **로봇 콜라이더는 건드리지 않는다** (2026-08-06 실측). 이 로봇의 휠은
#    실린더 프리미티브(r8 h12)라 PhysX 가 convex-core 로 다룬다. 거기에
#    contactOffset 0.5mm 를 강제로 씌웠더니 narrow phase 에서
#    `contactConvexCoreConvex` 로 **세그폴트**가 났다(코어 덤프).
#    → 로봇은 usda 가 저작한 값 그대로 두고, **배관에만** 건다.
_n_off = 0
for _p in stage.Traverse():
    if not str(_p.GetPath()).startswith("/World/Pipe_"):
        continue
    if _p.HasAPI(UsdPhysics.CollisionAPI) or _p.IsA(UsdGeom.Mesh):
        _px = PhysxSchema.PhysxCollisionAPI.Apply(_p)
        _px.CreateContactOffsetAttr(CONTACT_OFFSET)
        _px.CreateRestOffsetAttr(REST_OFFSET)
        _n_off += 1
_n_wheel = sum(1 for _p in stage.Traverse()
               if _p.GetName().endswith("_hull")
               and _p.HasAPI(UsdPhysics.CollisionAPI))
print(f"[준비] 주행 {TARGET_SPEED_MPS * 1000:.0f} mm/s "
      f"(한 스텝 {TARGET_SPEED_MPS / PHYSICS_HZ * 1000:.3f}mm) → "
      f"contactOffset {CONTACT_OFFSET * 1000:.2f}mm, 프림 {_n_off}개")
print(f"         (배관 프림에만 건다 — 로봇 콜라이더는 usda 값 그대로. "
      f"교체한 convexHull 콜라이더 {_n_wheel}개는 손대지 않았다)")

tick("로봇 조립 완료 (콜라이더 교체·배치 포함)")
for r in robots:
    r["art"] = SingleArticulation(prim_path=r["path"], name=f"bot_{r['name']}")
    world.scene.add(r["art"])
world.reset()
tick("world.reset() 완료 — 메시 콜라이더 cook")

# ── 초기 뷰포트 — **건물을 바라보게** (2026-08-11 사용자 지시: 기본 시점이
#    건물을 등지고 있었다). 활성 코스 중심선의 평균점을 표적으로 잡는다.
if not HEADLESS:
    try:
        from isaacsim.core.utils.viewports import set_camera_view
        _tgt = np.mean(np.vstack([paths[_nm].p for _nm in RUN_NAMES]),
                       axis=0)
        _eye = _tgt + np.array([-1.8, -1.6, 1.4])
        set_camera_view(eye=_eye.tolist(), target=_tgt.tolist())
        print(f"[준비] 뷰포트 — eye ({_eye[0]:.1f},{_eye[1]:.1f},"
              f"{_eye[2]:.1f}) → 표적 ({_tgt[0]:.1f},{_tgt[1]:.1f},"
              f"{_tgt[2]:.1f})")
    except Exception as _ve:
        print(f"[경고] 뷰포트 설정 실패(무해): {_ve}")

for r in robots:
    dof = list(r["art"].dof_names or [])
    types = [str(t).upper()
             for t in r["art"]._articulation_view.get_dof_types()]
    jd = r["jd"]
    # 🚨 **DOF 는 이름만으로 고를 수 없다** — 구동(회전)과 서스펜션(직동)이
    #    같은 이름을 쓴다(`RearBody_A0` 이 `DriveJoints` 와 `SpringJoints`
    #    양쪽에 있다). Isaac 이 뒤에 오는 쪽에 `_0` 을 붙이는데 **어느 쪽이
    #    붙는지는 보장이 없다.** → 이름(접미사·D6 축 접미사 무시) **AND
    #    DOF 종류**로 고른다. D6 조인트는 `J0:rotY` 처럼 축이 붙어 온다.
    # 🚨 **`_0` 을 무조건 떼면 안 된다.** 용접기 v2 는 이름이 겹쳐서 Isaac 이
    #    뒤쪽에 `_0` 을 붙이므로 떼야 하지만, 벨로우즈는 `seg0_wheel_0_0` 처럼
    #    **원래 `_0` 으로 끝나는 이름**이라 떼면 매칭이 깨진다(휠 6/12, 다리
    #    4/6, 중앙 3/4 로 조용히 절반만 잡혔다 — 검증이 잡아냈다).
    #    → **원래 이름을 먼저 보고, 그때만 `_0` 을 떼어 다시 본다.**
    def _base(n, names):
        b = n.split(":")[0]
        return b if b in names else b.removesuffix("_0")

    _wn = {j.GetName() for j in jd["wheels"]}
    _ln = {j.GetName() for j in jd["legs"]}
    _bn = {j.GetName() for j in jd["bend"]}
    r["wheel"] = [k for k, n in enumerate(dof)
                  if _base(n, _wn) in _wn and "ROTATION" in types[k]]
    r["piston"] = {}
    _seen = {}
    for k, n in enumerate(dof):
        if _base(n, _ln) not in _ln or "TRANSLATION" not in types[k]:
            continue
        sg = jd["leg_seg"].get(_base(n, _ln), 0)
        _seen[sg] = _seen.get(sg, 0) + 1
        r["piston"][(sg, _seen[sg] - 1)] = k
    # 🎯 세그먼트 번호 → 바디 프림 — 완전 탈출 때 "이 세그먼트가 관구 밖인가"
    #    를 위치로 판단해 다리를 접는 데 쓴다(2026-08-10).
    r["seg_prim_of"] = {}
    for _j in jd["legs"]:
        _sg2 = jd["leg_seg"].get(_j.GetName())
        if _sg2 is not None and _sg2 not in r["seg_prim_of"]:
            _tg = _j.GetRelationship("physics:body0").GetTargets()
            if _tg:
                r["seg_prim_of"][_sg2] = stage.GetPrimAtPath(_tg[0])
    # 🎯 다리 DOF → 암 프림 — 탈출 접기를 **다리 단위**로 (2026-08-10 밤).
    #    세그먼트째 접으면 몸통 두 줄(±24mm) 중 아직 벽이 있는 아랫줄까지
    #    한꺼번에 잃어 앞 3다리만 남는 구간이 일찍 온다(s=62 정지 실측).
    r["leg_arm_of"] = {}
    for _j in jd["legs"]:
        _tg = _j.GetRelationship("physics:body1").GetTargets()
        if not _tg:
            continue
        _pr2 = stage.GetPrimAtPath(_tg[0])
        _nm2 = _j.GetName()
        for k2, n2 in enumerate(dof):
            if _base(n2, _ln) == _nm2 and "TRANSLATION" in types[k2]:
                r["leg_arm_of"][k2] = _pr2
    r["bel"] = [k for k, n in enumerate(dof) if _base(n, _bn) in _bn]
    # 🚨 **개수를 상수로 박지 말 것.** 다리는 자산마다 다르다(벨로우즈 6 /
    #    용접기 v2·126 은 12 / **welder_short 는 9**)고 고쳤으면서 휠은 12 로
    #    남겨 뒀다가, 앞 세그먼트를 3다리로 줄인 short 를 멀쩡한 매핑인데도
    #    실패로 판정했다(2026-08-08 실측). **구조 탐색이 찾은 수와 비교한다** —
    #    그것이 discover() 를 둔 이유다.
    if len(r["wheel"]) != len(jd["wheels"]) \
            or len(r["piston"]) != len(jd["legs"]) \
            or not r["bel"]:
        raise SystemExit(
            f"[중단] {r['name']} DOF 매핑 실패 — "
            f"휠 {len(r['wheel'])}({len(jd['wheels'])}) / "
            f"다리 {len(r['piston'])}({len(jd['legs'])}) / "
            f"중앙 {len(r['bel'])}. DOF: {dof}")
    # 🔑 다리별 **시계각** — 자산의 조인트 부착 위치에서 읽는다(실측 0/120/240°,
    #    축방향 두 줄 ±24mm, 반경 40mm). 어느 방향 벽을 잃었는지 알아야
    #    그쪽으로 몰 수 있다. **도면이 아니라 로봇 제원이다.**
    r["leg_clock"] = {}
    for _j in jd["legs"]:
        _p0 = _j.GetAttribute("physics:localPos0").Get()
        for (s2, _i), k in r["piston"].items():
            pass
        _nm = _j.GetName()
        for k2, n2 in enumerate(dof):
            if _base(n2, _ln) == _nm and "TRANSLATION" in types[k2]:
                r["leg_clock"][k2] = math.degrees(
                    math.atan2(float(_p0[2]), float(_p0[1]))) % 360.0
    # 🎯 **바퀴별 시계각** (2026-08-08 사용자 지시 — 차동 속도용). 바퀴는
    #    다리 끝에 달리므로 **같은 이름의 다리**에서 시계각을 물려받는다
    #    (구동 조인트와 서스펜션이 이름을 공유하는 이 자산 구조가 근거).
    #    이것이 있어야 "굽힘 바깥쪽 바퀴는 더 멀리 간다"를 계산할 수 있다.
    _clk_by_name = {}
    for _j in jd["legs"]:
        _p0 = _j.GetAttribute("physics:localPos0").Get()
        _clk_by_name[_j.GetName()] = math.degrees(
            math.atan2(float(_p0[2]), float(_p0[1]))) % 360.0
    r["wheel_clock"] = {}
    for _k in r["wheel"]:
        _b = _base(dof[_k], _wn)
        if _b in _clk_by_name:
            r["wheel_clock"][_k] = _clk_by_name[_b]
    if len(r["wheel_clock"]) != len(r["wheel"]):
        print(f"[경고] {r['name']}: 바퀴 시계각 "
              f"{len(r['wheel_clock'])}/{len(r['wheel'])} 만 매핑 — "
              f"차동 속도를 끈다")

    # ── 🔧 용접 DOF·결함 사이트 (v1_3) ─────────────────────────────
    r["ring_dof"] = [k for k, n in enumerate(dof)
                     if n.split(":")[0].removesuffix("_0") == "RingRotate"]
    r["torch_dof"] = [k for k, n in enumerate(dof)
                      if n.split(":")[0].removesuffix("_0") == "TorchExtend"]
    r["weld_sites"] = _WELD_SITES.get(r["name"], [])
    r["weld"] = None
    r["ring_link"] = stage.GetPrimAtPath(
        f"{r['path']}/{TUNE.get('weld_ring_link', '')}") \
        if TUNE.get("weld_ring_link") else None
    r["torch_link"] = stage.GetPrimAtPath(
        f"{r['path']}/{TUNE.get('weld_torch_link', '')}") \
        if TUNE.get("weld_torch_link") else None
    if r["weld_sites"] and not (r["ring_dof"] and r["torch_dof"]):
        print(f"[경고] {r['name']}: 용접 DOF 못 찾음 (링 {len(r['ring_dof'])} "
              f"토치 {len(r['torch_dof'])}) — 용접 시연 생략")
        r["weld_sites"] = []

    # 조향용 — 중앙 관절의 피치/요 DOF 를 갈라 둔다 (D6 는 `J0:1`·`J0:2`)
    # ── 조향 축 매핑 — 자산 방식에 따라 갈린다 ─────────────────────
    # 🎯 **rollbend** (v9): 롤 관절이 굽힘 평면을 겨누고, 굽힘 관절 한 축이
    #    꺾는다. 피치/요 배분이 필요 없다 — 방향과 크기가 분리된다.
    #    welder 의 pitchyaw 는 D6 두 축에 sin/cos 로 나눠 주는 방식이라
    #    비틀림이 잠긴 몸에서 배분식이 요동친다(오늘 브레이크 댄스의 한 원인).
    r["roll_dof"], r["bend_dof"] = [], []
    if TUNE.get("steer_mode") == "rollbend" and STEER_ON:
        _rn = set(TUNE.get("roll_joints", ()))
        _bn = set(TUNE.get("bend_joints", ()))
        r["roll_dof"] = [k for k, n in enumerate(dof) if _base(n, _rn) in _rn]
        r["bend_dof"] = [k for k, n in enumerate(dof) if _base(n, _bn) in _bn]
        # 앞/뒤를 가른다 — **곡관은 앞만 꺾고 뒤는 자유롭게 두는 것이 최고**
        # 였다(단독 실측: 앞만 +126mm / 앞뒤 C자 +123 / 같은 부호 S자 +16).
        _fj = TUNE["bend_joints"][0]
        r["bend_front"] = [k for k, n in enumerate(dof)
                           if _base(n, _bn) == _fj]
        r["roll_front"] = [k for k, n in enumerate(dof)
                           if _base(n, _rn) == TUNE["roll_joints"][0]]
        if not (r["bend_front"] and r["roll_front"]):
            raise SystemExit(
                f"[중단] {r['name']} 롤·굽힘 DOF 매핑 실패 — "
                f"롤 {len(r['roll_dof'])} 굽힘 {len(r['bend_dof'])}. "
                f"중앙 DOF: {[dof[k] for k in r['bel']]}")
        print(f"  {r['name']}: 조향 **롤+굽힘** — 롤 {len(r['roll_dof'])} "
              f"(앞 {len(r['roll_front'])}) / 굽힘 {len(r['bend_dof'])} "
              f"(앞 {len(r['bend_front'])})")
    _ax = TUNE.get("steer")
    if _ax and STEER_ON:
        r["bel_pitch"] = [k for k, n in enumerate(dof)
                          if _base(n, _bn) in _bn and n.endswith(":" + _ax[0])]
        r["bel_yaw"] = [k for k, n in enumerate(dof)
                        if _base(n, _bn) in _bn and n.endswith(":" + _ax[1])]
        if not (r["bel_pitch"] and r["bel_yaw"]):
            raise SystemExit(
                f"[중단] {r['name']} 조향 축 매핑 실패 — 피치 "
                f"{len(r.get('bel_pitch', []))} / 요 {len(r.get('bel_yaw', []))}"
                f". 중앙 DOF 이름: {[dof[k] for k in r['bel']]}")
    else:
        r["bel_pitch"] = r["bel_yaw"] = []
    print(f"  {r['name']}: DOF {len(dof)}개 중 휠 12 · 다리 "
          f"{len(r['piston'])} · 중앙 {len(r['bel'])}"
          + (f" (조향 피치 {len(r['bel_pitch'])} / 요 {len(r['bel_yaw'])})"
             if r["bel_pitch"] else " (조향 없음)") + " 매핑 완료")
    _rear, _front = seg_body_prims(stage.GetPrimAtPath(r["path"]), jd)
    r["seg0"], r["seg1"] = _rear, _front
    # 🎯 **드럼** — 앞 롤 관절이 돌리는 링크(v9 의 SphF). 굽힘 평면의 절대
    #    방위를 여기서 읽는다(조원 스크립트와 같은 근거). 없으면 롤 제어를
    #    끄고 조용히 넘어가지 않는다 — 경고를 찍는다.
    r["drum_prim"] = None
    r["drum_rear_prim"] = None
    r["mid_prim"] = None
    if TUNE.get("steer_mode") == "rollbend":
        _root_p = stage.GetPrimAtPath(r["path"])
        for _nm2, _key in ((TUNE.get("drum", "SphF"), "drum_prim"),
                           (TUNE.get("drum_rear", "SphR"), "drum_rear_prim"),
                           (TUNE.get("seg_mid", "Body"), "mid_prim")):
            _dp = next((c for c in Usd.PrimRange(_root_p)
                        if c.GetName() == _nm2), None)
            if _dp is None:
                print(f"[경고] {r['name']}: 링크 {_nm2} 를 못 찾았다")
            r[_key] = _dp
    # 끼임 진단용 — 휠 링크 프림을 조인트에서 거꾸로 찾는다(이름 규약 무관)
    r["wheel_prims"] = []
    for _j in jd["wheels"]:
        _t = _j.GetRelationship("physics:body1").GetTargets()
        if _t:
            r["wheel_prims"].append(stage.GetPrimAtPath(_t[0]))
    r["leg_prims"] = []
    for _j in jd["legs"]:
        _t0 = _j.GetRelationship("physics:body0").GetTargets()
        r["leg_prims"].append(_t0[0].name if _t0 else "?")
    # 로봇 로컬 정규직교 프레임 — 전방을 x̂ 로 놓는다. 자산마다 전방 축이
    # 다르므로(−Z / +X) 여기서 한 번 만들어 두고 조향은 이 프레임에서만 잰다.
    _x = np.asarray(r["fw"], dtype=np.float64)
    _t = np.array([0.0, 0.0, 1.0]) if abs(_x[2]) < 0.9 \
        else np.array([1.0, 0.0, 0.0])
    _y = np.cross(_t, _x)
    _y /= max(np.linalg.norm(_y), 1e-12)
    r["fr"] = np.vstack([_x, _y, np.cross(_x, _y)])   # 행 = x̂, ŷ, ẑ

for r in robots:          # 다리를 벽에 닿은 자세로 놓는다 (첫 스텝 전)
    if r["piston"] and PISTON_INIT > 0:
        _q = np.array(r["art"].get_joint_positions())
        _q[list(r["piston"].values())] = PISTON_INIT
        r["art"].set_joint_positions(_q)
print(f"[준비] 다리 초기 신장 {PISTON_INIT * 1000:.2f}mm — 벽에 닿은 자세로 시작 "
      f"(휠 중심 {TUNE['wheel_center_mm'] + PISTON_INIT * 1000:.1f}mm, "
      f"관벽 {(PIPE_IR - WHEEL_R) * 1000:.1f}mm)")

# 카메라 프림은 **reset 뒤에** 만든다.
if not CAMERAS:
    print("[준비] 카메라 없음 — 이 연습장은 영상을 안 쓴다. "
          "필요하면 `--cam` (GUI 렌더 비용이 크게 는다)")
try:
    if not CAMERAS:
        raise ImportError("카메라 비활성")
    from isaacsim.sensors.camera import Camera             # noqa: E402
    _SQ = 0.70710678
    _ncam = 0
    for r in robots:
        for nm, seg, x, fwd in CAM_SPECS:
            # 🚨 vision 모드는 **전방 카메라만** 만든다 (2026-08-07 사용자
            #    지적 "카메라 다 켜서 렌더링 터진다"). 후방은 아무도 안 읽는데
            #    render product 가 생기는 순간 매 렌더마다 RTX 를 태운다 —
            #    4대면 카메라 8대 중 4대가 순수 낭비다.
            if NAV == "vision" and not fwd:
                continue
            # 🎯 **전방 카메라는 용접 카메라가 없는 층만** (2026-08-11 사용자:
            #    "2층 전면카메라도 끄자, 어차피 정답지로 이동하잖아").
            #    NAV=blueprint 는 도면으로 달리므로 주행에 영상이 필요 없다 —
            #    카메라는 관찰·웹 표시용이고, 결함이 있는 층은 용접 카메라가
            #    그 역할을 한다. 로봇당 정확히 1대 = 렌더 1대.
            # 🚨 **전방 카메라 비활성** (2026-08-11 사용자 지시 — 용접
            #    카메라를 되살리면서 전방은 끈다. 리소스 절약).
            #    되살리려면 `FRONT_CAM=1`. 코드는 그대로 둔다.
            if fwd and os.environ.get("FRONT_CAM", "0") != "1" \
                    and NAV != "vision":
                continue
            if not stage.GetPrimAtPath(f"{r['path']}/{seg}").IsValid():
                continue
            # 코스 접두사 — build_robot 의 리그 이름과 같은 규칙이어야 한다
            nm = f"{r['name']}_{nm}"
            cam = Camera(prim_path=f"{r['path']}/{seg}/{nm}_rig/{nm}",
                         translation=np.array([x, 0.0, 0.0]),
                         frequency=10, resolution=(CAM_W, CAM_H))
            cam.initialize()
            # 🚨 USD 카메라는 **로컬 −Z 를 본다.** 이 로봇의 전방은 +X 이므로
            #    항등이면 옆을 본다. 원하는 카메라 축(로봇 로컬 기준):
            #      전방  −Z_cam=+X, +Y_cam=+Z(화면 위)  → X_cam=−Y
            #      후방  −Z_cam=−X, +Y_cam=+Z          → X_cam=+Y
            #    행벡터 규약이라 행이 곧 카메라 축이다.
            _M = Gf.Matrix3d(0, -1, 0, 0, 0, 1, -1, 0, 0) if fwd \
                else Gf.Matrix3d(0, 1, 0, 0, 0, 1, 1, 0, 0)
            _q = Gf.Rotation(_M.ExtractRotation()).GetQuat()
            cam.set_local_pose(
                orientation=np.array(
                    [_q.GetReal(), *[float(v) for v in _q.GetImaginary()]]),
                camera_axes="usd")
            # 🔑 v1_3: 역할별 카메라를 전부 들고 있는다 — 웹 발행(동민)이
            #    "활성 카메라 1대만" 규약으로 골라 쓴다.
            r.setdefault("cams", {})["front" if fwd else "rear"] = cam
            if fwd:
                # 🚨 깊이는 어노테이터로 받는다(기록된 함정: writer 를 같은
                #    render product 에 붙이면 rgb/depth 가 깨진다).
                try:
                    cam.add_distance_to_camera_to_frame()
                except Exception as exc:
                    print(f"[경고] {r['name']} depth 어노테이터 실패({exc})")
                r["cam_front"] = cam
                # 🔑 카메라 프림을 들고 있는다 — 매 스텝 **실제 월드 자세**로
                #    "월드 오른쪽이 화면 몇 도인가"를 재기 위해서다(오른손
                #    법칙을 중력 기준으로 못박는다).
                r["cam_prim"] = stage.GetPrimAtPath(
                    f"{r['path']}/{seg}/{nm}_rig/{nm}")
            cam.set_clipping_range(0.005, 5.0)
            cam.set_focal_length(3.0 * F_PX * 1e-6)
            cam.set_horizontal_aperture(3.0 * CAM_W * 1e-6)
            try:
                cam.set_opencv_fisheye_properties(
                    cx=CAM_W / 2, cy=CAM_H / 2, fx=F_PX, fy=F_PX,
                    fisheye=[0.0, 0.0, 0.0, 0.0])
            except Exception as exc:
                print(f"[경고] {nm} 어안 설정 실패({exc}) — 핀홀로 진행")
            _ncam += 1
    # ── 🔧 토치 카메라 (v1_3, 동연 계층) — 용접링에 달아 토치가 미는
    #    방향(링 로컬 +Z = 토치 신장축)을 본다. ALIGN~ARC 동안의 활성 카메라.
    # 🎯 **용접 카메라 활성** (2026-08-11 사용자 지시로 복원). 끄려면
    #    `WELD_CAM=0`.
    for r in (robots if os.environ.get("WELD_CAM", "1") == "1" else []):
        _wr = TUNE.get("weld_ring_link")
        if not _wr:
            continue
        # 🎯 용접 카메라는 **결함이 있는 층에만** (2026-08-11 사용자 지시 —
        #    floor1 은 결함이 없으니 렌더 낭비다). 하드코딩이 아니라 결함
        #    목록 기준이라, WELD_SITES_floor1 을 주면 자동으로 다시 생긴다.
        if not _WELD_SITES.get(r["name"]):
            print(f"  {r['name']}: 결함 없음 — 용접 카메라 생략")
            continue
        _wrp = f"{r['path']}/{_wr}"
        if not stage.GetPrimAtPath(_wrp).IsValid():
            print(f"[경고] {r['name']}: {_wr} 링크 없음 — 토치 카메라 생략")
            continue
        # 🚨 **링·토치에 붙이면 안 된다** (동연 실측: 조인트 체인 하류 바디는
        #    set_local_pose 의 로컬 회전이 렌더에 반영되지 않아 48~70° 어긋
        #    난다). → 아티큘레이션 루트에 가까운 **몸통**에 붙이고 자세는
        #    매 틱 토치 팁을 향해 다시 계산한다(_aim_weld_cam).
        # 🔑 위치 = 링 중심에서 **반경 38mm**(동연 값. 22mm 는 링에 가려
        #    용접점이 프레임 밖으로 밀렸다는 그쪽 실측). 토치 팁이 41→48mm 로
        #    나가므로 카메라 앞을 지나 벽으로 내려가는 장면이 그대로 잡힌다.
        _bodyp = f"{r['path']}/{TUNE.get('seg_mid', 'Body')}"
        _tc = Camera(prim_path=f"{_bodyp}/{r['name']}_weld_camera",
                     translation=np.array([0.0, 0.0, 0.038]),
                     frequency=10, resolution=(CAM_W, CAM_H))
        _tc.initialize()
        r["weld_cam"] = _tc
        r["weld_cam_body"] = stage.GetPrimAtPath(_bodyp)
        try:
            _tc.add_distance_to_camera_to_frame()
        except Exception:
            pass
        _tc.set_clipping_range(0.003, 5.0)
        # 🚨 화각은 **한 곳에서만** 정한다 — 생성부가 env 기본값 60° 를
        #    따로 읽어 조준 계산(95°)과 어긋나 있었다(2026-08-11: 화면이
        #    결함 하나로 꽉 찬 원인). 전방 카메라와 같은 어안 설정을 쓴다.
        _fpx = (CAM_W / 2.0) / math.tan(math.radians(WELD_CAM_HFOV / 2.0))
        _tc.set_focal_length(3.0 * _fpx * 1e-6)
        _tc.set_horizontal_aperture(3.0 * CAM_W * 1e-6)
        try:
            _tc.set_opencv_fisheye_properties(
                cx=CAM_W / 2, cy=CAM_H / 2, fx=_fpx, fy=_fpx,
                fisheye=[0.0, 0.0, 0.0, 0.0])
        except Exception:
            pass
        print(f"  {r['name']}: 용접 카메라 화각 {WELD_CAM_HFOV:.0f}° "
              f"(f={_fpx:.1f}px)")
        r.setdefault("cams", {})["torch"] = _tc
        # 용접부 조명 + 아크 라이트(평소 0, ARC 때 올린다)
        _tl = UsdLux.SphereLight.Define(stage, f"{_bodyp}/torch_cam_light")
        _tl.CreateIntensityAttr(0.8e6)   # 동연 실측값(4e5 로는 어둡다)
        _tl.CreateRadiusAttr(0.002)
        UsdGeom.Xformable(_tl).AddTranslateOp().Set(
            Gf.Vec3d(0.060, 0.0, 0.020))
        _al = UsdLux.SphereLight.Define(
            stage, f"{r['path']}/{TUNE.get('weld_torch_link', _wr)}/arc_light")
        _al.CreateIntensityAttr(0.0)
        _al.CreateRadiusAttr(0.0015)
        _al.CreateColorAttr(Gf.Vec3f(0.75, 0.85, 1.0))
        r["arc_light"] = _al
        # 🎯 **용접봉 = 레몬색** (2026-08-11 사용자: 관 색과 비슷해 안 보인다).
        _lm = UsdShade.Material.Define(stage, "/World/TorchMat")
        _lms = UsdShade.Shader.Define(stage, "/World/TorchMat/S")
        _lms.CreateIdAttr("UsdPreviewSurface")
        _lms.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*[float(x) for x in os.environ.get(
                "TORCH_COLOR", "0.96,0.92,0.30").split(",")]))
        _lms.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
        _lms.CreateInput("emissiveColor",
                         Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.25, 0.23, 0.05))      # 어두운 관 속에서도 보이게
        _lm.CreateSurfaceOutput().ConnectToSource(_lms.ConnectableAPI(),
                                                  "surface")
        for _tp9 in (TUNE.get("weld_torch_link"), TUNE.get("weld_ring_link")):
            _pr9 = stage.GetPrimAtPath(f"{r['path']}/{_tp9}") if _tp9 else None
            if _pr9 and _pr9.IsValid():
                UsdShade.MaterialBindingAPI.Apply(_pr9).Bind(
                    _lm,
                    bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        _ncam += 1
    print(f"[준비] 카메라 {_ncam}대 (어안 {CAM_HFOV:.0f}°, {CAM_W}x{CAM_H}) "
          f"— 이름 <코스>_{{front,weld}}_camera "
          f"(결함 있는 층은 weld, 없는 층은 front 한 대씩). "
          f"발행은 상황에 맞는 1대")
except ImportError:
    pass
except Exception as exc:
    print(f"[경고] 카메라 초기화 실패 ({exc}) — 하우징·조명만 남는다")


# ── 🔧 용접 카메라 재조준 (동연 방식) ─────────────────────────────────
# 🚨 **고정 로컬 오프셋으로는 못 맞춘다** — 로봇이 정렬·용접하며 자세를 바꾸고
#    링이 돌기 때문이다(동연 실측: 사용자 "정확히 토치쪽을 바라봐야 함").
#    ALIGN/EXTEND/ARC 매 틱, 링 중심 기준 반경 방향으로 카메라를 옮기고
#    팁 끝을 다시 겨눈다. 광축을 위로 살짝(0.28≈16°) 틀어 용접점이 화면
#    아래쪽에 오게 하고, 90° 롤을 준다(그쪽이 관람하기 좋다는 사용자 확인).
# 🔑 **구도는 자산 기하가 정한다.** 동연 로봇은 용접봉이 반경 20→48mm 로
#    나가 카메라(38mm) 앞을 가로질렀지만, v11 은 팁이 **41→48mm** 로 7mm 만
#    움직인다 — 같은 자리에 두면 토치·링이 화면을 꽉 채워 아무것도 안 보인다
#    (2026-08-11 사용자: "전혀 담지 못해"). → **토치 옆(축 둘레 90°)에서
#    측면으로** 본다: 용접봉이 벽으로 내려가는 것도, 결함도, 스파크도 한 화면.
# 🚨 몸통 안(반경 30·축 22mm)에 두면 **로봇 제 부품이 시야를 막는다**
#    (2026-08-11 사용자: "너무 붙어있어서 확인이 안돼"). 관 안에는 반경으로
#    물러날 자리가 없다(바퀴 40~48 · 관벽 50) → **축방향으로 코 앞까지 빼고**
#    화각을 좁혀 용접부를 당겨 본다(동연도 같은 취지의 주석을 남겼다).
WELD_CAM_R = float(os.environ.get("WELD_CAM_R", 0.038))    # (미사용)
WELD_CAM_OFF = float(os.environ.get("WELD_CAM_OFF", 0.036))   # 원주 90° 옆으로
WELD_CAM_AX = float(os.environ.get("WELD_CAM_AX", 0.000))      # 중심선 위, 링에서 앞으로
WELD_CAM_TILT = float(os.environ.get("WELD_CAM_TILT", 0.0))
WELD_CAM_LIFT = float(os.environ.get("WELD_CAM_LIFT", 0.012))  # (미사용)
WELD_CAM_CIRC = float(os.environ.get("WELD_CAM_CIRC", 60.0))  # 결함에서 원주각
# 감시자 시점 — 결함 기준. 결함 반대편으로 물러난 거리 / 관 축 앞쪽 비껴선 거리
WELD_CAM_BACK7 = float(os.environ.get("WELD_CAM_BACK7", 0.030))
WELD_CAM_AHEAD7 = float(os.environ.get("WELD_CAM_AHEAD7", 0.075))
WELD_CAM_ROLL = float(os.environ.get("WELD_CAM_ROLL", 90.0))


def weld_tip_world(r):
    """토치 팁의 월드 위치 — **링의 실제 자세**에서 낸다(결함 방향이 아니라).
    링이 돌면 이 값이 같이 돌고, 카메라도 따라 돈다."""
    if r.get("ring_link") is None or r.get("torch_link") is None:
        return None
    _e = wrot(r["ring_link"]) @ np.array([0.0, 0.0, 1.0])
    return wpos(r["torch_link"]) + _e * float(
        TUNE.get("torch_tip_r0", 0.041))


def aim_weld_cam(r, tip_world=None, diag=False):
    """동연 `aim_torch_camera` 와 **같은 로직** (2026-08-11 사용자 지시:
    *"용접링 위치만 다르잖아, 그 로직은 그대로 반영할 수 있잖아"*).
      기준점 = 토치가 실제로 달린 자리(용접링 중심)
      카메라 = 기준점에서 팁 방향으로 WELD_CAM_R(38mm)
      광축   = 팁 방향 + 위쪽 0.28(≈16°, 용접점을 화면 아래로) · 롤 90°
    """
    _cam = r.get("weld_cam")
    if _cam is None or r.get("ring_link") is None:
        return
    if tip_world is None:
        tip_world = weld_tip_world(r)
        if tip_world is None:
            return
    _Rb = wrot(r["weld_cam_body"])
    _ref = wpos(r["ring_link"])            # 토치 장착 위치(링 중심)
    _f = tip_world - _ref
    _d = float(np.linalg.norm(_f))
    if _d < 1e-6:
        return
    _f = _f / _d
    _e0 = _f.copy()                        # 링 반경 방향(팁 쪽) — 롤 기준
    # 🎯 **FPS 구도** (2026-08-11 사용자 레퍼런스): 카메라가 총 **뒤**에
    #    있어야 총열과 총구, 그리고 그 너머 표적이 한 화면에 들어온다.
    #    링 중심에서 팁 **반대쪽**으로 물러나고(관 축을 가로질러) 축방향으로
    #    비껴 서면, 토치 몸통이 화면 앞쪽에 걸리고 팁·관벽이 그 너머로 보인다.
    # 🚨 **몸체에 파고들면 아무것도 안 보인다** — 동연이 겪은 그 실패와
    #    같은 증상(사용자: "화면이 너무 확대돼서 아무것도 안 보여").
    #    반경 32mm·축 50mm 자리는 다리 암(반경 40mm, x 55~75) 사이라 막힌다.
    #    → **관 축 위, 로봇 코 앞**(축 +100mm, 반경 0)으로 뺀다. 거기서
    #    뒤돌아보면 앞 디스크(r22)는 12° 만 가리고, 토치(r41~50)는 27° 쪽에
    #    있어 **총열처럼 화면에 걸린다**. 동연 구조(축방향으로 떨어져 축을
    #    바라봄)와 같은 관계이고, 자리는 우리 로봇 기하에 맞춘 것이다.
    # 🚨 **몸통 축으로 곧게 100mm 나가면 곡관에서 관을 뚫는다**
    #    (2026-08-11 사용자 지적, 옳다). 관 밖으로 나간 카메라는 벽 속을
    #    보므로 화면이 새까맣거나 극단적 근접이 된다.
    #    → blueprint 는 중심선을 안다. **카메라를 중심선 위에** 올려
    #    관 안에 있는 것을 보장한다(곡관에서도 자동으로 휘어 따라간다).
    # 🎯 **링 평면 옆자리** (2026-08-11, 저장 프레임 실측으로 확정).
    #    앞쪽(축 110mm)에서 보면 시선이 x≈65 에서 반경 34mm 를 지나는데
    #    거기를 **다리 암이 몸통(r16)→바퀴(r40) 로 가로지른다** → 렌즈가
    #    통째로 막혀 화면이 균일한 회색이었다(저장 프레임 확인).
    #    로봇에서 유일하게 트인 곳은 **링 평면(x≈0)** 이다 — 몸통 다리는
    #    x=±24 두 줄이라 그 사이가 비어 있다. 거기서 토치를 **옆(원주
    #    방향 90°)** 에서 본다: 시선이 다리 줄을 지나지 않는다.
    # 🎯 **링 평면 옆자리** (2026-08-11 사용자 확정: 결함 기준 감시자 시점은
    #    로봇 코앞이라 앞 디스크가 화면을 채웠다 — 직전 방식이 낫다).
    #    바퀴 가림은 카메라를 옮겨서가 아니라 **결함 각도를 다리 사이로**
    #    두어 피한다(사용자 제안 — 90° 결함이 딱 안 가렸다).
    _axw = _Rb @ np.array([1.0, 0.0, 0.0])
    _sidew = np.cross(_axw, _e0)
    _ns9 = float(np.linalg.norm(_sidew))
    if _ns9 < 1e-6:
        return
    # 🎯 카메라의 **원주 각도**를 결함에서 +90° → +45° 로 (2026-08-11
    #    사용자 지적: 다리가 120° 간격이면 그 사이를 쓰면 되는데 왜 가장
    #    자리냐). 90° 를 쓰면 결함~카메라 구간이 다리 사이(120°)를 거의
    #    다 먹어 결함이 가장자리로 밀린다. 45° 면 카메라가 **다리 사이
    #    정중앙**(결함 15°→카메라 60°, 255°→300°)에 서고 시선 구간도
    #    절반이라 여유가 두 배다. **결함 각도는 그대로 둔다.**
    _th7 = math.radians(WELD_CAM_CIRC)
    _dirc = _e0 * math.cos(_th7) + (_sidew / _ns9) * math.sin(_th7)
    _pos_w = (_ref + _dirc * WELD_CAM_OFF + _axw * WELD_CAM_AX)
    _w7 = r.get("weld")
    if _w7 is not None:                       # 용접 중엔 팁·결함 중간 조준
        _st7 = _w7["site"]
        tip_world = (np.asarray(tip_world)
                     + _st7["pos"] + _st7["dir"] * PIPE_IR) * 0.5
    _f = tip_world - _pos_w                    # 팁을 향해 다시 겨눈다
    _f /= max(float(np.linalg.norm(_f)), 1e-9)
    # 🎯 **링과 같이 돈다** (2026-08-11 사용자 확인): 화면의 "위" 를 월드 Z
    #    로 고정하면 링이 90° 돌 때 토치가 화면에서 옆으로 미끄러진다.
    #    위쪽 기준을 **링의 반경 방향 반대**로 잡으면, 링이 어디로 돌든
    #    토치·용접점이 화면 **아래쪽 같은 자리**에 온다(총을 든 시점처럼).
    _up = -_e0
    if abs(float(np.dot(_f, _up))) > 0.95:     # 시선과 겹치면 축으로 대체
        _up = _Rb @ np.array([1.0, 0.0, 0.0])
    _f = _f + _up * WELD_CAM_TILT          # 광축을 위로 살짝(용접점을 아래로)
    _f /= np.linalg.norm(_f)
    _rt = np.cross(_f, _up)
    _rt /= max(float(np.linalg.norm(_rt)), 1e-9)
    _u2 = np.cross(_rt, _f)
    _rl = math.radians(WELD_CAM_ROLL)      # 광축 롤
    _c, _sn = math.cos(_rl), math.sin(_rl)
    _rt2 = _rt * _c + np.cross(_f, _rt) * _sn
    _u3 = _u2 * _c + np.cross(_f, _u2) * _sn
    # USD 카메라: −Z 시선, +Y 화면 위. wrot 은 열 기저 → Gf(행 기저)에 전치.
    _Ml = _Rb.T @ np.column_stack([_rt2, _u3, -_f])
    _M = Gf.Matrix3d(*[float(v) for v in _Ml.T.reshape(-1)])
    _q = Gf.Rotation(_M.ExtractRotation()).GetQuat()
    _cam.set_local_pose(
        translation=_Rb.T @ (_pos_w - wpos(r["weld_cam_body"])),
        orientation=np.array([_q.GetReal(),
                              *[float(v) for v in _q.GetImaginary()]]),
        camera_axes="usd")
    if diag:
        # 📐 화면 좌표 계측 — 팁·결함이 **화면 안에 있는가**를 각도로 낸다.
        #    화각 절반보다 크면 프레임 밖이다(추측 대신 숫자로 가린다).
        _Rc = wrot(_cam.prim)
        _vw = _Rc @ np.array([0.0, 0.0, -1.0])     # 시선
        _rw = _Rc @ np.array([1.0, 0.0, 0.0])      # 화면 오른쪽
        _uw = _Rc @ np.array([0.0, 1.0, 0.0])      # 화면 위
        _cp = wpos(_cam.prim)
        _hf = WELD_CAM_HFOV / 2.0
        _vf = math.degrees(math.atan(math.tan(math.radians(_hf))
                                     * CAM_H / CAM_W))
        _tgt = [("팁", tip_world)]
        _w9 = r.get("weld")
        if _w9 is not None:
            _tgt.append(("결함", _w9["site"]["pos"]
                         + _w9["site"]["dir"] * PIPE_IR))
        _tgt.append(("링", _ref))
        _msg = []
        for _nm, _pt in _tgt:
            _d9 = _pt - _cp
            _z9 = float(np.dot(_d9, _vw))
            _x9 = float(np.dot(_d9, _rw))
            _y9 = float(np.dot(_d9, _uw))
            if _z9 <= 1e-6:
                _msg.append(f"{_nm}=뒤쪽(안보임)")
                continue
            _ax9 = math.degrees(math.atan2(_x9, _z9))
            _ay9 = math.degrees(math.atan2(_y9, _z9))
            _in = "안" if (abs(_ax9) < _hf and abs(_ay9) < _vf) else "**밖**"
            _msg.append(f"{_nm}={float(np.linalg.norm(_d9))*1000:.0f}mm "
                        f"좌우{_ax9:+.0f}°/상하{_ay9:+.0f}° 프레임{_in}")
        # 시선이 로봇 축에 최근접하는 반경 — 22mm(앞 디스크)보다 작으면
        # 디스크가 시야를 막는다. 40mm 를 넘으면 다리 암에 걸린다.
        _a0 = wpos(r["weld_cam_body"])
        _ah = _Rb @ np.array([1.0, 0.0, 0.0])
        _seg = tip_world - _cp
        _rmin = 1e9
        for _t9 in np.linspace(0.0, 1.0, 21):
            _q9 = _cp + _seg * _t9
            _v9 = _q9 - _a0
            _rr = float(np.linalg.norm(_v9 - _ah * float(np.dot(_v9, _ah))))
            _xx = float(np.dot(_v9, _ah))
            if -0.030 < _xx < 0.090:          # 로봇 몸통이 있는 축 구간
                _rmin = min(_rmin, _rr)
        _blk = ("디스크가 가림" if _rmin < 0.022 else
                "다리에 걸림" if _rmin > 0.040 else "트임")
        print(f"           [용접캠] 화각 {WELD_CAM_HFOV:.0f}°(반 {_hf:.0f}°) "
              f"세로반 {_vf:.0f}° | " + " | ".join(_msg)
              + f" | 시선 최근접반경 {_rmin * 1000:.0f}mm → {_blk}")
        # 자가검증 — 조준이 실제로 팁을 향하는가(규약 실수는 여기서 드러난다)
        _vw = wrot(_cam.prim) @ np.array([0.0, 0.0, -1.0])
        _to = tip_world - wpos(_cam.prim)
        _dd = float(np.linalg.norm(_to))
        _ang = math.degrees(math.acos(max(-1.0, min(1.0, float(
            np.dot(_vw, _to / max(_dd, 1e-9)))))))
        print(f"           [용접캠] 팁까지 {_dd * 1000:.0f}mm · 조준오차 "
              f"{_ang:.1f}° (0 이면 정확히 겨눔)")


# ── 🔍 검출 창 (--detect) ────────────────────────────────────────────
try:
    import cv2                                             # noqa: E402
except ImportError:
    cv2 = None
_DETW = {"n": 0, "on": DETECT_WIN and not HEADLESS and cv2 is not None,
         "path": "/dev/shm/cobot3_detect.jpg",
         "tmp": "/dev/shm/cobot3_detect_tmp.jpg"}
if _DETW["on"]:
    print(f"[준비] 🔍 검출 프레임 → {_DETW['path']} (10Hz) — 볼 때:\n"
          f"         python3 {SON}/tools/detect_view.py")


def detect_window(r):
    """전방 카메라 프레임 + 검출 오버레이를 창으로 띄운다."""
    if not _DETW["on"]:
        return
    _c = r.get("cams", {}).get("front") or r.get("cams", {}).get("torch")
    if _c is None:
        return
    try:
        _im = _c.get_rgba()
        if _im is None or not getattr(_im, "size", 0):
            return
        img = np.ascontiguousarray(np.asarray(_im)[:, :, :3].astype(np.uint8))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        h, w = img.shape[:2]
        cx0, cy0 = w // 2, h // 2
        # ① 화면 중앙 십자 — 카메라 광축
        cv2.line(img, (cx0 - 14, cy0), (cx0 + 14, cy0), (90, 90, 90), 1)
        cv2.line(img, (cx0, cy0 - 14), (cx0, cy0 + 14), (90, 90, 90), 1)
        # ② 관경 추적 링 (θ=50° 등거리 어안 → 화면 반경 f·θ)
        _rr = int(F_PX * math.radians(50.0))
        cv2.circle(img, (cx0, cy0), _rr, (200, 160, 60), 1)
        cv2.putText(img, "bore ring", (cx0 + _rr - 60, cy0 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 160, 60), 1)
        # ③ 센터링 — 개구(관 저 끝) 중심: 가장 어두운 큰 덩어리
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _thr = max(12, int(np.percentile(gray, 8)))
        _m = (gray <= _thr).astype(np.uint8)
        _n, _lab, _st, _ce = cv2.connectedComponentsWithStats(_m, 8)
        _best = None
        for i in range(1, _n):
            if _st[i, cv2.CC_STAT_AREA] < 300:
                continue
            if _best is None or _st[i, cv2.CC_STAT_AREA] > _st[_best,
                                                               cv2.CC_STAT_AREA]:
                _best = i
        if _best is not None:
            ax, ay = int(_ce[_best][0]), int(_ce[_best][1])
            cv2.circle(img, (ax, ay), 9, (60, 220, 60), 2)
            cv2.arrowedLine(img, (cx0, cy0), (ax, ay), (60, 220, 60), 2,
                            tipLength=0.2)
            _inc = math.degrees(math.hypot(ax - cx0, ay - cy0) / F_PX)
            _clk = math.degrees(math.atan2(ay - cy0, ax - cx0))
            cv2.putText(img, f"center err {_inc:.1f}deg @{_clk:+.0f}",
                        (8, h - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (60, 220, 60), 1)
        # ④ 결함 검출 — 동연 finders (키워드 인자 필수)
        if _DY_FINDERS is not None:
            # 🚨 **Depth 를 반드시 넘긴다** (2026-08-11 실측: 없으면 관 저 끝을
            #    결함으로 오검해 발견 전부터 박스가 뜬다). finders 주석 그대로
            #    — 벽면 구멍은 테두리가 근접 벽(≈0.1m), 관 저 끝은 0.3~0.5m 다.
            _dep = None
            try:
                _fr = _c.get_current_frame()
                _dep = _fr.get("distance_to_camera")
            except Exception:
                _dep = None
            try:
                _hole = _DY_FINDERS.find_wall_hole(
                    np.asarray(_im)[:, :, :3].astype(np.uint8),
                    expect_px=None, depth=_dep)
            except Exception:
                _hole = None
            # 🎯 깊이 판정을 **통과한 것만** 그린다 (사용자: 발견 전에는
            #    출력되지 않게). 깊이가 없으면 아예 안 그린다.
            if _hole and (_dep is None or _hole.get("rim_m", 9.9)
                          > DETECT_NEAR_M):
                _hole = None
            if _hole:
                _hx, _hy = int(_hole["cx"]), int(_hole["cy"])
                _hr = max(6, int(_hole.get("r_eq_px", 8)))
                cv2.circle(img, (_hx, _hy), _hr + 4, (0, 80, 255), 2)
                cv2.rectangle(img, (_hx - _hr - 6, _hy - _hr - 6),
                              (_hx + _hr + 6, _hy + _hr + 6), (0, 80, 255), 1)
                cv2.putText(img, f"DEFECT {_hole['area_px']:.0f}px",
                            (_hx - _hr, _hy - _hr - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 80, 255), 2)
        # ⑤ **단절 판정** (설계 문서 8.2 근거 그대로): 관이 끊기면 그 너머는
        #    빈 공간이라 Depth 가 **무효(inf)** 로 돌아오고, 캡으로 막혔으면
        #    **전방 거리가 급감**한다. 둘 다 여기서 직접 재서 띄운다
        #    (2026-08-11 사용자: floor2 단절이 뷰어에 안 나온다).
        _dz = None
        try:
            _dz = _c.get_current_frame().get("distance_to_camera")
        except Exception:
            _dz = None
        if _dz is not None and getattr(_dz, "size", 0):
            _d9 = np.asarray(_dz, dtype=np.float64)
            _fin = np.isfinite(_d9) & (_d9 > 0)
            _inv = 1.0 - float(_fin.mean())
            # 전방 거리 = 화면 중앙 패치(±10%)의 중앙값
            _hh, _ww = _d9.shape[:2]
            _p9 = _d9[int(_hh * 0.4):int(_hh * 0.6),
                      int(_ww * 0.4):int(_ww * 0.6)]
            _pf = _p9[np.isfinite(_p9) & (_p9 > 0)]
            _fr9 = float(np.median(_pf)) if _pf.size else 0.0
            # 🚨 **전방 거리는 단절 신호가 아니다** (2026-08-11 실측):
            #    주행 중에도 69~147mm 를 오가 100mm 문턱을 계속 넘나든다 —
            #    관 안에서는 곡관·벽을 가까이 보는 게 정상이기 때문이다.
            _dis = _inv > DETECT_INV_MAX
            if os.environ.get("DETECT_DIAG") == "1" and _DETW["n"] % 20 == 0:
                print(f"           [단절판정] 무효 {_inv * 100:5.1f}% "
                      f"(문턱 {DETECT_INV_MAX * 100:.0f}) 전방 "
                      f"{_fr9 * 1000:5.0f}mm (문턱 "
                      f"{DETECT_STOP_M * 1000:.0f}) → "
                      f"{'단절' if _dis else '정상'}")
            cv2.putText(img, f"depth invalid {_inv * 100:.1f}%   "
                             f"forward {_fr9 * 1000:.0f}mm",
                        (8, h - 46), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (200, 200, 90), 1)
            if _dis:
                cv2.rectangle(img, (0, 26), (w, 52), (0, 0, 140), -1)
                cv2.putText(img, "!! DISCONNECTED / DEAD END !!",
                            (int(w * 0.22), 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # ⑦ 상태 줄
        _w9 = r.get("weld")
        _st9 = _w9["st"] if _w9 else r.get("state", "")
        _s9 = float(r.get("s_hint") or 0.0) * 1000.0
        cv2.rectangle(img, (0, 0), (w, 22), (30, 30, 30), -1)
        cv2.putText(img, f"{r['name']}  {_st9}  s={_s9:.0f}mm", (8, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1)
        # 🚨 Isaac 내장 cv2 는 **headless 빌드**라 `imshow` 가 없다(실측:
        #    "The function is not implemented ... Rebuild with GTK+").
        #    tkinter·PyQt5 도 없다. → 프레임을 RAM 디스크에 쓰고 **별도
        #    뷰어 프로세스**(시스템 python3, GUI cv2)가 띄운다.
        # 🚨 임시파일도 **.jpg 확장자**여야 한다 — cv2 는 확장자로 인코더를
        #    고르므로 ".tmp" 면 "could not find a writer" 로 실패한다(실측).
        _ok, _buf = cv2.imencode(".jpg", img,
                                 [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if _ok:
            with open(_DETW["tmp"], "wb") as _fh:
                _fh.write(_buf.tobytes())
            os.replace(_DETW["tmp"], _DETW["path"])
            # 🎯 **웹으로도 보낸다** (2026-08-11 사용자 지시) — 동연 계층의
            #    `opencv_debug` 토픽이 바로 이 용도다(판정 근거 시각화).
            #    로컬 뷰어와 같은 그림이 웹 화면에도 그대로 뜬다.
            _dy = r.get("dy_pub")
            if _dy is not None and RB is not None:
                try:
                    from sensor_msgs.msg import CompressedImage as _CI8
                    _m8 = _CI8()
                    _m8.header.stamp = RB.node.get_clock().now().to_msg()
                    _m8.header.frame_id = "detect"
                    _m8.format = "jpeg"
                    _m8.data = _buf.tobytes()
                    _dy["debug"].publish(_m8)
                except Exception:
                    pass
        _DETW["n"] += 1
    except Exception as _de:
        if _DETW["n"] == 0:
            print(f"[경고] 검출 창 실패(끔): {_de}")
        _DETW["on"] = False


# ── ✨ 용접 스파크 (welder/spark_fx.py) ──────────────────────────────
# 🎯 세기는 **초기값 쪽으로 낮춘다** (2026-08-11 사용자: 이전 강한 설정은
#    용접 장면을 묻어버렸다. 크게 키웠던 이유였던 "멀어서 안 보임" 은 이제
#    용접 카메라가 근접해 보므로 성립하지 않는다).
#      발생률 700 → 250/s (주석의 초기 실측 120/s 와 현행 700 의 사이)
#      발광 배율 22/12/5 → 8/5/2.5 (블룸이 화면을 태우지 않게)
#    둘 다 env 로 조절: SPARK_RATE · SPARK_GAIN (0 이면 스파크 끔)
SPARK_RATE = float(os.environ.get("SPARK_RATE", 250.0))
SPARK_GAIN = float(os.environ.get("SPARK_GAIN", 8.0))
if SPARK_RATE > 0 and not HEADLESS:
    try:
        sys.path.insert(0, str(SON))
        from welder import spark_fx as _sfx                 # noqa: E402
        _sfx._R *= float(os.environ.get("SPARK_SCALE", 1.6))
        _sfx._L *= float(os.environ.get("SPARK_SCALE", 1.6))
        _sfx._BALL_R *= float(os.environ.get("SPARK_SCALE", 1.6))
        _sfx._STAGES = tuple(
            (nm, rgb, frac, SPARK_GAIN * mul)
            for (nm, rgb, frac, _g), mul in zip(
                _sfx._STAGES, (1.0, 0.625, 0.3125)))
        for r in robots:
            if not r.get("weld_sites"):
                continue
            r["sparks"] = _sfx.SparkFX(
                stage, f"/World/Sparks_{r['name']}", flooded=False)
            r["sparks"].p["rate"] = SPARK_RATE
        print(f"[준비] ✨ 용접 스파크 — 발생률 {SPARK_RATE:.0f}/s, 발광 "
              f"×{SPARK_GAIN:.1f} (SPARK_RATE=0 으로 끔)")
    except Exception as _se:
        print(f"[경고] 스파크 초기화 실패(무해): {_se}")

# ── 🎥 관 밖 관찰 카메라 (--obs) ────────────────────────────────────
# 🔑 좌표는 **코스에서 직접 계산**한다(하드코딩 아님): 단절 끝·지정 코너를
#    코스 점에서 뽑고, 관에서 비스듬히 물러난 자리에 세워 그 지점을 겨눈다.
# 🚨 시야 가림 방지: ① 관과 같은 높이가 아니라 **위로 130mm** 띄운다
#    (바닥 슬래브·관 자체에 안 묻힌다) ② 방 안쪽(x 0.33~1.50, y 0.10~1.40)
#    을 벗어나지 않게 잡는다 — 밖으로 나가면 복도 벽(유리 아님)이 가린다.
tick("카메라·센서 준비 완료")

# ── 🌐 웹 연결 (동민 ros_bridge — 규약 단일 출처 pipe_comm/contract.py) ──
# 로봇마다 네임스페이스(/floor1/..., /floor2/...) — both 모드에서도 안 섞인다.
# 발행: 상태/odom/imu/joint_states(바퀴)/코스/메시/사건 + 활성 카메라 1대.
# 수신: mission(시작·정지), cmd_vel. ROS_PUB=0 으로 끈다.
RB = None
_DY_FINDERS = None
# 🚨 **ROS 발행 기본 끔** (2026-08-11 사용자 지시 — 실측상 가장 비싸다:
#    1.21x → 0.93x). 웹 연동이 필요할 때만 `ROS_PUB=1`.
if os.environ.get("ROS_PUB", "0") == "1":
    sys.path.insert(0, str(SON.parent / "dongmin" / "isaac_bridge"))
    try:
        import ros_bridge as _rosb                        # noqa: E402
        if _rosb.available():
            RB = _rosb.Bridge([r["name"] for r in robots],
                              node_name="isaac_v13")
            from pipe_comm import contract as _ct         # noqa: E402
            for r in robots:
                _rp = RB.robot(r["name"])
                # 🚨 **토픽 호환** (2026-08-11): 인계 문서에는 floor2 가
                #    `torch/rgb` 로 적혀 있는데 용접 카메라를 끄면서 전방
                #    (`rgb`)으로 바뀌어 웹이 못 받았다(실측). 같은 영상을
                #    **두 역할로 등록**해 어느 토픽을 구독하든 받게 한다 —
                #    annotator 를 공유하므로 렌더 비용은 늘지 않는다.
                for _role, _key in (("front", "front"),
                                    ("torch", "torch"),
                                    ("torch", "front")):
                    _c = r.get("cams", {}).get(_key)
                    if _c is None:
                        continue
                    if _role == "torch" and _key == "front" \
                            and "torch" in r.get("cams", {}):
                        continue          # 진짜 용접 카메라가 있으면 생략
                    _rp.attach_cameras([(_c, CAM_W, CAM_H, F_PX)],
                                       role=_role)
                _cl0 = r["cl"]
                # 웹 관 튜브용 표본 ~60개 (스트라이드 40은 4개뿐이라 튜브가
                # 각졌다 — 실측)
                _st0 = max(1, len(_cl0.p) // 60)
                _rp.publish_course(
                    [[float(_cl0.s[i]), float(_cl0.p[i][0]),
                      float(_cl0.p[i][1]), float(_cl0.p[i][2])]
                     for i in range(0, len(_cl0.p), _st0)], ir_m=PIPE_IR)
            try:
                RB.robot(robots[0]["name"]).publish_mesh(
                    str(MAPS / "restroom_final0807.webmesh"),
                    usd=str(_MAP), z_shift_mm=-_Z1)
            except Exception as _me:
                print(f"[경고] CAD 메시 발행 실패(무해): {_me}")
            # ── 동연 용접·판정 토픽 (로봇별 네임스페이스, 2026-08-11 확정:
            #    수신부 토픽명은 사용자가 프론트에 맞춘다) ─────────────
            from rclpy.qos import (QoSProfile, ReliabilityPolicy,  # noqa
                                   HistoryPolicy)
            from sensor_msgs.msg import CompressedImage as _CImg  # noqa
            from std_msgs.msg import String as _Str               # noqa
            _dyq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
            for r in robots:
                _ns0 = r["name"]
                r["dy_pub"] = dict(
                    rgb=RB.node.create_publisher(
                        _CImg,
                        f"/{_ns0}/repair_robot/active_cam/rgb/compressed",
                        _dyq),
                    which=RB.node.create_publisher(
                        _Str, f"/{_ns0}/repair_robot/active_cam/which", _dyq),
                    debug=RB.node.create_publisher(
                        _CImg,
                        f"/{_ns0}/repair_robot/opencv_debug/compressed", _dyq),
                    judge=RB.node.create_publisher(
                        _Str,
                        f"/{_ns0}/repair_robot/opencv_judgement/json", _dyq))
            print("[준비] 🌐 동연 계층 — /<코스>/repair_robot/"
                  "{active_cam,opencv_debug,opencv_judgement} 발행 준비")
            # 동연 검출 모듈(finders) — 네 원본의 상위호환. 🚨 인자 순서가
            # 달라졌으므로 **키워드 인자로만** 부른다 (다른 세션 확인 사항).
            try:
                import importlib.util as _ilu
                # 🔑 경로는 **이 파일 기준 상대**다 (`SON.parent` = `src/`).
                #    워크스페이스를 어디에 풀든 따라온다.
                _fpath = SON.parent / "dongyeon" / "detect" / "finders.py"
                _fs = _ilu.spec_from_file_location("dy_finders", str(_fpath))
                _DY_FINDERS = _ilu.module_from_spec(_fs)
                _fs.loader.exec_module(_DY_FINDERS)
                print("[준비] 🌐 동연 finders 로드 — find_wall_hole/"
                      "find_weld_bead (키워드 호출)")
            except Exception as _fe:
                _DY_FINDERS = None
                print(f"[경고] finders 로드 실패(판정 시각화 생략): {_fe}")
        else:
            print("[경고] ROS 브릿지 사용 불가 — 웹 발행 없이 진행")
    except Exception as exc:
        RB = None
        print(f"[경고] ROS 브릿지 비활성 ({exc}) — 웹 발행 없이 진행")
# ── 자율주행 모듈 배선 (NAV=vision) ─────────────────────────────────
# 🔑 **연습장이 판단을 만들지 않는다.** `src/son` 의 정본 모듈이 판단한다:
#      condition/detector.py   깊이 → 관 상태(개방도·입사각·오프셋)
#      condition/odometry.py   영상 → **시각 속도** (휠이 헛돌아도 진짜 전진량)
#      driver/control.py       위 둘 + 휠속도 → 속도 지령 · 끼임 판정 · FSM
if NAV == "vision":
    sys.path.insert(0, str(SON))
    from condition.detector import PipeConditionDetector      # noqa: E402
    from condition.odometry import VisualOdometry             # noqa: E402
    from driver.control import DriveController                # noqa: E402
    _INTR = dict(fx=F_PX, fy=F_PX, ppx=CAM_W / 2.0, ppy=CAM_H / 2.0,
                 f_fish=F_PX)
    for r in robots:
        r["det"] = PipeConditionDetector(_INTR)
        r["vo"] = VisualOdometry(f_px=F_PX)
        # BRANCH_RULE 노브 — 오른손 법칙 진단용 (right/all/none)
        # CENTER_GAIN / CENTER_GAIN_CURVE — 센터링 조향 진단용 (0 = 끔)
        _ctl_kn = dict(branch_rule=os.environ.get("BRANCH_RULE", "right"))
        for _env, _key in (("CENTER_GAIN", "center_gain"),
                           ("CENTER_GAIN_CURVE", "center_gain_curve")):
            if os.environ.get(_env) is not None:
                _ctl_kn[_key] = float(os.environ[_env])
        r["ctl"] = DriveController(_ctl_kn)
        r["cond"] = None
        r["vis_mps"] = None
        r["vis_t"] = 0.0
        r["started"] = False
    print(f"[항법] vision — 판단을 `condition/` + `driver/` 정본 모듈에 맡긴다 "
          f"(카메라 {len(robots)}대 사용)")

_XC = UsdGeom.XformCache()


def wpos(prim):
    _XC.Clear()
    t = _XC.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])])


def _set_pos(r, values, indices):
    """관절 위치 목표를 **바로** 쓴다.

    🚨 `apply_action` 을 한 스텝에 여러 번 부르면 **마지막 것만 남는다** —
       다리·휠을 각각 부르면 휠만 먹고 나머지는 조용히 사라진다(실측).
    """
    a = r["art"]
    v = np.array(values, dtype=np.float32)
    i = np.array(indices)
    try:
        a._articulation_view.set_joint_position_targets(v.reshape(1, -1),
                                                        joint_indices=i)
    except Exception:
        a.apply_action(ArticulationAction(joint_positions=v, joint_indices=i))


def wrot(prim):
    """프림의 월드 자세 — **열이 기저**인 3×3 (world = C @ v_local).

    🚨 USD 는 행벡터 규약이라 `Gf.Matrix4d` 의 위 3×3 은 **행**이 기저다.
       그대로 쓰면 회전이 전치돼 조향 부호가 뒤집힌다.
    """
    _XC.Clear()
    a = np.array(_XC.GetLocalToWorldTransform(prim),
                 dtype=np.float64).reshape(4, 4)[:3, :3]
    a = a / np.maximum(np.linalg.norm(a, axis=1, keepdims=True), 1e-12)
    return a.T


def _set_eff(r, values, indices):
    """관절 힘 지령. 🚨 위치/속도 지령과 **다른 채널**이라 서로 안 지운다."""
    a = r["art"]
    v = np.array(values, dtype=np.float32)
    i = np.array(indices)
    try:
        a._articulation_view.set_joint_efforts(v.reshape(1, -1),
                                               joint_indices=i)
        return True
    except Exception as exc:
        if not r.get("eff_warned"):
            r["eff_warned"] = True
            print(f"[경고] {r['name']}: 힘 지령 실패({exc}) — 위치 모드로 돈다")
        return False


# ── 🎯 **다리 신장 상한을 추적 관경에 묶는다** (2026-08-09 신설) ────────
# 🚨 상한 35mm 는 **DN150 용**이다(관벽 67mm − 다리 밑동 40mm = 27mm 필요).
#    DN100 에서는 관벽이 42mm 라 **2mm** 면 충분한데, 상한이 열려 있으니
#    곡관에서 다리가 벽을 잠깐 놓치면 **한계까지 래칫으로 뻗어** 얇은 관 메시를
#    뚫고 나간다(GUI 실측: 다리 하나가 곡관 바깥으로 튀어나옴, 도달 71mm).
# 🔑 관경은 이미 추적하고 있다 — 검출기의 `bore_ref_mm`(원통 전제 재설계에서
#    만든 값). 메모에 *"다리 스트로크 상한, bore_ref_mm 필드 준비됨, 미착수"*
#    로 남아 있던 항목이 이것이다.
#      허용 신장 = (추적 관벽 − 휠반경) − 다리 밑동 반경 + 여유
#      DN100: (50−8)−40 = 2mm → 여유 4mm 로 6mm
#      DN150: (75−8)−40 = 27mm → 31mm
# 🚨 관절 한계를 런타임에 바꾸는 것은 PhysX 로 안 넘어간다(기록) → **힘 지령**
#    으로 막는다. 상한을 넘은 다리는 '벽 없음' 과 같게 안쪽으로 당긴다.
LEG_BORE_CAP = os.environ.get("LEG_BORE_CAP", "1") == "1"
# 🚨 여유가 너무 빡빡하면 **정상 다리까지 '상한 초과 = 벽 없음'으로 걸린다**
#    (실측: 여유 4mm → 상한 3mm → 뜬다리 4~6개가 상시 발생). 추적 관경 자체가
#    46~47mm 로 실제(49~50)보다 작게 읽히는 계측 오차도 여기에 얹힌다.
#    → 여유 10mm. 정상 다리(신장 0~3mm)는 안 걸리고, 폭주한 다리(31mm)는
#      9mm 에서 막힌다. **정밀 제한이 아니라 폭주 방지턱**이다.
LEG_CAP_MARGIN = float(os.environ.get("LEG_CAP_MARGIN", 0.010))


def leg_max_ext(r):
    """추적 관경이 허용하는 다리 신장 상한(m). 판정이 없으면 상한을 안 건다."""
    if not LEG_BORE_CAP:
        return PISTON_STROKE
    _c = r.get("cond")
    _b = float(getattr(_c, "bore_ref_mm", 0.0) or 0.0) / 1000.0 if _c else 0.0
    if _b <= 0.0:
        _b = PIPE_IR                      # 판정 전에는 설계 DN100 으로 본다
    # 🎯 **거름망 하우징 안에서는 상한을 푼다** (2026-08-11 실측 규명).
    #    blueprint 는 관경 판정이 없어 DN100 으로 가정 → 상한 12mm → 도달
    #    40+12+8 = 60mm. 그런데 하우징 사각 벽은 **65mm**(대각 92) 라
    #    **애초에 닿을 수가 없었다** — 탈출이 막히던 진짜 이유가 이것이다.
    #    하우징 구간(exit_box)에서는 스트로크 전량을 쓴다.
    _cap = (_b - WHEEL_R) - TUNE["wheel_center_mm"] / 1000.0 + LEG_CAP_MARGIN
    return float(min(max(_cap, 0.002), PISTON_STROKE))


def exit_fold_legs(r):
    """관구 밖(중심선 시작점에 클램프)으로 나간 **다리 DOF** 집합.

    🎯 완전 탈출용 (2026-08-10 사용자: *"몸 전체가 배수구 관 밖으로"*).
       "전부 벽을 잃으면 전부 민다" 규칙은 관 안 접합부용이다 — 관 밖에서
       벽을 찾겠다고 다리를 만개하면 **관구 테두리에 우산처럼 걸린다**
       (실측: 복귀 s=41→21 끼임 12회, 몸통·뒤 디스크 다리 6~9개가 35mm
       만개, 휠 도달 62~118mm — 앞 3다리만 관 안). 관구 밖 세그먼트는 접고,
       아직 관 안인 세그먼트가 남은 몸을 민다.
    """
    if r.get("phase") not in ("BACK", "EXIT") or r.get("cl") is None:
        if r.get("exit_folded"):
            r["exit_folded"].clear()
        return ()
    _sh6 = float(r.get("s_hint") or 1.0)
    if min(_sh6, r["cl"].total - _sh6) > 0.30:    # 관구(양끝) 근처에서만 잰다
        return ()
    out = r.setdefault("exit_folded", set())
    # 🚨 기준은 중심선 s 가 아니라 **관구 평면 위 높이** (2026-08-10 사용자
    #    제보로 정정): 관구(중심선 끝, z=0) 위에 거름망 하우징 보어가
    #    EXIT_BORE_H(85mm 실측)까지 이어져 **거기까지는 짚을 벽이 있다**.
    #    s 기준(3mm)으로 접었더니 포켓 안에서 접지를 잃고 중력에 미끄러졌다
    #    (사용자 GUI 관찰과 일치). 접기 = 보어 끝 위 / 펴기 = 40mm 히스테리시스.
    # 🎯 **다리 단위로 접는다**: 세그먼트째 접으면 아직 벽이 있는 줄까지
    #    한꺼번에 잃는다. 각 다리의 암 위치로 개별 판단.
    _cl = r["cl"]
    _p0 = _cl.p[0]
    _u = _cl.tangent(0)               # +s = 관 안쪽(아래) 방향
    _hmax = -1.0                       # 가장 높은 다리 — 저속·증강 게이트
    for _k, _pr in r.get("leg_arm_of", {}).items():
        _h = float(np.dot(_p0 - wpos(_pr), _u))   # 관구 평면 위 높이(m)
        _hmax = max(_hmax, _h)
        if _h > EXIT_RIM_H:               # 림 위 = 다시 편다(맨틀링)
            out.discard(_k)
        elif _h > EXIT_BORE_H:            # 공동·구멍 = 접는다
            out.add(_k)
        elif _h < EXIT_BORE_H - 0.04:
            out.discard(_k)
    # 🚨 감속 게이트도 **히스테리시스** — 경계에 걸치면 78↔20mm/s 가 매
    #    스텝 널뛰어 덜컥거리다 끼임 판정을 받는다(GUI 실측: 감속선 −20mm
    #    에서 물러남 반복). 켜기 −20 / 끄기 −60(되물림 50mm 가 넘겨 준다).
    if _hmax > EXIT_SLOW_H:
        r["exit_cavity"] = True
    elif _hmax < EXIT_SLOW_H - 0.04:
        r["exit_cavity"] = False
    return out


def force_legs(r):
    """다리 힘 지령 — 예압 + 중심 복원 + 뜬 다리 접기. 위치 목표를 안 쓴다."""
    if not r.get("piston"):
        return [], []
    q = np.asarray(r["art"].get_joint_positions())
    _cap = leg_max_ext(r)
    r["leg_cap"] = _cap
    # 🎯 **개구 탐색 다리** (2026-08-10 저녁, T 쐐기 t6 실측으로 신설).
    #    blueprint 는 관경 추적이 없어 캡이 12mm 고정인데, T 통과는 다리가
    #    분기관 벽(82mm)을 짚어야 성립한다(만개 35 필요조건 — F 스윕 실측).
    #    스케줄 조향이 "지금 원호 안, 개구는 선회 안쪽" 을 알려 주면
    #    (sched_open_dir) **그쪽을 향한 다리만** 만개를 허용한다.
    #    바깥쪽 다리는 캡 유지 — 곡관 외벽 래칫 관통 방지턱은 살아 있다.
    # 리딩 디스크 접기 (t38) — 접기 대상 = 리딩 드럼 70mm 내의 다리
    _fold_p = r.get("sched_fold_p")
    _fold_ks = set()
    if _fold_p is not None and r.get("leg_arm_of"):
        _fr8 = float(os.environ.get("BP_TEE_FOLD_R", 0.045))
        for _k8, _pr8 in r["leg_arm_of"].items():
            if float(np.linalg.norm(wpos(_pr8) - _fold_p)) < _fr8:
                _fold_ks.add(_k8)
    # 🎯 T 구간 다리 소극 모드 (t45) — 접합부 반경 150mm 내 다리는 탐색·밀기
    #    금지, 신장 ~5mm 만 유지 (바퀴 반경 45mm — 벽 있는 곳만 살짝 구른다).
    _tee_p = r.get("sched_tee_p")
    _tee_ks = set()
    if _tee_p is not None and r.get("leg_arm_of"):
        _tr9 = float(os.environ.get("BP_TEE_CALM_R", 0.150))
        for _k9, _pr9 in r["leg_arm_of"].items():
            if float(np.linalg.norm(wpos(_pr9) - _tee_p)) < _tr9:
                _tee_ks.add(_k9)
    # 발차기 다리 (t65) — 선회 반대측을 향한 다리: 강제 신장으로 몸을 민다
    # 🎯 **위치 적응 다리** (t67 — 사용자 설계: 부위 구분 없이, 다리마다
    #    "자기 위치에서 닿는 관"에 실린더를 맞춘다). T 구간의 각 다리를
    #    두 관축(현재 가지 + 다른 가지) 중 가까운 쪽으로 판정:
    #    회랑 안(<55mm) = 그 관 벽까지 자유 신장·파지(_open_ks 재사용 —
    #    순수 예압·풀캡·접힘 면제) / 공동 위 = 소극(_tee_ks 아래 재구성).
    _kick_ks = set()   # (t65 발차기는 회전 모멘트 사고로 폐지 — 파지가 견인)
    _boost_ks = set()  # (t73 철회 — 미사용)
    _mid9_ks = set()   # 몸통 전용 접기 세트 (t79 — 머리 세트와 완전 분리)
    # 🚨 위치 적응은 **지정 T 한정** (t67 사용자: 가지→본관(복귀)만.
    #    본관→가지(나가는 T)는 완성된 종전 레시피 그대로 둔다).
    _adapt = {int(x) for x in
              os.environ.get("BP_TEE_ADAPT_ARCS", "").split(",") if x}
    _x9 = (r.get("sched_tee_mi") == -1)   # 크로스바 직진 횡단 (가상 구역)
    if (_tee_p is not None and r.get("leg_arm_of")
            and (r.get("sched_tee_mi") in _adapt or _x9)):
        _cl6 = r["cl"]
        _alt6 = r.get("sched_tee_alt")
        _tee_ks2 = set()
        # 🎯 **몸통만 움츠림** (t69 사용자 설계 — 머리=방향타·파지, 뒤=추진
        #    이라 힘 유지, 몸통 다리의 강성 저항이 코너 진입을 막는다는
        #    추측). 세그먼트를 드럼 거리로 분류해 두고, 코너 입구를 지난
        #    **몸통 다리만 접는다.** 머리·뒤는 특수 모드 없이 정상 예압.
        _sg_of = {k9: s9 for (s9, _i9), k9 in r["piston"].items()}
        if "sched_mid_sg" not in r and r.get("mid_prim") is not None:
            # 🚨 몸통 식별은 **Body 링크 실거리**로 (t70 — 드럼 거리 유추가
            #    머리를 몸통으로 오인해 머리 다리를 접었다, 사용자 육안 확인).
            _pm9 = wpos(r["mid_prim"])
            _dd9 = {}
            for _sg9 in set(_sg_of.values()):
                _arms9 = [wpos(r["leg_arm_of"][k9]) for k9 in _sg_of
                          if _sg_of[k9] == _sg9 and k9 in r["leg_arm_of"]]
                if _arms9:
                    _c9 = np.mean(np.array(_arms9), axis=0)
                    _dd9[_sg9] = float(np.linalg.norm(_c9 - _pm9))
            r["sched_mid_sg"] = (min(_dd9, key=_dd9.get) if _dd9 else None)
            if r["sched_mid_sg"] is not None:
                _nleg9 = sum(1 for v9 in _sg_of.values()
                             if v9 == r["sched_mid_sg"])
                print(f"[{r['name']:8s}] 몸통 세그 판별 = seg"
                      f"{r['sched_mid_sg']} (다리 {_nleg9}개, Body 링크 기준)")
        _ent6 = r.get("sched_tee_entry")
        _mid_sg = r.get("sched_mid_sg")
        for _k6 in list(_tee_ks):
            _pa6 = wpos(r["leg_arm_of"][_k6])
            _s6a, _off6, _ = _cl6.nearest(_pa6, r.get("s_hint"))
            if _x9:
                # 🎯 직진 횡단 (t76 — 전부 소극으로 눌러 회랑 안 다리까지
                #    무력화했었다): 회랑 55mm 안 = 완전 파지 / 공동 = 소극.
                if _off6 < 0.055:
                    _kick_ks.add(_k6)     # 아래에서 파지(_open_ks)로 승격
                else:
                    _tee_ks2.add(_k6)
                continue
            # 🎯 접기 게이트 = **몸통 중심** 기준 (t73 사용자: 6개 중 3개만
            #    접힘 — 다리별 게이트라 몸통이 입구에 걸친 동안 반만 접혔다).
            if _mid_sg is not None and _sg_of.get(_k6) == _mid_sg:
                # 🎯 접기 게이트 = **개구 창** (t80 계산 — "원호 진입" 게이트는
                #    개구 128mm 전 멀쩡한 관에서 접어 접지만 잃었다. 몸통
                #    중심이 립 구간(접합부 −50~+90mm)일 때만 접는다.)
                # 접기 창도 **월드 y 직접** (t81 — 투영 s 바이어스 제거):
                # 몸통 중심 y ∈ [795, 905] = 개구 립 구간 그 자체.
                if r.get("mid_prim") is not None:
                    _my6 = float(wpos(r["mid_prim"])[1])
                    if 0.795 <= _my6 <= 0.905:
                        _mid9_ks.add(_k6)  # 립 구간 — 6다리 일괄 접기
        _tee_ks = _tee_ks2                # 소극 모드는 비움 (머리·뒤 정상)
    _open = r.get("sched_open_dir")
    _open_ks = set()
    if _open is not None and r.get("leg_arm_of"):
        _cl5 = r["cl"]
        for _k5, _pr5 in r["leg_arm_of"].items():
            _pa = wpos(_pr5)
            _s5, _o5, _i5 = _cl5.nearest(_pa, r.get("s_hint"))
            _rad = _pa - _cl5.p[_i5]
            _t5 = _cl5.tangent(_i5)
            _rad = _rad - _t5 * float(np.dot(_rad, _t5))
            _n5 = float(np.linalg.norm(_rad))
            if _n5 > 1e-6 and float(np.dot(_rad / _n5, _open)) > 0.5:
                _open_ks.add(_k5)
    _open_ks |= _kick_ks                 # 회랑 안 다리 = 파지(순수 예압·풀캡)
    _kick_ks = set()
    r["leg_open_n"] = len(_open_ks)
    vel = np.asarray(r["art"].get_joint_velocities())
    free = r.setdefault("leg_free", {})
    idx, val, nfree = [], [], 0
    _xf = exit_fold_legs(r)              # 관구 밖 다리 → 강제 접기
    for sg in sorted({s2 for s2, _i in r["piston"]}):
        ks = [k for (s2, _i), k in sorted(r["piston"].items()) if s2 == sg]
        for k in ks:
            if k in _xf:
                idx.append(k)
                val.append(-LEG_FOLD_N)  # 당겨 접는다 — 우산 금지
                free[k] = True
                nfree += 1
        ks = [k for k in ks if k not in _xf]
        if not ks:
            continue
        # 서 있는 다리 = 벽에 닿은 다리 → 그것들의 평균이 관벽 추정치
        stop = [k for k in ks if abs(float(vel[k])) < LEG_STOP_V]
        ref = float(np.mean(q[stop if stop else ks]))
        for k in ks:
            out = float(vel[k]) > LEG_FREE_V and q[k] > ref + LEG_REF_GAP
            # 🎯 추적 관경이 허용하는 상한을 넘으면 **벽이 없는 것으로 본다**
            #    — 관 안이라면 거기까지 뻗을 이유가 없다(관벽을 뚫은 것).
            #    개구 탐색 다리(_open_ks)는 만개까지 허용 — 분기관 벽을 찾는다.
            # 🚨 개구 탐색도 30mm 까지만 — 만개(35) 허용 + out 판정 면제
            #    조합이 관벽 래칫 관통을 낳았다(2026-08-10 GUI 사용자 육안:
            #    "다리가 배관을 뚫고 나온다"). 30mm 면 분기 개구 안 접지는
            #    되고(t8 실측 31mm 도달) 관통 방지 판정은 살아 있다.
            _cap_k = (min(PISTON_STROKE,
                          float(os.environ.get("BP_OPEN_CAP", 0.030)))
                      if k in _open_ks else _cap)
            if out or q[k] >= _cap_k or q[k] >= PISTON_STROKE - LEG_FREE_MARGIN:
                free[k] = True
            elif q[k] < ref + 0.5 * LEG_REF_GAP:
                free[k] = False
        cont = [k for k in ks if not free.get(k)]
        if not cont:                      # 전부 허공이면 접지 말고 벽을 찾는다
            cont = ks
            for k in ks:
                free[k] = False
        # 개구 탐색 다리는 평균에서 뺀다 — 만개 다리가 평균을 끌어올려
        # 나머지 다리의 중심 복원을 흐트리지 않게.
        _cm = [k for k in cont if k not in _open_ks] or cont
        m = float(np.mean(q[_cm]))
        # 탈출(공동 진입)이 시작되면 남은 접지 다리의 예압을 증강한다 —
        # 줄어든 다리 수로 자중을 이겨야 한다(위 EXIT_PUSH_N 근거).
        # 🎯 T 접합부 공동(포켓+팔+본관 합류)에서 뜬다리가 3개 이상이면
        #    남은 접지 다리 예압 증강 — 탈출 공동과 같은 원리 (t20: 정점에서
        #    뜬다리 5~6, 견인 구멍으로 정체).
        _push = EXIT_PUSH_N if (_xf or r.get("exit_cavity")
                                or (r.get("sched_arc")
                                    and r.get("leg_free_n", 0) >= 3)) \
            else PISTON_MAXF
        if r.get("tape_boost"):
            # 되감기 T 역통과 견인 증강 (t91 실측: 헛돎 — 예압 2배)
            _push = max(_push, float(os.environ.get("BP_TAPE_N", 18.0)))
        for k in ks:
            idx.append(k)
            if k in _mid9_ks:
                # 🎯 립 창 소극 −3mm (t82 매트릭스 결론 — 접기(-15 풀수축)는
                #    지지 상실로 역효과, 소극 +5mm 는 립을 8mm 찌른다.
                #    목표만 −3mm 로 내려 바퀴를 립 안쪽으로 살짝 들인다.)
                val.append(float(np.clip(
                    LEG_KC * (-0.003 - float(q[k])), -20.0, 8.0)))
                free[k] = False
            elif k in _fold_ks:
                # 🎯 머리 3단계 (t83 단면 계산 — 머리+다리 단면 50~53mm 는
                #    개구 구경 50mm 를 못 지난다. 구멍 통과 순간만 −8mm 수축
                #    (단면 40 → 클리어런스 10mm), 본관 안에서 +2 재접지):
                #    접근(x>720) 소극 +5 / 통과(640~720) 수축 −8 / 안(<640) +2.
                _hx8 = float(_fold_p[0]) if _fold_p is not None else 9.0
                if 0.640 <= _hx8 <= 0.720:
                    _tq8 = -0.008
                elif _hx8 < 0.640:
                    _tq8 = 0.002
                else:
                    _tq8 = 0.005
                val.append(float(np.clip(
                    LEG_KC * (_tq8 - float(q[k])), -25.0, 10.0)))
                free[k] = False
            elif k in _kick_ks:
                # 발차기 — 반대측 벽을 차서 몸을 본관으로 민다 (t65)
                val.append(float(os.environ.get("BP_TEE_KICK_N", 30.0)))
                free[k] = False
            elif k in _tee_ks:
                # 소극 모드: 5mm 목표로 온건한 비례력 — 뻗지도 접지도 않는다
                val.append(float(np.clip(
                    LEG_KC * (0.005 - float(q[k])), -20.0, 12.0)))
                free[k] = False
            elif free.get(k):
                val.append(-LEG_FOLD_N)
                nfree += 1
            elif k in _open_ks:
                # 🎯 개구 탐색 — 중심 복원 없이 순수 예압으로 민다. 복원항을
                #    같이 주면 평균(~2mm)보다 나간 만큼 도로 당겨져 **만개가
                #    원리적으로 불가능**했다(t7 실측: 캡 풀어도 15mm 정체).
                val.append(_push)
            else:
                val.append(_push + LEG_KC * (m - float(q[k])))
    r["leg_free_n"] = nfree
    r["leg_free_max"] = max(r.get("leg_free_max", 0), nfree)
    r["leg_spread"] = float(np.ptp(q[list(r["piston"].values())]))
    return idx, val


def preload_legs(r):
    """다리 목표를 **스트로크 밖**에 둔다 = 상수력 예압 (안착 구간용).

    🚨 **`set_joint_positions()` 는 PhysX 런타임 드라이브 타깃을 지운다**
       (기록된 함정). 다리 초기 신장을 넣는 그 한 줄 때문에 예압이 0 이 되어
       **로봇이 벽을 아예 안 짚었다.** 수평 코스는 중력이 아래 벽에 눌러 줘서
       증상이 가려졌고, **수직관에서만** 드러났다 — 안착 중 자유낙하의 86%
       속도로 떨어지고 휠 각속도가 26°/s(굴렀다면 7,160°/s).
    """
    if not r.get("piston"):
        return [], []
    ks = list(r["piston"].values())
    return ks, [PISTON_STROKE * 2.0] * len(ks)


def leg_efforts(r):
    """관절이 실제로 내고 있는 힘(N) — 다리별 벽 접촉 판정용.

    🚨 Isaac 버전마다 이름이 달라 **있는 것을 찾아 쓰고, 없으면 판정을 끈다**
       (조용히 틀린 값을 쓰느니 안 쓰는 게 낫다). 처음 한 번 어느 API 를
       썼는지 로그로 남긴다.
    """
    if r.get("eff_api") == "none":
        return None
    av = r["art"]._articulation_view
    names = ([r["eff_api"]] if r.get("eff_api")
             else ["get_measured_joint_efforts", "get_applied_joint_efforts",
                   "get_measured_joint_forces"])
    for nm in names:
        fn = getattr(av, nm, None)
        if fn is None:
            continue
        try:
            e = np.asarray(fn())
        except Exception:
            continue
        if e is None or e.size == 0:
            continue
        e = e.reshape(-1) if e.ndim == 1 else e[0].reshape(-1)
        if e.shape[0] < max(r["piston"].values()) + 1:
            continue
        if not r.get("eff_api"):
            r["eff_api"] = nm
            _s = ", ".join(f"{float(e[k]):+.2f}"
                           for k in sorted(r["piston"].values()))
            print(f"  {r['name']}: 다리 힘 판정에 `{nm}` 사용 — 초기값(N) "
                  f"[{_s}]")
        return e
    r["eff_api"] = "none"
    print(f"[경고] {r['name']}: 관절 힘을 읽을 API 가 없다 — "
          f"다리별 접촉 판정을 끈다")
    return None


def center_legs(r):
    """세그먼트별로 다리 신장을 맞춘다 = 몸체를 관 중앙으로.

    🔑 **벽이 없는 다리는 따로 다룬다** — 평균에서 빼고 접는다. 안 그러면
       구멍 위에서 끝까지 뻗은 값이 평균을 끌어올려 **나머지 다리가 몸을
       구멍 쪽으로 민다**(사용자 표현: 왼발차기로 벽을 밀어야 하는데,
       지금은 허공을 짚는 발까지 똑같이 밀라고 하고 있었다).
    🚨 한 세그먼트의 다리가 **전부** 벽이 없으면 접으면 안 된다 — 주저앉는다.
       그때는 전부 밀어 벽을 찾는다.
    """
    if not r.get("piston"):
        return [], []
    q = np.asarray(r["art"].get_joint_positions())
    eff = leg_efforts(r)
    free = r.setdefault("leg_free", {})
    idx, val, nfree = [], [], 0
    _xf = exit_fold_legs(r)              # 관구 밖 다리 → 강제 접기
    for sg in sorted({s for s, _i in r["piston"]}):
        ks = [k for (s2, _i), k in sorted(r["piston"].items()) if s2 == sg]
        for k in ks:
            if k in _xf:
                idx.append(k)
                val.append(-PISTON_RETRACT if PISTON_RETRACT > 0 else LEG_FOLD)
                free[k] = True
                nfree += 1
        ks = [k for k in ks if k not in _xf]
        if not ks:
            continue
        # 🚨 **중앙값 기준으로 미리 잡으려던 것은 되돌렸다** (실측 악화:
        #    `tee_in` 끼임 2→3회). 접합부에서는 한 세그먼트의 다리 **대부분**이
        #    벽을 잃어 중앙값 자체가 뜬 다리 쪽으로 끌려가므로 기준이 못 된다.
        #    다수결이 성립하지 않는 곳이라 중앙값은 원리적으로 안 맞는다.
        if LEG_FREE_ON:
            for k in ks:
                if q[k] >= PISTON_STROKE - LEG_FREE_MARGIN:
                    free[k] = True
                elif q[k] < PISTON_STROKE - LEG_FREE_CLEAR:
                    free[k] = False
        cont = [k for k in ks if not free.get(k)]
        if not cont:                      # 짚을 벽이 하나도 없다 → 전부 민다
            cont = ks
            for k in ks:
                free[k] = False
        m = float(np.mean(q[cont])) + CENTER_DELTA
        for k in ks:
            idx.append(k)
            if free.get(k):
                val.append(LEG_FOLD)
                nfree += 1
            else:
                val.append(m)
    r["leg_free_n"] = nfree
    r["leg_free_max"] = max(r.get("leg_free_max", 0), nfree)
    r["leg_spread"] = float(np.ptp(q[list(r["piston"].values())]))
    return idx, val


def sync_rollbend_cmds(r):
    """방향 전환 순간 롤·굽힘 지령 적분기를 **실측 관절각으로 동기화**한다.

    🚨 끼임 되물림에서 지령이 물러나기 이전 값(예: 롤 −145°)에 남아 있으면
       복원 순간 목표 접선이 뒤집혀 오차가 한 번에 벌어지고, 45°/s 슬루가
       다리가 분기 벽에 물린 채로 몸을 비틀어 **개구로 사출**시켰다
       (2026-08-10 floor1 A런 실측: 복원 직후 이탈 134mm 즉사).
    """
    if not r.get("bend_front"):
        return
    _qq = np.asarray(r["art"].get_joint_positions())
    r["cmd_roll"] = float(np.sum(_qq[r["roll_front"]]))
    r["cmd_bend"] = float(np.sum(_qq[r["bend_front"]]))
    r["cmd_roll_r"] = float(np.sum(_qq[[k for k in r["roll_dof"]
                                        if k not in r["roll_front"]]]))
    r["cmd_bend_r"] = float(np.sum(_qq[[k for k in r["bend_dof"]
                                        if k not in r["bend_front"]]]))


def diag_stuck(r):
    """끼임 순간의 몸통·다리·휠 상태를 찍는다.

    끼임의 원인을 **가른다**: 다리가 끝까지 뻗어 있으면 밀 벽이 없는 것
    (개구로 빠짐)이고, 다리가 눌려 있으면 벽에 물린 것이다. 휠 반경은
    관 중심선에서 얼마나 떨어졌는가 — 관벽(50 − 휠 8 = 42mm)이 기준이다.
    """
    cl = r["cl"]
    q = np.asarray(r["art"].get_joint_positions())
    # 탈출 접기 상태 — 왜 접기/저속이 발동 안 하는지 가른다 (2026-08-10)
    _u0 = cl.tangent(0)
    _hs = {k: float(np.dot(cl.p[0] - wpos(_pr), _u0))
           for k, _pr in r.get("leg_arm_of", {}).items()}
    print(f"           [진단] 탈출: phase={r.get('phase')} "
          f"cavity={r.get('exit_cavity')} 접힘={sorted(r.get('exit_folded', set()))} "
          f"매핑={len(r.get('leg_arm_of', {}))}개 "
          f"다리높이(mm)={[f'{v * 1000:+.0f}' for v in sorted(_hs.values())]}")
    p0, p1 = wpos(r["seg0"]), wpos(r["seg1"])
    s0, o0, _ = cl.nearest(p0, r.get("s_hint"))
    s1, o1, _ = cl.nearest(p1, r.get("s_hint"))
    print(f"           [진단] 뒤 s={s0 * 1000:.0f} 이탈{o0 * 1000:5.1f} "
          f"월드({p0[0] * 1000:+.0f},{p0[1] * 1000:+.0f},{p0[2] * 1000:+.0f}) | "
          f"앞 s={s1 * 1000:.0f} 이탈{o1 * 1000:5.1f} "
          f"월드({p1[0] * 1000:+.0f},{p1[1] * 1000:+.0f},{p1[2] * 1000:+.0f})")
    for sg in sorted({s for s, _i in r["piston"]}):
        ks = [k for (s2, _i), k in sorted(r["piston"].items()) if s2 == sg]
        ext = ", ".join(f"{q[k] * 1000:5.1f}" for k in ks)
        _fr = "".join("F" if r.get("leg_free", {}).get(k) else "."
                      for k in ks)
        print(f"           [진단] seg{sg} 접촉 [{_fr}] (F=벽없음)")
        print(f"           [진단] seg{sg} 다리 신장(mm, 상한 "
              f"{PISTON_STROKE * 1000:.0f}) [{ext}]")
    rad = []
    for wp in r["wheel_prims"]:
        _s, _o, _ = cl.nearest(wpos(wp), r.get("s_hint"))
        rad.append(_o * 1000)
    print(f"           [진단] 휠 중심선 거리(mm, 관벽 "
          f"{(PIPE_IR - WHEEL_R) * 1000:.0f}) "
          f"[{', '.join(f'{v:4.1f}' for v in rad)}]")
    print(f"           [진단] 관절합 P{r.get('q_pitch', 0):+.0f}"
          f"/Y{r.get('q_yaw', 0):+.0f}  지령 "
          f"{r.get('cmd_pitch', 0):+.1f}/{r.get('cmd_yaw', 0):+.1f}×4  "
          f"목표 P{r.get('want', (0, 0))[0]:+.0f}/Y{r.get('want', (0, 0))[1]:+.0f}"
          f"  휠 각속도 실측/지령 {r.get('gov_meas', 0) * 100:.0f}%")


# 분기 방위를 시야에서 잃은 뒤에도 이만큼은 그 방향을 유지한다(진입 중에는
# 개구가 화면 밖으로 벗어난다).
BRANCH_HOLD_S = float(os.environ.get("BRANCH_HOLD_S", 3.0))


def _cl_arcs(cl):
    """중심선의 원호 구간 목록 [(s0, s1, i0, i1), …] — 접선 회전율로 검출."""
    n = len(cl.s)
    ts = np.array([cl.tangent(i) for i in range(n)])
    out, bent = [], None
    for i in range(1, n):
        dth = math.acos(max(-1.0, min(1.0, float(np.dot(ts[i], ts[i - 1])))))
        ds = max(float(cl.s[i] - cl.s[i - 1]), 1e-9)
        if dth / ds > BP_ARC_KAPPA:
            if bent is None:
                bent = i - 1
        elif bent is not None:
            out.append((float(cl.s[bent]), float(cl.s[i - 1]), bent, i - 1))
            bent = None
    if bent is not None:
        out.append((float(cl.s[bent]), float(cl.s[n - 1]), bent, n - 1))
    return out


def _cl_arc_meta(cl):
    """원호 구간에 진입·출구 접선과 꺾임각을 붙인다 — 스케줄 조향의 지도."""
    n = len(cl.s)
    out = []
    for s0, s1, i0, i1 in _cl_arcs(cl):
        t_in = cl.tangent(max(i0 - 2, 0))
        t_out = cl.tangent(min(i1 + 2, n - 1))
        ang = math.degrees(math.acos(max(-1.0, min(1.0,
                                                   float(np.dot(t_in, t_out))))))
        out.append(dict(s0=s0, s1=s1, t_in=np.asarray(t_in),
                        t_out=np.asarray(t_out), ang=ang))
    return out


def steer_bp_sched(r):
    """🎯 **스케줄 조향** (2026-08-10 저녁) — v9 T 통과 레시피의 정답지판.

    구조는 BP_SCHED 상수 주석 참조. 드럼(앞/뒤)마다 제 위치 s 로 국면을 정한다:
      PRE(진입 전 직선) : 굽힘을 펴고 롤을 원호 평면으로 조준.
                          지령 = **실측 관절각 + 오차**(앵커) — 적분 폭주 불가.
      ARC/LAG(원호 안~끝+LAG) : 롤 동결, 고정 굽힘(부호는 PRE 에서 확정).
      그 외(직선)       : 굽힘 0, 롤 유지(아무것도 안 쫓는다).
    국면이 겹치면 ARC 중심부 > PRE > LAG — 연속 원호(floor2 곡관1→2)에서
    앞 원호의 LAG 유지보다 다음 원호의 재조준이 우선해야 하기 때문.
    """
    if not r.get("bend_front"):
        return [], []
    cl = r["cl"]
    p1 = wpos(r["seg1"])
    s1, _o, _ = cl.nearest(p1, r.get("s_hint"))
    lim = math.radians(STEER_RATE) * PHYSICS_DT

    # 관구 접근은 라이저 직선 — 굽힘을 편다 (EXIT_STRAIGHT_S 주석).
    # 한붓그리기 코스(끝 = 배수구)는 **끝쪽** 라이저에서도 같게 편다.
    if r.get("phase") in ("BACK", "EXIT") and (
            s1 < EXIT_STRAIGHT_S or (cl.total - s1) < EXIT_STRAIGHT_S):
        _cb = r.get("cmd_bend", 0.0)
        _cbr2 = r.get("cmd_bend_r", 0.0)
        _cb += max(-lim, min(lim, -_cb))
        _cbr2 += max(-lim, min(lim, -_cbr2))
        r["cmd_bend"], r["cmd_bend_r"] = _cb, _cbr2
        r["bp_f_deg"], r["bp_r_deg"] = math.degrees(_cb), math.degrees(_cbr2)
        _rb2 = [k for k in r["bend_dof"] if k not in r["bend_front"]]
        _rr2 = [k for k in r["roll_dof"] if k not in r["roll_front"]]
        return (r["roll_front"] + r["bend_front"] + _rr2 + _rb2,
                [r.get("cmd_roll", 0.0)] * len(r["roll_front"])
                + [_cb] * len(r["bend_front"])
                + [r.get("cmd_roll_r", 0.0)] * len(_rr2)
                + [_cbr2] * len(_rb2))

    metas = r.get("cl_arc_meta")
    if metas is None:
        metas = r["cl_arc_meta"] = _cl_arc_meta(cl)
        print(f"  {r['name']}: 스케줄 원호 "
              + ", ".join(f"#{i}[{m['s0'] * 1000:.0f}~{m['s1'] * 1000:.0f}mm "
                          f"{m['ang']:.0f}°]" for i, m in enumerate(metas)))
    # 🚨 국면 판정은 **임무 방향**으로 — 순간 dir(끼임 되물림에 뒤집힘)로
    #    하면 물러날 때마다 PRE 가 앞뒤 원호로 널뛰며 롤이 요동한다
    #    (t4 실측: s=481 데드락, 끼임 16회, 롤 -20↔-67° 왕복).
    d = -1 if r.get("phase") in ("BACK", "EXIT") else 1
    st = r.setdefault("sched", {})
    _q = np.asarray(r["art"].get_joint_positions())

    def _drum_cmd(drum, roll_dofs, bend_dofs, cr_key, cb_key, aim_sign, tag):
        _cr = r.get(cr_key, 0.0)
        _cb = r.get(cb_key, 0.0)
        if drum is None:
            return _cr, _cb
        sd, _od, _ = cl.nearest(wpos(drum), r.get("s_hint"))
        # ── 국면 판정 (ARC 중심부 > PRE > LAG) ──────────────────────
        zone, meta, mi = None, None, -1
        for pri in ("CORE", "PRE", "LAG"):
            for i, m in enumerate(metas):
                if pri == "CORE" and m["s0"] <= sd <= m["s1"]:
                    zone, meta, mi = "ARC", m, i
                elif pri == "PRE" and (
                        (d > 0 and m["s0"] - BP_SCHED_PRE <= sd < m["s0"]) or
                        (d < 0 and m["s1"] < sd <= m["s1"] + BP_SCHED_PRE)):
                    zone, meta, mi = "PRE", m, i
                elif pri == "LAG" and (
                        (d > 0 and m["s1"] < sd <= m["s1"] + BP_SCHED_LAG) or
                        (d < 0 and m["s0"] - BP_SCHED_LAG <= sd < m["s0"])):
                    zone, meta, mi = "ARC", m, i
                if zone:
                    break
            if zone:
                break
        _zk, _sk = tag + "_zone", tag + "_sign"
        if zone is None:                       # 직선 — 편다, 롤은 그대로 둔다
            _cb += max(-lim, min(lim, -_cb))
            st.pop(tag + "_go", None)          # 뒤 발동 래치 해제 (다음 원호용)
            st.pop("seat_s", None)             # 안착 래치 해제
            if st.pop(_sk, None) is not None or st.get(_zk) is not None:
                st[_zk] = None
            r[cr_key], r[cb_key] = _cr, _cb
            return _cr, _cb

        # 진행 방향 기준 진입/출구 접선 → 굽힘 평면 목표(출구가 벌어지는 쪽)
        _u = meta["t_in"] if d > 0 else -meta["t_out"]
        _v = meta["t_out"] if d > 0 else -meta["t_in"]
        _pl = _v - _u * float(np.dot(_v, _u))
        _pn = float(np.linalg.norm(_pl))

        _Rd = wrot(drum)
        _xd = _Rd @ np.array([1.0, 0.0, 0.0])
        _bd = np.cross(_Rd @ np.array([0.0, 1.0, 0.0]), _xd)

        def _aim_once():
            """현재 자세에서 (롤 오차, 굽힘 부호) 한 번 측정."""
            _w = _pl - _xd * float(np.dot(_pl, _xd))
            _n2 = float(np.linalg.norm(_w))
            if _pn < 1e-6 or _n2 < 1e-6:
                return 0.0, st.get(_sk, 1.0)
            _w = _w / _n2
            _e = math.atan2(float(np.dot(np.cross(_bd, _w), _xd)),
                            float(np.dot(_bd, _w)))
            _sg = 1.0
            if _e > math.pi / 2:
                _e -= math.pi
                _sg = -1.0
            elif _e < -math.pi / 2:
                _e += math.pi
                _sg = -1.0
            return _e, _sg

        if zone == "PRE":
            _cb += max(-lim, min(lim, -_cb))       # 펴면서 조준한다
            # 🚨 굽힘이 남아 있으면 롤을 안 돌린다 (v9 ROLL_LOCK_AT 의 역방향
            #    게이트) — 연속 원호에서 직전 굽힘이 펴지기 전에 돌리면 그게
            #    곧 요동이다. 펴진 뒤에만 조준한다.
            _e, _sg = math.pi, st.get(_sk, 0.0)    # 미조준 기본값
            if abs(_cb) <= math.radians(8.0):
                _e, _sg = _aim_once()
                st[_sk] = _sg
                # 🔑 앵커: 목표 = **실측 롤** + 오차 (v9 base=_roll_actual 이식)
                _qr = float(np.sum(_q[roll_dofs]))
                _stp = math.radians(BP_SCHED_STEP)
                _tr = _qr + aim_sign * max(-_stp, min(_stp, _e))
                _cr += max(-lim, min(lim, _tr - _cr))
            # 🎯 조준이 덜 됐으면 **감속** — floor1 은 곡관→T 직선이 80mm 뿐
            #    이라(원호#0 끝 421 → #1 시작 501) 순항 속도로는 90° 평면
            #    전환(45°/s = 2s)이 물리적으로 안 끝난다. v9 는 긴 조주로가
            #    있었다 — 여기서는 속도를 내려 같은 시간을 번다.
            if abs(_e) > math.radians(15.0):
                r["sched_slow"] = min(r.get("sched_slow", 1.0), 0.25)
            if st.get(_zk) != ("PRE", mi):
                st[_zk] = ("PRE", mi)
                print(f"[{r['name']:8s}] 🧭 {tag} 원호#{mi} 조준 시작 "
                      f"(s={sd * 1000:.0f}mm, 오차 {math.degrees(_e):+.0f}°, "
                      f"부호 {_sg:+.0f})")
        else:                                      # ARC/LAG — 롤 동결, 고정 굽힘
            r["sched_arc"] = True                  # 접합부 공동 대응 게이트
            if (tag == "앞") == (d > 0):           # 리딩 드럼 위치 공유 (LAG용)
                r["sched_lead_sd"] = sd
            _sg = st.get(_sk)
            if _sg is None:                        # 조준 없이 원호에 든 경우
                _e, _sg = _aim_once()              # 부호만 한 번 정하고 동결
                st[_sk] = _sg
            # 🚨 **오른손 법칙 검증** (2026-08-10 GUI — 사용자 육안: 로봇이
            #    왼팔(+Y)로 꺾었다). 조준 부호가 초기 조건에 따라 뒤집힌다
            #    (t16 롤F +21 vs 이전 런 -91). 원호 안에서 매 스텝 실측
            #    꺾임 방향(_bd×부호)과 코스 선회 방향(_pl)을 대조해, 반대면
            #    부호를 뒤집는다. 히스테리시스 -0.3 — 경계 널뛰기 방지.
            if _pn > 1e-9:
                _w4 = _pl - _xd * float(np.dot(_pl, _xd))
                _n4 = float(np.linalg.norm(_w4))
                if _n4 > 1e-6:
                    _dd4 = float(np.dot(_bd, _w4 / _n4)) * _sg
                    if _dd4 < -0.3:
                        _sg = st[_sk] = -_sg
                        print(f"[{r['name']:8s}] ↪ {tag} 굽힘 부호 반전 — "
                              f"실측 꺾임이 코스 반대(오른손 위반, "
                              f"dot={_dd4:+.2f}, s={sd * 1000:.0f}mm)")
            # 🎯 **끼임 단계 증강** (2026-08-10 t8 — v9 스케줄 재계산 근거).
            #    v9 조원 스케줄은 T 에서 앞 관절 피크를 **90°** 까지 올린다
            #    (TOTAL_BEND 90, tF−tB_front 프로파일). 반면 곡관은 85° 가
            #    악화 실측(D/E 스윕) — 그래서 일괄 90 이 아니라 **끼임이
            #    증명한 만큼만** 올린다: 기본 40°, 이 원호에서 끼임 1회당
            #    +BP_SCHED_ESC(15°), 상한 = 원호 꺾임각. 곡관은 안 끼므로
            #    40 유지, T 쐐기는 40→55→70→85→90.
            _esc = max(0, int(r.get("stuck", 0)) - int(st.get(tag + "_st0", 0)))
            # 🎯 **평탄 증강 — 리딩 관절만** (2026-08-10 맵 v2 후 t18 확정).
            #    맵 v2 는 T 가 실제 R150 필렛이다 → 원호 전체에서 일정하게
            #    꺾는 것이 옳다. 필렛인데 40° 로는 정면 포켓(외벽이 비는
            #    구간)으로 흘러 캡에 박힌다(t18: 진입 추적 완벽, 코너 중심
            #    s=705 포켓행) → 끼임이 증명한 만큼 리딩 관절만 +15°씩.
            #    트레일링까지 같이 올리면 U자 쐐기(t9 실측).
            _lead = (tag == "앞") == (d > 0)
            # 트레일링 굽힘 배율 (t24 — "앞만 꺾는 게 최고" 단독 실측의
            # 스케줄판. 0 = 뒤 곧게 펴고 직진 추력 전담)
            _rsc = float(os.environ.get("BP_SCHED_REAR", 1.0))
            # 🎯 굽힘각 분리 (t45 — 사용자 육안: 수직곡관부터 문제. T용
            #    70° 가 env 로 **모든 원호**에 걸려 곡관 과굴절을 만들었다.
            #    곡관은 floor2 검증값 40°(BP_SCHED_BEND 기본), T 만 BP_TEE_BEND).
            # 🚨 **T 레시피는 그 코스 전용이다** (2026-08-11 실측 회귀).
            #    원호 번호는 코스마다 뜻이 다르다 — floor1 의 1·6 은 T 진입/
            #    복귀 원호지만 **floor2 의 1 은 평범한 곡관**(680,850)이다.
            #    v1_3 이 이 값을 전역 기본값으로 구우면서 floor2 복귀가 그
            #    곡관에서 70° 과굴절로 쐐기가 됐다(s=727 끼임4 실측).
            #    → 코스별 env 로만 받는다: BP_TEE_ARCS_<코스>.
            _tee_env = os.environ.get(f"BP_TEE_ARCS_{r['name']}", "")
            _tee = {int(x) for x in _tee_env.split(",") if x}
            _bb = (float(os.environ.get("BP_TEE_BEND", 70.0))
                   if mi in _tee else BP_SCHED_BEND)
            if _esc > 0 and _lead:
                r["sched_corner"] = True
                _bm = min(meta["ang"],
                          _bb
                          + float(os.environ.get("BP_SCHED_ESC", 15.0)) * _esc)
            else:
                _bm = min(meta["ang"], _bb)
            # 🎯 2단 각도 (t25 — 필렛 추종은 70°가 정답이나 90° 코너의
            #    마지막 팔 진입은 정렬각이 더 필요해 s=758 점근 정체).
            #    정점(중점) 이후 리딩 관절만 BEND2 로 올린다. 위치 기반 —
            #    끼임 기반 증강(잭나이프 전례)과 다르게 경로 정위에서 상승.
            # 🚨 2단 각도는 **T 원호에만** (t26 실측: 전 원호 적용 시 진입
            #    곡관 후반 88° → 곡관 탈출 자세 붕괴 → T 선조준이 틀린
            #    평면(+54)에 잠김). 정답지는 도면을 아는 주행 — T 지정 정당.
            _b2 = float(os.environ.get("BP_SCHED_BEND2", 0.0))
            if _b2 > 0 and _lead and mi in _tee:
                _u5 = d * (sd - 0.5 * (meta["s0"] + meta["s1"]))
                if _u5 > 0:
                    _bm = min(meta["ang"], max(_bm, _b2))
            if not _lead:
                # 🎯 뒤 관절 지연 발동 (t27 — 뒤 0° 고정은 진입엔 정답이나
                #    몸통이 코너에 닿는 s=744 부터는 일자 강체가 코너에 안
                #    낀다. v9 의 LAG 개념: 뒤는 곧게 밀다 코너 근접 시 꺾는다).
                # 🎯 v9 원형 LAG (t32 확정 — 힘 6배로도 s=758 불변 = 뒤 0°
                #    고정은 기하 한계. 몸이 코너를 돌려면 뒤도 결국 꺾여야
                #    하고, 그 시점은 **리딩 드럼이 코너+LAG 를 지날 때**다.
                #    리딩 드럼 위치는 정지해도 유효한 신호 — t29 순환 없음).
                _lag7 = float(os.environ.get("BP_SCHED_LAG2", -1.0))
                _dly9 = {int(x) for x in
                         os.environ.get("BP_TEE_DELAY_ARCS", "").split(",")
                         if x}
                if mi in _dly9 and r.get("sched_xfer_w") is not None:
                    # 굽힘 이관의 뒤쪽 절반 — 뒤 = 총량 × w (래치 대체)
                    _bm = min(meta["ang"],
                              float(os.environ.get("BP_TEE_BEND", 70.0))) \
                        * float(r["sched_xfer_w"])
                elif _lag7 >= 0 and mi in _tee:
                    # 🎯 래치 발동 (t33 — 정지점이 런마다 705~758 로 흔들려
                    #    거리 트리거는 도박. **끼임 이벤트 = 강체 국면 종료
                    #    신호**로 쓴다, 임무 규칙 8 과 같은 원리). 거리(LAG)
                    #    트리거는 보조로 병행. 발동 후 원호 이탈까지 유지.
                    if not st.get(tag + "_go"):
                        _ld7 = r.get("sched_lead_sd")
                        _sc7 = 0.5 * (meta["s0"] + meta["s1"])
                        _pp6 = (d * (_ld7 - _sc7)
                                if _ld7 is not None else -1.0)
                        _jam7 = (int(r.get("stuck", 0))
                                 > int(st.get(tag + "_st0", 0))) and \
                            d * (float(r.get("s_hint") or sd) - _sc7) > -0.03
                        if _pp6 > _lag7 or _jam7:
                            st[tag + "_go"] = True
                            print(f"[{r['name']:8s}] 🔓 뒤 굽힘 발동 "
                                  f"({'끼임 래치' if _jam7 else 'LAG'}, "
                                  f"s={sd * 1000:.0f}mm)")
                    if not st.get(tag + "_go"):
                        _bm = 0.0          # 발동 전: 직진 추력
                elif mi in _tee:
                    _bm *= _rsc
                # 🎯 T 외 원호(곡관)는 뒤도 그대로 꺾는다 (t46 로그 대조:
                #    floor2 정상 신호 = F-40/R-40 인데 BP_SCHED_REAR=0 이
                #    전 원호에 걸려 곡관 자세가 달랐다 — T 한정으로 격리)
            # (구) 각진 코너용 이동 범프 + 뒤 컴플라이언스 — 맵 v2 에서는
            # 역효과(꺾음 지연 → 포켓 진입 후 꺾음). BP_SCHED_SHARP=1 로만 복원.
            if _esc > 0 and os.environ.get("BP_SCHED_SHARP", "0") == "1":
                def _ss3(x):
                    x = max(0.0, min(1.0, x))
                    return x * x * (3.0 - 2.0 * x)
                _sc = 0.5 * (meta["s0"] + meta["s1"])
                _pp = d * (sd - _sc)
                _W = float(os.environ.get("BP_SCHED_W", 0.056))
                _LG = float(os.environ.get("BP_SCHED_JLAG", 0.048))
                _bm *= _ss3((_pp + _W) / _W) - _ss3((_pp - _LG) / _W)
            # 🎯 굽힘 발동 지연 (t63 사용자 관찰 — 복귀 T 에서 코너 128mm
            #    전부터 꺾인 머리가 옆면으로 벽을 긁어 입구 전에 정지.
            #    지정 원호의 리딩 굽힘을 코너 −BP_TEE_BEND_AT 까지 0 유지.
            #    25mm/s 에서 70° 완성에 40mm — 60mm 전 발동이면 코너 직전 완성).
            _dly8 = {int(x) for x in
                     os.environ.get("BP_TEE_DELAY_ARCS", "").split(",") if x}
            if mi in _dly8 and _lead:
                # 🎯 **곡선 램프** (t71 사용자: "한 번에 팍 꺾으니 벽 밀기가
                #    된다 — 곡선으로 천천히 접어 방향을 맞춰라"). 종전
                #    지연-계단(코너 60mm 전 0 → 40mm 만에 70°)을 폐기하고,
                #    진입→정점 128mm 에 걸쳐 smoothstep 으로 0→70°
                #    (25mm/s 기준 ~14°/s — 매 순간 경로 곡률과 일치).
                _sc8 = 0.5 * (meta["s0"] + meta["s1"])
                # 🎯 램프 트리거 = **월드 좌표 직접** (t81 사용자 지적 적중 —
                #    필렛 투영 s 는 코너 근처에서 실제보다 30~40mm 앞서 읽혀
                #    "입구 전 꺾임"을 만들었다. 머리 드럼의 물리 y 로 건다:
                #    y=960 시작 → y=905(개구선 도달) 만각.)
                _dpy = wpos(drum) if drum is not None else None
                if _dpy is not None:
                    _u8 = max(0.0, min(1.0,
                                       (0.960 - float(_dpy[1])) / 0.055))
                else:
                    _u8 = 0.0
                _bm *= _u8 * _u8 * (3.0 - 2.0 * _u8)
                # 🎯 굽힘 이관은 **머리 안착 후에만** (총 90° 보존, 앞→뒤).
                #    안착 = 머리 드럼이 본관 회랑(x<690, |y−850|<45) 진입.
                _w9 = 0.0
                _dp10 = wpos(drum) if drum is not None else None
                if (_dp10 is not None and float(_dp10[0]) < 0.690
                        and abs(float(_dp10[1]) - 0.850) < 0.045
                        and st.get("seat_s") is None):
                    st["seat_s"] = float(r.get("s_hint") or sd)
                    print(f"[{r['name']:8s}] ⚓ 머리 본관 안착 — 굽힘 이관 시작"
                          f" (x={_dp10[0] * 1000:.0f}mm)")
                if st.get("seat_s") is not None:
                    _w9 = max(0.0, min(1.0, d * (float(r.get("s_hint") or sd)
                                                 - st["seat_s"]) / 0.10))
                _bm *= (1.0 - _w9)
                r["sched_xfer_w"] = _w9   # 뒤 관절이 같은 w 를 쓴다
            _bt = _sg * math.radians(_bm)
            if os.environ.get("BP_SCHED_SHARP", "0") == "1" \
                    and r["sched_corner"] and tag == "뒤":
                _bt = float(np.sum(_q[bend_dofs]))
            _cb += max(-lim, min(lim, _bt - _cb))
            # 🎯 T 구간 다리 군기 (2026-08-10 저녁 — 사용자 육안: "다리가
            #    쓸데없이 발산". 공동에서 개구 탐색·허공 밀기가 구멍마다
            #    다리를 쑤셔넣어 갈림날·이음새에 걸렸다. v9 T 통과 조건이
            #    다리 ±4mm 고정이었던 것과 정합 — T 에선 다리를 붙인다):
            #    접합부 중심을 force_legs 에 넘겨 반경 내 다리를 소극 모드로.
            if mi in _tee:
                _ci = int(np.argmin(np.abs(
                    cl.s - 0.5 * (meta["s0"] + meta["s1"]))))
                r["sched_tee_p"] = cl.p[_ci]
                r["sched_tee_mi"] = mi     # 어느 T 인지 (적응 다리 게이트용)
                # 코너 입구 s (진행 방향 기준) — "입구를 지난 다리만 소극/적응"
                r["sched_tee_entry"] = ((meta["s0"] if d > 0 else meta["s1"]),
                                        d)
                r["sched_tee_sc"] = 0.5 * (meta["s0"] + meta["s1"])
                # 다른 가지의 중심 s — 위치 적응 다리의 2차 탐색창 (t67)
                _oth = [j for j in _tee if j != mi and j < len(metas)]
                r["sched_tee_alt"] = (0.5 * (metas[_oth[0]]["s0"]
                                             + metas[_oth[0]]["s1"])
                                      if _oth else None)
                # 🎯 T 원호 안 저속 캡 (t56 — 복귀 T 에서 45mm/s 로는 굽힘이
                #    자세를 만들기 전에 턴인 지점을 지나쳐 오른팔로 드리프트.
                #    나가는 T 성공 조건이 조준 감속 25mm/s 였던 것과 정합).
                r["sched_slow"] = min(r.get("sched_slow", 1.0),
                                      float(os.environ.get("BP_TEE_V", 0.25)))
            elif _pn > 1e-9:
                # 개구 탐색은 T 이외 원호에서만 (기존 로직 유지)
                r["sched_open_dir"] = _pl / _pn
            # 🎯 **리딩 디스크 접기** (t37 — 토크 16배로도 s=731 불변 = 걸림.
            #    welder_126 의 T 통과 실측(08-07)은 다리 **수축**이었다.
            #    T 공동에서 리딩 디스크 다리를 접어 걸림을 없애고, 뒤가
            #    접지·추진을 전담한다 — 배수구 탈출 접기와 같은 원리).
            # 접기는 지정 원호에만 (t59 — 양쪽 T 에 걸자 잘 되던 나가는 T 가
            # 머리 접지 상실로 몸롤 +146° 구름. 사용자 지시: 복귀 T 한정)
            _fold_arcs = {int(x) for x in
                          os.environ.get("BP_TEE_FOLD_ARCS", "").split(",")
                          if x}
            if mi in _fold_arcs and _lead and drum is not None:
                # 🎯 2단 머리 다리 (t60 로그 — 접힌 머리는 열린 본관 입구에서
                #    누를 벽이 없어 앵커 부재 → 뒤 추력이 회전 모멘트를 못
                #    만든다. 머리가 본관 보어 회랑에 들어오면 접기를 풀어
                #    벽을 재파지 = 앵커 생성, 뒤 추력이 그 축으로 몸을 꺾음).
                # 머리 세트는 **항상** 지정 (t79 — 안착 게이트가 세트 생성을
                # 막아 머리가 소극으로 떨어졌던 버그 제거. grip/fold 는
                # force_legs 의 머리 전용 분기가 정한다.)
                _dpF = wpos(drum)
                r["sched_fold_p"] = _dpF
                # 🎯 진입 확정 저속 (t85 계산 — 45mm/s 는 확정 창(오버슛
                #    허용 ~30mm)을 0.7초에 지나쳐 "훅 통과". 필요 시간 2.1초
                #    (뒤 LAG 램프 1.6s + 정착 0.5s) → v ≤ 14 → 12mm/s.
                #    구간: 머리 y<965(입구 55mm 전) ~ 머리 x<660(본관 물림).
                if float(_dpF[1]) < 0.965 and float(_dpF[0]) > 0.660:
                    r["sched_slow"] = min(
                        r["sched_slow"],
                        float(os.environ.get("BP_TEE_V2", 0.12)))
            # [진단] 굽힘 방향이 실제로 원호 평면을 향하는가 — dot(+굽힘 방향,
            # 목표 평면)×부호 가 +1 이어야 맞다. 사출(2026-08-10 t3) 원인 규명용.
            st[tag + "_n"] = st.get(tag + "_n", 0) + 1
            if os.environ.get("BP_SCHED_DBG") == "1" \
                    and st[tag + "_n"] % int(PHYSICS_HZ) == 0:
                _w3 = _pl - _xd * float(np.dot(_pl, _xd))
                _n3 = float(np.linalg.norm(_w3))
                if _n3 > 1e-9:
                    _dd = float(np.dot(_bd, _w3 / _n3))
                    print(f"[sched] {tag} s={sd * 1000:.0f} dot(bd,평면)"
                          f"={_dd:+.2f} 부호={_sg:+.0f} → 유효꺾임 "
                          f"{_dd * _sg:+.2f} (want +1) 굽힘{math.degrees(_cb):+.0f}°")
            if st.get(_zk) != ("ARC", mi):
                st[_zk] = ("ARC", mi)
                st[tag + "_st0"] = int(r.get("stuck", 0))   # 증강 기준점
                print(f"[{r['name']:8s}] 🔒 {tag} 원호#{mi} 진입 — 롤 동결, "
                      f"굽힘 {math.degrees(_bt):+.0f}° 유지 "
                      f"(s={sd * 1000:.0f}mm)")
        r[cr_key], r[cb_key] = _cr, _cb
        return _cr, _cb

    r["sched_slow"] = 1.0
    r["sched_open_dir"] = None
    r["sched_corner"] = False
    r["sched_arc"] = False
    r["sched_fold_p"] = None
    r["sched_tee_p"] = None
    r["sched_kick_dir"] = None
    r["sched_xfer_w"] = None
    # 🎯 크로스바 직진 횡단 (t75 — 코너를 없앴더니 원호가 사라져 접합부
    #    공동 위에서 T 특수 처리가 안 걸림 → 일반 다리 캡이 무력화돼 끼임.
    #    닫힘 접합부(코스 끝 −250mm) ±120mm 를 가상 T 구역으로: 소극+저속).
    if r["name"] == "floor1" and os.environ.get("BP_XING", "1") == "1":
        _xs = cl.total - 0.250
        if abs(s1 - _xs) < 0.12:
            _xi = int(np.argmin(np.abs(cl.s - _xs)))
            r["sched_tee_p"] = cl.p[_xi]
            r["sched_tee_mi"] = -1        # 적응 게이트 미해당 (소극만)
            r["sched_slow"] = min(r["sched_slow"], float(
                os.environ.get("BP_TEE_V", 0.25)))
    _crf, _cbf = _drum_cmd(r.get("drum_prim"), r["roll_front"],
                           r["bend_front"], "cmd_roll", "cmd_bend", 1.0, "앞")
    _rear_roll = [k for k in r["roll_dof"] if k not in r["roll_front"]]
    _rear_bend = [k for k in r["bend_dof"] if k not in r["bend_front"]]
    _crr, _cbr = _drum_cmd(r.get("drum_rear_prim"), _rear_roll, _rear_bend,
                           "cmd_roll_r", "cmd_bend_r", BP_REAR_SIGN, "뒤")
    r["bp_f_deg"], r["bp_r_deg"] = math.degrees(_cbf), math.degrees(_cbr)
    return (r["roll_front"] + r["bend_front"] + _rear_roll + _rear_bend,
            [_crf] * len(r["roll_front"]) + [_cbf] * len(r["bend_front"])
            + [_crr] * len(_rear_roll) + [_cbr] * len(_rear_bend))


def steer_bp_rollbend(r):
    """🎯 **정답지 조향** (2026-08-09) — 도면(중심선)으로 롤·굽힘을 몬다.

    목적은 자율주행이 아니라 **"이 몸이 이 코스를 물리적으로 완주하는가"의
    정답지**다(NAV=blueprint 전용 — 성능 수치를 자율 근거로 쓰지 말 것).
    🔑 롤기하 프로브가 확정한 사실 위에 선다:
       · 굽힘 평면은 곡관 평면과 일치한다(어긋남 ±180 = 부호만 음수로)
       · 정지 원인은 각도 부족 — 필요 90° 에 지령 40°(상한) + 뒤 0°(방치)
    → 필요 각의 정답을 중심선에서 직접 뽑아 **앞·뒤 관절에 나눠 준다**:
       앞 굽힘 = (몸통 축 ↔ 앞쪽 접선) 각,  뒤 굽힘 = (뒤쪽 접선 ↔ 몸통 축) 각
       부호는 각 드럼의 +굽힘 방향에 투영해 정한다(프레임 논쟁 금지 — 실측).
    """
    if not r.get("bend_front"):
        return [], []
    cl = r["cl"]
    _mid = r.get("mid_prim")
    if _mid is None:
        return [], []
    _ax = wrot(_mid) @ np.array([1.0, 0.0, 0.0])      # 몸통 축
    p1 = wpos(r["seg1"])
    s1, _o, _ = cl.nearest(p1, r.get("s_hint"))
    lim = math.radians(STEER_RATE) * PHYSICS_DT

    # 관구 접근(복귀)은 라이저 직선 — 굽힘을 편다 (위 EXIT_STRAIGHT_S 주석)
    if r.get("phase") in ("BACK", "EXIT") and s1 < EXIT_STRAIGHT_S:
        _cb = r.get("cmd_bend", 0.0)
        _cbr2 = r.get("cmd_bend_r", 0.0)
        _cb += max(-lim, min(lim, -_cb))
        _cbr2 += max(-lim, min(lim, -_cbr2))
        r["cmd_bend"], r["cmd_bend_r"] = _cb, _cbr2
        r["bp_f_deg"], r["bp_r_deg"] = math.degrees(_cb), math.degrees(_cbr2)
        _rb2 = [k for k in r["bend_dof"] if k not in r["bend_front"]]
        _rr2 = [k for k in r["roll_dof"] if k not in r["roll_front"]]
        return (r["roll_front"] + r["bend_front"] + _rr2 + _rb2,
                [r.get("cmd_roll", 0.0)] * len(r["roll_front"])
                + [_cb] * len(r["bend_front"])
                + [r.get("cmd_roll_r", 0.0)] * len(_rr2)
                + [_cbr2] * len(_rb2))

    def _aim(drum, tgt_dir, cr_key, cb_key, sign=1.0, plane_dir=None):
        """드럼 하나에 (롤, 굽힘) 지령 쌍 — 평면을 돌리고 그 평면에서 꺾는다.

        🚨 초판은 굽힘만 주고 **롤을 아예 안 돌렸다** — 곡관1(수직 평면)에서
           곡관2/T(수평 평면)로 넘어가는 경계(floor2 s=510 / floor1 s=501)에서
           두 코스가 똑같이 섰다. 관절은 자유인데 돌리라는 지령이 없었던 것.
        🔑 카메라 롤 제어에서 오늘 검증한 부품을 그대로 쓴다(표현 선택 +
           115° 히스테리시스). 입력만 카메라 → 중심선.
        """
        _Rd = wrot(drum)
        _xd = _Rd @ np.array([1.0, 0.0, 0.0])
        _bd = np.cross(_Rd @ np.array([0.0, 1.0, 0.0]), _xd)
        # 🎯 **평면과 각도를 분리한다** (2026-08-10 A/I 대조로 확정).
        #    롤(평면)은 절대 기하(plane_dir = 원호 출구 접선)로 — 도는 디스크
        #    뒤쫓기(적분 폭주, A/B/G 사출)가 원리적으로 사라진다.
        #    굽힘(각도)은 look 접선으로 점진 — 출구 접선으로 각도까지 주면
        #    진입 즉시 60° 만곡을 요구해 입구 턱에 쐐기가 된다(I: 끼임 44회).
        _pv = plane_dir if plane_dir is not None else tgt_dir
        _perp = _pv - _ax * float(np.dot(_pv, _ax))
        _n = float(np.linalg.norm(_perp))
        ang = math.acos(max(-1.0, min(1.0, abs(float(np.dot(tgt_dir, _ax))))))
        _cr = r.get(cr_key, 0.0)
        _cb = r.get(cb_key, 0.0)
        if _n < 1e-6 or ang < math.radians(2.0):
            _cb += max(-lim, min(lim, 0.0 - _cb))     # 곧게 편다
            # 원호 선조준: 직선(펴진 상태)에서만 롤을 목표 평면으로 돌린다 —
            # H런의 "틀린 평면에 동결" 이 여기서 풀린다(PRE 구간 = 직선).
            if plane_dir is not None and _n > 1e-6 and \
                    abs(_cb) <= math.radians(BP_ROLL_LOCK_AT):
                _perp2 = _perp / _n
                _e2 = math.atan2(float(np.dot(np.cross(_bd, _perp2), _xd)),
                                 float(np.dot(_bd, _perp2)))
                _t2 = _cr + sign * _e2
                if abs(_t2 - _cr) > math.radians(115.0):
                    _a2 = _t2 - math.copysign(math.pi, _t2 - _cr)
                    if abs(_a2 - _cr) < abs(_t2 - _cr):
                        _t2 = _a2
                _cr += max(-lim, min(lim, _t2 - _cr))
            r[cr_key], r[cb_key] = _cr, _cb
            return _cr, _cb
        _perp /= _n
        _err = math.atan2(float(np.dot(np.cross(_bd, _perp), _xd)),
                          float(np.dot(_bd, _perp)))
        _tgt_r = _cr + sign * _err
        _bnd = min(ang, math.radians(BP_BEND_MAX))
        if abs(_tgt_r - _cr) > math.radians(115.0):   # 반대 표현이 가깝다
            _alt = _tgt_r - math.copysign(math.pi, _tgt_r - _cr)
            if abs(_alt - _cr) < abs(_tgt_r - _cr):
                _tgt_r, _bnd = _alt, -_bnd
        # 🚨 굽힘이 걸려 있으면 롤을 동결한다 (BP_ROLL_LOCK_AT) — 자유 롤이
        #    벽 토크로 돌 때 그걸 쫓으면 적분 폭주다. 평면 전환은 펴진 채로.
        if abs(_cb) > math.radians(BP_ROLL_LOCK_AT):
            _tgt_r = _cr
        _cr += max(-lim, min(lim, _tgt_r - _cr))
        _cb += max(-lim, min(lim, _bnd - _cb))
        r[cr_key], r[cb_key] = _cr, _cb
        return _cr, _cb

    # **리딩 끝만 앞(진행 방향)을 본다** — 트레일링 끝은 제 위치의 접선.
    # 🚨 초판은 앞 디스크가 항상 s+LOOK 을 봤다 — 전진 전용 가정. 복귀(−s)
    #    에서는 앞이 트레일링인데 **이미 지나온 구간의 접선**으로 꺾인 채
    #    코너에 박혔다(2026-08-10 실측: 복귀 라이저 하단 s=185 끼임 4회,
    #    뒤 디스크는 s=62 까지 올라갔는데 앞이 코너에 걸려 휠 122% 헛돎).
    p0 = wpos(r["seg0"])
    s0, _o0, _ = cl.nearest(p0, r.get("s_hint"))
    if r.get("dir", 1) > 0:            # 전진: 앞이 리딩
        jf = int(np.argmin(np.abs(cl.s - (s1 + STEER_LOOK))))
        jr = int(np.argmin(np.abs(cl.s - s0)))
    else:                              # 복귀: 뒤가 리딩
        jf = int(np.argmin(np.abs(cl.s - s1)))
        jr = int(np.argmin(np.abs(cl.s - (s0 - STEER_LOOK))))
    # 원호 구간(+PRE)에서는 리딩 목표를 **원호 출구 접선**으로 (위 BP_ARC 주석)
    arcs = r.get("cl_arcs")
    if arcs is None:
        arcs = r["cl_arcs"] = _cl_arcs(cl)
        print(f"  {r['name']}: 원호 구간 "
              f"{[(round(a * 1000), round(b * 1000)) for a, b, _x, _y in arcs]}"
              f"mm (출구 조준 대상)")
    _pl_f = _pl_r = None                  # 원호 평면 목표(출구 접선) — 롤 전용
    if BP_ARC_AIM:                        # 기본 꺼짐 — 위 BP_ARC_AIM 주석
        for _a0, _a1, _ia, _ib in arcs:
            if r.get("dir", 1) > 0 and _a0 - BP_ARC_PRE <= s1 <= _a1:
                _pl_f = cl.tangent(min(_ib + 2, len(cl.s) - 1))
                break
            if r.get("dir", 1) < 0 and _a0 <= s1 <= _a1 + BP_ARC_PRE:
                _pl_r = cl.tangent(max(_ia - 2, 0))
                break
    tf = cl.tangent(jf)
    if float(np.dot(tf, _ax)) < 0.0:
        tf = -tf
    tr = cl.tangent(jr)
    if float(np.dot(tr, _ax)) < 0.0:
        tr = -tr
    if _pl_f is not None and float(np.dot(_pl_f, _ax)) < 0.0:
        _pl_f = -_pl_f
    if _pl_r is not None and float(np.dot(_pl_r, _ax)) < 0.0:
        _pl_r = -_pl_r
    _crf = _cbf = _crr = _cbr = 0.0
    if r.get("drum_prim") is not None:
        _crf, _cbf = _aim(r["drum_prim"], tf, "cmd_roll", "cmd_bend",
                          plane_dir=_pl_f)
    if r.get("drum_rear_prim") is not None:
        _crr, _cbr = _aim(r["drum_rear_prim"], tr, "cmd_roll_r", "cmd_bend_r",
                          sign=BP_REAR_SIGN, plane_dir=_pl_r)
    r["bp_f_deg"], r["bp_r_deg"] = math.degrees(_cbf), math.degrees(_cbr)
    _rear_bend = [k for k in r["bend_dof"] if k not in r["bend_front"]]
    _rear_roll = [k for k in r["roll_dof"] if k not in r["roll_front"]]
    return (r["roll_front"] + r["bend_front"] + _rear_roll + _rear_bend,
            [_crf] * len(r["roll_front"]) + [_cbf] * len(r["bend_front"])
            + [_crr] * len(_rear_roll) + [_cbr] * len(_rear_bend))


def steer_rollbend(r):
    """**롤로 겨누고 굽힘 한 축으로 꺾는다** (v9 구조, 2026-08-09 신설).

    🔑 컨트롤러가 주는 것은 `(세기 mag, 화소 방위 clock)` 하나다. 지금까지는
       그것을 피치·요 두 축에 sin/cos 로 **배분**했는데, 비틀림이 잠긴 몸에서는
       배분식이 자세에 따라 요동쳐 관절이 좌우로 왕복했다(브레이크 댄스).
       v9 는 롤이 **자유회전**이라 평면 자체를 돌릴 수 있다 →
           롤 목표 = 방위,  굽힘 목표 = 세기
       방향과 크기가 분리되어 배분이 사라진다.
    🚨 **뒤 굽힘은 건드리지 않는다.** 단독 실측에서 앞만 꺾은 것이 최고였다
       (앞만 +126mm / 앞뒤 반대부호 C자 +123 / 같은 부호 S자 +16mm).
       뒤는 자유롭게 두면 벽이 알아서 맞춰 준다.
    🚨 **굽힘 부호는 음수가 "위"다** — 굽힘축이 Y 라 오른손 법칙으로 양수가
       아래다(단독 시험에서 +37 을 줘 머리를 바닥에 처박은 실측).
    """
    if not r.get("bend_front"):
        return [], []
    q = np.asarray(r["art"].get_joint_positions())
    mag, clock = r.get("ctl_steer", (0.0, 0.0))
    _c = r.get("cond")
    # 직진 장면에서는 편다 — v1_1 의 완화 규약과 같다.
    # 🚨 **완화 조건에서 분기비율 항을 뺀다** (2026-08-09). v1_1 은 "분기비율
    #    <8% 이고 입사각<12°" 를 요구하는데, 실전 맵은 분기 신호가 5~60% 로 늘
    #    떠 있어 **완화가 거의 성립하지 않는다** — 한 번 접힌 몸이 안 펴진다
    #    (사용자 GUI 지적: *"다시 수평을 유지하지 않으니 방향을 못 잡는다"*).
    #    조향 세기(mag)가 이미 입사각에서 나온 값이므로 그것만 보면 충분하다.
    if (r.get("straighten_n", 0) > 0
            or r.get("ctl_state") == "RECOVER"
            or mag <= 0.0):
        _lim = math.radians(RELAX_RATE) * PHYSICS_DT
        idx = r["bend_front"]
        # 누적 지령도 같이 0 으로 되감는다 — 안 그러면 완화가 끝난 첫 프레임에
        # 묵은 지령이 튀어 다시 접힌다(v1_1 에서 겪은 그대로).
        for _k in ("cmd_bend",):
            _v = r.get(_k, 0.0)
            r[_k] = _v - max(-_lim, min(_lim, _v))
        return (idx, [float(q[k]) - max(-_lim, min(_lim, float(q[k])))
                      for k in idx])
    # 🎯 **롤은 측정으로 몬다** (2026-08-09 사용자 지적으로 재작성).
    # 🚨 한때 롤 목표를 `방위 + 180°` 라는 **절대 각도**로 열어 놓고 줬다.
    #    롤 관절은 한계가 없어(자유회전) 관절 0점과 화소 방위 0점이 서로
    #    무관하다 — 기준점이 없으니 어디까지 감길지 아무도 모른다. 실측:
    #    **롤이 +190° 까지 감겨** 몸이 통째로 뒤집혔다(꺾임 194.5°).
    # 🔑 조원 스크립트가 같은 이유로 **드럼의 실제 월드 자세**를 읽어 쓴다 —
    #    *"굽힘 평면의 절대 방위가 우리가 정말 원하는 것"*. 우리도 같은 것을
    #    가지고 있다: 카메라 프림의 월드 자세(오른손 법칙 계산에 이미 쓴다).
    #    · 드럼 X·Y 축 → **양(+)의 굽힘이 앞 디스크를 보내는 월드 방향**
    #    · 카메라 자세 + 개구 화소 방위 → **가고 싶은 월드 방향**
    #    · 둘 사이의 각을 관 축 둘레로 재서 그만큼만 롤을 돌린다
    #    열린 루프가 아니라 오차를 줄이는 것이라 감길 일이 없다.
    _drum = r.get("drum_prim")
    _camp = r.get("cam_prim")
    _roll_err = None
    if _drum is not None and _camp is not None:
        _Rd = wrot(_drum)                       # 열이 기저
        _xd = _Rd @ np.array([1.0, 0.0, 0.0])
        _yd = _Rd @ np.array([0.0, 1.0, 0.0])
        _bend_dir = np.cross(_yd, _xd)          # +굽힘이 디스크를 보내는 방향
        _n = float(np.linalg.norm(_bend_dir))
        _Rc = wrot(_camp)
        _img_r = _Rc @ np.array([1.0, 0.0, 0.0])
        _img_d = _Rc @ np.array([0.0, -1.0, 0.0])
        _th = math.radians(clock)
        _want_dir = math.cos(_th) * _img_r + math.sin(_th) * _img_d
        _wn = float(np.linalg.norm(_want_dir))
        if _n > 1e-6 and _wn > 1e-6:
            _bend_dir /= _n
            _want_dir /= _wn
            # 관 축(_xd) 둘레의 부호 있는 각 — 이만큼 롤을 돌리면 평면이 맞는다
            _roll_err = math.atan2(
                float(np.dot(np.cross(_bend_dir, _want_dir), _xd)),
                float(np.dot(_bend_dir, _want_dir)))
    if _roll_err is None:                       # 측정 불가 — 롤을 안 건드린다
        _roll_err = 0.0
    r["roll_err_deg"] = math.degrees(_roll_err)
    # 🔬 보정 시험 (ROLL_CAL=도) — 롤을 **고정 각도**로 돌려 놓고 측정 오차가
    #    그만큼 따라 변하는지 본다. 측정식의 부호·배율을 이론 없이 실측으로
    #    확정하는 유일한 방법(부호 이론 논쟁 금지 규칙).
    if ROLL_CAL is not None:
        return (r["roll_front"], [math.radians(ROLL_CAL)] * len(r["roll_front"]))
    # 🚨 **부호는 실측으로 가른다** (기록된 규칙). 보정 시험(ROLL_CAL 스윕):
    #    롤 0→39→80° 에 오차 −127→−157→+134° — 롤+ 가 오차− 를 만든다.
    #    이득이 1:1 이 아닌 것(≈0.77)은 카메라가 앞 디스크에 붙어 롤과 같이
    #    돌기 때문(측정 일부 상쇄) — 방향은 유지되므로 적분이 수렴한다.
    # 🚨 **±180° 클램프 철회** (궤적 실측: 클램프에 박혀 오차 +63~107° 가
    #    전 구간 안 닫혔다). 조원 스크립트 머리말의 처방을 그대로 쓴다 —
    #    *"(롤, +굽힘) 과 (롤±180°, −굽힘) 은 같은 평면이다. 지금 지령에
    #    가까운 표현을 골라라."* 목표가 90° 넘게 멀면 롤을 180° 당기고 굽힘
    #    부호를 뒤집는다. 클램프가 필요 없어진다.
    # 🚨 이 함수는 **편집이 세 겹 쌓여** 새 계산이 옛 줄에 덮이는 사고가
    #    있었다(_cr 리셋 → 옛 적분기 실행). 지령 계산부는 이 블록 하나다 —
    #    덧대지 말고 여기를 고칠 것.
    _bend = -math.radians(min(mag, STEER_MAX_V9))     # 음수 = 위
    lim = math.radians(STEER_RATE) * PHYSICS_DT
    _cr = r.get("cmd_roll", 0.0)
    _cb = r.get("cmd_bend", 0.0)
    # ── 🎯 **역할 분리** (2026-08-09 확정 — 실측 역설이 근거) ──────────
    # 🚨 조향 결함 6개를 다 고쳐 롤·굽힘이 일관되게 작동하자 **오히려
    #    나빠졌다**(조향 없음 s=601 / 조향 정상작동 s=264). 버그가 아니라
    #    전략이 틀렸던 것: 곡관에서 카메라는 반쯤 장님(관경 18~34mm)이라
    #    그 신호로 롤을 조준해 관절 하나에 세게 주면, 틀리는 순간 96mm
    #    강체가 통째로 벽에 박힌다.
    # 🔑 단독 실측이 역할을 이미 갈라 줬다:
    #      곡관: 앞 굽힘만 + 밀기 → +126mm ✅  (롤 조준 불필요)
    #      T   : 롤 조준 + 굽힘   → 전 방향 ✅ (조준이 핵심)
    #    조원 스크립트가 분기 전용이고 곡관 조향이 없는 것도 같은 설계다.
    #      · 곡관·직관 → 롤 **유지**, 지금 평면에 투영해 굽힘 **부호만** 고른다
    #      · BRANCH/T  → 롤 조준 전체(측정+표현선택+히스테리시스+잠금)
    _bend = -math.radians(min(mag, STEER_MAX_V9))     # 음수 = 위(관례)
    lim = math.radians(STEER_RATE) * PHYSICS_DT
    _cr = r.get("cmd_roll", 0.0)
    _cb = r.get("cmd_bend", 0.0)
    _aim = (r.get("ctl_state") == "BRANCH_ENTRY"
            or (_c is not None and _c.state == "BRANCH"))
    if not _aim:
        # 곡관·직관 — 개구가 지금 굽힘 평면의 어느 쪽인지만 본다.
        # |오차|<90° 면 +굽힘 쪽, 아니면 −굽힘 쪽. 롤은 안 돌린다.
        _bnd = (+1.0 if abs(_roll_err) < math.pi / 2.0 else -1.0)             * math.radians(min(mag, STEER_MAX_V9))
        _cb += max(-lim, min(lim, _bnd - _cb))
        r["cmd_bend"] = _cb
        r["roll_tgt_deg"] = math.degrees(_cr)
        r["steer_err"] = mag
        r["steer_max"] = max(r.get("steer_max", 0.0), mag)
        return (r["bend_front"], [_cb] * len(r["bend_front"]))
    # ── T 분기 — 롤 조준 (측정 + 표현 선택 + 히스테리시스 + 잠금) ────
    # 굽힘 부호 반영: 측정은 +굽힘 방향 기준, 기본 지령은 음수(위)라
    # 정렬 상태가 +180° 로 찍힌다 — 조원이 롤·굽힘을 한 쌍으로 두는 이유.
    _e = _roll_err
    if _bend < 0.0:
        _e = ((_e - math.pi) + math.pi) % (2.0 * math.pi) - math.pi
    _tgt = _cr + ROLL_SIGN * _e
    _bnd = _bend
    if abs(_tgt - _cr) > math.radians(115.0):    # 경계 여유 (조원 값)
        _alt = _tgt - math.copysign(math.pi, _tgt - _cr)
        if abs(_alt - _cr) < abs(_tgt - _cr):
            _tgt, _bnd = _alt, -_bnd
    r["roll_tgt_deg"] = math.degrees(_tgt)
    _bent = abs(math.degrees(float(np.sum(q[r["bend_front"]]))))         > ROLL_LOCK_AT_V9
    if not _bent:                                # 꺾인 몸으로 롤 금지
        _cr += max(-lim, min(lim, _tgt - _cr))
    _cb += max(-lim, min(lim, _bnd - _cb))
    r["cmd_roll"], r["cmd_bend"] = _cr, _cb
    r["steer_err"] = mag
    r["steer_max"] = max(r.get("steer_max", 0.0), mag)
    return (r["roll_front"] + r["bend_front"],
            [_cr] * len(r["roll_front"]) + [_cb] * len(r["bend_front"]))


def steer_vision(r):
    """**검출기의 분기 방위(`branch_deg`)로 조향한다** (2026-08-07 배선).

    🔑 화소 방위 → 로봇 몸 프레임 변환이 전부다. 전방 카메라 축은
       X_cam = −Y_robot, Y_cam = +Z_robot 이고 **화소 y 는 아래로 증가**하므로
       화면 방위 φ 에 보이는 개구는 로봇 (y,z) 평면에서
           y_r = −cos φ,  z_r = −sin φ    →   θ_robot = φ + 180°
    🔑 진입을 시작하면 개구가 화각 밖으로 벗어나 BRANCH 가 끊긴다 —
       마지막 방위를 `BRANCH_HOLD_S` 동안 유지해 마저 꺾는다.
    🚨 개구를 본 적이 없으면 지령 0 — 목표를 현재 각도로 두어 관절 스프링이
       벽이 만든 꺾임과 싸우지 않게 한다(`steer_onboard` 와 같은 규약).
    """
    if not r.get("bel_pitch"):
        return [], []
    q = np.asarray(r["art"].get_joint_positions())
    # 🎯 2026-08-07 — 조향은 **`DriveController` 의 분기 진입 기동 지시**를
    #    실행할 뿐이다. 여기서 판단하지 않는다(비례 조향은 실측 기각 —
    #    구멍으로 통째로 빠졌다). 컨트롤러가 방위를 래치하고 호 길이로
    #    기동을 끝내므로, 실행기는 화소 방위 → 몸 프레임 변환만 한다.
    mag, clock = r.get("ctl_steer", (0.0, 0.0))
    # 🚨 **직진 장면에서는 관절을 서서히 편다** (2026-08-07 사용자 스크린샷
    #    보고: *"일자관인데 계속 웅크려 있다"*). 지령 0 = "현재 각 유지"
    #    규약은 곡관에서 벽과 안 싸우려는 것인데, T 진입이 만든 40° 굽힘이
    #    직관에서도 영구 잔류하는 부작용이 있었다. 판정이 곧은 관(NORMAL·
    #    분기 미미·입사각 작음)일 때만 초당 RELAX_RATE 로 0° 를 향해 편다 —
    #    곡관·분기·기동 중에는 기존 규약 그대로.
    _c = r.get("cond")
    # MISALIGNMENT 도 편다 — 정렬 이탈이야말로 몸을 곧게 펴야 고쳐지는
    # 장면이다(실측: 안착 요동 굽힘 10° 가 유지되며 시야가 비뚤어져
    # MISALIGNMENT 가 영영 안 풀리던 reducer 출발 정체).
    # 재출발 유예(straighten_n) 중에는 판정과 무관하게 편다 — 접합부 앞이라
    # 판정이 BRANCH 라서 안 펴지는 것을 뚫는 것이 유예의 목적이다.
    # RECOVER 중에도 편다 — 굽힘을 유지한 채 흔들면 모서리에서 양방향 쐐기
    # (실측 s=178 동결). 회복 = 몸을 펴고 살살 빠져나오는 동작.
    if (r.get("straighten_n", 0) > 0
            or r.get("ctl_state") == "RECOVER"
            or (mag <= 0.0 and _c is not None
                and _c.state in ("NORMAL", "MISALIGNMENT")
                and _c.branch_ratio < 0.08
                and abs(_c.incidence_deg) < 12.0)):
        _lim = math.radians(RELAX_RATE) * PHYSICS_DT
        # 누적 조향 지령도 같이 0 으로 되감는다 — 안 그러면 완화가 끝난 뒤
        # 첫 조향 프레임에 묵은 지령(target = q + cmd)이 튀어 다시 굽힌다.
        _dl = RELAX_RATE * PHYSICS_DT
        for _k in ("cmd_pitch", "cmd_yaw"):
            _v = r.get(_k, 0.0)
            r[_k] = _v - max(-_dl, min(_dl, _v))
        idx = r["bel_pitch"] + r["bel_yaw"]
        return (idx,
                [float(q[k]) - max(-_lim, min(_lim, float(q[k])))
                 for k in idx])
    th = math.radians(clock + 180.0)      # 전방 카메라 장착 변환 (φ+180°)
    np_, ny = len(r["bel_pitch"]), len(r["bel_yaw"])
    tp = float(np.clip(-mag * math.sin(th) / np_, -STEER_MAX, STEER_MAX))
    ty = float(np.clip(+mag * math.cos(th) / ny, -STEER_MAX, STEER_MAX))
    lim = STEER_RATE * PHYSICS_DT
    cp, cy = r.get("cmd_pitch", 0.0), r.get("cmd_yaw", 0.0)
    cp += max(-lim, min(lim, tp - cp))
    cy += max(-lim, min(lim, ty - cy))
    r["cmd_pitch"], r["cmd_yaw"] = cp, cy
    r["steer_err"] = mag
    r["steer_max"] = max(r.get("steer_max", 0.0), mag)
    return (r["bel_pitch"] + r["bel_yaw"],
            [float(q[k]) + math.radians(cp) for k in r["bel_pitch"]]
            + [float(q[k]) + math.radians(cy) for k in r["bel_yaw"]])


def steer_onboard(r):
    """**도면 없이** 조향한다 — 앞 세그먼트가 벽을 잃은 방향으로 몬다.

    🔑 쓰는 신호는 전부 로봇 것이다: 다리별 접촉(힘/속도로 판정) + 다리
       시계각(로봇 제원) + 중앙 관절 엔코더. 중심선을 안 본다.
    🔑 분기에서 개구는 **벽이 사라진 방향**으로 나타난다. 그쪽으로 몸을 몰면
       가지로 들어간다(사용자 표현: 왼발차기).
    🚨 벽을 잃은 다리가 없으면 **목표를 현재 각도로 둔다** — 관절 스프링이
       벽이 만든 꺾임과 싸우지 않게 한다(강성 60 은 조향용이라 세다).
    """
    if not r.get("bel_pitch"):
        return [], []
    q = np.asarray(r["art"].get_joint_positions())
    free = r.get("leg_free", {})
    fs = max(sg for sg, _i in r["piston"])          # 앞 세그먼트
    ths = [r["leg_clock"].get(k, 0.0)
           for (sg, _i), k in r["piston"].items() if sg == fs and free.get(k)]
    if ths:
        _c = float(np.mean(np.cos(np.radians(ths))))
        _s = float(np.mean(np.sin(np.radians(ths))))
        th = math.atan2(_s, _c)
        mag = STEER_ONBOARD_DEG * min(len(ths) / 2.0, 1.0)
    else:
        th, mag = 0.0, 0.0
    np_, ny = len(r["bel_pitch"]), len(r["bel_yaw"])
    # 개구 방향 θ 로 코를 돌린다: yaw ∝ +cosθ, pitch ∝ −sinθ
    tp = float(np.clip(-mag * math.sin(th) / np_, -STEER_MAX, STEER_MAX))
    ty = float(np.clip(+mag * math.cos(th) / ny, -STEER_MAX, STEER_MAX))
    lim = STEER_RATE * PHYSICS_DT
    cp, cy = r.get("cmd_pitch", 0.0), r.get("cmd_yaw", 0.0)
    cp += max(-lim, min(lim, tp - cp))
    cy += max(-lim, min(lim, ty - cy))
    r["cmd_pitch"], r["cmd_yaw"] = cp, cy
    r["steer_err"] = mag
    r["steer_max"] = max(r.get("steer_max", 0.0), mag)
    r["want"] = (math.degrees(-math.sin(th)) * 0, mag)
    # 목표 = **현재 각도 + 지령** → 지령이 0 이면 벽이 만든 꺾임을 안 거스른다
    return (r["bel_pitch"] + r["bel_yaw"],
            [float(q[k]) + math.radians(cp) for k in r["bel_pitch"]]
            + [float(q[k]) + math.radians(cy) for k in r["bel_yaw"]])


def steer(r, i_now):
    """중앙 관절로 **중심선을 따라간다** — 앞을 내다보고 헤딩 오차를 닫는다.

    🔑 곡관은 벽이 꺾어 주지만 **분기는 직진로가 열려 있어 안 꺾인다.**
       가지로 들어가려면 관절에 목표각을 직접 줘야 한다.
    🚨 목표각은 **초당 각도로 제한**한다 — 한 번에 던지면 발산한다(기록된 사고).
    🚨 관절 위치 목표는 `center_legs` 것과 **한 번에 같이** 써야 한다.
       나눠 쓰면 뒤에 쓴 것만 남는다(기록된 `apply_action` 함정).
    """
    if not r.get("bel_pitch"):
        return [], []
    cl = r["cl"]
    # 🚨 **뒤 세그먼트의 헤딩 오차로 닫으면 안 된다** — 관절을 꺾어도 뒤
    #    세그먼트는 안 돌아가므로 오차가 영영 안 닫히고 목표각이 포화된다
    #    (실측: 직관에서 조향오차 79°, 관절 33° 꺾임). 관절이 실제로 만드는
    #    각은 **두 세그먼트 사이에서 경로가 꺾이는 각**이다 → 그것을 준다.
    # 🔑 진행 방향과 무관하다(전진·후진 같은 식) — 몸통 사슬을 관 모양에
    #    맞추는 것이지 진행 방향을 쫓는 것이 아니다.
    p0, p1 = wpos(r["seg0"]), wpos(r["seg1"])
    span = p1 - p0
    s1, _o1, i1 = cl.nearest(p1, r.get("s_hint"))
    j = int(np.argmin(np.abs(cl.s - (s1 + STEER_LOOK * r["dir"]))))
    d = cl.tangent(j)
    if float(np.dot(d, span)) < 0.0:          # 접선 부호를 몸통 방향에 맞춘다
        d = -d
    e = r["fr"] @ (wrot(r["seg0"]).T @ d)     # 로봇 전방 정렬 프레임에서 본 목표
    e = e / max(np.linalg.norm(e), 1e-12)
    yaw = math.degrees(math.atan2(e[1], e[0]))
    pitch = math.degrees(math.atan2(-e[2], math.hypot(e[0], e[1])))
    np_, ny = len(r["bel_pitch"]), len(r["bel_yaw"])
    tp = float(np.clip(STEER_KP * pitch / np_, -STEER_MAX, STEER_MAX))
    ty = float(np.clip(STEER_KP * yaw / ny, -STEER_MAX, STEER_MAX))
    lim = STEER_RATE * PHYSICS_DT
    cp, cy = r.get("cmd_pitch", 0.0), r.get("cmd_yaw", 0.0)
    cp += max(-lim, min(lim, tp - cp))
    cy += max(-lim, min(lim, ty - cy))
    r["cmd_pitch"], r["cmd_yaw"] = cp, cy
    # [진단] 실제 몸통 사슬이 어느 쪽으로 꺾였는가 — 목표(e)와 같은 프레임에서
    # 재서 나란히 본다. 축 매핑(`:1`=피치 가정)이 틀리면 여기서 갈린다.
    u = r["fr"] @ (wrot(r["seg0"]).T @ (span / max(np.linalg.norm(span), 1e-12)))
    r["act_yaw"] = math.degrees(math.atan2(u[1], u[0]))
    r["act_pitch"] = math.degrees(math.atan2(-u[2], math.hypot(u[0], u[1])))
    r["want"] = (pitch, yaw)
    # 관절이 지령을 실제로 따라갔는가 (드라이브 힘 부족 ↔ 벽에 눌림을 가른다)
    _q = np.asarray(r["art"].get_joint_positions())
    r["q_pitch"] = math.degrees(float(np.sum(_q[r["bel_pitch"]])))
    r["q_yaw"] = math.degrees(float(np.sum(_q[r["bel_yaw"]])))
    r["steer_err"] = math.hypot(pitch, yaw)
    r["steer_max"] = max(r.get("steer_max", 0.0), r["steer_err"])
    return (r["bel_pitch"] + r["bel_yaw"],
            [math.radians(cp)] * np_ + [math.radians(cy)] * ny)


def curve_speed(r):
    """관절 엔코더 → 목표 속도(m/s). **카메라를 쓰지 않는다.**

    중앙 관절 4개(X·Y·X·Y)를 합성해 잰다 — 수직곡관은 피치축이, 수평곡관은
    요축이 꺾이므로 한 축만 보면 나머지 방향을 못 알아본다.
    """
    if not r.get("bel"):
        return TARGET_SPEED_MPS
    ang = np.degrees(np.abs(
        np.asarray(r["art"].get_joint_positions())[r["bel"]]))
    bend = float(np.linalg.norm(ang))
    f = 1.0 - (1.0 - CURVE_MIN) * min(bend / max(BEND_REF, 1e-6), 1.0)
    r["bend_deg"], r["curve_f"] = bend, f
    r["bend_max"] = max(r.get("bend_max", 0.0), bend)
    return TARGET_SPEED_MPS * f


def leg_speed(r):
    """다리 실린더 → 속도 배율. **관절이 못 보는 분기·관경 변화를 본다.**

    기준선(느린 EMA)에서 평균이 벗어난 양과 다리 편차 중 큰 쪽으로 감속한다.
    둘 다 **로봇이 자기 관절에서 직접 읽는 값**이라 도면이 필요 없다.
    """
    if not LEG_SLOW_ON or not r.get("piston"):
        return 1.0
    q = np.asarray(r["art"].get_joint_positions())
    ks = list(r["piston"].values())
    spread = float(np.ptp(q[ks]))
    mean = float(np.mean(q[ks]))
    ema = r.get("leg_ema", mean)
    r["leg_ema"] = ema + 0.002 * (mean - ema)      # 느린 기준선
    dev = abs(mean - r["leg_ema"])
    # 🚨 **편차(spread)는 쓰지 않는다** (2026-08-07 실측). 정상 직관·곡관에서도
    #    8~18mm 라 임계 8mm 가 상시 발동해 속도가 내내 0.45 로 눌렸고, 리듀서는
    #    왕복 2→1회로 오히려 느려졌다. 분기에서도 정상 대비 분리가 없다.
    # 🔑 **평균의 기준선 이탈은 관경 변화를 40배 차이로 가른다** —
    #    리듀서 테이퍼 4.2~5.8mm vs 그 외 0.1~1.4mm. 이것만 쓴다.
    #    ⚠ 분기는 이 신호로 **안 잡힌다** — 다리가 개구로 뻗지 않기 때문이다.
    u = dev / LEG_DEV_REF
    f = 1.0 - (1.0 - LEG_MIN) * min(u, 1.0)
    r["leg_spread_now"], r["leg_dev"], r["leg_f"] = spread, dev, f
    r["leg_spread_max"] = max(r.get("leg_spread_max", 0.0), spread)
    r["leg_ext_max"] = max(r.get("leg_ext_max", 0.0), float(np.max(q[ks])))
    return f


def ramp(r, want):
    """비대칭 가감속 (설계 0.08 / 0.40 m/s²)."""
    cur = r.get("v_cmd", 0.0)
    lim = (ACC_MPS2 if want > cur else DEC_MPS2) * PHYSICS_DT
    cur += max(-lim, min(lim, want - cur))
    r["v_cmd"] = cur
    return cur


def drive(r, deg_s):
    """휠 각속도 지령 — **실측 각속도 되먹임으로 처짐을 보정한다.**

    🔑 속도 드라이브의 힘은 `댐핑 × (지령 − 실제)` 라 부하가 걸리면 실제가
       지령보다 낮은 자리에서 평형을 잡는다(열린 루프 실측 93%). 배율로 민다.
    🚨 maxForce 에 걸리면 배율을 올려도 소용없다 → 상한 1.5.
    🚨 정지·방향 전환에서 리셋한다(적분 폭주 방지).
    """
    a = r["art"]
    if GOV_ON and abs(deg_s) > 1e-6:
        want = math.radians(abs(deg_s))
        meas = float(np.mean(np.abs(
            np.asarray(a.get_joint_velocities())[r["wheel"]])))
        if r.get("gov_dir") != (deg_s > 0):
            r["gov"], r["gov_dir"] = 1.0, (deg_s > 0)
        r["gov"] = float(np.clip(
            r.get("gov", 1.0) + GOV_KI * ((want - meas) / want) * PHYSICS_DT,
            GOV_MIN, GOV_MAX))
        r["gov_meas"] = meas / want
        r.setdefault("gov_hist", []).append(r["gov_meas"])
        deg_s *= r["gov"]
    elif GOV_ON:
        r["gov"], r["gov_dir"] = 1.0, None
    # 🎯 **바퀴 차동 속도** (2026-08-08 사용자 지시 — *"앞대가리에만 영향을
    #    주니 관절마다, 꼬리쪽은 더 심하게 비틀린다"*). 지금까지 12개 바퀴에
    #    **똑같은 각속도**를 줬는데, 굽은 관에서는 바깥쪽 바퀴가 더 먼 거리를
    #    가야 한다 — 같은 속도를 주면 바깥은 끌리고 안쪽은 밀려 그 마찰이
    #    몸통을 비트는 토크가 된다(명령이 나오는 앞에서 먼 꼬리가 가장 심함).
    # 🔑 로봇 자기 신호만 쓴다(도면 무관): 중앙 관절 엔코더 → 총 굽힘각 θ 와
    #    굽힘 방위, 세그먼트 간격 L → 곡률 반경 **R = L/θ**. 바퀴가 시계각 φ
    #    (자산 제원)에 반경 r_w 로 달려 있으면 그 바퀴의 선회 반경은
    #    R − r_w·cos(φ − φ_bend) 이므로
    #        속도배율 = 1 − (r_w/R)·cos(φ − φ_bend)
    #    굽힘 안쪽(φ = φ_bend)은 느리고 바깥쪽은 빠르다.
    # 🚨 배율은 [0.5, 1.5] 로 자른다 — θ 가 크면 R 이 작아져 발산한다.
    _mult = None
    if DIFF_ON and r.get("wheel_clock") and len(r["wheel_clock"]) == len(r["wheel"]) \
            and r.get("bel_pitch") and r.get("seg_span", 0) > 1e-6:
        _q = np.asarray(r["art"].get_joint_positions())
        _tp = float(np.sum(_q[r["bel_pitch"]]))
        _ty = float(np.sum(_q[r["bel_yaw"]]))
        _th = math.hypot(_tp, _ty)
        if _th > math.radians(3.0):          # 3° 밑은 직관 — 차동 불필요
            # 🚨 **곡률 반경에 바닥을 둔다** (2026-08-08 실측 교훈). 진입
            #    기동 중에는 관절이 60° 넘게 꺾여 R = L/θ 가 74mm 까지 작아지고
            #    (실제 관은 R150) 배율이 ±0.57 로 벌어졌다. 설계 최소 곡관
            #    반경(SR R=100mm, v3 §4.2)을 바닥으로 쓴다 — 맵 정보가 아니라
            #    **배관 설계 규격**이라 자율 원칙에 어긋나지 않는다.
            _R = max(r["seg_span"] / _th, 0.10)
            # 굽힘 방위 — 조향 규약(tp ∝ −sinθ, ty ∝ +cosθ)의 역
            _bc = math.degrees(math.atan2(-_tp, _ty))
            _c = r.get("cond")
            _bore = (float(_c.bore_ref_mm) / 1000.0
                     if _c is not None and getattr(_c, "bore_ref_mm", 0) > 0
                     else PIPE_IR)
            _rw = max(_bore - WHEEL_R, 0.005)   # 휠 중심이 도는 반경
            # 🔑 **아무 바퀴도 기준 속도보다 느리게 주지 않는다** (핵심 수정).
            #    속도 드라이브는 지령보다 빨리 도는 바퀴를 **잡아채므로**,
            #    안쪽 바퀴에 0.5배를 주면 그것이 곧 브레이크가 된다 — 실측
            #    실패가 정확히 그 모습이었다(이탈 1.4mm 로 잘 정렬된 채
            #    **전진만 못 함**, floor2 s 969→355).
            #    → 비율은 유지하되 **최솟값이 1.0 이 되도록 통째로 올린다.**
            #      안쪽 바퀴는 그대로, 바깥 바퀴만 더 빨리 돈다. 전체가 조금
            #      빨라지는 것은 기존 실측 되먹임(gov)이 알아서 되돌린다.
            _raw = []
            for _k in r["wheel"]:
                _phi = r["wheel_clock"][_k]
                _raw.append(1.0 - DIFF_SIGN * DIFF_GAIN * (_rw / _R)
                            * math.cos(math.radians(_phi - _bc)))
            _lo = max(min(_raw), 0.3)
            _mult = [float(np.clip(x / _lo, 1.0, 1.8)) for x in _raw]
            r["diff_spread"] = max(_mult) - min(_mult)
            r["diff_max"] = max(r.get("diff_max", 0.0), r["diff_spread"])
    if _mult is None:
        r["diff_spread"] = 0.0
        v = np.array([math.radians(deg_s)] * len(r["wheel"]), dtype=np.float32)
    else:
        v = np.array([math.radians(deg_s) * m for m in _mult], dtype=np.float32)
    i = np.array(r["wheel"])
    try:
        a._articulation_view.set_joint_velocity_targets(v.reshape(1, -1),
                                                        joint_indices=i)
    except Exception:
        a.apply_action(ArticulationAction(joint_velocities=v,
                                          joint_indices=i))


print("=" * 78)
_m = sum(float(UsdPhysics.MassAPI(q).GetMassAttr().Get() or 0.0)
         for q in stage.Traverse()
         if str(q.GetPath()).startswith(robots[0]["path"] + "/")
         and q.HasAPI(UsdPhysics.RigidBodyAPI)
         and UsdPhysics.MassAPI(q).GetMassAttr())
print(f"로봇  1대 질량 {_m * 1000:.0f} g  중량 {_m * 9.81:.2f} N "
      f"(용접기 내장, 카메라 2대 부착)")
print(f"      주행 {TARGET_SPEED_MPS * 1000:.0f} mm/s  편도+복귀 1회  "
      f"휠 토크 {WHEEL_MAXFORCE:.3f} N·m")
print("-" * 78)

_t_mark, _step_mark = time.time(), 0
step, was_playing = 0, True
# 🎯 FAIL_S 10→5s (2026-08-07 사용자 지시: *"끼였으면 그 자체로 이미 문제.
#    기다리지 말고 짧게 끊어야 다음 테스트로 개선한다"*). 하한은 의도된
#    정지 구간(출발 HOLD ~3s, 진입 확인 1.5s)보다는 길어야 오탐이 없다.
STUCK_S = 2.0
REPORT_S = float(os.environ.get("REPORT_S", 3.0))   # 진단 때 1s 로 좁힌다
FAIL_S = float(os.environ.get("FAIL_S", 5.0))
# 🎯 **순진행 워치독** (2026-08-09 사용자 지시: *"끼이거나 문제가 생겨서 위치
#    변화가 없으면 타임아웃으로 끊어라 — 움직이지도 않는 것을 계속 붙잡고 있어
#    다양한 시도가 막히는 것이 더 큰 문제다"*).
# 🚨 위 `FAIL_S` 만으로는 **못 잡는 정체가 있다.** FAIL_S 는 "s 가 3mm 이상 안
#    변한 채 N 초" 를 보는데, RECOVER 가 물러났다 나아갔다를 되풀이하면 s 가
#    3mm 넘게 오가므로 **타이머가 매번 리셋된다** — 제자리 왕복만 하면서 스텝
#    예산을 끝까지 태운다(2026-08-08 floor1 T 실패가 정확히 이 모양이었다:
#    진입 실패 → RECOVER 반복 → 사망까지 전 구간을 붙잡고 있었다).
# 🚨 **"기준점에서 STALL_WIN 밖으로 나갔나" 로는 못 잡는다** (2026-08-09 실측으로
#    확인하고 고침). 첫 판이 그렇게 짰는데, floor1 T 앞에서 로봇이 s 560↔599 를
#    오갔다 — 폭 39mm 가 창(30mm)보다 넓어 **기준점이 매번 갱신되며** 타이머가
#    리셋됐다. 잡으려던 바로 그 왕복에 그대로 뚫린 것이다.
# 🔑 왕복과 전진을 가르는 것은 이동량이 아니라 **최고 도달점의 갱신**이다.
#    나갈 때는 s 의 최댓값, 복귀할 때는 최솟값이 그것이다 — 제자리 왕복은
#    아무리 크게 흔들려도 이 값을 못 넘는다. (`build_robot` 의 `best` 필드가
#    원래 이 자리를 위해 잡혀 있었으나 비어 있었다.)
# ⚠ 의도된 정지·회복 동작보다는 넉넉해야 오탐이 없다(정면 T 정지-탐색 1.5~3s,
#   RECOVER 한 사이클 수 초). 기본 40초는 그 10배 이상이다.
STALL_S = float(os.environ.get("STALL_S", 40.0))
STALL_WIN = float(os.environ.get("STALL_WIN", 0.030))
WELD_ARC_S = float(os.environ.get("WELD_ARC_S", 4.0))   # 아크 유지 시간
WELD_LEAD = float(os.environ.get("WELD_LEAD", 0.020))   # 넉넉히 미리 정지
WELD_FINE_M = float(os.environ.get("WELD_FINE_M", 0.0015))  # 미세 정렬 허용
_last_report = 0

tick("첫 물리 스텝 진입 — 여기서부터 로봇이 움직인다")
# 🚨 **NAV=vision 은 헤드리스여도 렌더해야 한다.** 안 그러면 카메라가
#    프레임을 못 만들어 `get_rgba()` 가 빈 배열이고, 판정이 없어 컨트롤러가
#    HOLD 로 멈춘다(실측: v=0 으로 제자리).
# 🎯 **단, 매 스텝 렌더하지 않는다** (2026-08-07 사용자 질문 "왜 3대에
#    0.10배인가"의 답) — 판정·카메라는 10Hz 만 쓰는데 물리 240Hz 마다
#    렌더하면 **필요한 것보다 24배** RTX 를 태운다. 그것이 GUI 0.10×,
#    헤드리스 vision 0.33× 의 정체였다. 렌더는 카메라 주기에만:
#    RENDER_EVERY 스텝마다 한 장. 아래 판정 게이트(PHYSICS_HZ/10)와 같은
#    주기라 렌더 직후 스텝에서 프레임을 읽는다 (신선도 1스텝).
#    GUI 뷰포트도 같은 렌더를 나눠 쓴다 — GUI_EVERY(기본 8 = 30fps)로
#    따로 조절. 뷰포트가 어느 쪽이든 렌더는 합집합으로 돈다.
RENDER_EVERY = max(1, int(round(PHYSICS_HZ / 10.0)))
# 🎯 카메라를 켠 GUI 는 렌더 프로덕트가 로봇당 3개(2기면 6개)라, 뷰포트
#    30fps(GUI_EVERY 8)로는 0.35x 까지 떨어진다(v1_3 both 실측). 발행이
#    10Hz 니 뷰포트도 10fps 로 맞춰 렌더를 1/3 로 줄인다 — 눈으로 보기에는
#    충분하고, 필요하면 GUI_EVERY=8 로 되돌린다.
GUI_EVERY = max(1, int(os.environ.get("GUI_EVERY", 24 if CAMERAS else 8)))
while True:
    _render = (NAV == "vision" and step % RENDER_EVERY == 0) \
        or (not HEADLESS and step % GUI_EVERY == 0) \
        or (RB is not None and CAMERAS and step % RENDER_EVERY == 0)
    world.step(render=_render)
    step += 1
    if step == 1:
        tick("첫 스텝 완료(GUI 는 여기서 RTX 셰이더를 굽는다)")

    if not HEADLESS:
        _playing = world.is_playing()
        if _playing and not was_playing:
            world.reset()
            for r in robots:
                r.update(state="SETTLE", t=0, dir=+1, lap=0, stuck=0,
                         s_last=0.0, mark=step, dead=False, wheel_rad=0.0)
            print("[재시작] Stop → Play 감지 — 처음부터 다시 돈다")
        was_playing = _playing
        if not simulation_app.is_running():
            break
        if not _playing:
            continue

    if step > STEPS and STEPS > 0 and not (HOLD and not HEADLESS):
        break

    # ── 🎥 **뷰포트 카메라 좌표 로그** (VIEW_LOG=1, 기본 켜짐) ──────────
    #    마우스로 원하는 각도를 잡아 두면 그 위치·시선이 3초마다 찍힌다.
    #    그 숫자를 그대로 고정 관찰 카메라로 박으면 같은 구도가 재현된다
    #    (2026-08-11 사용자 요청 — 사진으로 역산하면 오차가 크다).
    if (not HEADLESS and VIEW_LOG
            and step % int(3 * PHYSICS_HZ) == 0):
        try:
            from omni.kit.viewport.utility import get_active_viewport
            _vp = get_active_viewport()
            _cpz = stage.GetPrimAtPath(str(_vp.camera_path))
            if _cpz and _cpz.IsValid():
                _pz = wpos(_cpz)
                _fz = wrot(_cpz) @ np.array([0.0, 0.0, -1.0])
                _tg = {"단절끝": np.array([1.500, 0.600, 2.490]),
                       "곡관": np.array([1.160, 1.360, 2.490]),
                       "라이저": np.array([0.330, 0.850, 2.490])}
                _near = min(_tg.items(),
                            key=lambda kv: np.linalg.norm(kv[1] - _pz))
                print(f"  [뷰포트] 위치 ({_pz[0]:+.3f}, {_pz[1]:+.3f}, "
                      f"{_pz[2]:+.3f})  시선 ({_fz[0]:+.2f}, {_fz[1]:+.2f}, "
                      f"{_fz[2]:+.2f})  가장 가까운 지점 {_near[0]} "
                      f"{np.linalg.norm(_near[1] - _pz) * 1000:.0f}mm")
        except Exception as _ve:
            if not globals().get("_VIEWLOG_WARNED"):
                globals()["_VIEWLOG_WARNED"] = True
                print(f"[경고] 뷰포트 좌표 로그 불가: {_ve}")

    if step - _step_mark >= 5 * PHYSICS_HZ:
        _dt = time.time() - _t_mark
        _rate = (step - _step_mark) / max(_dt, 1e-9)
        print(f"  [속도] 물리 {_rate:6.1f} step/s "
              f"(실시간 대비 {_rate / PHYSICS_HZ:4.2f}x, 로봇 {len(robots)}대)")
        _t_mark, _step_mark = time.time(), step

    for r in robots:
        if r["dead"]:
            continue
        r["t"] += 1
        p1 = wpos(r["seg1"])
        # 🚨 닫힌 루프 — 직전 s 를 힌트로 창 안에서만 찾는다(전역 argmin 은
        #    T 근처에서 끝점에 붙어 "코스 끝" 오판을 낸다, 실측).
        s_now, off_now, i_now = r["cl"].nearest(p1, r.get("s_hint"))
        r["s_hint"] = s_now

        # 🔬 롤 기하 프로브 (ROLL_GEO=1, 2026-08-09) — s=342 곡관 정지의 유력
        #    후보("동결된 롤 방위가 곡관 평면과 안 맞는다")를 가르는 계측.
        #    기준은 **중심선(심판 전용)** — 곡관 평면의 정답은 맵이 안다.
        #      +굽힘 방향(드럼 실측) vs 앞쪽 접선의 수직 성분(필요 방향)
        #      어긋남 0°/±180° = 평면 일치(부호로 해결 가능) / ±90° = 수직(최악)
        if os.environ.get("ROLL_GEO") == "1"                 and r.get("drum_prim") is not None                 and r["t"] % int(PHYSICS_HZ) == 0:
            _Rd = wrot(r["drum_prim"])
            _xd = _Rd @ np.array([1.0, 0.0, 0.0])
            _bd = np.cross(_Rd @ np.array([0.0, 1.0, 0.0]), _xd)
            _s1p, _o1p, _ = r["cl"].nearest(wpos(r["seg1"]), r.get("s_hint"))
            _jp = int(np.argmin(np.abs(r["cl"].s - (_s1p + 0.10))))
            _tgp = r["cl"].tangent(_jp)
            _need = _tgp - _xd * float(np.dot(_tgp, _xd))
            _nn = float(np.linalg.norm(_need))
            if _nn > 1e-6:
                _need = _need / _nn
                _ang = math.degrees(math.atan2(
                    float(np.dot(np.cross(_bd, _need), _xd)),
                    float(np.dot(_bd, _need))))
                print(f"           [롤기하] s={_s1p * 1000:4.0f} "
                      f"+굽힘→({_bd[0]:+.2f},{_bd[1]:+.2f},{_bd[2]:+.2f}) "
                      f"필요→({_need[0]:+.2f},{_need[1]:+.2f},{_need[2]:+.2f}) "
                      f"어긋남 {_ang:+4.0f}°  굽힘지령 "
                      f"{math.degrees(r.get('cmd_bend', 0.0)):+.0f}°")
        # 🔬 카메라 부착 프로브 (CAM_PROBE=1, 2026-08-09 사용자 관찰 검증):
        #    카메라가 링크를 실제로 따라다니는지 — 월드 좌표 간격을 직접 잰다.
        if os.environ.get("CAM_PROBE") == "1" and r.get("cam_prim") is not None                 and r["t"] % int(PHYSICS_HZ) == 0:
            _cp = wpos(r["cam_prim"])
            _dp = wpos(r["seg1"])
            print(f"           [캠프로브] 카메라({_cp[0]*1000:+.0f},"
                  f"{_cp[1]*1000:+.0f},{_cp[2]*1000:+.0f}) 앞디스크"
                  f"({_dp[0]*1000:+.0f},{_dp[1]*1000:+.0f},{_dp[2]*1000:+.0f}) "
                  f"간격 {float(np.linalg.norm(_cp-_dp))*1000:.1f}mm")
        # (🎥 카메라 렌더 on/off 분기는 **철회했다** — 2026-08-11 사용자:
        #  "필요한 것들만 남겼으니 분기처리는 철회". 로봇당 카메라가 1~2대뿐
        #  이라 켜 두는 편이 단순하고, 주행 중 화면이 비는 문제도 없어진다.)

        # 🎯 **탈출 강제 상승** (위 EXIT_ASSIST_* 주석 — 사용자 지시).
        #    복귀 국면에서 라이저 구간(s < EXIT_STRAIGHT_S)에 있는데
        #    EXIT_ASSIST_S 초 동안 순진행이 없으면, 관 축 방향(위)으로
        #    EXIT_ASSIST_M 만큼 EXIT_ASSIST_T 초에 걸쳐 밀어 올린다.
        if (EXIT_ASSIST_M > 0 and r.get("phase") in ("BACK", "EXIT")
                and s_now < EXIT_STRAIGHT_S and r.get("art") is not None):
            _as = r.setdefault("assist", {"s": s_now, "mark": step, "n": 0,
                                          "left": 0.0})
            if s_now < _as["s"] - 0.002:            # 순진행 중 — 감시 리셋
                _as["s"], _as["mark"] = s_now, step
            # 🎯 **관구부터는 연속 상승** (2026-08-11 실측): 하우징 안은
            #    짚을 벽이 없어(뜬다리 9개) 30mm 씩 끊어 올리면 그때마다
            #    중력이 되돌린다 — 12회 반복해도 제자리(관 밖 33mm).
            #    관구(s≈0)에 닿으면 완전 탈출까지 **쉬지 않고** 밀어 올린다.
            if s_now <= EXIT_ASSIST_MOUTH_S:
                _pw, _qw = r["art"].get_world_pose()
                _up = -r["cl"].tangent(0)
                r["art"].set_world_pose(
                    position=np.asarray(_pw)
                    + _up * (EXIT_ASSIST_V / PHYSICS_HZ), orientation=_qw)
                _as["mark"] = step
                if not _as.get("cont"):
                    _as["cont"] = True
                    print(f"[{r['name']:8s}] ⬆ 관구 도달 — 완전 탈출까지 "
                          f"연속 상승 {EXIT_ASSIST_V * 1000:.0f}mm/s")
            elif _as["left"] > 0.0:                 # 상승 중
                _d = min(_as["left"],
                         EXIT_ASSIST_M / max(EXIT_ASSIST_T * PHYSICS_HZ, 1))
                _pw, _qw = r["art"].get_world_pose()
                _up = -r["cl"].tangent(0)           # 관구 바깥(위) 방향
                r["art"].set_world_pose(
                    position=np.asarray(_pw) + _up * _d, orientation=_qw)
                _as["left"] -= _d
                if _as["left"] <= 1e-9:
                    _as["s"], _as["mark"] = s_now, step
                    print(f"[{r['name']:8s}] ⬆ 강제 상승 "
                          f"{EXIT_ASSIST_M * 1000:.0f}mm 완료 "
                          f"({_as['n']}회째, s={s_now * 1000:.0f}mm)")
            elif step - _as["mark"] > EXIT_ASSIST_S * PHYSICS_HZ:
                _as["left"] = EXIT_ASSIST_M
                _as["n"] += 1
                print(f"[{r['name']:8s}] ⬆ 탈출 정체 "
                      f"{EXIT_ASSIST_S:.1f}s — 강제 상승 "
                      f"{EXIT_ASSIST_M * 1000:.0f}mm 시작 "
                      f"(s={s_now * 1000:.0f}mm, {_as['n']}회째)")

        # 🎥 **용접 카메라는 항상 팁을 본다** (2026-08-11 사용자 지시).
        #    주행 중에도 링 자세를 따라 돌므로, 링이 결함 쪽으로 돌면 화면도
        #    같이 돈다. 4스텝(60Hz)마다면 화면상 충분하다.
        if _DETW["on"] and r["t"] % max(1, int(PHYSICS_HZ / 10)) == 0:
            detect_window(r)
        if r.get("weld_cam") is not None and r["t"] % 4 == 0:
            aim_weld_cam(r, diag=(os.environ.get("WELD_CAM_DIAG") == "1"
                                  and r["t"] % int(2 * PHYSICS_HZ) == 0))

        if r["state"] == "SETTLE":
            # 🚨 안착 중에도 예압을 **매 스텝 다시 써야** 한다 — reset 뒤의
            #    `set_joint_positions()` 가 드라이브 타깃을 지웠기 때문이다.
            if LEG_FORCE:
                _li, _lv = force_legs(r)
                if _li:
                    _set_eff(r, _lv, _li)
            else:
                _li, _lv = preload_legs(r)
                if _li:
                    _set_pos(r, _lv, _li)
            # [진단] 안착 궤적 — 수직관에서 흘러내리는지 가른다.
            #   s 가 **가속**하면 접촉 없이 자유낙하, **등속**이면 굴러 내려가는
            #   것(휠 드라이브가 못 잡는 것), 안 변하면 정상.
            if SETTLE_TRACE and r["t"] % int(0.25 * PHYSICS_HZ) == 0:
                _q = np.asarray(r["art"].get_joint_positions())
                _v = np.asarray(r["art"].get_joint_velocities())
                print(f"           [안착 {r['t'] / PHYSICS_HZ:4.2f}s] "
                      f"{r['name']} s={s_now * 1000:6.1f} "
                      f"이탈{off_now * 1000:5.1f}  z={p1[2] * 1000:+7.1f}  "
                      f"다리평균 {np.mean(_q[list(r['piston'].values())]) * 1000:5.2f}mm  "
                      f"휠각속도 {np.degrees(np.mean(np.abs(_v[r['wheel']]))):7.1f}°/s")
            if r["t"] > 1.5 * PHYSICS_HZ:
                # [진단] 조향 프레임이 맞는지 — 안착 직후 로봇은 관과 나란하니
                # 헤딩과 접선이 거의 같아야 하고 로컬 목표는 (+1,0,0) 이어야 한다.
                _hd = wrot(r["seg0"]) @ np.asarray(r["fw"])
                _tg = r["cl"].tangent(i_now)
                _el = r["fr"] @ (wrot(r["seg0"]).T @ _tg)
                print(f"[{r['name']:8s}] 안착 s={s_now * 1000:.1f}mm "
                      f"이탈 {off_now * 1000:.1f}mm — 주행 시작")
                # 🎯 **탈출 전용 신속 시험** (2026-08-10): 안착 즉시 복귀
                #    국면으로 — 왕복 15분을 기다리지 않고 라이저 탈출만
                #    3분 내로 반복한다. `EXIT_TEST=1` 로 켠다.
                if os.environ.get("EXIT_TEST", "0") == "1":
                    r["phase"] = "BACK"
                    r["dir"] = -1
                    print(f"[{r['name']:8s}] 🧪 EXIT_TEST — 안착 즉시 복귀 "
                          f"국면(라이저 탈출만 시험)")
                print(f"           [진단] 헤딩({_hd[0]:+.2f},{_hd[1]:+.2f},"
                      f"{_hd[2]:+.2f}) 접선({_tg[0]:+.2f},{_tg[1]:+.2f},"
                      f"{_tg[2]:+.2f}) 로컬목표({_el[0]:+.2f},{_el[1]:+.2f},"
                      f"{_el[2]:+.2f}) — 로컬목표가 (+1,0,0) 이어야 한다")
                # 안착이 이렇게 어긋났으면 주행 결과를 보기 전에 원인을 봐야
                # 한다(다리가 벽에 안 닿은 채 시작한 것인지 가른다).
                if off_now > 0.010:
                    print(f"           [경고] 안착 이탈 {off_now * 1000:.1f}mm "
                          f"— 출발부터 어긋났다. 아래 상태를 먼저 볼 것")
                    diag_stuck(r)
                r.update(state="RUN", t=0, s_last=s_now, mark=step)
            continue

        # ── 🔧 용접 FSM (v1_3) — OUT 전진 중 결함 도달 → 정지·정렬·용접·재개
        #    정렬은 링 서보(방사방향을 결함 방향에), 신장은 반경 실측 종료,
        #    성공 = 프림 가시성 전환(설계 확정 방식). 주행·조향은 그동안 스킵.
        if r["weld_sites"]:
            _w = r.get("weld")
            if _w is None and r.get("phase", "OUT") == "OUT" and r["dir"] > 0:
                # 🎯 정지 기준 = **용접링 자신의 진행거리** (2026-08-11 사용자
                #    지적: "용접기 위치와 결함 위치가 다르다"). s_now 는 앞
                #    디스크(seg1) 기준인데 용접링은 몸통에 있어 그만큼 **뒤**
                #    다 — 그 자리에 세우면 토치가 결함 뒤를 지진다. 로봇이
                #    바뀌면 이 간격도 바뀌므로 상수로 박지 않고 매번 잰다.
                _sr9 = s_now
                if r.get("ring_link") is not None:
                    _sr9 = r["cl"].nearest(wpos(r["ring_link"]), s_now)[0]
                # 🎯 감속 오버슈트(실측 +8~9mm)만큼 **미리** 세운다 —
                #    그래야 토치 끝이 결함 바로 위에 선다(2026-08-11 사용자:
                #    "용접봉 끝 위치가 결함과 정확히 안 맞는다").
                for _site in r["weld_sites"]:
                    if not _site["done"] and _sr9 >= _site["s"] - WELD_LEAD:
                        r["weld"] = _w = dict(site=_site, st="CREEP",
                                              t0=r["t"], ok_t=0,
                                              sign=1.0, err0=None)
                        print(f"[{r['name']:8s}] 🔧 결함 도달 s="
                              f"{_site['s'] * 1000:.0f}mm 시계 "
                              f"{_site['clock']:.0f}° — 정렬 시작 "
                              f"(용접링 s={_sr9 * 1000:.0f}mm · 축오차 "
                              f"{(_sr9 - _site['s']) * 1000:+.1f}mm, "
                              f"머리 s={s_now * 1000:.0f}mm)")
                        if RB is not None:
                            RB.robot(r["name"]).emit(
                                "DEFECT", "결함 앞 정지",
                                s_mm=_site["s"] * 1000,
                                clock_deg=_site["clock"])
                        break
            if _w is not None:
                _site = _w["site"]
                _creep = (_w["st"] == "CREEP")
                # (조준은 아래 주행 루프에서 **매 틱 항상** 한다)
                if not _creep:
                    drive(r, 0.0)
                if LEG_FORCE:
                    _fi, _fv = force_legs(r)
                    if _fi:
                        _set_eff(r, _fv, _fi)
                r["mark"], r["s_last"] = step, s_now   # 끼임 워치독 무장해제
                _qj = np.asarray(r["art"].get_joint_positions())
                _qr = float(_qj[r["ring_dof"][0]])
                _qt = float(_qj[r["torch_dof"][0]])
                if _w["st"] == "CREEP":
                    # 🎯 **미세 정렬** (2026-08-11): 감속 오버슈트가 4~18mm 로
                    #    들쭉날쭉해 고정 선행보정으로는 못 맞춘다(실측).
                    #    멈춘 뒤 저속으로 기어 **용접링 s = 결함 s** 를 맞춘다.
                    #    이게 맞아야 팁이 결함 정중앙에 서고, 결함(r7)이
                    #    팁(r2) 둘레로 빨갛게 드러난다(카메라에 담기는 조건).
                    # 🚨 중심선 s 는 표본 간격(13mm)으로 **양자화**돼 있어
                    #    오차가 −17.7 → +8.4 → +21.5 로 뛴다(실측) — 1.5mm
                    #    수렴이 원리적으로 불가능했다. 축오차는 **결함 지점의
                    #    접선에 투영**해 연속값으로 잰다.
                    _i8 = int(np.argmin(np.abs(r["cl"].s - _site["s"])))
                    _tg8 = r["cl"].tangent(_i8)
                    _er8 = float(np.dot(wpos(r["ring_link"]) - _site["pos"],
                                        _tg8))
                    _sr8 = _site["s"] + _er8
                    if abs(_er8) < WELD_FINE_M:
                        _w["st"], _w["t0"] = "ALIGN", r["t"]
                        print(f"[{r['name']:8s}] 🔧 미세 정렬 완료 — 축오차 "
                              f"{_er8 * 1000:+.1f}mm")
                    elif r["t"] - _w["t0"] > 6.0 * PHYSICS_HZ:
                        _w["st"], _w["t0"] = "ALIGN", r["t"]
                        print(f"[{r['name']:8s}] ⚠ 미세 정렬 시한 — 축오차 "
                              f"{_er8 * 1000:+.1f}mm 로 진행")
                    else:
                        if (r["t"] - _w["t0"]) % int(0.5 * PHYSICS_HZ) == 0:
                            print(f"           [미세] 축오차 "
                                  f"{_er8 * 1000:+6.1f}mm  링s "
                                  f"{_sr8 * 1000:.1f}  목표 "
                                  f"{_site['s'] * 1000:.1f}")
                        # 🚨 바퀴 저속 기기는 **정지 마찰에 막힌다**(실측:
                        #    8초에 2mm, 축오차 −17.7mm 로 시한 초과).
                        #    탈출 강제 상승과 같은 방식으로 **관 축을 따라
                        #    직접** 옮겨 오차를 없앤다(blueprint 규약이다).
                        _st8 = min(abs(_er8), 0.030 / PHYSICS_HZ)
                        _pw8, _qw8 = r["art"].get_world_pose()
                        r["art"].set_world_pose(
                            position=np.asarray(_pw8)
                            + _tg8 * math.copysign(_st8, -_er8),
                            orientation=_qw8)
                elif _w["st"] == "ALIGN":
                    # 링 서보 — 토치 방사방향(링 로컬 +Z)을 결함 방향에.
                    # 부호는 자가교정(1초 뒤 오차가 커져 있으면 반전) —
                    # 링 축 규약을 자산에 묻지 않는다.
                    _Rr = wrot(r["ring_link"])
                    _e = _Rr @ np.array([0.0, 0.0, 1.0])
                    _xb = _Rr @ np.array([1.0, 0.0, 0.0])
                    _d = _site["dir"]
                    _ang = math.atan2(float(np.dot(np.cross(_e, _d), _xb)),
                                      float(np.dot(_e, _d)))
                    if _w["err0"] is None:
                        _w["err0"] = abs(_ang)
                    if r["t"] - _w["t0"] == int(1.0 * PHYSICS_HZ) \
                            and abs(_ang) > _w["err0"] + 0.05:
                        _w["sign"] = -_w["sign"]
                        _w["err0"] = abs(_ang)
                    if abs(_ang) < math.radians(1.2):
                        _w["ok_t"] += 1
                        if _w["ok_t"] >= int(0.5 * PHYSICS_HZ):
                            _w["st"], _w["t0"] = "EXTEND", r["t"]
                            # 🎯 **결함을 토치에 맞춘다** (2026-08-11 사용자
                            #    지시 — 훨씬 간단하고 확실하다). 로봇이 관
                            #    중심에서 ~12mm 치우쳐 서기 때문에 토치를
                            #    결함에 맞추려면 이탈 보정·행정 확장이 줄줄이
                            #    필요했다. 결함은 우리가 놓는 프림이므로
                            #    **토치가 실제로 닿을 자리로 옮긴다**.
                            _snap = wpos(r["torch_link"]) + _e * float(
                                TUNE.get("torch_tip_r0", 0.041))
                            _rl = _snap - _site["pos"]
                            _axl = float(np.dot(_rl, _site["tan"]))
                            _rv = _rl - _site["tan"] * _axl
                            _nv = float(np.linalg.norm(_rv))
                            if _nv > 1e-6:
                                _u = _rv / _nv
                                _rot = Gf.Rotation(
                                    Gf.Vec3d(0, 0, 1),
                                    Gf.Vec3d(*[float(v) for v in _u]))
                                for _pk, _off in (("prim_d", 0.0006),
                                                  ("prim_b", 0.0012)):
                                    _c = (_site["pos"] + _site["tan"] * _axl
                                          + _u * (PIPE_IR - _off))
                                    _xf = UsdGeom.Xformable(_site[_pk])
                                    _xf.ClearXformOpOrder()
                                    _xf.AddTransformOp().Set(
                                        Gf.Matrix4d().SetRotate(_rot)
                                        * trans(float(_c[0]), float(_c[1]),
                                                float(_c[2])))
                                _site["dir"] = _u          # 이후 계산도 일치
                                print(f"[{r['name']:8s}] 🎯 결함을 토치 자리로 "
                                      f"스냅 (축 {_axl * 1000:+.1f}mm)")
                            print(f"[{r['name']:8s}] 🔧 정렬 완료 (오차 "
                                  f"{math.degrees(_ang):+.1f}°) — 토치 신장")
                    else:
                        _w["ok_t"] = 0
                    _set_pos(r,
                             [_qr + _w["sign"] * max(-0.6, min(0.6, _ang))],
                             [r["ring_dof"][0]])
                elif _w["st"] == "EXTEND":
                    # 🎯 종료 판정 = **팁의 실제 반경** (2026-08-11 관통 사고
                    #    수정). 토치 팁은 링크 원점보다 tip_r0(41mm) 앞에
                    #    있고, 링크가 신장만큼 더 나간다. 관벽 50mm 에서
                    #    용접 간극 2mm 를 남긴 48mm 에서 멈춘다.
                    _tp = wpos(r["torch_link"])
                    _tipw = _tp + _site["dir"] * float(
                        TUNE.get("torch_tip_r0", 0.041))
                    _rel = _tipw - _site["pos"]
                    _rad = float(np.linalg.norm(
                        _rel - _site["tan"]
                        * float(np.dot(_rel, _site["tan"]))))
                    _goal = PIPE_IR - float(os.environ.get("WELD_GAP_M",
                                                           0.002))
                    if "qcmd" not in _w:
                        _w["qcmd"] = _qt
                    if (r["t"] - _w["t0"]) % int(0.5 * PHYSICS_HZ) == 0:
                        print(f"           [용접] {r['name']} 신장 "
                              f"{_qt * 1000:5.1f}mm  팁 반경 "
                              f"{_rad * 1000:5.1f}mm / 목표 "
                              f"{_goal * 1000:.0f}mm (관벽 "
                              f"{PIPE_IR * 1000:.0f})")
                    if _rad >= _goal or _qt >= TUNE.get("torch_stroke",
                                                        0.010) - 0.0005:
                        _w["st"], _w["t0"] = "ARC", r["t"]
                        print(f"[{r['name']:8s}] 🔧 토치 도달 — 팁 반경 "
                              f"{_rad * 1000:.1f}mm (신장 "
                              f"{_qt * 1000:.1f}mm, 관벽까지 "
                              f"{(PIPE_IR - _rad) * 1000:.1f}mm) — "
                              f"아크 점화 {WELD_ARC_S:.0f}s")
                        if RB is not None:
                            RB.robot(r["name"]).emit(
                                "WELD_BEGIN", "아크 점화",
                                s_mm=_site["s"] * 1000,
                                clock_deg=_site["clock"])
                    else:
                        _w["qcmd"] = min(TUNE.get("torch_stroke", 0.010),
                                         _w["qcmd"] + 0.006 / PHYSICS_HZ)
                        _set_pos(r, [_w["qcmd"]], [r["torch_dof"][0]])
                elif _w["st"] == "ARC":
                    # 📸 용접 순간 프레임 저장 (WELD_CAM_SHOT=1) — 화면을
                    #    말로 추측하지 않고 직접 본다.
                    if (os.environ.get("WELD_CAM_SHOT") == "1"
                            and not _w.get("shot")
                            and r["t"] - _w["t0"] > int(0.7 * PHYSICS_HZ)):
                        _w["shot"] = True
                        aim_weld_cam(r, diag=True)   # 저장 순간 수치도 남긴다
                        try:
                            _c9 = r.get("cams", {}).get("torch")
                            _im = _c9.get_rgba() if _c9 else None
                            if _im is not None and getattr(_im, "size", 0):
                                import imageio.v2 as _iio
                                _fn = (f"/tmp/claude-1000/-home-rokey/"
                                       f"3df81582-84b7-4a5d-a0a5-"
                                       f"8700bab76459/scratchpad/"
                                       f"weldcam_{r['name']}_"
                                       f"{_site['clock']:.0f}.png")
                                _iio.imwrite(_fn, np.asarray(
                                    _im)[:, :, :3].astype(np.uint8))
                                print(f"           [용접캠] 📸 저장 {_fn}")
                        except Exception as _pe:
                            print(f"           [용접캠] 저장 실패 {_pe}")
                    if r.get("sparks") is not None:
                        _tipw = wpos(r["torch_link"]) + _site["dir"] * float(
                            TUNE.get("torch_tip_r0", 0.041))
                        r["sparks"].step(
                            PHYSICS_DT, emitting=True, origin=_tipw,
                            normal=-_site["dir"],
                            light=r.get("arc_light"), light_base=3.0e5,
                            confine=(_site["pos"], _site["tan"], PIPE_IR))
                    elif r.get("arc_light") is not None:
                        _fl = 1.0 + 0.6 * math.sin(r["t"] * 0.9)
                        r["arc_light"].CreateIntensityAttr(2.0e6 * _fl)
                    if r["t"] - _w["t0"] >= WELD_ARC_S * PHYSICS_HZ:
                        UsdGeom.Imageable(_site["prim_d"]).MakeInvisible()
                        UsdGeom.Imageable(_site["prim_b"]).MakeVisible()
                        if r.get("arc_light") is not None:
                            r["arc_light"].CreateIntensityAttr(0.0)
                        _w["st"], _w["t0"] = "RETRACT", r["t"]
                        print(f"[{r['name']:8s}] 🔧 용접 완료 — 결함 숨김·"
                              f"비드 표시, 토치 수납")
                        if RB is not None:
                            RB.robot(r["name"]).emit(
                                "WELD_DONE", "비드 형성",
                                s_mm=_site["s"] * 1000,
                                clock_deg=_site["clock"])
                elif _w["st"] == "RETRACT":
                    _set_pos(r, [max(0.0, _qt - 0.020 / PHYSICS_HZ)],
                             [r["torch_dof"][0]])
                    if _qt <= 0.002:
                        _set_pos(r, [0.0], [r["ring_dof"][0]])
                        _site["done"] = True
                        if r.get("sparks") is not None:
                            r["sparks"].clear()
                        r["weld"] = None
                        r["s_last"], r["mark"] = s_now, step
                        print(f"[{r['name']:8s}] 🔧 용접 절차 종료 — "
                              f"주행 재개")
                continue

        # ── 🌐 웹 정지 지령 (mission stop) — 재개 지령까지 그 자리 유지 ──
        if r.get("hold_cmd"):
            drive(r, 0.0)
            if LEG_FORCE:
                _fi, _fv = force_legs(r)
                if _fi:
                    _set_eff(r, _fv, _fi)
            r["mark"], r["s_last"] = step, s_now
            continue

        # ── RUN : 끝까지 갔다가 되돌아오고, 다시 간다 ──────────────
        # 🚨 다리(중심 유지)와 중앙 관절(조향)의 위치 목표를 **한 번에** 쓴다.
        #    나눠 부르면 뒤에 쓴 것만 남는다(기록된 함정).
        # 🚨 `CENTER=0` 이어도 **예압은 매 스텝 써야** 한다. 안 쓰면 reset 뒤의
        #    `set_joint_positions()` 가 지운 드라이브 타깃이 그대로 0 이라
        #    다리가 벽을 안 민다(수평에서는 중력이 가려 준다).
        # ── NAV=vision : 판단을 정본 모듈에 맡긴다 ─────────────────
        if NAV == "vision" and r.get("ctl") is not None:
            # 🚨 **부호 있는** 휠 속도를 먹인다. |ω| 로 주면 복귀 중에도
            #    distance_m 이 계속 늘어 RETURN 이 영영 안 끝난다(실측 —
            #    복귀 재트리거 끼임 반복의 절반이 이것).
            _wmps = float(np.mean(np.asarray(
                r["art"].get_joint_velocities())[r["wheel"]])) * WHEEL_R
            # 카메라·판정은 10Hz (설계값). 매 스텝 굽지 않는다.
            if step - r.get("vis_step", -999) >= PHYSICS_HZ / 10.0:
                r["vis_step"] = step
                _cam = r.get("cam_front")
                if _cam is not None:
                    try:
                        _fr = _cam.get_current_frame()
                        _dep = _fr.get("distance_to_camera")
                        _rgba = _cam.get_rgba()
                        # 첫 프레임 전에는 빈 1차원 배열이 온다 — 건너뛴다
                        _rgb = (_rgba[:, :, :3]
                                if _rgba is not None
                                and getattr(_rgba, "ndim", 0) == 3 else None)
                        if _dep is not None and _rgb is not None \
                                and getattr(_dep, "size", 0):
                            _ja = math.degrees(float(np.max(np.abs(
                                np.asarray(r["art"].get_joint_positions())
                                [r["bel"]]))))
                            r["cond"] = r["det"].run(np.asarray(_dep),
                                                     joint_angle_deg=_ja,
                                                     rgb=np.asarray(_rgb))
                            # 판별자 설계용 depth 덤프 (DUMP_DEPTH=디렉터리,
                            # DUMP_MIN=분기비율 문턱 — 0 이면 전 프레임)
                            if os.environ.get("DUMP_DEPTH") and \
                                    r["cond"].branch_ratio >= float(
                                        os.environ.get("DUMP_MIN", 0.04)) and \
                                    r.get("dump_n", 0) < 40:
                                r["dump_n"] = r.get("dump_n", 0) + 1
                                np.save(f"{os.environ['DUMP_DEPTH']}/"
                                        f"{r['name']}_s{int(s_now * 1000):04d}"
                                        f"_{step:06d}.npy", np.asarray(_dep))
                            _fl = r["vo"].step(np.asarray(_rgb),
                                               np.asarray(_dep), 0.1)
                            r["vis_mps"] = (float(_fl.speed_mps)
                                            if _fl.valid else None)
                            r["vis_valid"] = bool(_fl.valid)
                    except Exception as exc:
                        if not r.get("vis_warned"):
                            r["vis_warned"] = True
                            print(f"[경고] {r['name']} 영상 판단 실패({exc}) "
                                  f"— 이 로봇은 지령 속도로만 돈다")
            # 🚨 `DriveController` 는 **dict** 를 받는다(ROS 메시지 경유 규약).
            #    `Condition` 객체를 그대로 넘기면 `cond.get` 에서 죽는다.
            _cd = r.get("cond")
            # 재출발 유예 중에는 start 를 주지 않는다 — 컨트롤러가 IDLE(v=0)
            # 로 서 있는 동안 steer_vision 이 관절을 편다.
            _hold_start = r.get("straighten_n", 0) > 0
            if _hold_start:
                r["straighten_n"] -= 1
            # 🎯 **중력 기준 절대 롤로 "오른쪽"을 잡는다** (2026-08-08, 실전
            #    맵에서 드러난 결함). 오른손 법칙의 오른쪽은 **월드** 기준
            #    (진행방향 × 중력반대)인데, 컨트롤러는 개구 방위를 **몸**
            #    프레임 clock 으로 판정한다. 연습장은 전 코스가 수평 진입이라
            #    둘이 같았지만, 실전 맵은 **수직 라이저 → 곡관 → 수평**으로
            #    들어오면서 로봇이 축을 중심으로 굴러(롤) 몸 기준 오른쪽이
            #    월드 왼쪽이 된다. 실측: T 에서 개구가 몸 clock 355°(왼쪽으로
            #    판정)로 보여 진입 거부 → 정체 사망.
            # 🔑 설계 문서가 이미 답을 준다 — *"롤: IMU 중력 기준 **절대 롤**"*.
            #    도면이 아니라 **센서**로 푸는 것이라 자율 원칙에 어긋나지
            #    않는다. 매 스텝 롤을 재서 pixel→body 오프셋에 더한다.
            # 🚨 **부호를 이론으로 따지지 않는다** (몸 clock 논쟁이 세 번
            #    반복된 교훈). 카메라 프림의 **실제 월드 자세**를 읽어
            #    "월드 오른쪽" 이 화면 몇 도에 찍히는지 직접 계산한다.
            #    검출기의 화소 방위 규약: φ = atan2(dy, dx), dx = 화면 오른쪽,
            #    dy = 화면 **아래**. USD 카메라는 로컬 −Z 를 보고 +X 가 화면
            #    오른쪽, +Y 가 화면 위다 → 화면아래 = −Y_cam.
            _camp = r.get("cam_prim")
            if _camp is not None:
                _Rc = wrot(_camp)
                _img_r = _Rc @ np.array([1.0, 0.0, 0.0])    # 화면 오른쪽
                _img_d = _Rc @ np.array([0.0, -1.0, 0.0])   # 화면 아래
                _fwd_w = _Rc @ np.array([0.0, 0.0, -1.0])   # 광축(전방)
                _up_w = np.array([0.0, 0.0, 1.0])
                # 임무 규칙의 오른쪽 = 진행방향 × 중력반대 (월드·IMU 기준)
                _right_w = np.cross(_fwd_w, _up_w)
                _n = float(np.linalg.norm(_right_w))
                if _n > 1e-6:
                    _right_w /= _n
                    _phi_r = math.degrees(math.atan2(
                        float(np.dot(_right_w, _img_d)),
                        float(np.dot(_right_w, _img_r))))
                    # 컨트롤러 규약: body = 화소 + offset(180), right 는
                    # branch_right_deg 와 비교 → right 를 화소에서 직접 준다.
                    r["phi_right"] = _phi_r
                    r["ctl"].k["branch_right_deg"] = (_phi_r + 180.0) % 360.0
                    # 절대 롤(표시용) — 화면 위가 월드 위에서 얼마나 돌았나
                    _img_u = -_img_d
                    _upp = _up_w - _fwd_w * float(np.dot(_up_w, _fwd_w))
                    if float(np.linalg.norm(_upp)) > 1e-6:
                        _upp /= float(np.linalg.norm(_upp))
                        r["roll_abs"] = math.degrees(math.atan2(
                            float(np.dot(_upp, _img_r)),
                            float(np.dot(_upp, _img_u))))
            _ds = r["ctl"].step(PHYSICS_DT, recall=r.get("recall", False),
                                suppress_branch=r.get("leg_dev", 0.0) > 0.0018,
                                cond=None if _cd is None else _cd.as_dict(),
                                wheel_mps=_wmps, visual_mps=r.get("vis_mps"),
                                start=not r["started"] and not _hold_start)
            if not _hold_start:
                r["started"] = True
            # 🎯 [턴검증] 몸 clock 논쟁을 끝내는 정본 판정 (2026-08-07) —
            #    진입 기동의 실제 턴 방향을 **월드 기준**(오른쪽 = 진행방향 ×
            #    상방 +Z)으로 잰다. 몸 프레임 clock 만으로는 거울상 코스를 못
            #    가른다는 것이 세 번 반복된 교훈이라, 화면(사용자)과 같은
            #    기준으로 로그에 박는다.
            _prev_st = r.get("ctl_state")
            if _ds.state == "BRANCH_ENTRY" and _prev_st != "BRANCH_ENTRY":
                r["turn_h0"] = wrot(r["seg0"]) @ np.asarray(r["fw"])
            elif _prev_st == "BRANCH_ENTRY" and _ds.state != "BRANCH_ENTRY" \
                    and r.get("turn_h0") is not None:
                _h1 = wrot(r["seg0"]) @ np.asarray(r["fw"])
                _rgt = np.cross(r["turn_h0"], np.array([0.0, 0.0, 1.0]))
                _dp = float(np.dot(_h1, _rgt))
                print(f"[{r['name']:8s}] [턴검증] 진입 기동 종료 — 월드 기준 "
                      f"{'✅ 오른손 턴' if _dp > 0.2 else '🚨 왼손 턴' if _dp < -0.2 else '? 직진(꺾임 미미)'}"
                      f" (h0→h1·right = {_dp:+.2f}, 절대롤 "
                      f"{r.get('roll_abs', 0.0):+.0f}°, 월드오른쪽=화소 "
                      f"{r.get('phi_right', 0.0):+.0f}°)")
                r["turn_h0"] = None
            r["ctl_state"], r["slip"] = _ds.state, _ds.slip_ratio
            r["gates"] = dict(_ds.gates)      # 속도 관문 계측 (상태줄에 표시)
            # 🚨 부호를 살린다 — RETURN 은 음수 지령으로 뒤로 나온다.
            #    연습장 dir 는 vision 에서 항상 +1 로 두고 부호는 여기서 진다.
            r["v_cmd"] = float(_ds.v_cmd)
            r["dir"] = +1
            r["ctl_steer"] = (float(_ds.steer_deg),
                              float(_ds.steer_clock_deg))

        if LEG_FORCE:
            _fi, _fv = force_legs(r)
            if _fi:
                _set_eff(r, _fv, _fi)
            _pi, _pv = [], []
        else:
            _pi, _pv = center_legs(r) if CENTER_ON else preload_legs(r)
        # 🎯 **되감기 테이프** (2026-08-10 밤 — 사용자 설계: 나가는 턴의
        #    관절 궤적을 위치 기준으로 기록해 두고, 복귀는 오른팔 스텁에서
        #    후진하며 같은 위치에 같은 관절값을 재생한다. 준정적 기동이라
        #    "위치→모양" 은 방향 무관 — 미러 계산 불필요, 시간만 역재생.)
        _tape = os.environ.get("BP_TAPE")
        if _tape and r["name"] == "floor1":
            _sh12 = float(r.get("s_hint") or 0.0)
            if os.environ.get("BP_TAPE_REC") == "1" and r["dir"] > 0 \
                    and r.get("phase", "OUT") == "OUT" \
                    and 0.40 <= _sh12 <= 1.00:
                if "tape_fh" not in r:
                    r["tape_fh"] = open(_tape, "w")
                r["tape_fh"].write(
                    f"{_sh12:.4f} {r.get('cmd_roll', 0.0):.4f} "
                    f"{r.get('cmd_bend', 0.0):.4f} "
                    f"{r.get('cmd_roll_r', 0.0):.4f} "
                    f"{r.get('cmd_bend_r', 0.0):.4f}\n")
            elif os.environ.get("BP_TAPE_REC") != "1" \
                    and r.get("phase") == "BACK" and 0.40 <= _sh12 <= 1.00:
                if "tape_arr" not in r:
                    try:
                        _rows = np.loadtxt(_tape)
                        r["tape_arr"] = _rows[np.argsort(_rows[:, 0])]
                        print(f"[{r['name']:8s}] 🎞 되감기 테이프 로드 — "
                              f"{len(r['tape_arr'])}행")
                    except Exception as _e12:
                        r["tape_arr"] = None
                        print(f"[경고] 테이프 로드 실패: {_e12}")
                if r.get("tape_arr") is not None:
                    _ta = r["tape_arr"]
                    _i12 = int(np.searchsorted(_ta[:, 0], _sh12))
                    _i12 = max(0, min(len(_ta) - 1, _i12))
                    _row = _ta[_i12]
                    r["cmd_roll"], r["cmd_bend"] = float(_row[1]), float(_row[2])
                    r["cmd_roll_r"], r["cmd_bend_r"] = (float(_row[3]),
                                                        float(_row[4]))
                    _rb12 = [k for k in r["bend_dof"]
                             if k not in r["bend_front"]]
                    _rr12 = [k for k in r["roll_dof"]
                             if k not in r["roll_front"]]
                    _si = (r["roll_front"] + r["bend_front"] + _rr12 + _rb12)
                    _sv = ([float(_row[1])] * len(r["roll_front"])
                           + [float(_row[2])] * len(r["bend_front"])
                           + [float(_row[3])] * len(_rr12)
                           + [float(_row[4])] * len(_rb12))
                    r["tape_boost"] = True   # 되감기 견인 증강 (t91: s=678
                    #   바퀴 99% 헛돎 = 견인 한계 — 사용자: "힘만 더")
                    if LEG_FORCE:
                        _fi, _fv = force_legs(r)
                        if _fi:
                            _set_eff(r, _fv, _fi)
                    r["tape_boost"] = False
                    _set_pos(r, _sv, _si)
                    _v12 = TARGET_SPEED_MPS * 0.45
                    drive(r, math.degrees(_v12 / WHEEL_R) * r["dir"])
                    r["wheel_rad"] += (_v12 / WHEEL_R) * PHYSICS_DT
                    r["off_hist"] = r.get("off_hist", []) + [off_now]
                    continue
        _si, _sv = (((steer_bp_sched(r) if BP_SCHED else steer_bp_rollbend(r))
                     if r.get("bend_front")
                     else steer(r, i_now)) if NAV == "blueprint"
                    else (steer_rollbend(r) if r.get("bend_front")
                          else steer_vision(r)) if NAV == "vision"
                    else steer_onboard(r))
        if _pi or _si:
            _set_pos(r, _pv + _sv, _pi + _si)
        # 🔑 **속도를 스스로 정한다** — 관절이 꺾이면 줄이고 펴지면 올린다.
        # 🔑 판단층 둘(관절 꺾임 / 다리 실린더) 중 **더 느린 쪽**을 택한다.
        if NAV == "vision" and r.get("ctl") is not None:
            leg_speed(r)                         # 계측만 (판단은 컨트롤러가)
            # 🚨 bend_max 는 curve_speed() 안에서만 갱신되는데 vision 경로는
            #    그걸 안 부른다 — "최대 꺾임 0.0°" 로 찍히던 지표 구멍(실측).
            if r.get("bel"):
                _ang = np.degrees(np.abs(
                    np.asarray(r["art"].get_joint_positions())[r["bel"]]))
                r["bend_max"] = max(r.get("bend_max", 0.0),
                                    float(np.linalg.norm(_ang)))
            _v = r["v_cmd"]                      # ← `driver/control` 이 정한다
        elif AUTO_SPEED:
            curve_speed(r)                       # r["curve_f"] 갱신
            _f = min(r.get("curve_f", 1.0), leg_speed(r),
                     r.get("sched_slow", 1.0))   # 스케줄 조준 미완 감속
            # 🎯 코너 모드는 감속 면제 (t13 — 정적 평형은 관성으로 돌파한다.
            #    v9 의 T 통과는 75mm/s 풀속도였고, 우리는 곡관 감속이 45→20
            #    mm/s 로 깎아 정적으로 밀다 쐐기가 됐다).
            if r.get("sched_corner"):
                _f = max(_f, float(os.environ.get("BP_CORNER_F", 1.0)))
            r["want_f"] = _f
            _v = ramp(r, TARGET_SPEED_MPS * _f)
        else:
            _v = TARGET_SPEED_MPS
        if r.get("unjam_s0") is not None:
            _v *= 0.5                     # 되물림은 반속 — 사출 방지
        # 탈출 마지막 구간은 저속 — 휠 감쇠가 토크를 다 먹는다 (EXIT_V 주석)
        # 🚨 위치(관구 220mm 이내)로 걸면 **출발부터** 걸려 등반 추력이 모자라
        #    미끄러진다(GUI 실측: 185→211 후퇴 후 왕복). 접힌 다리가 실제로
        #    생긴 순간(= 토크 기근 구간의 시작)부터만 건다.
        if r.get("exit_folded") or r.get("exit_cavity"):
            _v = min(_v, EXIT_V)
        drive(r, math.degrees(_v / WHEEL_R) * r["dir"])
        r["wheel_rad"] += (_v / WHEEL_R) * PHYSICS_DT
        r["off_hist"] = r.get("off_hist", []) + [off_now]

        # 🚨 관 밖으로 튄 것은 되살리지 않는다 — 조용히 계속 돌면 로그가
        #    거짓말이 된다. 그 로봇만 멈추고 나머지는 계속 간다.
        # ⚠ 탈출 단계(EXIT)는 면제 — 관 밖에서는 중심선 끝점과의 거리가
        #    탈출 거리만큼 커지는 게 **정상**이다(끝점 클램프). 면제하지
        #    않으면 +80mm 시점에 이 감시가 탈출 성공보다 먼저 죽인다.
        # ⚠ **원호 구간은 한계를 넓힌다** (2026-08-10 저녁 — t5 기하 계산).
        #    각진 T 의 채점 중심선은 R162 필렛 **모델**이다. 물리적으로 옳은
        #    코너링(정면 벽 x=780 까지 파고든 뒤 선회 — v9 통과 기전)은 그
        #    필렛에서 최대 ~100mm 떨어진다(원호 중심 (568,688)→벽 (775,850)
        #    = 263mm − R162 = 101mm). 80mm 일괄 한계가 **옳은 통과를 매번
        #    s=638 에서 죽이고 있었다** — 어제 A~J 트라이얼의 사망 지점 전부.
        #    직선 구간은 그대로 80mm — 진짜 사출은 여전히 즉사한다.
        _off_lim = PIPE_IR + 0.03
        if NAV == "blueprint" and BP_SCHED:
            for _m4 in r.get("cl_arc_meta") or []:
                if _m4["s0"] - 0.05 <= s_now <= _m4["s1"] + 0.05:
                    _off_lim = PIPE_IR + float(
                        os.environ.get("BP_CORNER_OFF", 0.08))
                    break
        if r.get("phase") != "EXIT" and (
                off_now > _off_lim or not np.isfinite(p1).all()):
            print(f"[{r['name']:8s}] ❌ 코스 이탈 — 중심선에서 "
                  f"{off_now * 1000:.0f}mm. 이 로봇은 여기서 멈춘다")
            drive(r, 0.0)
            r["dead"] = True
            continue

        # ── NAV=vision : 복귀를 컨트롤러의 RETURN/DONE 으로 돈다 ────
        # 🚨 연습장의 방향 뒤집기(dir=−1)와 컨트롤러가 **싸운다**(실측:
        #    복귀 중 BRANCH 재트리거 → s=155 끼임 반복). vision 에서는
        #    연습장이 개입하지 않는다 — 코스 끝에서 `recall=True` 를 주고,
        #    복귀·끼임 회복은 전부 컨트롤러(RETURN/RECOVER)가 한다.
        if NAV == "vision" and r.get("ctl") is not None:
            if not r.get("recall") and s_now >= r["cl"].total - END_S:
                r["recall"] = True
                print(f"[{r['name']:8s}] ✅ 코스 끝 도달"
                      f" — recall 지시, 컨트롤러가 후진 복귀한다")
            # 🎯 조기 실패 판정 (2026-08-07 사용자 지적) — 끼인 채 스텝 예산을
            #    다 태우며 기다리지 않는다. 진행이 FAIL_S 동안 3mm 미만이면
            #    그 시점에서 실패로 끊는다. 시도 회전이 몇 배 빨라진다.
            if abs(s_now - r.get("s_prog", -1.0)) > 0.003:
                r["s_prog"], r["prog_mark"] = s_now, step
            elif step - r.get("prog_mark", step) > FAIL_S * PHYSICS_HZ:
                print(f"[{r['name']:8s}] ❌ 조기 중단 — {FAIL_S:.0f}초째 "
                      f"제자리 (s={s_now * 1000:.0f}mm, "
                      f"상태 {r.get('ctl_state', '?')}). 실패로 판정")
                drive(r, 0.0)
                r["dead"] = True
                continue
            # 🎯 순진행 워치독 (위 STALL_S 주석) — 제자리 **왕복**을 끊는다.
            #    FAIL_S 가 못 잡는 정체(RECOVER 물러남↔전진 반복)가 여기 걸린다.
            # 🔑 나갈 때는 **최댓값**, 복귀할 때는 **최솟값**이 최고 도달점이다.
            #    recall 로 임무 국면이 바뀌면 기준을 새로 잡는다.
            _out = not r.get("recall")
            if r.get("best") is None or r.get("stall_out") != _out:
                r["best"], r["stall_out"], r["stall_mark"] = s_now, _out, step
            elif (s_now - r["best"] if _out else r["best"] - s_now) > STALL_WIN:
                r["best"], r["stall_mark"] = s_now, step
            elif step - r["stall_mark"] > STALL_S * PHYSICS_HZ:
                print(f"[{r['name']:8s}] ❌ 순진행 정체 — {STALL_S:.0f}초 동안 "
                      f"최고 도달점(s={r['best'] * 1000:.0f}mm)을 "
                      f"{STALL_WIN * 1000:.0f}mm 이상 못 넘었다 "
                      f"(현재 s={s_now * 1000:.0f}mm, "
                      f"{'전진' if _out else '복귀'}, "
                      f"상태 {r.get('ctl_state', '?')}, "
                      f"끼임회복 {r['ctl'].s.recover_try}회). 실패로 판정")
                diag_stuck(r)
                drive(r, 0.0)
                r["dead"] = True
                continue
            if r.get("ctl_state") == "DONE":
                # 🎯 **실전 임무는 왕복 반복이 아니다** (2026-08-08 사용자 확정:
                #    *"왕복 무한 반복은 map_test 때처럼 짧은 구간을 확인하려던
                #    것. 복귀의 끝지점은 샤워 배수구를 밖으로 나가는 것"*).
                # 🚨 컨트롤러의 DONE 은 **적산 거리가 0 에 닿았다**는 뜻이지
                #    "입구에 도착했다"가 아니다. 나갈 때 시각 오도메트리가
                #    부풀려 세면(실측 슬립 2.00 = 두 배로 읽음) 돌아올 때
                #    **관 한복판에서** 0 에 닿는다 → 예전 코드는 그것을 랩
                #    완료로 보고 재출발시켜 **다시 관으로 들어갔다**
                #    (사용자 관찰). 이제 위치로 판정한다.
                if s_now > EXIT_S:
                    # 아직 관 안이다 — 복귀를 이어 간다. 적산이 모자란 만큼
                    # 채워 주고 상태를 RETURN 으로 되돌린다.
                    # ⚠ 이 보충은 **채점 계층이 오도메트리 결함을 메우는 것**
                    #    이다. 근본 해결은 시각 오도메트리의 과대적산 수정.
                    if not r.get("_odom_warned"):
                        print(f"[{r['name']:8s}] ⚠ 적산 소진(DONE)인데 아직 관 "
                              f"안(s={s_now * 1000:.0f}mm > 출구 "
                              f"{EXIT_S * 1000:.0f}mm) — 복귀 계속 "
                              f"(오도메트리 과대적산)")
                        r["_odom_warned"] = True
                    r["ctl"].s.state = "RETURN"
                    r["ctl"].s.distance_m = float(s_now)
                    continue
                print(f"[{r['name']:8s}] ✅ **임무 완료** — 샤워 배수구로 복귀 "
                      f"(s={s_now * 1000:.0f}mm)")
                r["lap"] = 1               # 결과 줄의 '임무완료' 표시용
                drive(r, 0.0)
                r["dead"] = True
                continue
            continue

        # ── 완전 탈출 단계 — s 는 관 밖에서 0 에 클램프되므로 월드 변위로 잰다
        if r.get("phase") == "EXIT":
            _d = float(np.linalg.norm(wpos(r["seg1"]) - r["exit_p0"]))
            if _d >= EXIT_EXTRA:
                print(f"[{r['name']:8s}] ✅ **임무 완료** — 몸 전체가 배수구 "
                      f"관 밖으로 탈출 (관구 밖 +{_d * 1000:.0f}mm)")
                r["lap"] = 1
                drive(r, 0.0)
                r["dead"] = True
            elif step - r["mark"] > 20.0 * PHYSICS_HZ:
                # 탈출 단계는 s 기반 끼임 판정이 안 통한다 — 자체 시한을 둔다
                print(f"[{r['name']:8s}] ⚠ 탈출 정체 — 관구 밖 "
                      f"{_d * 1000:.0f}/{EXIT_EXTRA * 1000:.0f}mm 에서 "
                      f"20초 무진행. 실패로 판정")
                drive(r, 0.0)
                r["dead"] = True
            continue

        # 🎯 **T 3점턴** (2026-08-10 밤, t57 확정 — 복귀 T 는 정면이 열린
        #    관이라 몸을 세워줄 벽이 없다: 끼임이 안 나니 되물림 스윙도 안
        #    나고 미끄러져 지나가다 심판에 죽는다). 이탈이 문턱을 넘으면
        #    = 입구를 지나치기 시작 → 굽힘 유지한 채 의도적 후진 스윙을
        #    발동한다(트레일러 후진). 전진 복귀는 기존 UNJAM_M 복원이 맡는다.
        if (r.get("sched_tee_p") is not None and r.get("unjam_s0") is None
                and r["dir"] > 0
                and off_now > float(os.environ.get("BP_TEE_SWING", 0.060))):
            r["stuck"] += 1
            r["dir"] = -1
            r["unjam_s0"] = s_now
            sync_rollbend_cmds(r)
            r["s_last"], r["mark"] = s_now, step
            r["off_at_mark"] = off_now
            print(f"[{r['name']:8s}] ↪ T 3점턴 — 이탈 {off_now * 1000:.0f}mm, "
                  f"굽힘 유지 후진 스윙 (s={s_now * 1000:.0f}mm)")
            continue

        # 진행 방향 기준으로 나아갔는가
        adv = (s_now - r["s_last"]) * r["dir"]
        if adv > 0.003:
            r["s_last"], r["mark"] = s_now, step
            r["off_at_mark"] = off_now
            # 🎯 끼임 탈출 물러남이 UNJAM_M 에 닿으면 **임무 방향으로 복원**
            #    (2026-08-10). 복원이 없으면 복귀 중 끼임 한 번에 코스를 한
            #    바퀴 더 돈다 — GUI 런 실측(라이저 끼임 → 2바퀴째).
            if r.get("unjam_s0") is not None and \
                    abs(s_now - r["unjam_s0"]) >= UNJAM_M:
                r["unjam_s0"] = None
                r["dir"] = -1 if r.get("phase") == "BACK" else 1
                sync_rollbend_cmds(r)     # 지령 적분기 ← 실측 (사출 방지)
                r["s_last"], r["mark"] = s_now, step
                print(f"[{r['name']:8s}] ↩ 끼임 탈출 물러남 "
                      f"{UNJAM_M * 1000:.0f}mm 완료 — 임무 방향 복원 "
                      f"({'복귀' if r['dir'] < 0 else '전진'}, "
                      f"s={s_now * 1000:.0f}mm)")
        elif step - r["mark"] > (float(os.environ.get("STUCK_ARC_S", 8.0))
                                 if r.get("sched_arc") else STUCK_S) \
                * PHYSICS_HZ:
            # 🎯 원호(접합부) 안은 인내 8초 — 공동 교량 통과(머리가 건너편
            #    팔 벽에 닿을 때까지 미는 것)는 3초보다 길다 (2026-08-10 t20).
            # 🎯 **코너 선회는 s 로 안 보인다** (2026-08-10 t10 실측). 각진 T
            #    선회는 회전 운동이라 최근접 s 가 제자리(638)인데, 이탈은
            #    84.8→57mm 로 끼임 34회에 걸쳐 꾸준히 줄었다 — 로봇은 돌고
            #    있었고 워치독이 3초마다 후진시켜 진행을 되돌린 것.
            #    코너 모드 중 이탈이 2mm+ 줄었으면 진행으로 인정하고 참는다.
            if r.get("sched_corner") and \
                    off_now < r.get("off_at_mark", 1.0) - 0.002:
                r["mark"] = step
                r["off_at_mark"] = off_now
                continue
            # 끼임 → **방향을 뒤집어 빠져나온다** (임무 규칙 8 의 연습장판)
            r["stuck"] += 1
            r["dir"] *= -1
            r["unjam_s0"] = s_now        # 물러남 시작점 — UNJAM_M 뒤 복원
            sync_rollbend_cmds(r)        # 지령 적분기 ← 실측 (사출 방지)
            r["s_last"], r["mark"] = s_now, step
            r["off_at_mark"] = off_now
            print(f"[{r['name']:8s}] ⚠ 끼임 {r['stuck']}회 "
                  f"(s={s_now * 1000:.0f}mm, 이탈 {off_now * 1000:.1f}mm) "
                  f"→ 방향 전환 {'전진' if r['dir'] > 0 else '후진'}")
            if r["stuck"] <= 2:          # 처음 두 번만 자세히 (로그 폭주 방지)
                diag_stuck(r)
            continue

        # 코스 끝에 닿으면 복귀, **몸 전체가 배수구 밖으로 나오면** 임무 종료.
        # 🎯 **왕복 반복은 폐지했다** (2026-08-09 사용자 확정: *"이제 왕복
        #    필요없어. 실전으로 반영하는 단계야"*). 무한 왕복은 연습장에서
        #    짧은 구간을 되풀이해 보려던 규약이고, 실전 임무는 **편도 주행 +
        #    복귀 1회**로 끝난다(vision 경로는 이미 그렇게 돈다 — 위 EXIT_S).
        # 🚨 **코스 끝은 관의 열린 끝단이다.** 마지막 점까지 가면 로봇 앞
        #    절반이 관 밖으로 나가고, 벽을 잃은 다리가 스트로크 한계까지 뻗어
        #    **관 밖으로 튀어나온다**(GUI 로 확인).
        #    → 출발과 같은 여유(END_S)를 남기고 되돌아간다.
        # 🎯 종점은 출발 인덱스(i0)가 아니라 **관 밖**이다 (2026-08-10 사용자:
        #    *"완전히 튀어나와서 몸 전체가 배수구 관 밖으로"*. i0 는 라이저
        #    한가운데라 "끝지점까지만 가고 말았"다) — 방향 뒤집기 오판을
        #    막기 위해 phase(OUT/BACK/EXIT)로 임무 국면을 따로 든다.
        if r.get("phase", "OUT") == "OUT" and r["dir"] > 0 \
                and s_now >= r["cl"].total - END_S:
            # 🎯 **역재생 귀가 스냅** (floor1) — 코스 끝 = 오른팔 스텁.
            #    후진 복귀는 코스를 되밟는 것이 아니라 **나가는 가지 s 로
            #    갈아타** 원길(T 역통과→본관→라이저)을 되밟아야 한다.
            #    같은 물리 지점의 나가는 branch s(≈850mm 부근)로 스냅.
            # floor1: 스냅 없이 **코스 후진 그대로** — 루프를 되밟아 오른팔로
            # 후진 접근한 뒤, s∈[400,1000] 에서 테이프 되감기가 이어받는다.
            r["phase"] = "BACK"
            r["dir"] = -1
            r["unjam_s0"] = None
            r["s_last"], r["mark"] = s_now, step
            print(f"[{r['name']:8s}] ✅ 코스 끝 도달 — 복귀 시작")
        elif r.get("phase") == "BACK" and r["dir"] < 0 and s_now <= EXIT_S:
            r["phase"] = "EXIT"
            r["exit_p0"] = wpos(r["seg1"]).copy()
            r["s_last"], r["mark"] = s_now, step
            print(f"[{r['name']:8s}] 🚪 배수구 관구 도달"
                  f"(s={s_now * 1000:.0f}mm) — 몸 전체 탈출까지 "
                  f"+{EXIT_EXTRA * 1000:.0f}mm 더 후진")

    # ── 🌐 웹 발행(10Hz) + 지령 수신 (v1_3, 동민 ros_bridge) ────────────
    if RB is not None:
        RB.spin()
        if step % RENDER_EVERY == 0:
            for r in robots:
                _rp = RB.robot(r["name"])
                _m9 = _rp.pop_mission()
                if _m9:
                    _cmd9 = str(_m9.get("cmd", "")).lower()
                    if _cmd9 in ("stop", "hold", "estop"):
                        r["hold_cmd"] = True
                        print(f"[{r['name']:8s}] 🌐 지령 {_cmd9} — 정지 유지")
                    elif _cmd9 in ("start", "resume", "go"):
                        r["hold_cmd"] = False
                        print(f"[{r['name']:8s}] 🌐 지령 {_cmd9} — 재개")
                _w9 = r.get("weld")
                _wst = _w9["st"] if _w9 else None
                # 🎯 활성 카메라 1대 규약 (2026-08-11 사용자 확정):
                #    전진→front / 후진→rear / 정렬~아크→torch. 렌더·인코딩
                #    비용이 로봇당 1대 몫으로 준다 (2기여도 최대 2대).
                # 🎥 **발행 분기 철회** (2026-08-11 사용자 지시) — 카메라가
                #    로봇당 1대뿐이라 "상황에 맞는 것만" 골라 봐야 의미가
                #    없다. 있는 카메라를 **항상** 내보내고, 웹은 두 토픽을
                #    계속 구독한다. `_role` 은 이제 보고용 이름일 뿐이다.
                # 🎯 웹에는 **전방 카메라**를 뿌린다 (2026-08-11 사용자 지시 —
                #    용접 카메라를 끈 지금 전방이 임무 시야다).
                _role = ("front" if "front" in r.get("cams", {})
                         else next(iter(r.get("cams", {})), "front"))
                if r["dead"]:
                    _st9 = "DEAD"
                elif _wst in ("ALIGN", "EXTEND"):
                    _st9 = "INSPECT"
                elif _wst is not None:
                    _st9 = "REPAIR"
                elif r["state"] == "SETTLE":
                    _st9 = "SETTLE"
                elif r.get("hold_cmd"):
                    _st9 = "HOLD"
                elif r.get("phase", "OUT") in ("BACK", "EXIT"):
                    _st9 = "RETURN"
                else:
                    _st9 = "RUN"
                _p9 = wpos(r["seg1"])
                _sn9, _off9, _ = r["cl"].nearest(_p9, r.get("s_hint"))
                _rp.publish_state(
                    state=_st9, direction=int(r["dir"]),
                    speed_mps=TARGET_SPEED_MPS * float(r.get("sched_slow",
                                                             1.0)),
                    s_mm=_sn9 * 1000.0, s_total_mm=r["cl"].total * 1000.0,
                    off_mm=_off9 * 1000.0, lap=r.get("lap", 0),
                    stuck=r.get("stuck", 0), step=step,
                    roll_deg=float(r.get("body_roll_abs", 0.0)),
                    art=r["art"], wheel_idx=r.get("wheel"),
                    pos=_p9, cam=_role)
                if not CAMERAS:
                    continue
                _rp.publish_camera()          # 등록된 카메라 전부(1대)
                _dy = r.get("dy_pub")
                if _dy is None:
                    continue
                from sensor_msgs.msg import CompressedImage as _CI9
                from std_msgs.msg import String as _S9
                _dy["which"].publish(_S9(data=f"{r['name']}_{_role}_camera"))
                # 동연 active_cam rgb + 판정 JSON — **항상** 발행(위 철회와
                # 같은 이유). 판정은 용접 국면에만 뜻이 있으므로 그때만 붙인다.
                _tc9 = next((c for c in _rp.cams if c[0] == _role), None)
                if _tc9 is None:
                    continue
                try:
                    _rgb9 = _tc9[1].get_data()
                    if _rgb9 is None or not getattr(_rgb9, "size", 0):
                        continue
                    _a9 = np.asarray(_rgb9)[:, :, :3].astype(np.uint8)
                    _im9 = _CI9()
                    _im9.header.stamp = RB.node.get_clock().now().to_msg()
                    _im9.header.frame_id = "torch_camera"
                    _im9.format = "jpeg"
                    _im9.data = _rosb.codec.rgb_to_jpeg(_a9)
                    _dy["rgb"].publish(_im9)
                    if _DY_FINDERS is not None and _wst is not None:
                        _dep9 = (_tc9[2].get_data()
                                 if _tc9[2] is not None else None)
                        # 🚨 인자 순서가 원본과 다르다 — 키워드로만 부른다
                        _h9 = _DY_FINDERS.find_wall_hole(
                            _a9, expect_px=None, depth=_dep9)
                        _b9 = _DY_FINDERS.find_weld_bead(_a9, expect_px=None)
                        _dy["judge"].publish(_S9(data=json.dumps(
                            dict(frame="torch_camera", phase=_wst,
                                 t=round(step / PHYSICS_HZ, 2),
                                 hole=_h9, bead=_b9),
                            ensure_ascii=False, default=str)))
                except Exception:
                    pass      # 판정 발행은 보조 — 실패해도 주행은 간다

    # 🎯 전멸 즉시 종료 (2026-08-07 사용자 지시) — 죽은 로봇을 두고 스텝
    #    예산을 태우지 않는다. 실패는 이미 났고, 다음 시도가 더 싸다.
    if robots and all(r["dead"] for r in robots):
        print(f"[중단] 로봇 전멸 (step {step}/{STEPS}) — 즉시 종료")
        break

    if step - _last_report >= REPORT_S * PHYSICS_HZ:
        _last_report = step
        _line = []
        for r in robots:
            p1 = wpos(r["seg1"])
            s_now, off_now, _i = r["cl"].nearest(p1, r.get("s_hint"))
            if NAV == "vision":
                _c = r.get("cond")
                # 🎯 **몸통이 얼마나 휘어 있는가** (2026-08-09 사용자 관찰:
                #    *"직선 구간에서도 자꾸 휜 채로 이동하는 듯하다"*). vision
                #    경로는 `curve_speed()` 를 안 불러 관절합이 상태줄에 없었다
                #    — 눈으로 본 것을 숫자로 확인할 수 없었다는 뜻이다.
                #    관절 4개 × 상한 STEER_MAX 17° = 68° 가 만석이다.
                # 🎯 자산 방식에 따라 **다른 것을 찍는다** (2026-08-09).
                #    v1_1 의 피치/요 표시는 D6 전제라, v9(롤+굽힘)에서는
                #    항상 0 으로 찍혀 아무것도 안 보였다 — 오늘 배운 그대로
                #    "결과만 보고 원인을 못 보는" 상태가 된다.
                _qp = _qy = 0.0
                _rb = ""
                _qq = np.asarray(r["art"].get_joint_positions())
                if r.get("bend_front"):
                    _bf = math.degrees(float(np.sum(_qq[r["bend_front"]])))
                    _br = math.degrees(float(np.sum(
                        _qq[[k for k in r["bend_dof"]
                             if k not in r["bend_front"]]])))
                    _rf = math.degrees(float(np.sum(_qq[r["roll_front"]])))
                    _rb = (f" 굽힘앞{_bf:+.0f}/뒤{_br:+.0f}° 롤앞{_rf:+.0f}°"
                           f"(오차{r.get('roll_err_deg', 0.0):+.0f}°)"
                           f" 지령굽힘{math.degrees(r.get('cmd_bend', 0.0)):+.0f}"
                           f"/롤{math.degrees(r.get('cmd_roll', 0.0)):+.0f}°")
                elif r.get("bel_pitch") or r.get("bel_yaw"):
                    _qp = math.degrees(float(np.sum(_qq[r["bel_pitch"]])))
                    _qy = math.degrees(float(np.sum(_qq[r["bel_yaw"]])))
                _line.append(
                    f"{r['name']} s={s_now*1000:5.0f}"
                    # 중심선까지의 거리 = **중심을 잡고 있는가**의 정본 지표.
                    # 관벽까지 50mm 이므로 한 자릿수여야 정상이다.
                    f" 이탈{off_now*1000:4.1f}"
                    f" {r.get('ctl_state','?')}"
                    f" 롤{r.get('roll_abs', 0.0):+.0f}°우측{r.get('phi_right', 0.0):+.0f}°"
                    f" v={r.get('v_cmd',0)*1000:3.0f}mm/s"
                    f" 시각속도={'--' if r.get('vis_mps') is None else format(r['vis_mps']*1000,'.0f')}"
                    f" 슬립={r.get('slip',0):.2f}"
                    # 🎯 다리 거부권 계측 (2026-08-09) — `suppress_branch` 는
                    #    이 값이 1.8mm 를 넘으면 서고, 그동안 **분기 확인
                    #    타이머가 통째로 리셋**된다. T 에서 진입이 한 번도
                    #    안 걸리는 원인 후보라 상태줄에 드러낸다.
                    f" 다리{r.get('leg_dev', 0) * 1000:.1f}mm"
                    f"{'*거부' if r.get('leg_dev', 0.0) > 0.0018 else ''}"
                    f"/뜬{r.get('leg_free_n', 0)}"
                    f"/편차{r.get('leg_spread_now', 0) * 1000:.0f}"
                    f"/상한{r.get('leg_cap', 0) * 1000:.0f}"
                    + (_rb if _rb else f" 관절합P{_qp:+.0f}/Y{_qy:+.0f}")
                    + f" 차동{r.get('diff_spread', 0.0):.2f}"
                    # 🎯 **속도가 0 이 되기까지의 관문을 전부 찍는다**
                    #    (2026-08-09 사용자 지적: *"테스트 때 확인할 수 있는
                    #    것인데 왜 자꾸 같은 문제로 회귀하나"*). 그 지적이 맞다 —
                    #    그동안 상태줄이 **결과**(도달 s·이탈)만 보여 줬고,
                    #    "왜 바퀴에 0 이 나갔나"는 안 보였다. 그래서 원인을
                    #    한 칸씩 더듬으며 같은 자리로 되돌아왔다.
                    # 🔑 속도 법칙은 `v = v_max × 개방도 × 곡관 × 판정 × 전방`
                    #    이라 **어느 항이 0 인지만 보면 원인이 바로 나온다.**
                    #    컨트롤러가 이미 `gates` 로 들고 있는데 안 찍고 있었다.
                    f" 게이트{r.get('gates', {})}"
                    # 판정이 **왜** 그렇게 났는지 — 검출기가 남기는 사유.
                    f" 사유[{'--' if _c is None else _c.reason[:38]}]"
                    f" 판정={'--' if _c is None else _c.state}"
                    f"(개방{0 if _c is None else _c.aperture_px}px"
                    f" 관경{0 if _c is None else _c.bore_mm:.0f}mm"
                    f" 분기{0 if _c is None else _c.branch_ratio * 100:.0f}%"
                    f"@{0 if _c is None else _c.branch_deg:+.0f}°"
                    # 🎯 **구멍이 몇 개로 세어지는가** (2026-08-09 사용자 지시의
                    #    핵심). T/곡관 판별 규칙이 "구멍 2개 = T / 1개 = 곡관"
                    #    이므로, 곡관에서 BRANCH 가 뜬다면 **없는 둘째 구멍을
                    #    만들어 내고 있다**는 뜻이다. 폭(arc)이 0 이면 없는 것.
                    f"[폭{0 if _c is None else _c.branch_arc_deg:.0f}"
                    f"/2째{0 if _c is None else _c.branch2_arc_deg:.0f}°"
                    f"@{0 if _c is None else _c.branch2_deg:+.0f}°]"
                    f" 전방{0 if _c is None else _c.forward_range_m:.2f}m"
                    f"{'곡관' if _c is not None and _c.curve_ahead else ''}"
                    f"{'확' if _c is not None and _c.curve_sure else ''}"
                    f" 입사{0 if _c is None else _c.incidence_deg:.0f}°"
                    f"@{0 if _c is None else _c.incidence_clock_deg:+.0f}°"
                    f" 오프셋{0 if _c is None else _c.offset_mm:.0f}mm"
                    f"@{0 if _c is None else _c.joint_range_m:.2f}m)")
                continue
            _w = r.get("want", (0.0, 0.0))
            if r.get("bend_front"):
                # 🎯 정답지 롤+굽힘 판단 사슬 (2026-08-10) — 종전에는 D6 용
                #    피치/요 칸이 전부 0 으로 찍혀 **T 이탈 80mm 의 원인을
                #    지령/실측 어느 쪽에서도 볼 수 없었다**(어제 교훈 재발:
                #    결과만 보고 원인을 못 보는 상태줄).
                _qq2 = np.asarray(r["art"].get_joint_positions())
                _bf2 = math.degrees(float(np.sum(_qq2[r["bend_front"]])))
                _br2 = math.degrees(float(np.sum(_qq2[
                    [k for k in r["bend_dof"] if k not in r["bend_front"]]])))
                _rf2 = math.degrees(float(np.sum(_qq2[r["roll_front"]])))
                _rr2 = math.degrees(float(np.sum(_qq2[
                    [k for k in r["roll_dof"] if k not in r["roll_front"]]])))
                # 🎯 **몸통 절대 롤** (2026-08-10 저녁 — 사용자 가설: 수직곡관
                #    하강 중 몸이 비틀린 채 T 에 도착해 다리 클로킹이 복불복.
                #    임무 규격대로 후방 세그 기준·중력 참조로 잰다. 런별
                #    T 도착 롤과 정지 양상의 상관을 이 값으로 검증한다.)
                _Rb4 = wrot(r["seg0"])
                _bx4 = _Rb4 @ np.array([1.0, 0.0, 0.0])
                _by4 = _Rb4 @ np.array([0.0, 1.0, 0.0])
                _up4 = np.array([0.0, 0.0, 1.0]) \
                    - _bx4 * float(_bx4[2])
                _n4b = float(np.linalg.norm(_up4))
                _rabs = 0.0
                if _n4b > 1e-6:
                    _up4 /= _n4b
                    _rabs = math.degrees(math.atan2(
                        float(np.dot(np.cross(_up4, _by4), _bx4)),
                        float(np.dot(_up4, _by4))))
                r["body_roll_abs"] = _rabs
                _x = (f" {r.get('v_cmd', 0) * 1000:3.0f}mm/s"
                      f" 이탈{off_now * 1000:4.1f}mm"
                      f" 몸롤{_rabs:+.0f}°"
                      f" 굽힘F{r.get('bp_f_deg', 0.0):+.0f}"
                      f"→{_bf2:+.0f}/R{r.get('bp_r_deg', 0.0):+.0f}"
                      f"→{_br2:+.0f}°"
                      f" 롤F{math.degrees(r.get('cmd_roll', 0.0)):+.0f}"
                      f"→{_rf2:+.0f}"
                      f"/R{math.degrees(r.get('cmd_roll_r', 0.0)):+.0f}"
                      f"→{_rr2:+.0f}°"
                      f" 다리편차{r.get('leg_spread_now', 0) * 1000:.1f}"
                      f"/뜬{r.get('leg_free_n', 0)}"
                      f" 배율{r.get('want_f', 1):.2f}")
            else:
                _x = (f" {r.get('v_cmd', 0) * 1000:3.0f}mm/s"
                      f" 목표P{_w[0]:+.0f}/Y{_w[1]:+.0f}"
                      f" 지령{r.get('cmd_pitch', 0):+.0f}"
                      f"/{r.get('cmd_yaw', 0):+.0f}"
                      f" 관절합P{r.get('q_pitch', 0):+.0f}"
                      f"/Y{r.get('q_yaw', 0):+.0f}"
                      f" 다리편차{r.get('leg_spread_now', 0) * 1000:.1f}"
                      f"/기준벗어남{r.get('leg_dev', 0) * 1000:.1f}mm"
                      f" 배율{r.get('want_f', 1):.2f}"
                      f" 실제P{r.get('act_pitch', 0):+.0f}"
                      f"/Y{r.get('act_yaw', 0):+.0f}")
            _line.append(f"{r['name']} s={s_now * 1000:5.0f}"
                         f"{'→' if r['dir'] > 0 else '←'}"
                         f" 끼임{r['stuck']}{_x}"
                         + ("(정지)" if r["dead"] else ""))
        print(f"  [{step / PHYSICS_HZ:6.1f}s] " + " | ".join(_line))

    if all(r["dead"] for r in robots):
        print("[중단] 세 대 모두 코스를 벗어났다")
        break

print("=" * 78)
for r in robots:
    p1 = wpos(r["seg1"])
    s_now, off_now, _i = r["cl"].nearest(p1, r.get("s_hint"))
    print(f"결과  {r['name']:8s} {'임무완료' if r['lap'] else '미완'}  "
          f"끼임 {r['stuck']}회  "
          f"s={s_now * 1000:.0f}mm  이탈 {off_now * 1000:.1f}mm "
          f"(평균 {np.mean(r.get('off_hist', [0])) * 1000:.1f} / 최대 "
          f"{np.max(r.get('off_hist', [0])) * 1000:.1f}mm)  실속도/지령 "
          f"{np.mean(r.get('gov_hist', [1])) * 100:.1f}%  최대 꺾임 "
          f"{r.get('bend_max', 0):.1f}°  조향오차 최대 "
          f"{r.get('steer_max', 0):.1f}°  뜬다리 최대 "
          f"{r.get('leg_free_max', 0)}개  다리편차 최대 "
          f"{r.get('leg_spread_max', 0) * 1000:.1f}mm"
          f"  다리 최대신장 {r.get('leg_ext_max', 0) * 1000:.1f}mm"
          f"(도달 {TUNE['wheel_center_mm'] + r.get('leg_ext_max', 0) * 1000:.0f}"
          f" / 관벽 {(PIPE_IR - WHEEL_R) * 1000:.0f}mm)"
          + ("  ❌ 이탈로 정지" if r["dead"] else ""))
sys.stdout.flush()
simulation_app.close()
