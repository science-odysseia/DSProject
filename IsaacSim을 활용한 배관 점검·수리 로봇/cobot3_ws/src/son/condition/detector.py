"""[공용] 관 상태 판정 — ROS 비의존 순수 로직.

판정 순서 (사양서):
  1. Depth 무효 픽셀 비율 > 임계        → DISCONNECTED
  2. 전방 개구부 원형도 < 임계          → UNDETERMINED
  3. 원 피팅 중심 이탈량 = 축 오프셋(mm) → NORMAL / MISALIGNMENT 등급

곡관 예외: 관절 엔코더 각도가 임계 이상이면 위를 전부 무시하고 NORMAL.

Disconnected 와 Misalignment 의 구분은 Depth 반사 유무 하나로 한다.
어긋난 조인트는 먼 관 끝면이 초승달로 노출돼 전 영역에서 Depth 가 유효하지만,
관이 끊겼으면 그 너머가 빈 공간이라 반사가 없어 무효가 된다.
"""

from dataclasses import dataclass, asdict

import cv2
import numpy as np

NORMAL = "NORMAL"
MISALIGNMENT = "MISALIGNMENT"
DISCONNECTED = "DISCONNECTED"
UNDETERMINED = "UNDETERMINED"
# 🎯 2026-08-07 신설 — 주행이 **분기**와 **관경 변화**를 구분해야 다리(실린더)를
#    상황에 맞게 몰 수 있다. 둘 다 통행 가능(passable)이고 감속 신호다.
BRANCH = "BRANCH"            # 옆으로 벽이 사라졌다 = T 분기 개구
BORE_CHANGE = "BORE_CHANGE"  # 관경이 달라졌다 = 리듀서/확관

SPEED_FULL = "full"
SPEED_SLOW = "slow"
SPEED_STOP = "stop"


DEFAULTS = dict(
    projection="fisheye_equidistant",
    invalid_ratio_max=0.02,
    invalid_mode="empty_is_zero",
    max_range_m=1.5,
    circularity_min=0.60,
    roughness_max=0.015,
    aperture_min_px=400,
    # 🚨 주행용 재보정 (2026-08-07 심야): 1/3/8mm 는 용접 정렬 정밀도급
    #    수치가 주행 판정에 이식된 것 — 실측으로 **정상 주행의 이탈이 평균
    #    5~15mm** 라(모든 코스 결과 라인), 그 문턱이면 정상 주행도 감속·정지
    #    판정을 받는다(실측: reducer 안착 오프셋 4mm → 감속 12mm/s → 정지
    #    마찰을 못 이겨 출발 정체 사망). 용접 정렬은 별도 단계의 몫이다.
    offset_normal_mm=6.0,
    offset_slow_mm=12.0,
    offset_max_mm=25.0,
    bend_angle_deg=8.0,
    center_patch_frac=0.10,
    blur_ksize=5,
    morph_ksize=5,
    # ── 분기·관경 (2026-08-07 신설) ──────────────────────────────
    # 🔑 **중간 시야 링 하나로 둘 다 잡는다.** 어안 등거리 규약(ρ = f·θ)에서
    #    광축과 θ 를 이루는 광선은 관벽을 거리 d 에서 만나고, 관 내반경은
    #    R = d·sinθ 다. 그 링을 방위별로 보면:
    #      · 벽이 온전 → 전 방위에서 R 이 일정  → 그 R 이 곧 관경
    #      · 벽이 사라짐 → 그 방위만 무효/원거리 → 분기 개구 방향
    #    앞쪽 개구부(전방 관 끝)는 광축 근처(θ 작음)라 이 링에 안 걸린다.
    bore_ring_deg=50.0,        # 링을 어느 θ 에서 뜰 것인가
    bore_ring_band_deg=6.0,    # 링 두께
    bore_dn100_mm=50.0,        # 기준 관 내반경
    bore_change_mm=8.0,        # 이만큼 벗어나면 관경 변화로 본다
    branch_ratio_min=0.08,     # 링에서 벽 없는 화소 비율이 이보다 크면 분기
    branch_arc_min_deg=25.0,   # 그 방위 폭이 이보다 넓어야 분기(잡음 배제)
    # 🎯 **T 와 곡관을 가르는 개구 폭** (2026-08-09 덤프 실측). 개구가 이보다
    #    좁으면 곡관(바깥벽이 물러난 띠), 넓으면 T(가지 관이 통째로 뚫림).
    #      진입 곡관 40장 : 20 ~ **40°**
    #      정면 T   15장 : **50°** ~ 110°
    #    겹침 0 — 사이값 45 를 쓴다. 사용자 설계 규칙("구멍 2개면 T, 1개면
    #    곡관")의 실측 가능한 대리 지표다: 둘째 개구의 **폭**은 T 에서 10~60°
    #    로 흔들려 세는 것 자체가 불안정한 반면, 제1 개구 폭은 안 겹친다.
    branch_tee_arc_min_deg=45.0,
    # 🚨 상한도 필요하다 (2026-08-07 실측) — DN150 **확관**은 벽이 전 방위에서
    #    고르게 멀어져 "벽 없음" 이 사방에 흩어진다(방위 폭이 아주 커진다).
    #    상한 없이는 확관이 BRANCH 로 오판돼 직관 한가운데서 진입 기동이
    #    발동했다(reducer s=690 쐐기 박힘). 분기 개구는 한쪽에 몰린다.
    branch_arc_max_deg=140.0,
    # 🚨 far 임계는 **고정값**이어야 한다 (2026-08-07 실측). bore 추정 기반
    #    (1.6×bore)으로 했더니 확관에서 bore 중앙값이 근벽에 눌려 39mm 로
    #    나왔고, 1.6×39 = 62mm 가 DN150 벽(75mm)을 "벽 없음" 으로 분류해
    #    **확관이 통째로 BRANCH 오탐**이 됐다(진입 기동 발동 → 쐐기 박힘).
    #    분기 개구는 광선이 가지 관 안으로 90mm 이상 들어간다 — 90 으로 갈린다
    #    (120 은 분기 실검출까지 죽였다: tee_back 분기 8% 로 미달, 실측).
    branch_far_mm=90.0,
    # 🎯 확관/분기의 결정타 (2026-08-07, 세 번째 시도) — **결손 중 무효(inf)
    #    비율.** 확관은 벽이 멀어질 뿐 항상 있으므로 결손이 전부 유한(far)이고
    #    inf 가 0 이다. 분기는 개구 너머가 열린 관이라 inf 가 섞인다.
    #    → ❌ **이것도 기각** (실측): 각진 T 가지도 광선이 가지 벽에 맞아
    #      inf≈0 이라 진짜 분기까지 죽였다. 계측 필드는 남긴다(참고용).
    #    (반경 임계 2회, 방위폭 상한 — 반경 60~90 vs 75 겹침으로 기각.
    #     결정판은 시간 축: driver 의 확인 타이머 × 다리 거부권 리셋.)
    branch_inf_min=0.0,
    # 🎯 곡관 판별자 (2026-08-07, 덤프 실측): far 반경 중앙값이 bore 의 이
    #    배수를 넘으면 분기가 아니라 **곡관 접근**이다. 실측 마진 2배
    #    (분기 1.9× vs 곡관 3.9×)의 가운데. run() 의 BRANCH 게이트에서 사용.
    branch_far_bore_ratio=2.6,
    # 위 far 판별자를 쓸 것인가 — 실전 맵에서는 방향이 반대라 기본 OFF
    # (2026-08-09 덤프 실측, `_curve_like` 주석 참조).
    branch_far_curve_on=False,
    # 🎯 **6번째 시도 — depth 프레임 80장 실측으로 설계한 결정판** (2026-08-07).
    #    진짜 분기: 링 벽 반경 R50 = 49~73mm (관벽 50 정상, 결손만 국소).
    #    확관 오탐: R50 = 13~40mm — **자기 몸체(토치 링 ~30mm)가 시야를 가려**
    #    링 통계가 무너진 프레임이었다(확관 기하 자체가 아니었다!).
    #    → 벽 반경이 정상일 때만 BRANCH 를 믿는다.
    # 🎯 **40 → 30 으로 내린다** (2026-08-09 덤프 실측). 40mm 는 곡관·분기
    #    **안에 들어간** 프레임을 통째로 기각해 왔다 — 굽이 안에서는 카메라가
    #    안쪽 벽을 가까이 보므로 관경이 작게 읽히는 것이 기하학적으로 정상이다.
    #      진입 곡관 40장 : 32.6 ~ 41.4mm  → 40 기준에서 **11장만 통과**
    #      정면 T   15장 : 0 ~ 51.2mm
    #    이 값 하나가 곡관(밀고 들어가기 불발)과 T(진입 기동 불발) 둘 다를
    #    막고 있었다. 실측 최솟값 32.6 밑으로 내려 곡관 40/40 을 살린다.
    # 🚨 **이 값은 T/곡관을 못 가른다** — 낮추면 T 도 같이 통과한다(실측:
    #    32mm 에서 곡관 40/40·T 11/15). 종류 판별은 `branch_tee_arc_min_deg`
    #    (개구 폭)가 하고, 이 값은 "링 통계가 쓸 만한가"만 본다.
    # 40 → 30 → **20** (2026-08-09). 이제 이 값은 "링이 완전히 무의미한가" 만
    #    보는 최후 방어선이다 — 실측된 정상 최솟값이 28mm 이므로 그 아래에 둔다.
    branch_bore_sane_mm=20.0,
    # ── 원통 전제 (2026-08-08 사용자 설계 지시) ──────────────────────
    # 🎯 **관은 어디서나 원통이고, 기준 지름은 고정(DN100)이 아니라 추적값이다.**
    #    reducer 와 일반관은 지름만 다른 같은 원통 — DN100 상수에 묶으면
    #    지름이 넓어질 때 판정 전부가 착오를 일으킨다(사용자 진단). 링에서
    #    벽이 온전히 잡히는 프레임마다 기준 반경을 슬루 제한으로 따라간다.
    #    bore_dn100_mm 는 이제 **초기값**이다.
    bore_ref_slew_mm=2.0,       # 프레임당 기준 반경 이동 상한 (10Hz → 20mm/s;
    #                             테이퍼 4.76°×0.1m/s = 8mm/s 를 여유로 따라감)
    # BRANCH 신뢰 조건(벽 반경 정상)도 추적 기준에 비례시킨다 — DN150 에서
    # 고정 40mm 는 너무 느슨하다(벽 75 대비 절반). 고정값은 하한으로만 남긴다.
    # 🚨 0.80 → 0.62 (2026-08-09) — 위 하한을 30 으로 내려도 **이 비례항이
    #    상한을 잡아 무의미해진다**: DN100 기준 50mm 에서 0.80 이면 40mm 라
    #    바뀐 게 없다(`_sane = max(하한, 기준×frac)`). 곡관 실측 최솟값
    #    32.6mm 를 통과시키려면 기준 50 대비 0.65 미만이어야 한다.
    # 🚨 0.80 → 0.62 → **0.0** (2026-08-09). 비례항은 DN150 에서 문턱을 자동으로
    #    올리려던 것인데(벽 75 대비 40mm 는 느슨하다), DN100 굽이 안에서
    #    28~33mm 로 읽히는 **정상 프레임까지 기각**한다(기준 50 × 0.62 = 31).
    #    확관 방어는 `branch_arc_max_deg`(140°)와 다리 거부권이 맡는다 —
    #    위 `_sane` 주석의 근거 참조. 되살리려면 0.62 로 되돌린다.
    branch_bore_sane_frac=0.0,
    # 🎯 **T/곡관 판별 = 양옆 개구 유무** (2026-08-08 사용자 설계 지시).
    #    개구가 하나뿐이고 **직진로까지 막혔으면** 선택지가 없다 = 곡관.
    #    (T 는 양옆이 뚫렸거나(정면 T) 한쪽 개구 + 직진로 열림(측면 T)이다.)
    #    곡관은 발차기(BRANCH_ENTRY) 없이 센터링 조향으로 민다 — 사용자 규칙
    #    "왼발차기는 T 에서만". 직진로 열림 문턱; 곡관 접근 실측 fr 0.10~0.25,
    #    측면 T 접합부의 직진로(가로대 반대팔)는 ≥0.35 로 갈린다.
    front_open_m=0.30,
    # 🎯 **곡관 정면 접근의 전방 문턱** (2026-08-08, 실전 맵 실측 설계).
    #    긴 직관 끝의 굽이를 정면으로 보면 개구 윤곽이 거칠어(0.147) 변형 관
    #    검사에 걸려 UNDETERMINED 로 선다 — 그런데 개구가 아직 화면 중앙이라
    #    (입사 1°) 기존 곡관 예외(입사 ≥15°)가 안 받는다. 실전 맵 floor1
    #    출발점에서 그대로 사망했다.
    #    🔑 가르는 신호는 **전방 거리**다 (같은 잣대로 잰 실측):
    #        변형 관(합성 회귀 정답 UNDETERMINED) 0.257 / 0.308 / 0.339m
    #        굽이 정면(실전 맵 floor1 s520)          **0.153m**
    #      굽이는 시야를 막고, 변형된 직관은 여전히 멀리까지 보인다.
    #    0.20 = 두 무리의 가운데. 단절(0.210m·무효 4.7%)은 무효 화소 조건으로
    #    따로 막는다 — 이 규칙은 무효 <2%(단절 문턱) 일 때만 산다.
    curve_front_m=0.20,
    # 🎯 **닫힌 곡관 판별자** (2026-08-08 실전 맵, 덤프 114장 실측 설계).
    #    실전 맵의 곡관 코너에서 링이 "벽 없음 50%@폭80°" 를 내 T 로 오탐하고
    #    왼발차기가 나갔다(루프 주행 중 사망). 어제의 `branch_far_bore_ratio`
    #    (>2.6 = 곡관)는 **열린-끝 연습 곡관** 기준이라 이 맵에서는 반대로
    #    나온다 — 실전 곡관은 출구 너머에 관이 이어져 far 가 작다.
    #    🔑 같은 잣대로 잰 분포가 **완전히 갈린다**:
    #        새 맵 곡관   far/bore 1.13 ~ **1.15** (40장, 매우 좁게 뭉침)
    #        tee_go  (T)          **1.81** ~ 2.44 (40장)
    #        tee_back(T)          **1.74** ~ 1.81 (34장)
    #      곡관 최대 1.149 < T 최소 1.616 — 겹침 0.
    # 🚨 **코너마다 다르다** (2026-08-08, 2차 실측): 돌자마자 T 로 이어지는
    #    코너(730,1400)는 광선이 그 관 안으로 달아나 1.36~2.21 로 T 와 겹친다
    #    (메모리의 "긴 가지에서는 반경으로 원리적으로 못 가른다" 와 같은 한계).
    #    → 문턱은 **T 를 하나도 잃지 않는 상한**(T 최소 1.616)에서 안전여유를
    #      뺀 1.50 으로 둔다: T 74장 손실 0, 곡관 80장 중 44장이 곡관으로
    #      확정된다. 나머지 깜빡이는 프레임은 **장면 래치**(곡관을 한 번 보면
    #      2초간 발차기 금지, driver.curve_kick_block_s)가 흡수한다.
    #    물리도 맞는다: 곡관은 **바깥벽이 물러난 것**이라 거리가 관 반경의
    #    1.1배대에 머물고, T 는 광선이 **가지 관 안으로 들어가** 훨씬 멀다.
    branch_far_bore_min=1.50,
)


# 이 값보다 큰 유한값이 있으면 mm 로 온 것으로 본다. 관 안에서 5m 를 넘는
# 유효 깊이는 나올 수 없고, mm 라면 최소 수백이 나온다.
UNIT_THRESHOLD_M = 100.0


def normalize_depth(depth):
    """Depth 를 미터로 맞춘다. mm 로 오면 1/1000 한다.

    반환 (미터 depth, 변환했는가).

    🚨 **유한값만 보고 판단해야 한다.** Isaac Sim 은 빈 공간(관이 끊긴
    너머)을 `inf` 로 준다 — 2026-08-04 실기 실측으로 확정됐다. 그런데
    `np.nanmax` 는 NaN 만 무시하고 `inf` 는 그대로 돌려준다. 그래서

        nanmax(inf 포함) = inf  >  100  →  전 화소를 1000 으로 나눔
        관벽 50mm  →  0.05mm

    **단절 픽셀이 하나만 들어와도 Depth 전체가 1/1000 이 되고, 오프셋도
    1/1000 이 되어 무슨 일이 있어도 NORMAL 이 나온다.** 에러도 경고도 안
    난다. 실제로 `condition/node.py` 가 이 형태로 틀려 있었다.

    ROS 노드가 아니라 여기 있는 이유는 **오프라인에서 시험할 수 있어야 하기
    때문**이다. 노드 안에 있으면 rclpy 없이는 검증할 방법이 없다.
    """
    d = np.asarray(depth, dtype=np.float64)
    finite = d[np.isfinite(d)]
    if finite.size and float(finite.max()) > UNIT_THRESHOLD_M:
        return d / 1000.0, True
    return d, False


@dataclass
class Condition:
    state: str = UNDETERMINED
    circularity: float = 0.0
    roughness: float = 0.0
    forward_range_m: float = 0.0
    joint_range_m: float = 0.0
    incidence_deg: float = 0.0
    # 🎯 개구부 중심의 화면 방위 (2026-08-07, 센터링 조향용) — `branch_deg`
    #    와 같은 화소 규약(atan2(dy,dx), ±180°). incidence_deg 가 크기,
    #    이것이 방향이다. 함께 봐야 "관 축이 어느 쪽으로 벗어났나"가 된다.
    incidence_clock_deg: float = 0.0
    offset_mm: float = 0.0
    invalid_ratio: float = 0.0
    aperture_px: int = 0
    bore_mm: float = 0.0        # 실측 관 내반경 (0 = 못 쟀다)
    # 🎯 추적 기준 반경 (2026-08-08 원통 전제) — 판정·주행이 "지금 관"의
    #    반경으로 삼는 값. DN100 상수가 아니라 벽이 온전한 프레임마다
    #    슬루 제한으로 실측을 따라간다. 다리 스트로크 상한 등에 쓸 것.
    bore_ref_mm: float = 0.0
    branch_ratio: float = 0.0   # 중간 링에서 벽이 없는 화소 비율
    branch_deg: float = 0.0     # 제1 개구 방위 (화면 화소 좌표, 도)
    branch_arc_deg: float = 0.0  # 제1 개구 방위 폭
    # 🎯 2026-08-07 — **정면 T** 는 개구가 좌·우 양쪽에 동시에 열린다(본관에서
    #    가로대를 정면으로 만나는 장면). 단일 방위 평균은 양쪽이 상쇄돼 방위가
    #    널뛰고 방위폭이 상한을 넘겨 **아예 안 잡혔다**(실측). 개구를 방위
    #    클러스터로 갈라 큰 것부터 두 개까지 보고한다. 오른손 법칙(어느 것에
    #    들어갈지)은 검출기가 아니라 주행 제어의 몫이다.
    branch2_deg: float = 0.0    # 제2 개구 방위 (없으면 arc=0)
    branch2_arc_deg: float = 0.0
    branch_inf_frac: float = 0.0  # 결손 중 무효(inf) 비율 — 확관 판별자
    branch_far_p50_mm: float = 0.0  # far 화소 반경 중앙값 — 곡관 판별자
    # 곡관 접근으로 분류된 프레임 표시 — 센터링 조향이 이 프레임에만 강한
    # 이득을 쓴다(일반 프레임에 강한 이득을 쓰면 T 접합부 근처에서 과조향
    # 사출: 실측 이탈 294mm).
    curve_ahead: bool = False
    # 🎯 곡관 **확증** 표시 (2026-08-08) — 링에 양성 증거(far 분포 ×2.6, 또는
    #    단일 개구+직진로 막힘)가 있는 곡관만 True. 곡관 정면 접근(링 사각,
    #    분기 0%)은 정면 T 와 센서 신호가 같을 수 있는 **모호 장면**이라
    #    False 로 남는다 — 주행의 발차기 금지 래치는 이 값만 믿는다
    #    (curve_ahead 로 래치를 걸었더니 tee_back 2랩 재진입 발차기까지
    #    막아 이탈 80mm 사출, 실측).
    curve_sure: bool = False
    passable: bool = False
    speed: str = SPEED_STOP
    reason: str = ""

    def as_dict(self):
        return asdict(self)


class PipeConditionDetector:
    def __init__(self, intrinsics, params=None):
        self.k = dict(DEFAULTS)
        if params:
            self.k.update({a: b for a, b in params.items() if a in self.k})
        self.fx = float(intrinsics["fx"])
        self.fy = float(intrinsics["fy"])
        self.ppx = float(intrinsics["ppx"])
        self.ppy = float(intrinsics["ppy"])
        self.f_fish = float(intrinsics.get("f_fish", self.fx))
        # 원통 전제 — 기준 반경 추적 상태. 초기값은 DN100(투입관 규격).
        self.bore_ref = float(self.k["bore_dn100_mm"])

    def incidence_angle(self, cx, cy):
        """개구부 중심 픽셀 -> 광축에서 벌어진 각(rad).

        어안은 등거리 r = f*theta, 핀홀은 theta = atan(r/f) 로 서로 다르다.
        140도 어안에서 핀홀 식을 쓰면 가장자리로 갈수록 크게 틀어진다.
        """
        r = np.hypot((cx - self.ppx), (cy - self.ppy))
        if self.k["projection"] == "fisheye_equidistant":
            return float(r / self.f_fish)
        return float(np.arctan2(r, self.fx))

    def invalid_mask(self, depth):
        # normalize_depth 와 짝이다. 여기서 무효로 보는 것과 거기서 최대값
        # 계산에서 빼는 것이 같아야 한다.
        if self.k["invalid_mode"] == "empty_is_far":
            return ~np.isfinite(depth) | (depth <= 0) | (
                depth > self.k["max_range_m"])
        return ~np.isfinite(depth) | (depth <= 0)

    def aperture(self, depth, invalid):
        """전방 개구부 = 무효 픽셀 + 먼 쪽 유효 픽셀. Otsu 로 원근을 가른다."""
        valid = ~invalid
        far = np.zeros(depth.shape, np.uint8)
        if valid.sum() > 100:
            d = depth[valid]
            lo, hi = float(d.min()), float(d.max())
            if hi - lo > 1e-4:
                norm = np.zeros(depth.shape, np.uint8)
                norm[valid] = np.clip((depth[valid] - lo) / (hi - lo) * 255,
                                      0, 255).astype(np.uint8)
                t, _ = cv2.threshold(norm[valid], 0, 255,
                                     cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                far = ((norm >= t) & valid).astype(np.uint8)
        mask = (far | invalid.astype(np.uint8)) * 255
        k = int(self.k["morph_ksize"])
        if k > 1:
            el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, el)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, el)
        return mask

    def ring_scan(self, depth, invalid):
        """중간 시야 링에서 **관경**과 **벽이 사라진 방위**를 같이 잰다.

        반환 `(bore_mm, branch_ratio, branch_deg, branch_arc_deg)`.

        🔑 어안 등거리(ρ = f·θ)라 화면 반경이 곧 광축과의 각이다. 그 각 θ 로
           나간 광선이 관벽을 거리 d 에서 만나면 내반경은 **R = d·sinθ**.
        🚨 전방 개구부(관 저 끝)는 θ 가 작아 이 링에 안 들어온다 — 그래서
           "앞이 뚫린 것"과 "옆이 뚫린 것"이 섞이지 않는다. 이것이 분기를
           `DISCONNECTED` 와 가르는 핵심이다.
        """
        h, w = depth.shape
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float64)
        dx, dy = xx - self.ppx, yy - self.ppy
        rho = np.hypot(dx, dy)
        th = rho / max(self.f_fish, 1e-9)                  # 라디안
        t0 = np.radians(self.k["bore_ring_deg"])
        tb = np.radians(self.k["bore_ring_band_deg"]) * 0.5
        ring = (th > t0 - tb) & (th < t0 + tb)
        if ring.sum() < 50:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        wall = ring & (~invalid)
        # 관경 — 벽이 잡힌 화소에서만
        bore = 0.0
        bore_lo = 0.0
        if wall.sum() > 20:
            R = depth[wall] * np.sin(th[wall])
            R = R[np.isfinite(R) & (R > 0)]
            if R.size > 20:
                bore = float(np.median(R)) * 1000.0
                # 🚨 far 임계 기준은 **하위 분위수(p35)** — 접합부 접근에서
                #    개구 광선이 중앙값을 부풀리면(실측 47→56mm) 1.6×bore
                #    문턱이 가지 벽 명중거리(93mm) 위로 올라가 **개구를 통째로
                #    벽으로 삼킨다**(분기 0% → 진입 불발 → 직진 이탈, 2랩째
                #    실측). 다수 화소는 여전히 진짜 벽이라 p35 는 안 부푼다.
                #    DN150 확관은 전 화소가 정직하게 크므로 p35 도 커서(≈70)
                #    reducer 오탐 방지(1.6×70=112 > 벽 90)가 그대로 유지된다.
                bore_lo = float(np.percentile(R, 35)) * 1000.0
        # 벽이 사라진 방위 — 무효이거나, 벽 거리가 기준보다 훨씬 먼 화소
        gone = ring & invalid
        # 🚨 far 임계는 1.6×bore (분기 검출이 실증된 값). 고정값(90·120mm)은
        #    분기 광선(60~90mm)과 DN150 벽(75mm)이 겹쳐 **원리적으로 못
        #    가른다** — 90 은 분기 실검출을 죽였고 120 도 마찬가지(실측).
        #    확관 오탐은 여기서가 아니라 **다리(고유수용) 거부권**으로 막는다
        #    (driver.step 의 suppress_branch — 확관은 다리가 같이 뻗는다).
        far_p50 = 0.0
        if bore_lo > 0:
            far = ring & (~invalid) & (
                depth * np.sin(th) > (bore_lo / 1000.0) * 1.6)
            gone = gone | far
            # 🎯 곡관/분기 판별용 계측 (2026-08-07, 덤프 80장 실측 설계) —
            #    far(유한 원거리) 화소의 반경 중앙값. 분기는 개구 광선이
            #    가지 관 벽을 좁은 대역에서 맞아 bore×~1.9 에 몰리고(실측
            #    93mm/49, p10~p90 폭 22mm), 곡관은 멀어진 벽이 연속적으로
            #    물러나 bore×~3.9 까지 퍼진다(실측 184mm/47, 폭 139mm).
            if far.sum() > 20:
                Rf = (depth * np.sin(th))[far]
                Rf = Rf[np.isfinite(Rf)]
                if Rf.size > 20:
                    far_p50 = float(np.median(Rf)) * 1000.0
        ratio = float(gone.sum()) / float(ring.sum())
        inf_frac = (float((ring & invalid).sum()) / float(gone.sum())
                    if gone.sum() > 0 else 0.0)
        # 🚨 단일 원형 평균은 **정면 T(양쪽 개구)** 에서 무너진다 — 좌우가
        #    상쇄돼 방위가 널뛰고(±178° 요동 실측) 폭이 상한을 넘는다.
        #    → 10° 방위 빈으로 히스토그램을 만들고, 결손 비율이 높은 빈의
        #      **연속 구간(원형)** 을 개구 하나로 센다. 큰 것부터 두 개 보고.
        openings = []
        if gone.sum() > 20:
            nb = 36
            az_all = np.degrees(np.arctan2(dy[ring], dx[ring])) % 360.0
            az_gone = np.degrees(np.arctan2(dy[gone], dx[gone])) % 360.0
            tot, _ = np.histogram(az_all, bins=nb, range=(0, 360))
            gn, _ = np.histogram(az_gone, bins=nb, range=(0, 360))
            open_bin = (gn > 0.5 * np.maximum(tot, 1)) & (tot > 3)
            # 원형 연속 구간 찾기 — 시작점이 열린 빈이면 한 바퀴 돌려 붙인다
            runs, k = [], 0
            while k < nb:
                if open_bin[k]:
                    j = k
                    while open_bin[(j + 1) % nb] and j - k < nb - 1:
                        j += 1
                    runs.append((k, j))
                    k = j + 1
                else:
                    k += 1
            if len(runs) >= 2 and runs[0][0] == 0 and runs[-1][1] == nb - 1:
                a0, b0 = runs.pop()
                runs[0] = (a0 - nb, runs[0][1])       # 0° 걸친 구간 병합
            for a0, b0 in runs:
                width = (b0 - a0 + 1) * (360.0 / nb)
                mid = ((a0 + b0) / 2.0 + 0.5) * (360.0 / nb) % 360.0
                mid = (mid + 180.0) % 360.0 - 180.0   # ±180 표기
                openings.append((width, mid))
            openings.sort(reverse=True)
        o1 = openings[0] if openings else (0.0, 0.0)
        o2 = openings[1] if len(openings) > 1 else (0.0, 0.0)
        return bore, ratio, o1[1], o1[0], o2[1], o2[0], inf_frac, far_p50

    def _rim_depth(self, depth, mask, invalid, band=9):
        """개구부 테두리 바깥 띠의 깊이 중앙값 = 조인트 평면까지 거리."""
        el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (band, band))
        ring = (cv2.dilate(mask, el) > 0) & (mask == 0) & (~invalid)
        vals = depth[ring]
        vals = vals[np.isfinite(vals) & (vals > 0)]
        return float(np.median(vals)) if vals.size > 20 else 0.0

    def run(self, depth, joint_angle_deg=0.0, rgb=None):
        c = Condition()
        depth = np.asarray(depth, dtype=np.float64)
        if self.k["blur_ksize"] > 1:
            depth = cv2.medianBlur(depth.astype(np.float32),
                                   int(self.k["blur_ksize"])).astype(np.float64)

        invalid = self.invalid_mask(depth)
        c.invalid_ratio = float(invalid.mean())

        pf = self.k["center_patch_frac"]
        h, w = depth.shape
        y0, y1 = int(h * (0.5 - pf)), int(h * (0.5 + pf))
        x0, x1 = int(w * (0.5 - pf)), int(w * (0.5 + pf))
        patch = depth[y0:y1, x0:x1]
        pv = patch[np.isfinite(patch) & (patch > 0)]
        c.forward_range_m = float(np.median(pv)) if pv.size else 0.0

        # 🎯 관경·분기는 **판정 전에 항상 재 둔다** — 어느 갈래로 빠져나가도
        #    주행 쪽에서 쓸 수 있어야 한다(다리 스트로크·조향).
        (c.bore_mm, c.branch_ratio, c.branch_deg, c.branch_arc_deg,
         c.branch2_deg, c.branch2_arc_deg,
         c.branch_inf_frac, c.branch_far_p50_mm) = self.ring_scan(depth,
                                                                  invalid)

        # 🎯 원통 전제 — 기준 반경 추적 (2026-08-08). 링에서 벽이 온전히
        #    잡힌 프레임(결손 비율이 분기 문턱 미만)만 신뢰하고, 프레임당
        #    슬루 상한으로 실측 반경을 따라간다. reducer 테이퍼(8mm/s)는
        #    따라가고, 개구·자기몸체 오염 프레임은 기준을 못 흔든다.
        if c.bore_mm > 0 and c.branch_ratio < self.k["branch_ratio_min"]:
            _slew = self.k["bore_ref_slew_mm"]
            self.bore_ref += float(np.clip(c.bore_mm - self.bore_ref,
                                           -_slew, _slew))
        c.bore_ref_mm = self.bore_ref

        # 🎯 개구부 중심(≈전방 관 축 방향)도 **판정 전에 항상 재 둔다**
        #    (2026-08-07, 센터링 조향). 꼬리 갈래에서만 재면 곡관 예외·BRANCH
        #    조기 리턴이 사실상 매 프레임이라(관절이 물러 직관에서도 8° 를
        #    넘게 처진다) 실기에서 incidence 가 거의 항상 0 이었다.
        #    🚨 `aperture_px` 는 여기서 채우지 않는다 — 속도 법칙(openness)이
        #    그 값을 쓰므로, 조기 리턴 갈래에 채우면 감속 거동이 변한다.
        mask = self.aperture(depth, invalid)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_NONE)
        cnt = max(cnts, key=cv2.contourArea) if cnts else None
        area = float(cv2.contourArea(cnt)) if cnt is not None else 0.0
        peri = float(cv2.arcLength(cnt, True)) if cnt is not None else 0.0
        if cnt is not None and area >= self.k["aperture_min_px"] \
                and peri > 1e-6:
            (acx, acy), _ = cv2.minEnclosingCircle(cnt)
            c.incidence_deg = float(np.degrees(
                self.incidence_angle(acx, acy)))
            c.incidence_clock_deg = float(np.degrees(
                np.arctan2(acy - self.ppy, acx - self.ppx)))

        # 🚨 **분기는 곡관 예외보다도 먼저 본다** (2026-08-07 실측 수정).
        #    T 접합부에서는 로봇이 이미 꺾여 들어가는 중이라 관절이 8° 를
        #    넘는데, 곡관 예외를 먼저 두면 "곡관 — 판정 생략" 으로 빠져나가
        #    분기 39% 짜리 장면이 NORMAL 로 나왔다. 옆 링(θ≈50°)의 벽 결손은
        #    곡관에서는 안 생기므로(벽이 휘어질 뿐 사라지지 않는다) 분기
        #    판정을 관절 각도로 가릴 이유가 없다.
        #    같은 이유로 **단절보다도 먼저** 본다 — T 개구가 만드는 무효
        #    화소가 DISCONNECTED(정지·복귀) 오판을 만든다. 가르는 근거:
        #    단절은 **앞(광축)** 이 비고, 분기는 **옆**이 빈다. θ≈50° 링에는
        #    앞쪽 개구가 안 들어온다.
        # 🎯 **곡관 판별자** (2026-08-07, 덤프 실측 설계). 분기의 far 반경은
        #    가지 관 벽에 몰려 bore×1.9(93mm/49, 폭 22mm), 곡관 접근은 벽이
        #    연속으로 물러나 bore×3.9(184mm/47, 폭 139mm) — ×2.6 으로 가른다.
        #    ⚠ **유효 범위: 가지 길이가 짧은(245mm) 현 코스 한정.** 가지에
        #    600mm 연장을 붙이면 개구 광선이 관 안으로 달아나 곡관과 동일
        #    분포가 되어 진짜 분기를 죽인다(같은 날 실측·연장은 원복됨).
        #    긴 가지 실전 배관에서는 이 판별자를 재설계해야 한다.
        # 🚨 (2026-08-07 심야 왕복 실측 기록) inf 가드는 **양쪽 다 못 간다**:
        #    - "inf 위주 = 분기로 둔다"(현행) → 출구 너머가 허공인 곡관 배치
        #      에서 곡관에 BRANCH 가 붙어 왼발차기가 나간다(사용자 규칙 위반).
        #    - "inf 위주 = 곡관"(반전 실험) → 정면 T 접근(반대팔 너머로 광선이
        #      빠져 inf)까지 곡관으로 삼켜 tee_go 진입 불발·사망.
        #    θ50° 링 하나로는 원리적으로 못 가르는 지점 — 현행 유지가 전
        #    코스 통과를 보장한다. 곡관 발차기 근절은 별도 설계 필요(후보:
        #    곡관 출구에 연속관 스텁 = 열린-끝 광학 아티팩트 제거, 또는
        #    장면 래치 분류기).
        # 🚨 **기본 OFF** (2026-08-09 덤프 실측 — 이 규칙은 실전 맵에서 방향이
        #    거꾸로다). "far 가 관경의 2.6배를 넘으면 곡관" 은 **열린-끝 연습
        #    곡관**(출구 너머가 허공이라 far 가 무한대로 커짐) 기준이었다.
        #    실전 맵은 관이 이어져 있어 정반대로 나온다:
        #        진입 곡관 40장 : far/관경 1.62 ~ **1.71**
        #        정면 T    8장 : **2.68** ~ 4.14   ← 광선이 가지 관으로 들어간다
        #    즉 **큰 far 는 T 의 표식**인데 이 규칙이 그것을 곡관이라 부른다.
        # 🔑 2026-08-08 에 이미 "실전 맵에서 반대로 나온다"를 확인하고 새
        #    판별자(`_wall_receded`)를 만들었는데, **옛 규칙을 지우지 않고 OR 로
        #    묶어 둔** 탓에 새 판별자가 맞아도 옛 규칙이 T 를 통과시켰다.
        #    실측: 정면 T 15장 중 **8장이 이 경로로 곡관이 됐고**, 거기서
        #    밀기(curve_push)가 발동해 로봇이 관 밖 701mm 로 튕겨 나갔다.
        # ⚠ 연습장(`map_test_demo`)의 열린-끝 곡관은 이 규칙에 기대고 있었을 수
        #   있다. 되살릴 일이 있으면 `branch_far_curve_on=True` 로 켠다 —
        #   **단, 그 코스에서 T 덤프와 나란히 재고 나서.**
        _curve_like = (self.k["branch_far_curve_on"]
                       and c.branch_far_p50_mm > 0
                       and c.branch_inf_frac < 0.5
                       and c.branch_far_p50_mm
                       > c.bore_mm * self.k["branch_far_bore_ratio"])
        # 🎯 **벽이 물러났을 뿐 = 닫힌 곡관** (2026-08-08 실측, 위 상수 주석).
        #    T 는 광선이 가지 관으로 들어가 far 가 bore 의 1.6배를 넘는다.
        _wall_receded = (c.branch_far_p50_mm > 0 and c.bore_mm > 0
                         and c.branch_far_p50_mm
                         < c.bore_mm * self.k["branch_far_bore_min"])
        # 벽 반경 정상 문턱 — 고정 40mm 는 하한, 기준 반경의 80% 를 따라간다
        # (원통 전제: DN150 에서 40mm 는 벽 75 의 절반이라 너무 느슨하다).
        # 🎯 **"관이 크게 읽히나" 로 신뢰를 재던 것을 그만둔다** (2026-08-09).
        #    이 문턱의 목적은 주석대로 *"자기 몸체가 시야를 가려 링 통계가
        #    무너진 프레임을 거르는 것"* 인데, 그것을 **관경의 크기**로 대신
        #    재고 있었다. 굽이·분기 **안에서는 관경이 작게 읽히는 것이
        #    기하학적으로 정상**이라 이 대리 지표가 정상 프레임을 기각한다.
        # 🔑 실측이 "자기 몸 가림이 아니다" 라고 말한다 (진입 곡관 덤프 40장):
        #      · 무효 화소 **0%** — 관을 멀쩡히 보고 있다
        #      · 관경이 로봇이 전진하는 대로 **43→39→33→29→28mm 로 변한다**
        #        → 자기 몸은 강체로 붙어 있어 **거리가 안 변한다.** 변한다는
        #          것 자체가 진짜 벽을 보고 있다는 증거다.
        #    문턱을 40→30 으로 내려도 로봇이 그만큼 더 들어가 28mm 로 또
        #    걸렸다 — 값을 쫓아 내리는 것은 끝이 없다(실측 2회).
        # 🚨 **DN150 은 이 문턱이 아니라 다른 둘이 지킨다** (사용자 지적으로
        #    확인): ① 확관은 벽이 사방에서 고르게 멀어져 개구 방위 폭이 커지므로
        #    `branch_arc_max_deg`(140°)가 잡는다. ② 다리가 같이 뻗으므로
        #    `driver` 의 고유수용 거부권(`suppress_branch`)이 BRANCH 를 무효화
        #    한다. 반면 **다리로 중심을 잡는 동작 자체는 카메라와 무관하다** —
        #    `force_legs()` 는 관절 위치·속도만 쓰고 벽에 닿을 때까지 뻗는다.
        #    → 그래도 비례항을 없앤 것은 확관 판정을 헐겁게 만드는 변경이므로
        #      **리듀서 코스(`maps/reducer_dn100_dn150_dn100.usda`)로 확인할 것.**
        #      되돌리려면 `branch_bore_sane_frac` 을 0.62 로 되살린다.
        _sane = max(self.k["branch_bore_sane_mm"],
                    self.bore_ref * self.k["branch_bore_sane_frac"])
        _openingish = (c.branch_ratio > self.k["branch_ratio_min"]
                       and c.bore_mm >= _sane
                       and self.k["branch_arc_min_deg"] < c.branch_arc_deg
                       < self.k["branch_arc_max_deg"])
        # 🚨 곡관으로 가른 프레임을 그냥 흘려보내면 안 된다 — 개구 무게중심이
        #    화면 밖으로 크게 벗어나 MISALIGNMENT(통행 불가)로 빠지고 HOLD 로
        #    영영 선다(실측: elbow s=180 입사 31° HOLD 사망). **곡관 접근은
        #    통행 가능·감속**이다 — 조향(센터링)이 입사각을 보고 꺾어 들어간다.
        # 🎯 **T/곡관 판별 = 양옆 개구 유무** (2026-08-08 사용자 설계 지시).
        #    T 는 선택지가 있는 구조다 — 양옆이 다 뚫렸거나(정면 T), 한쪽
        #    개구 + 직진로 열림(측면 T). **개구가 하나뿐인데 직진로마저
        #    막혔으면 선택지가 없다 = 곡관** — 그 개구가 유일한 길이므로
        #    발차기(BRANCH)가 아니라 곡관 접근(센터링 조향)으로 분류한다.
        #    far 분포 판별자(×2.6)가 못 가르던 열린-끝 허공 곡관(far=inf 라
        #    _curve_like 불성립 → BRANCH 오탐 → 곡관 발차기, 어제 미해결)을
        #    이 구조 규칙이 잡는다.
        _two_sided = c.branch2_arc_deg > 0.0
        # 🎯 **T/곡관 판별의 정본 = 제1 개구의 폭** (2026-08-09 덤프 실측 확정).
        #    같은 잣대로 두 곳을 나란히 재서 얻은 값이다 —
        #      실전 맵 진입 곡관 40장 : 제1 폭 20 ~ **40°**  (중앙 30)
        #      실전 맵 정면 T   15장 : 제1 폭 **50°** ~ 110° (중앙 80)
        #    **한 장도 안 겹친다.** 경계 45° 에서 55장 전부 옳게 갈린다.
        #    물리도 맞는다 — T 는 가지 관이 통째로 뚫려 개구가 넓고, 곡관은
        #    바깥벽이 물러난 좁은 띠다.
        # 🚨 **둘째 개구 폭으로 가르려던 시도는 실측으로 기각됐다**(같은 날,
        #    같은 덤프). 둘째 폭이 곡관 10~20° / T **10~60°** 로 크게 겹쳐,
        #    "둘째가 25° 미만이면 곡관" 으로 두었더니 **T 15장 중 9장이 곡관으로
        #    표시**됐고 그 프레임에서 밀기(curve_push)가 벽을 들이받아 로봇이
        #    관 밖 701mm 로 튕겨 나갔다(실측). 곡관 덤프만 보고 정한 값이었다.
        # ⚠ 근거는 **진입 곡관 한 곳 + 정면 T 한 곳**의 55장이다. 루프 안쪽
        #   곡관들은 아직 같은 잣대로 안 쟀다 — 전 구간 주행으로 확인할 것.
        _narrow_opening = c.branch_arc_deg < self.k["branch_tee_arc_min_deg"]
        _fwd_blocked = 0.0 < c.forward_range_m < self.k["front_open_m"]
        if _openingish and (_curve_like or _wall_receded or _narrow_opening
                            or (not _two_sided and _fwd_blocked)):
            c.state, c.passable, c.speed = NORMAL, True, SPEED_SLOW
            c.curve_ahead = True
            c.curve_sure = True
            # 🚨 개방도(aperture_px)를 실어 준다 — 안 실으면 속도 법칙이
            #    바닥값(0.25)을 물어 v≈2mm/s 로 정지 마찰에 진다(실측).
            c.aperture_px = int(area)
            c.reason = ((f"곡관 접근 — 개구 폭 {c.branch_arc_deg:.0f}° < "
                         f"{self.k['branch_tee_arc_min_deg']:.0f}° (T 는 50° 이상)")
                        if _narrow_opening else
                        (f"곡관 접근 — far {c.branch_far_p50_mm:.0f}mm > "
                         f"{self.k['branch_far_bore_ratio']:.1f}×관경"
                         f"{c.bore_mm:.0f}") if _curve_like else
                        (f"곡관 접근 — 단일 개구 {c.branch_deg:+.0f}° + "
                         f"직진로 막힘 {c.forward_range_m:.2f}m = 선택지 없음"))
            return c
        if _openingish:
            # (클러스터 폭은 실제 연속 폭이다 — 정면 T 의 좌·우 개구는 각각
            #  60~90° 로 상한 140° 안에 들어와 이제 잡힌다)
            # 🚨 분기 중 관경은 **믿을 수 없다** — 링의 상당수가 벽이 아니라
            #    개구 자체를 보므로 중앙값이 무너진다(실측 14mm). 0 으로
            #    지워서 "못 쟀다" 를 명시한다. 관경이 필요한 쪽은 마지막
            #    유효값을 유지할 것.
            c.bore_mm = 0.0
            c.state, c.passable, c.speed = BRANCH, True, SPEED_SLOW
            c.reason = (f"분기 개구 — 링의 {c.branch_ratio * 100:.0f}% 가 벽 없음, "
                        f"방위 {c.branch_deg:+.0f}° 폭 {c.branch_arc_deg:.0f}°")
            return c

        # 🎯 **곡관 정면 접근** (2026-08-08 CTL_DEBUG 계측 설계) — 개구가 링
        #    사각이라 안 보여도(elbow_v 실측: 분기 0%), 전방이 막혔고(0.12m)
        #    관경이 기준과 정상 일치(46/46)하며 개구부 원반이 축에서 크게
        #    벗어나(입사 21°) 있으면 굽이가 바로 앞이다. curve_ahead 를 줘야
        #    센터링이 곡관 이득(1.2)으로 꺾어 들어가고 front 트리거(정면 T)
        #    타이머도 안 찬다. 진짜 정면 T 정지점은 관경이 팽창(61~127mm,
        #    ≥1.22×기준)이라 "관경 정상" 조건에 안 걸린다(덤프 40장).
        # 🚨 문턱은 `curve_front_m`(0.20) 이지 `front_open_m`(0.30) 이 아니다 —
        #    후자는 "T 의 직진로가 열렸나" 용이라 변형 관(0.257~)까지 삼킨다.
        # 🚨 무효 화소는 **단절 문턱(2%) 미만**일 때만 — 단절 장면이
        #    (전방 0.210m·무효 4.7%) 곡관으로 새는 것을 막는다.
        # 🚨 관경 ±8mm 조건이 **정면 T 를 배제한다** — 가로대 공동이 잡혀
        #    관경이 1.22배 이상 팽창하므로(덤프 40장) 이 창에 안 들어온다.
        if (0.0 < c.forward_range_m < self.k["curve_front_m"]
                and c.bore_mm > 0
                and abs(c.bore_mm - self.bore_ref)
                <= self.k["bore_change_mm"]
                and c.invalid_ratio < self.k["invalid_ratio_max"]
                and c.branch2_arc_deg <= 0.0
                # 🚨 **개구 폭 잣대를 여기에도 건다** (2026-08-09 덤프 실측).
                #    이 경로는 "개구가 링 사각이라 안 보이는" 곡관용이라 폭을
                #    안 봤는데, 그 탓에 **정면 T 15장 중 8장이 곡관으로 샜다**
                #    (관경이 기준과 비슷해 ±8mm 창을 통과). 연습장 T 는 관경이
                #    1.22배 팽창해 이 창에 안 걸렸지만 실전 맵 T 는 안 그렇다.
                #    개구가 T 폭(≥45°)으로 보이면 곡관일 수 없다.
                and c.branch_arc_deg < self.k["branch_tee_arc_min_deg"]):
            c.state, c.passable, c.speed = NORMAL, True, SPEED_SLOW
            c.curve_ahead = True
            if area > 0:
                c.aperture_px = int(area)
            c.reason = (f"곡관 접근(정면) — 전방 {c.forward_range_m:.2f}m 막힘"
                        f", 관경 {c.bore_mm:.0f}≈기준, 입사 {c.incidence_deg:.0f}°")
            return c

        if abs(joint_angle_deg) >= self.k["bend_angle_deg"]:
            c.state, c.passable, c.speed = NORMAL, True, SPEED_FULL
            # 🎯 **곡관 표시를 붙인다** (2026-08-09 — 상태줄 계측이 바로 잡아낸
            #    구멍). 이 경로는 "관절이 이만큼 꺾였으면 곡관이 확실하다"고
            #    **이름표까지 붙여 놓고** `curve_ahead` 를 안 세웠다. 그래서
            #    곡관 전용 동작(센터링 곡관 이득, `driver` 의 밀고 들어가기)이
            #    전부 발동하지 않았다.
            # 🔑 실측(floor1 진입 수직 곡관, 조향 0): 관절이 벽에 밀려 20.1° 로
            #    꺾여 이 경로에 들어왔는데, 표시가 없어 전방 정지 게이트가
            #    `forward=0.0` 으로 v=0 을 물었고 15초 무진행으로 사망했다.
            #      게이트{openness 0.25, curve 0.95, condition 1.0, **forward 0.0**}
            #      사유[곡관 (관절 20.1°) — 판정 생략]
            #    관절이 그만큼 꺾였다는 것은 **벽이 이미 몸을 꺾고 있다**는
            #    뜻이라 곡관 확증으로 충분하다(문턱 `bend_angle_deg`).
            c.curve_ahead = True
            c.curve_sure = True
            c.reason = f"곡관 (관절 {joint_angle_deg:.1f}°) — 판정 생략"
            return c

        # 관경 변화 — 벽은 온전한데 반경이 **추적 기준**에서 벗어났다.
        # 🎯 원통 전제(2026-08-08): 기준이 DN100 상수가 아니라 추적값이므로
        #    DN150 직관 구간은 기준이 75 로 수렴해 NORMAL 이 된다 — reducer 는
        #    "지름이 다른 같은 원통"이지 상시 경계 대상이 아니다(사용자 지시).
        #    BORE_CHANGE 는 슬루(20mm/s)보다 빨리 변하는 과도 구간에만 뜬다.
        if (c.bore_mm > 0
                and abs(c.bore_mm - self.bore_ref)
                > self.k["bore_change_mm"]):
            c.state, c.passable, c.speed = BORE_CHANGE, True, SPEED_SLOW
            c.reason = (f"관경 {c.bore_mm:.0f}mm "
                        f"(추적 기준 {self.bore_ref:.0f}mm)")
            return c

        if c.invalid_ratio > self.k["invalid_ratio_max"]:
            c.state, c.passable, c.speed = DISCONNECTED, False, SPEED_STOP
            c.reason = (f"무효 픽셀 {c.invalid_ratio * 100:.1f}% > "
                        f"{self.k['invalid_ratio_max'] * 100:.1f}%")
            return c

        # (mask·cnt·area·peri 는 위 상시 계측에서 이미 계산했다 — 재사용)
        if cnt is None:
            c.reason = "개구부 윤곽 없음"
            return c
        c.aperture_px = int(area)
        if area < self.k["aperture_min_px"] or peri < 1e-6:
            c.reason = f"개구부 과소 {area:.0f}px"
            return c

        c.circularity = float(4.0 * np.pi * area / (peri ** 2))
        # 원형도는 둘레에 좌우돼 완만한 요철에 둔감하다. 윤곽 반경의 상대
        # 표준편차가 같은 요철을 훨씬 민감하게 잡는다(합성 장면 실측).
        pts = cnt.reshape(-1, 2).astype(np.float64)
        rad = np.hypot(*(pts - pts.mean(0)).T)
        c.roughness = float(rad.std() / rad.mean()) if rad.mean() > 1e-9 else 1.0
        # 🎯 곡관 예외 (2026-08-07, elbow_v 첫 vision 주행 실측) — 곡관을
        #    정면으로 보면 개구가 **타원/초승달**이라 원형도·거칠기 검사에
        #    걸려 UNDETERMINED(정지)로 영영 선다(출발 HOLD 사망). 관벽이
        #    정상(DN100 대역)이고 무효 화소가 적으면 단절·붕괴가 아니라
        #    **관이 휘어 보이는 것** — 통행 가능·감속으로 분류하고 조향
        #    (센터링)이 입사각을 보고 꺾어 들어가게 한다.
        # 🚨 변형 관과의 분리는 **입사각** — 곡관은 개구가 축에서 크게 밀려
        #    난다(elbow_v 실측 27°). 변형 관은 개구가 중앙(입사≈0)인 채
        #    모양만 찌그러진다(합성 회귀 3장면이 이 차이를 잡았다). 중앙에
        #    있는 변형은 기존대로 UNDETERMINED 정지.
        _shape_bad = (c.circularity < self.k["circularity_min"]
                      or c.roughness > self.k["roughness_max"])
        if _shape_bad and abs(c.bore_mm - self.bore_ref) <= \
                self.k["bore_change_mm"] and c.invalid_ratio < 0.05 \
                and abs(c.incidence_deg) >= 15.0:
            c.state, c.passable, c.speed = NORMAL, True, SPEED_SLOW
            c.curve_ahead = True
            c.aperture_px = int(area)
            c.reason = (f"곡관 접근(개구 변형) — 원형도 {c.circularity:.2f} "
                        f"관경 {c.bore_mm:.0f}mm 정상")
            return c
        if c.circularity < self.k["circularity_min"]:
            c.state, c.passable, c.speed = UNDETERMINED, False, SPEED_STOP
            c.reason = (f"원형도 {c.circularity:.3f} < "
                        f"{self.k['circularity_min']:.2f}")
            return c
        if c.roughness > self.k["roughness_max"]:
            c.state, c.passable, c.speed = UNDETERMINED, False, SPEED_STOP
            c.reason = (f"윤곽 거칠기 {c.roughness:.4f} > "
                        f"{self.k['roughness_max']:.4f}")
            return c

        # 역투영 거리는 화면 중앙 깊이(먼 관 안쪽)가 아니라 개구부가 놓인
        # 조인트 평면까지의 거리다. 개구부 테두리 바깥 띠의 깊이가 그 값이다.
        c.joint_range_m = self._rim_depth(depth, mask, invalid)
        rng = c.joint_range_m if c.joint_range_m > 0 else c.forward_range_m
        # depth 는 distance_to_camera(실제 거리)이므로 횡방향 성분은 D*sin(theta).
        # 투영 방식은 theta 를 구하는 단계에만 들어간다.
        # (incidence 각·방위는 위 상시 계측에서 이미 c 에 들어 있다)
        theta = np.radians(c.incidence_deg)
        c.offset_mm = float(rng * np.sin(theta) * 1000.0)

        if c.offset_mm <= self.k["offset_normal_mm"]:
            c.state, c.passable, c.speed = NORMAL, True, SPEED_FULL
        elif c.offset_mm <= self.k["offset_slow_mm"]:
            c.state, c.passable, c.speed = MISALIGNMENT, True, SPEED_FULL
        elif c.offset_mm <= self.k["offset_max_mm"]:
            c.state, c.passable, c.speed = MISALIGNMENT, True, SPEED_SLOW
        else:
            c.state, c.passable, c.speed = MISALIGNMENT, False, SPEED_STOP
        c.reason = f"오프셋 {c.offset_mm:.2f} mm"
        return c
