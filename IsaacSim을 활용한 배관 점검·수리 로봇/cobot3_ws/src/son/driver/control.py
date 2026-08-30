"""[공용] 주행 제어 — ROS 비의존 순수 로직.

이 로봇에는 조향이 없다. 관절은 수동이고 cmd_vel 은 linear.x 만 쓴다.
6륜이 관벽에 압착되어 횡방향은 기계가 이미 잡아주므로, 제어기가 정하는 것은
**전진 속도 하나**다. 그래서 여기 있는 것은 조향 PID 가 아니라 속도 법칙이다.

  v = V_MAX x 전방개방도 x 곡관감속 x 상태게이트

여기에 FSM 과 끼임 판정을 얹는다. 끼임은 **바퀴는 도는데 실제로 안 나가는 것**
이므로 휠 오도메트리와 시각 오도메트리를 비교해야 알 수 있다.
"""

import math
import os

from dataclasses import dataclass, asdict, field

# 진입 트리거 계측 (CTL_DEBUG=1) — "어느 경로가, 어떤 판정값으로 발차기를
# 쐈는가"를 트리거 순간에 찍는다. 2026-08-08 elbow_v 오발 규명용으로 신설;
# 켜지 않으면 아무것도 안 한다.
_CTL_DEBUG = os.environ.get("CTL_DEBUG", "0") == "1"


def _dbg_cond(tag, cond, extra=""):
    if not _CTL_DEBUG:
        return
    c = cond or {}
    print(f"[CTL] {tag} | 상태 {c.get('state')} 관경 {c.get('bore_mm', 0):.0f}"
          f"/기준 {c.get('bore_ref_mm', 0):.0f}mm 분기 "
          f"{(c.get('branch_ratio', 0) or 0) * 100:.0f}%@{c.get('branch_deg', 0):+.0f}°"
          f"(폭{c.get('branch_arc_deg', 0):.0f}°) 2차@{c.get('branch2_deg', 0):+.0f}°"
          f"(폭{c.get('branch2_arc_deg', 0):.0f}°) inf {c.get('branch_inf_frac', 0):.2f} "
          f"far {c.get('branch_far_p50_mm', 0):.0f}mm 전방 "
          f"{c.get('forward_range_m', 0):.2f}m 입사 {c.get('incidence_deg', 0):.0f}°"
          f"@{c.get('incidence_clock_deg', 0):+.0f}° curve {c.get('curve_ahead')}"
          f" {extra}", flush=True)

IDLE = "IDLE"
CRUISE = "CRUISE"
SLOW = "SLOW"
HOLD = "HOLD"
RECOVER = "RECOVER"
# 🎯 2026-08-07 신설 — 분기 진입 기동. 검출기의 BRANCH 는 **트리거**로만 쓰고,
#    기동 자체는 래치한 방위로 정해진 호 길이만큼 수행한다.
#    비례 조향("개구를 가리켜라")은 실측으로 기각됐다 — 몸을 옆으로 밀기만
#    해서 구멍으로 통째로 빠진다(세기 40°/20° 낙하, 12° 는 끼임).
BRANCH_ENTRY = "BRANCH_ENTRY"
RETURN = "RETURN"
DONE = "DONE"

DEFAULTS = dict(
    v_max_mps=0.10,
    v_slow_frac=0.35,
    # 🚨 감속 인자(슬로게이트×개방도×입사각×전방)가 **곱으로 쌓이면** 3mm/s
    #    까지 떨어져 정지 마찰을 못 이긴다(실측: elbow 곡관 접근 제자리 사망).
    #    통행 가능 + 의도적 정지(전방 게이트 0)가 아니면 이 밑으로는 안 준다.
    #    9mm/s 는 SLOW 에서 실제로 움직인 실측값.
    # (0.009 → 0.012: 안착 요동 굽힘이 남은 채의 출발에서 9mm/s 로는 정지
    #  마찰을 못 이기는 사례 실측 — reducer 출발 5초 정체 사망)
    v_min_move_mps=0.012,
    # 🚨 0.12 → 0.035 (2026-08-07): 복귀가 진입(35mm/s)의 3.4배로 후진하다
    #    분기 모서리를 되지나며 관 입구를 지나쳐 **사출**됐다(실측: RETURN
    #    v=−94mm/s → 이탈 84mm 낙하). 들어온 호를 되그리는 후진이 진입보다
    #    빠를 이유가 없다.
    v_return_mps=0.035,
    accel_mps2=0.08,
    decel_mps2=0.40,

    aperture_ref_px=90000.0,
    aperture_min_frac=0.25,
    incidence_slow_deg=8.0,
    incidence_stop_deg=30.0,

    mission_range_m=30.0,

    stuck_wheel_min_mps=0.02,
    stuck_visual_frac=0.30,
    stuck_hold_s=1.5,
    recover_back_m=0.05,
    recover_max_try=3,

    condition_timeout_s=1.0,
    hold_release_s=2.0,

    # ── 부하 보상 증속 (2026-08-08 사용자 지시: "중력·곡관 구조로 감속이
    #    발생하면 자율주행이 알아서 속도를 높여라") ─────────────────────
    # 🔑 휠 드라이브 토크 = 댐핑 × (지령 − 실제) 이므로, 실속도가 지령에
    #    못 미치면(중력 오르막·곡관 마찰·정지 마찰) **지령을 올리는 것이
    #    곧 토크를 더 쓰는 것**이다. 지령의 boost_lag_frac 미만으로
    #    boost_after_s 처지면 배율을 올리고, 따라잡으면 서서히 내린다.
    #    끼임 판정(RECOVER)보다 먼저 발동해야 물러나기 전에 밀어붙인다
    #    (stuck_hold_s 1.5 보다 짧게).
    # 🚨 절대 상한 boost_v_cap 은 접촉 감지폭 제약: 데모 contactOffset 이
    #    SPEED_MPS 기준(0.10 → 0.5mm/스텝)이라, 절대속도가 이를 크게 넘으면
    #    바퀴가 관벽을 뛰어넘는다(기록된 함정). 저속 구간(12~35mm/s)은
    #    배율 2 라도 절대속도가 낮아 안전하다.
    boost_max=2.0,           # 지령 배율 상한
    boost_after_s=1.0,       # 이만큼 처져야 증속 시작 (< stuck_hold_s 1.5)
    boost_rate=0.5,          # 초당 배율 증가
    boost_decay=0.3,         # 따라잡은 뒤 초당 배율 감소
    boost_lag_frac=0.6,      # 실속도 < 지령×이 값 = 처짐
    boost_ok_frac=0.85,      # 실속도 ≥ 지령×이 값 = 따라잡음
    boost_v_cap_mps=0.12,    # 배율 적용 후 절대 상한
    # 출발점 이전(적산 거리 미달)에서는 발차기 금지 — RECOVER 물러남 반복으로
    # 입구 개방단까지 밀린 로봇이 입구를 개구로 오인해 허공으로 차는 사고
    # 방지(2026-08-08 GUI 2대 런 실측: elbow_h s=35 발차기 → 관 밖 사출).
    entry_min_dist_m=0.05,

    # ── 분기 진입 기동 ──────────────────────────────────────────
    # 🚨 **들어갈지 지나칠지는 임무 의도다** — 센서가 정하지 않는다.
    #    기본 True 는 연습장용이고, 실전 임무에서는 조율기가 정해 준다.
    branch_enter=True,
    # 🎯 **오른손 법칙** (2026-08-07 사용자 확정) — 임무 루프 전체가 이 규칙
    #    하나다: 나갈 때(본관→오른팔)도, 복귀할 때(왼팔→본관)도 **오른쪽에
    #    나타난 개구로 꺾어 들어간다.** 왼쪽 개구는 무시하고 직진(오른쪽 벽이
    #    이어진다). 방향 수정은 왼발차기(개구 반대쪽 벽을 미는 것) = 진입 기동.
    #      "right" = 오른쪽 개구만 진입 / "all" = 무조건(구 동작) / "none"
    branch_rule="right",
    # 개구 방위(화소, 검출기 좌표) → 몸 프레임 시계각 변환 오프셋.
    # 전방 카메라 장착 실측: θ_body = φ_화소 + 180°.
    branch_body_offset_deg=180.0,
    # 몸 프레임에서 "오른쪽" = −Y = 시계각 180° (X 전방·Y 왼쪽·Z 위 규약).
    # 🚨 경위 (2026-08-07, 두 번 뒤집힌 값): 한때 "tee_back 개구가 몸 시계각
    #    ~0° 로 찍힌다" 는 실측으로 0 으로 뒤집었으나, 그 실측의 코스 자체가
    #    **거울상**(월드 기준 왼손 턴)이었음이 수치로 확인됐다 — 왼쪽 개구가
    #    0° 에 찍힌 것이니 오히려 규약과 맞다. 코스를 바로잡으며 180 으로
    #    원복. 오른쪽 판정의 정본 검증은 몸 clock 이 아니라 **월드 기준**
    #    (진행방향×상방, 연습장 [턴검증] 로그)으로 한다.
    branch_right_deg=180.0,
    branch_right_window_deg=90.0,   # 오른쪽 ±90° 안에 있어야 진입
    # 🎯 **"허공에는 절대 차지 않는다"** (2026-08-08 사용자 원칙). 진짜 T
    #    개구는 광선이 가지 벽에 맞아 결손 중 무효(inf)가 낮다. 결손이 무효
    #    위주면 열린-끝 허공(연습장 광학 아티팩트)이니 발차기 후보에서 뺀다.
    # 🚨 0.5 는 위험 (CTL_DEBUG 실측): 정면 T(tee_go) 트리거 프레임의 inf 가
    #    0.49~0.51 로 경계에 걸린다 — 반대팔 너머로 광선이 inf 로 빠지는
    #    기하(기록된 inf 반전 실험이 tee_go 를 죽인 그 이유). 0.7 로 둔다.
    branch_inf_max=0.7,
    # 🎯 **장면 래치** (2026-08-08, 어제 설계안 2 의 최소 구현 — CTL_DEBUG
    #    실측 설계): 곡관 접근(curve_ahead)을 본 뒤 이 시간 동안은 링
    #    발차기를 금지한다. elbow_v 오발 = 곡관 분류로 굽이에 코를 들이민
    #    로봇이 곡관 연속면을 개구(31%@body155, 오른쪽 창 안)로 본 것.
    #    진짜 T 접근은 BRANCH 연속(곡관 프레임 0, tee 로그 실측)이라 안 걸림.
    curve_kick_block_s=2.0,
    # 🎯 **정지-탐색 교착 탈출** (2026-08-08 실전 맵 floor2 실측). 정면 T 는
    #    "벽 앞에 서서 탐색하다 진입" 이 정상 동작이라 전방 게이트가 v=0 을
    #    준다. 그런데 그 자리가 **곡관인데 BRANCH 로 읽힌 장면**이면 진입이
    #    영원히 안 걸리고(곡관에서 발차기 금지가 옳다), BRANCH 중에는 센터링
    #    조향도 꺼져 있어 **차지도 돌지도 못한 채 영영 선다** — 실측: floor2
    #    출발 26mm 앞 곡관에서 v=0·휠 정지로 굳음(사용자: "바퀴회전도 안하는데?").
    # 🔑 실제 관 속에서 서 있을 수는 없다. 이만큼 서 있었는데도 진입이 안
    #    걸렸으면 **그 장면은 T 가 아니다** — 곡관으로 재해석해 센터링이 밀고
    #    나가게 한다. 정면 T 는 front_confirm_s(1.5s) 안에 진입하므로 안 걸린다.
    branch_deadlock_s=3.0,
    # 🎯 front 트리거(정면 T 임무 규칙) 오발 가드 (2026-08-08 실측 설계) —
    #    **개구가 보이는데 그것이 오른쪽 창에 없으면 정면 T 가 아니라 곡관**
    #    이다(선택지가 그 개구 하나뿐 = T 아님, 사용자 구조 규칙). elbow_v
    #    실측: 관경 12~18mm(자기몸 가림)로 곡관 표시를 못 받은 프레임에서
    #    분기 48%@body−5°(오른쪽 아님)인데 front 트리거가 "정면 T"로 오발 →
    #    엉뚱한 방향 발차기 → 이탈 정지. 정면 T 정지 지점 실측은 분기
    #    6~8%(문턱 미달)라 이 가드에 안 걸린다(덤프 40장).
    front_opening_block_ratio=0.08,
    # 🎯 front 트리거 가드 2 — **결정판** (2026-08-08 CTL_DEBUG 계측). 곡관
    #    정지점은 링 관경이 **정상**(46mm ≈ 기준 46, 분기 0% — 링에 아무
    #    증거가 없다: 로봇은 아직 DN100 안이고 굽이만 앞에 있다)인 채 전방
    #    0.12m 막힘 + 무진행이 1.5s 차서 오발했다. 진짜 정면 T 정지점은
    #    가로대 내부 공동이 링에 잡혀 관경이 **팽창**한다(덤프 40장: 61→127,
    #    최소 1.22×기준). → 관경이 기준×1.1 미만(공동 증거 없음)이면 front
    #    타이머를 안 채운다. 실측 1.0(곡관) vs 1.22(T)의 사이값.
    front_bore_expand_frac=1.10,
    entry_bend_deg=40.0,     # 기동 중 관절 총 굽힘 지령(도)
    entry_arc_m=0.24,        # 이만큼 전진하면 기동 종료 (90° 호 ≈ (π/2)·R)
    entry_v_frac=0.35,       # 기동 중 속도 배율
    entry_cooldown_m=0.30,   # 종료 직후 이 거리 안에서는 재트리거 금지
    entry_confirm_s=1.5,     # BRANCH 연속 확인 — 0.4 로는 확관에서 다리 거부권이 따라붙기 전에 차 버렸다(실측)
    entry_margin_m=0.06,     # 후진 기동 구간을 진입 구간 앞뒤로 이만큼 확장

    # ── 시각 센터링 조향 (2026-08-07 사용자 설계: 관은 어디나 DN100 원통 —
    #    경로는 몰라도 화면의 개구부 중심(≈관 축)에 늘 몸을 맞출 수 있다) ──
    # 🚨 BRANCH 판정 중에는 끈다 — "개구를 가리켜라" 비례 조향이 구멍으로
    #    통째로 빠진 실측 전례. 센터링은 NORMAL 계열 한정·소이득이다.
    # 🚨 이득은 **장면에 따라 둘이어야 한다** (실측 두 사고의 교훈):
    #    - 0.5 로는 곡관을 못 돈다 (입사 30°×0.5 = 15°, 관절당 3.75° —
    #      elbow 곡관 입구 제자리 사망).
    #    - 1.2 를 일반 프레임에도 쓰면 T 접합부 근처(입사 30°가 흔하다)에서
    #      과조향으로 관 밖 사출(tee_go 이탈 294mm·꺾임 221° 실측).
    #    → 일반 0.5 / 검출기가 curve_ahead 로 표시한 곡관 접근 프레임만 1.2.
    center_gain=0.5,         # 조향 지령 = gain × incidence(관 축 이탈각)
    center_gain_curve=1.2,   # 곡관 접근(curve_ahead) 프레임 전용 이득
    # 🚨 상한 10°(관절당 2.5°)로는 곡관을 못 돈다 (실측: elbow 곡관 입구에서
    #    코가 바깥벽에 닿은 채 9mm/s 제자리 — 필요한 총 굽힘은 ~58°). 40°
    #    (관절당 10°, 진입 기동과 같은 급)로 올린다. 직관에서는 데드밴드
    #    3° 와 이득 0.5 가 지령을 여전히 작게 유지한다.
    center_max_deg=40.0,
    center_deadband_deg=3.0,  # 이 밑은 무시 — 관절이 미세 요동을 쫓지 않게

    # ── 전방 거리 게이트 (2026-08-07, 정면 T 대응) ──────────────────
    # 🎯 정면 T 는 가로대 맞은편 **벽**이 다가온다 — 설계 문서의 "곡관/단절
    #    판정" 표에 있던 그 신호(전방 거리 급감)다. 코가 벽에 박히면 다리가
    #    크게 벗어나 **다리 거부권이 분기 확인 타이머를 계속 리셋**해 진입이
    #    영영 안 걸린다(실측: s=189 BRANCH 14% 가 보이는데 SLOW 정체 → 조기
    #    중단). 박히기 **전에** 서면 다리가 정상이라 진입 게이트가 산다.
    #    fr < stop 이면 v=0 이되 상태는 SLOW 로 남는다 — HOLD 가 아니라서
    #    분기 트리거가 계속 활성이다(정지-탐색-진입).
    forward_slow_m=0.15,     # 이 밑에서 감속 시작
    forward_stop_m=0.08,     # 이 밑에서 정지 (탐색은 계속)
    # 🎯 **정면 T 진입** (2026-08-07, 덤프 실측 근거) — 정지 지점에서는
    #    어떤 θ 링(45~68°)도 좌·우 개구를 못 본다(HFOV 140° 한계: 68° 에서
    #    우측 개구 6~8% 로 문턱 미달, bore 61→127mm 연속 팽창 = 가로대
    #    내부가 벽으로 잡힘). 검출로는 못 들어간다 → **임무 규칙으로 든다**:
    #    전방이 막힌 채(front_confirm_s) 서 있고 단절(DISCONNECTED→HOLD)이
    #    아니면 그 자리가 정면 T 다. 오른손 법칙이 방향을 준다(오른쪽 턴).
    front_confirm_s=1.5,     # 전방 정지가 이만큼 지속되면 정면 T 로 본다
    # 막힘 판정 거리 — forward_slow(0.15)보다 넓게 잡는다. 곡관 입구는
    # 코가 벽에 닿기 전(fr 0.15~0.25)에 이미 전진이 죽는다(실측: elbow_h
    # s=205 정체, fr 이 창 밖이라 트리거 불발).
    front_range_m=0.25,
    # 🎯 **곡관은 벽이어도 밀고 들어간다** (2026-08-09 사용자 지시: *"R150 은
    #    밀고 들어가면 자동으로 꺾이게끔 설계한 것이다. 곡관이라 판정이 확실히
    #    되면 벽이어도 밀고 들어갈 수 있게 할 수 없나"*).
    # 🚨 위 전방 정지 게이트(`forward_stop_m`)는 **정면 T 전용 장치**다 — 코가
    #    박히기 전에 세워야 다리가 멀쩡해 진입 판정이 산다는 것이 도입 사유다.
    #    그런데 곡관 예외가 없어서, 굽이 입구에서 바깥벽이 다가오면 곡관에서도
    #    v=0 을 문다. 실측(floor1 진입 곡관, 센터링 0): 전방 0.08m 에서 12초
    #    동안 한 발도 못 나가고 사망 — **설계상 밀면 지나갈 자리에서 섰다.**
    # 🔑 곡관은 막다른 벽이 아니라 **휘어 이어지는 길**이다. 곡관으로 분류된
    #    프레임에서는 게이트를 이 값까지만 열어 두어 최소 속도가 살아 있게
    #    한다(0 이면 v_min_move 바닥조차 안 걸린다 — 위 속도 법칙 참조).
    # 🚨 이 값이 사는 것은 **곡관 분류가 맞을 때뿐**이다. 진짜 T 를 곡관으로
    #    오판하면 벽에 그대로 박는다 → 분류 정확도가 이 노브의 전제다
    #    (같은 날 `detector` 의 "둘째 개구 폭 하한" 결함을 함께 고친 이유).
    curve_push_frac=0.35,
)


@dataclass
class DriveState:
    state: str = IDLE
    v_cmd: float = 0.0
    v_target: float = 0.0
    distance_m: float = 0.0
    slip_ratio: float = 1.0
    stuck_for_s: float = 0.0
    recover_try: int = 0
    reason: str = ""
    gates: dict = field(default_factory=dict)
    # 분기 진입 기동의 조향 지시. steer_deg == 0 이면 조향 없음.
    # steer_clock_deg 는 **검출기 좌표(화면 화소 방위)** 그대로다 — 카메라
    # 장착 방향을 아는 쪽(로봇 드라이버)이 몸 프레임으로 변환한다.
    steer_deg: float = 0.0
    steer_clock_deg: float = 0.0

    def as_dict(self):
        return asdict(self)


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class DriveController:
    """조건 스칼라를 받아 전진 속도와 FSM 상태를 낸다.

    이미지는 보지 않는다. 설계 원칙상 이미지는 인지 노드 밖으로 나가지 않으며,
    여기로는 검출 결과와 치수만 들어온다.
    """

    def __init__(self, params=None):
        self.k = dict(DEFAULTS)
        if params:
            self.k.update({a: b for a, b in params.items() if a in self.k})
        self.s = DriveState()
        self._hold_s = 0.0
        self._recover_from = 0.0
        self._no_cond_s = 0.0
        self._last_cond = None
        self._entry_from = 0.0
        self._entry_clock = 0.0
        self._entry_done_at = -1e9
        self._branch_s = 0.0
        self._front_s = 0.0
        # RECOVER 가 물러나는 방향. 전진 중 끼임은 뒤로(−1), **후진(RETURN)
        # 중 끼임은 앞으로(+1)** 물러난다 (임무 규칙 8 의 방향 상대화).
        self._recover_dir = -1.0
        # RETURN 순진행 감시 (슬립 판정이 깜빡일 때의 안전망)
        self._ret_d0 = None
        self._ret_t = 0.0
        # RECOVER 정체 감시 (물러나는 방향조차 막힐 때 반대로 흔들기)
        self._rec_d0 = None
        self._rec_t = 0.0
        # 장면 래치 — 곡관 접근을 본 뒤 남은 발차기 금지 시간
        self._curve_hold_s = 0.0
        # 적응형 순항 부스트 상태
        self._boost = 1.0
        self._boost_ok_s = 0.0
        # 정지-탐색 교착 감시 (BRANCH 인데 진입도 못 하고 못 나가는 시간)
        self._frozen_s = 0.0

    # ── 속도 법칙 ────────────────────────────────────────────────
    def openness(self, aperture_px):
        """전방이 얼마나 열려 있나. 개구부 면적이 작을수록 감속."""
        if aperture_px is None or aperture_px <= 0:
            return self.k["aperture_min_frac"]
        r = aperture_px / self.k["aperture_ref_px"]
        return _clamp(r, self.k["aperture_min_frac"], 1.0)

    def curve_factor(self, incidence_deg):
        """개구부가 화면 중앙에서 벗어난 각. 곡관 접근의 선행 지표다.

        관절 엔코더는 이미 꺾인 뒤에야 반응하지만 이 각은 곡관에 닿기 전에
        커진다. 그래서 선제 감속에 쓴다.
        """
        a, b = self.k["incidence_slow_deg"], self.k["incidence_stop_deg"]
        d = abs(incidence_deg or 0.0)
        if d <= a:
            return 1.0
        if d >= b:
            return self.k["v_slow_frac"]
        t = (d - a) / (b - a)
        return 1.0 - t * (1.0 - self.k["v_slow_frac"])

    def gate(self, cond):
        """관 상태 판정이 내리는 통행 허가."""
        if cond is None:
            return 0.0, "판정 없음"
        if not cond.get("passable", False):
            return 0.0, f"{cond.get('state', '?')} — {cond.get('reason', '')}"
        sp = cond.get("speed", "stop")
        if sp == "stop":
            return 0.0, "정지 지시"
        if sp == "slow":
            return self.k["v_slow_frac"], "저속 지시"
        return 1.0, ""

    def _right_opening(self, cond):
        """진입 규칙(`branch_rule`)에 맞는 개구의 방위(화소 좌표). 없으면 None.

        - "right": 오른쪽 창 안의 개구만. 정면 T 는 개구가 두 개다 — 검출기가
          큰 것부터 둘을 주므로 **둘 다** 보고 오른쪽 것을 고른다. 이것이
          오른손 법칙의 실체다: 좌·우가 같이 열려 있으면 오른쪽으로 꺾는다.
        - "all": 방위 무관 첫 개구 (구 동작).
        - "none": 항상 None — 진입 금지.
        🚨 진입 게이트는 이 함수를 쓴다. 한때 `_branch_on_right` 만 규칙을
        보고 여기는 안 봐서 `BRANCH_RULE=none` 이 **죽은 노브**였다(실측:
        none 인데 elbow 오탐 진입 발동).
        """
        if self.k["branch_rule"] == "none":
            return None
        # 🎯 **허공에는 절대 차지 않는다** (2026-08-08 사용자 원칙) — 결손이
        #    무효(inf) 위주면 열린-끝 허공이지 T 개구가 아니다. 진입 금지.
        if (float(cond.get("branch_inf_frac", 0.0) or 0.0)
                >= self.k["branch_inf_max"]):
            return None
        for dg, arc in ((cond.get("branch_deg", 0.0),
                         cond.get("branch_arc_deg", 0.0)),
                        (cond.get("branch2_deg", 0.0),
                         cond.get("branch2_arc_deg", 0.0))):
            if arc <= 0.0:
                continue
            if self.k["branch_rule"] == "all":
                return float(dg)
            body = float(dg) + self.k["branch_body_offset_deg"]
            d = (body - self.k["branch_right_deg"] + 180.0) % 360.0 - 180.0
            if abs(d) <= self.k["branch_right_window_deg"]:
                return float(dg)
        # 🎯 (2026-08-08 폴백 제거) 구 "전방 막힘 = 개구가 유일한 길 = 방위
        #    불문 진입" 폴백은 곡관에 왼발차기를 쏘는 통로였다(사용자 규칙
        #    위반: 왼발차기는 T 전용). 이제 그 장면(단일 개구 + 직진로 막힘)
        #    은 **검출기가 곡관 접근으로 재분류**해 여기 오지 않는다 —
        #    곡관은 센터링 조향(curve_ahead 이득 1.2)이 민다. elbow_h 왼쪽
        #    개구 제자리 사망(이 폴백의 도입 사유)은 재분류가 같은 장면을
        #    받아 주므로 재발하지 않는다.
        return None

    def _branch_on_right(self, cond):
        """오른손 법칙 — 개구가 몸 오른쪽에 있는가.

        🚨 왼쪽 개구에서 진입을 막는 것이 핵심이다. 이것 없이는 "무조건
        진입" 이라 복귀 경로에서 왼쪽 가지로 잘못 들어가고, 사용자의 루프
        위상(오른손 법칙)이 통째로 무시된다.
        """
        if self.k["branch_rule"] == "all":
            return True
        if self.k["branch_rule"] != "right":
            return False
        body = (float(cond.get("branch_deg", 0.0))
                + self.k["branch_body_offset_deg"])
        d = (body - self.k["branch_right_deg"] + 180.0) % 360.0 - 180.0
        return abs(d) <= self.k["branch_right_window_deg"]

    # ── 끼임 판정 ────────────────────────────────────────────────
    def update_slip(self, wheel_mps, visual_mps, dt):
        """바퀴는 도는데 실제로 안 나가면 끼임이다.

        휠 엔코더만으로는 6륜이 동시에 미끄러지는 경우를 못 잡는다.
        시각 오도메트리와 비교해야 실제 이동을 안다.
        """
        w = abs(wheel_mps or 0.0)
        if w < self.k["stuck_wheel_min_mps"]:
            # 🚨 **바퀴 정체(stall)도 끼임이다** (2026-08-07 실측: tee_go 복귀
            #    쐐기에서 휠 드라이브(maxF 0.08N·m)가 아예 못 돌아 "바퀴는
            #    도는데" 전제가 무너졌다 — 슬립 1.0 으로 위장되어 RECOVER 가
            #    영영 안 걸렸다). 지령이 충분한데 바퀴가 안 돌면 끼임 누적.
            if abs(self.s.v_cmd) > 0.015:
                self.s.slip_ratio = 0.0
                self.s.stuck_for_s += dt
                return self.s.stuck_for_s >= self.k["stuck_hold_s"]
            self.s.slip_ratio = 1.0
            self.s.stuck_for_s = 0.0
            return False
        v = abs(visual_mps if visual_mps is not None else w)
        self.s.slip_ratio = _clamp(v / w, 0.0, 2.0)
        if self.s.slip_ratio < self.k["stuck_visual_frac"]:
            self.s.stuck_for_s += dt
        else:
            self.s.stuck_for_s = 0.0
        return self.s.stuck_for_s >= self.k["stuck_hold_s"]

    # ── 본체 ─────────────────────────────────────────────────────
    def step(self, dt, cond=None, wheel_mps=None, visual_mps=None,
             start=False, recall=False, suppress_branch=False):
        # suppress_branch: **고유수용 거부권** — 다리가 관경 변화를 느끼는
        # 동안(전 다리가 같이 뻗는다) 시각의 BRANCH 를 믿지 않는다. 확관에서
        # 위쪽 벽이 멀어진 것이 분기 개구와 광학적으로 안 갈리기 때문(실측:
        # far 반경이 60~90mm 로 겹친다). 분기에서는 다리가 안 뻗으므로
        # (기준벗어남 0.1~1.4 vs 확관 4~6mm) 진짜 진입은 안 막는다.
        s = self.s
        k = self.k

        if wheel_mps is not None:
            # 🚨 휠 적분만 쓰면 슬립이 거리를 부풀린다 (진입 기동 중 슬립
            #    0.19~0.28 실측 — 바퀴는 도는데 몸은 조금 나간다). 복귀가
            #    그 부푼 거리를 되감느라 **출발점을 지나쳐** 관 입구까지
            #    후진했다. → 크기는 시각 오도메트리(실제 이동), 부호는 휠
            #    (시각은 방향이 불안정하다). 시각이 튈 때 상한은 휠 **1배** —
            #    관 안에서 몸은 바퀴 표면속도보다 빨리 못 간다. 2배로 두었을
            #    때 접합부에서 시각이 튀며(개구 위 광류 붕괴, 리듀서 113mm/s
            #    실측) RETURN 거리가 실제의 두 배로 줄어 **복귀가 끝 구간을
            #    벗어나기 전에 DONE** 이 났다(실측: tee_back 가짜 랩 155회).
            mag = abs(wheel_mps)
            if visual_mps is not None:
                mag = min(abs(visual_mps), abs(wheel_mps))
                # 🚨 순수 min 은 **저속에서** 거리를 굶긴다 (실측: RECOVER 가
                #    12초에 51mm 실전진하는데 광류가 0 을 읽어 distance 가
                #    얼고 탈출 조건이 영영 안 참 — 저속은 화소 이동이 부족해
                #    광류가 못 잰다). 휠이 저속일 때만 휠의 40% 를 바닥으로.
                #    고속 슬립(휠 빠른데 시각 0 = 진짜 끼임/헛돎)에는 바닥을
                #    주면 안 된다 — 기준점(recover_from)이 오염되어 RECOVER
                #    탈출 거리가 영영 안 찬다(회귀 시험 실측).
                if abs(wheel_mps) < 0.02:
                    mag = max(mag, 0.4 * abs(wheel_mps))
            s.distance_m += math.copysign(mag, wheel_mps) * dt

        # 판정 주기(10Hz)가 제어 주기(20Hz)보다 느리다. 새 메시지가 없는 틱을
        # 그대로 "판정 없음"으로 다루면 매 틱 걸러 정지해 주행이 끊긴다.
        # 마지막 판정을 타임아웃까지 유지하고, 그 뒤에야 두절로 본다.
        if cond is not None:
            self._last_cond = cond
            self._no_cond_s = 0.0
        else:
            self._no_cond_s += dt
        cond = (None if self._no_cond_s > k["condition_timeout_s"]
                else self._last_cond)

        # 진입 확인 타이머 — BRANCH 가 끊기면 리셋 (한 프레임 오탐 차단)
        # 🔑 거부권은 **타이머 리셋**으로 먹인다 (2026-08-07 최종안) — 확관은
        #    시각(벽 멀어짐)이 다리(같이 뻗음)보다 먼저 뜬다. 순간 판정으로
        #    막으면 그 시차에 진입이 발동한다. 확인 타이머 0.4s 동안 다리
        #    거부권이 한 번이라도 뜨면 처음부터 다시 세므로, 다리가 따라붙는
        #    확관에서는 타이머가 영영 안 찬다. 분기에서는 다리가 안 뻗으므로
        #    타이머가 그대로 찬다.
        # 장면 래치 — 곡관 **확증**(curve_sure) 프레임만 발차기 금지를
        # 재장전한다. 모호한 곡관 정면 접근(curve_ahead 만 True)으로 걸면
        # 정면 T 재진입 발차기까지 막는다(tee_back 이탈 80mm 실측).
        if cond is not None and cond.get("curve_sure"):
            self._curve_hold_s = k["curve_kick_block_s"]
        else:
            self._curve_hold_s = max(0.0, self._curve_hold_s - dt)

        if suppress_branch or self._curve_hold_s > 0.0:
            self._branch_s = 0.0
        elif cond is not None and cond.get("state") == "BRANCH":
            self._branch_s += dt
            if _CTL_DEBUG and (self._branch_s // 0.5
                               != (self._branch_s - dt) // 0.5):
                _dbg_cond(f"branch타이머 {self._branch_s:.1f}s", cond)
        else:
            self._branch_s = 0.0

        # 정면 T 확인 타이머 — **전방이 가깝고(감속 거리 안) + 실제로 안
        # 나가고 있으면** 찬다. 단절(DISCONNECTED)은 passable=False 라 여기
        # 못 온다(HOLD 로 빠짐). 곡관 접근(curve_ahead) 프레임은 제외 —
        # 곡관 입구 정체에 오른손 턴을 쏘면 왼쪽 곡관에서 사고가 난다.
        # 🚨 "정지 거리(80mm) 안" 창으로는 놓친다(실측 GUI: 안착이 벽 앞
        #    s=178 에 떨어져 fr 이 창 밖인 채 코를 대고 9mm/s 헛지령 —
        #    다리 거부권이 링 진입도 막아 5초 조기 중단). 막힘의 실체는
        #    거리+무진행이다.
        _fr = float((cond or {}).get("forward_range_m") or 0.0)
        # 개구가 보이는데 오른쪽 창에 없으면(왼쪽/위/아래/허공) 정면 T 가
        # 아니다 — 곡관·측면 개구를 front 트리거가 삼키지 않게 리셋한다.
        _op_not_right = (cond is not None
                         and float(cond.get("branch_ratio", 0.0) or 0.0)
                         > k["front_opening_block_ratio"]
                         and self._right_opening(cond) is None)
        # 링 붕괴(자기몸 가림) 프레임 — 관경이 기준의 80% 미만으로 무너지면
        # 정면 T 증거로 안 센다(정면 T 는 관경이 팽창한다, 붕괴가 아니라).
        _ring_blind = False
        if cond is not None:
            _bm = float(cond.get("bore_mm", 0.0) or 0.0)
            _br = float(cond.get("bore_ref_mm", 0.0) or 0.0)
            # 붕괴(자기몸 가림) 또는 공동 증거 없음(관경 정상 = 곡관) —
            # 둘 다 정면 T 증거가 아니다. BRANCH 프레임(관경 0)은 링이
            # 개구를 본 것이므로 여기 안 걸린다(가드 1 이 방위를 본다).
            _ring_blind = (0.0 < _bm
                           < _br * k["front_bore_expand_frac"])
        if (s.state in (CRUISE, SLOW) and cond is not None
                and cond.get("passable", False)
                and not cond.get("curve_ahead")
                and not _op_not_right
                and not _ring_blind
                and 0.0 < _fr < k["front_range_m"]
                and abs(visual_mps or 0.0) < 0.005):
            self._front_s += dt
            if _CTL_DEBUG and (self._front_s // 0.5
                               != (self._front_s - dt) // 0.5):
                _dbg_cond(f"front타이머 {self._front_s:.1f}s", cond)
        else:
            self._front_s = 0.0

        # ③ 고유수용 거부권은 **판정 강등까지** — 진입만 막으면 BRANCH 의
        #    SPEED_SLOW 게이트가 남아 확관에서 9mm/s 로 기었다(s=680 정체 실측).
        #    다리가 관경 변화를 느끼는 동안 시각 BRANCH 는 통째로 무효다.
        if (suppress_branch and cond is not None
                and cond.get("state") == "BRANCH"):
            # 🚨 state 만 바꾸면 안 된다 — cond dict 에 speed="slow" 가 박혀
            #    있어 게이트가 그대로 0.35 를 문다(개방도 바닥 0.25 와 겹쳐
            #    9mm/s). 확관 감속은 개방도 항이 이미 하므로 full 로 되돌린다.
            cond = dict(cond, state="BORE_CHANGE", passable=True, speed="full",
                        reason="다리 거부권 — 확관으로 재해석")
        # 정지-탐색 교착 감시 — BRANCH 판정인데 실제로 못 나가고 있는 시간.
        # (진입이 걸리면 상태가 BRANCH_ENTRY 로 바뀌어 여기 안 온다)
        if (s.state in (CRUISE, SLOW) and cond is not None
                and cond.get("state") == "BRANCH"
                and abs(visual_mps or 0.0) < 0.005
                and abs(s.v_cmd) < 0.005):
            self._frozen_s += dt
        else:
            self._frozen_s = 0.0

        if (self._frozen_s >= k["branch_deadlock_s"] and cond is not None
                and cond.get("state") == "BRANCH"):
            # 🎯 교착 탈출 (위 상수 주석) — 서 있기만 한 BRANCH 는 T 가 아니다.
            cond = dict(cond, state="NORMAL", curve_ahead=True, speed="slow",
                        reason=(f"정지-탐색 {self._frozen_s:.1f}s 무진행 — "
                                f"곡관으로 재해석(교착 탈출)"))
            _dbg_cond("교착 탈출", cond)
        elif (self._curve_hold_s > 0.0 and cond is not None
                and cond.get("state") == "BRANCH"):
            # 🎯 장면 래치의 자기완결 (2026-08-08 실측): 래치가 발차기만 막고
            #    BRANCH 판정을 그대로 두면 **차지도 조향하지도 못하는 교착**
            #    이 된다 — BRANCH 중엔 센터링이 꺼져 있고 전방 게이트가 v=0
            #    을 무는 굽이 입구에서 12초 정지 사망(elbow_v 실측). 곡관을
            #    봤으면 그 뒤의 BRANCH 는 곡관 연속면이다 — 곡관 접근으로
            #    재해석해 센터링(곡관 이득)이 꺾어 들어가게 한다.
            cond = dict(cond, state="NORMAL", curve_ahead=True,
                        reason="장면 래치 — 곡관 연속면으로 재해석")

        stuck = self.update_slip(wheel_mps, visual_mps, dt)
        g, greason = self.gate(cond)

        # 🚨 RECOVER 는 recall 이 선점하면 안 된다 (2026-08-07 실측: 복귀 중
        #    쐐기 → RECOVER 전이가 다음 스텝에 "복귀 지시"로 되돌려져 1스텝
        #    짜리가 됐다 — 물러남 동작이 아예 없었다). RECOVER 가 끝나면
        #    CRUISE 로 나오고, 그때 recall 이 다시 RETURN 으로 데려간다.
        if recall and s.state not in (RETURN, DONE, RECOVER):
            # 🚨 진입 기동 **중에** 복귀 지시가 오면 (코스 끝이 접합부 바로
            #    뒤라 실측으로 흔하다) 기동 구간의 끝을 여기로 래치한다.
            #    이것 없이는 `_entry_done_at` 이 -1e9 로 남아 후진 기동
            #    (래치 굽힘 재적용) 구간이 비어, 모서리를 굽힘 없이 되지나다
            #    쐐기로 박힌다 (실측: tee_back s=309 고정).
            if s.state == BRANCH_ENTRY:
                self._entry_done_at = s.distance_m
            s.state, s.reason = RETURN, "복귀 지시"
        elif s.state == IDLE:
            if start:
                s.state, s.reason = CRUISE, "출발"
        elif s.state in (CRUISE, SLOW):
            if s.distance_m >= k["mission_range_m"]:
                s.state, s.reason = RETURN, f"{k['mission_range_m']:.0f}m 도달"
            elif g <= 0.0:
                s.state, s.reason = HOLD, greason
                self._hold_s = 0.0
            elif stuck:
                s.state = RECOVER
                s.reason = f"끼임 (슬립률 {s.slip_ratio:.2f})"
                s.recover_try += 1
                self._recover_from = s.distance_m
                self._recover_dir = -1.0
            elif (k["branch_enter"] and k["branch_rule"] != "none"
                    and cond is not None
                    and self._front_s >= k["front_confirm_s"]
                    and s.distance_m > k["entry_min_dist_m"]
                    and s.distance_m - self._entry_done_at
                    > k["entry_cooldown_m"]):
                # 🎯 정면 T — 전방이 막힌 채 서 있었다. 링은 개구를 못 보는
                #    기하(덤프 실측)라 **오른손 법칙이 방향을 준다**. 곡관은
                #    여기 못 온다(curve_ahead 분류가 먼저 걸러 센터링이
                #    맡는다 — 사용자 규칙: **왼발차기는 T 에서만**). 이
                #    경로는 다리 거부권을 안 탄다 — 코가 닿아 다리가 벗어난
                #    상태가 바로 이 트리거의 정상 입력이다.
                s.state = BRANCH_ENTRY
                s.reason = "정면 T — 전방 막힘, 오른손 턴"
                _dbg_cond("진입(front 트리거)", cond,
                          f"front_s {self._front_s:.1f}s")
                self._entry_clock = ((k["branch_right_deg"]
                                      - k["branch_body_offset_deg"] + 180.0)
                                     % 360.0 - 180.0)
                self._entry_from = s.distance_m
                self._front_s = 0.0
            elif (k["branch_enter"] and cond is not None
                    and cond.get("state") == "BRANCH"
                    and self._branch_s >= k["entry_confirm_s"]
                    and s.distance_m > k["entry_min_dist_m"]
                    and s.distance_m - self._entry_done_at
                    > k["entry_cooldown_m"]
                    and not suppress_branch
                    and self._right_opening(cond) is not None):
                # 🔑 방위를 **여기서 래치한다.** 진입하면서 검출 방위가 크게
                #    도는 것이 실측됐다(−178° → −26°) — 그걸 쫓으면 기동이
                #    산다. 트리거 순간의 값 하나로 호를 끝까지 그린다.
                s.state = BRANCH_ENTRY
                s.reason = (f"분기 진입 (방위 {cond.get('branch_deg', 0.0):+.0f}°"
                            f", 호 {k['entry_arc_m'] * 1000:.0f}mm)")
                _dbg_cond("진입(링 트리거)", cond,
                          f"branch_s {self._branch_s:.1f}s "
                          f"right={self._right_opening(cond)}")
                self._entry_from = s.distance_m
                # 오른쪽으로 **고른** 개구의 방위를 래치한다 (제1 개구가
                # 왼쪽일 수 있다 — 정면 T)
                _rb = self._right_opening(cond)
                self._entry_clock = (float(cond.get("branch_deg", 0.0))
                                     if _rb is None else _rb)
            else:
                s.state = SLOW if g < 1.0 else CRUISE
                s.reason = greason or "정상 주행"
        elif s.state == HOLD:
            self._hold_s += dt
            if g > 0.0 and self._hold_s >= k["hold_release_s"]:
                s.state, s.reason = CRUISE, "통행 재개"
            else:
                s.reason = greason or "정지 유지"
        elif s.state == BRANCH_ENTRY:
            # 🚨 판정 변화로는 안 나간다 — 진입 중에는 링이 개구·벽을 오가며
            #    아무 판정이나 낸다. 나가는 조건은 **전진 거리**와 끼임뿐.
            if stuck:
                s.state = RECOVER
                s.reason = f"진입 중 끼임 (슬립률 {s.slip_ratio:.2f})"
                s.recover_try += 1
                self._recover_from = s.distance_m
                self._recover_dir = -1.0
                self._entry_done_at = s.distance_m
            elif s.distance_m - self._entry_from >= k["entry_arc_m"]:
                s.state, s.reason = CRUISE, "분기 진입 완료"
                self._entry_done_at = s.distance_m
            else:
                s.reason = (f"분기 진입 중 "
                            f"({(s.distance_m - self._entry_from) * 1000:.0f}"
                            f"/{k['entry_arc_m'] * 1000:.0f}mm)")
        elif s.state == RECOVER:
            if s.recover_try > k["recover_max_try"]:
                s.state, s.reason = RETURN, "끼임 반복 — 임무 중단"
            elif (self._recover_from - s.distance_m) * (-self._recover_dir) \
                    >= k["recover_back_m"]:
                # 물러나기 완료 — 전진 재시도. RETURN 중이던 로봇은 연습장이
                # recall 을 계속 주므로 다음 스텝에 다시 RETURN 으로 돌아간다.
                s.state, s.reason = CRUISE, "물러남 완료 — 재시도"
                s.stuck_for_s = 0.0
                self._rec_d0 = None
            else:
                # 🚨 물러나는 방향조차 막히면 (실측: 복귀 종점이 접합부
                #    모서리라 전진 물러남도 바퀴 정체, 판정·거리 동결 15초)
                #    **반대 방향으로 흔든다.** 시도 횟수는 같이 센다.
                if self._rec_d0 is None:
                    self._rec_d0, self._rec_t = s.distance_m, 0.0
                self._rec_t += dt
                if self._rec_t >= k["stuck_hold_s"] + 1.0:
                    if abs(s.distance_m - self._rec_d0) < 0.005:
                        self._recover_dir *= -1.0
                        s.recover_try += 1
                        self._recover_from = s.distance_m
                    self._rec_d0, self._rec_t = s.distance_m, 0.0
                s.reason = f"물러나는 중 ({s.recover_try}/{k['recover_max_try']})"
        elif s.state == RETURN:
            # 🚨 RETURN 에도 끼임 처리가 있어야 한다 (2026-08-07 실측: tee_go
            #    복귀가 정면 모서리를 되지나다 쐐기, 휠 −35mm/s 에 시각 0 으로
            #    5초 정체 → 조기 중단). 후진 중 끼임이니 물러남은 **전진**이다.
            # 🚨 슬립 판정만으로는 못 잡는다 — 쐐기에서 시각속도가 간헐적으로
            #    튀며 1.5초 **연속** 조건이 안 찬다(실측: 6초 정체에 RECOVER
            #    미발동). **순진행 감시**를 안전망으로 둔다: 2.5초에 5mm 도
            #    못 물러났으면 끼임이다 (정상 후진 35mm/s 면 87mm 를 간다).
            if self._ret_d0 is None:
                self._ret_d0, self._ret_t = s.distance_m, 0.0
            self._ret_t += dt
            _ret_stalled = False
            if self._ret_t >= k["stuck_hold_s"] + 1.0:
                _ret_stalled = (self._ret_d0 - s.distance_m) < 0.005 \
                    and abs(s.v_cmd) > 0.015
                self._ret_d0, self._ret_t = s.distance_m, 0.0
            if stuck or _ret_stalled:
                s.state = RECOVER
                s.reason = (f"복귀 중 끼임 "
                            f"({'순진행 정체' if _ret_stalled else f'슬립률 {s.slip_ratio:.2f}'})")
                s.recover_try += 1
                self._recover_from = s.distance_m
                self._recover_dir = +1.0
            elif s.distance_m <= 0.0:
                s.state, s.reason = DONE, "복귀 완료"

        if s.state != RETURN:
            self._ret_d0 = None    # 순진행 감시는 RETURN 연속 구간만 잰다

        # 조향 지시 — 분기 진입 중, 그리고 **복귀가 그 구간을 되지날 때**
        if s.state == BRANCH_ENTRY:
            s.steer_deg = k["entry_bend_deg"]
            s.steer_clock_deg = self._entry_clock
        elif (s.state == RETURN and self._entry_done_at > -1e8
                and (self._entry_from - k["entry_margin_m"]
                     <= s.distance_m
                     <= self._entry_done_at + k["entry_margin_m"])):
            # 🚨 RECOVER 는 제외 (2026-08-07 실측) — 회복 흔들기까지 40° 굽힘을
            #    유지하면 모서리에서 양방향 다 쐐기가 되어 영영 못 나온다
            #    (판정·거리 동결 s=178). 회복은 몸을 펴고 살살 움직이는 것.
            # 🎯 후진 기동 (2026-08-07) — 카메라는 앞만 보므로 후진에는 검출이
            #    없다. 대신 **들어올 때 래치한 방위·굽힘을 그대로** 쓴다:
            #    관 형상이 같으니 몸이 그려야 할 호도 같다. 구간 판정은 거리
            #    (시각 오도메트리 적산)로 한다. 이것 없이 RETURN 이 접합부를
            #    지나면 관 밖으로 사출됐다(실측 낙하 25.9m).
            s.steer_deg = k["entry_bend_deg"]
            s.steer_clock_deg = self._entry_clock
        elif (s.state in (CRUISE, SLOW, RETURN) and cond is not None
                and cond.get("state") in ("NORMAL", "MISALIGNMENT",
                                          "BORE_CHANGE")
                and float(cond.get("incidence_deg", 0.0))
                > k["center_deadband_deg"]):
            # 🎯 **복귀(RETURN) 중에도 센터링을 쓴다** (2026-08-08 사용자 지적:
            #    *"복귀할 때 카메라를 사용 안 하는 구조인가? 왜 땅에 냅다
            #    처박지?"*). vision 모드는 렌더 부하 때문에 **전방 카메라만**
            #    만들어 후진 방향을 보는 눈이 없다. 그런데 전방 영상의 개구
            #    중심은 **관 축**을 알려주므로 뒤로 갈 때도 몸을 관에 맞추는
            #    데는 충분하다 — 조향은 진행 방향을 쫓는 것이 아니라 몸통
            #    사슬을 관 모양에 맞추는 것이라 전진·후진이 같은 식이다.
            #    실측 근거: floor2 복귀에서 조향이 없자 롤이 +3°→−54° 로
            #    누적되며 자세가 무너졌다(입사각도 13°→51°).
            #    ⚠ 진입 기동 구간(위 elif)은 래치한 굽힘이 우선이라 안 걸린다.
            # 🎯 센터링 — 개구부 중심(관 축)을 향해 코를 돌린다. 직관에서는
            #    정렬 유지(소이득), 곡관 접근(curve_ahead)에서는 진입 기동급
            #    이득으로 축이 휘는 쪽에 몸을 맞춘다.
            _cg = (k["center_gain_curve"] if cond.get("curve_ahead")
                   else k["center_gain"])
            s.steer_deg = min(_cg * float(cond["incidence_deg"]),
                              k["center_max_deg"])
            s.steer_clock_deg = float(cond.get("incidence_clock_deg", 0.0))
        else:
            s.steer_deg = 0.0

        # 부하 보상 증속 — 지령 대비 실속도가 처지면(중력·곡관 마찰·정지
        # 마찰) 배율을 올려 휠 토크를 더 쓴다. CRUISE/SLOW 한정: RECOVER·
        # 진입 기동·복귀는 각자의 속도 규약이 있다. 완전 정지(끼임)는
        # 여기 소관이 아니다 — update_slip → RECOVER 가 맡는다.
        if s.state in (CRUISE, SLOW) and abs(s.v_cmd) > 0.008:
            _act = abs(visual_mps or 0.0)
            if _act < abs(s.v_cmd) * k["boost_lag_frac"]:
                self._boost_ok_s += dt
                if self._boost_ok_s >= k["boost_after_s"]:
                    self._boost = min(k["boost_max"],
                                      self._boost + k["boost_rate"] * dt)
            elif _act >= abs(s.v_cmd) * k["boost_ok_frac"]:
                self._boost_ok_s = 0.0
                self._boost = max(1.0, self._boost - k["boost_decay"] * dt)
        else:
            self._boost_ok_s = 0.0
            self._boost = 1.0

        if s.state in (IDLE, HOLD, DONE):
            s.v_target = 0.0
        elif s.state == RECOVER:
            s.v_target = self._recover_dir * k["v_return_mps"] * 0.5
        elif s.state == RETURN:
            s.v_target = -k["v_return_mps"]
        elif s.state == BRANCH_ENTRY:
            s.v_target = k["v_max_mps"] * k["entry_v_frac"]
        else:
            openness = self.openness((cond or {}).get("aperture_px"))
            curve = self.curve_factor((cond or {}).get("incidence_deg"))
            # 전방 거리 게이트 — 벽이 다가오면 서서히, 문턱 밑에서는 완전히
            # 선다(정면 T). 상태는 SLOW 로 남아 분기 탐색·진입이 계속 산다.
            fr = float((cond or {}).get("forward_range_m") or 0.0)
            fwd = 1.0
            if fr > 0.0:
                a, b = k["forward_stop_m"], k["forward_slow_m"]
                fwd = _clamp((fr - a) / max(b - a, 1e-6), 0.0, 1.0)
                # 🎯 곡관은 밀고 들어간다 (위 curve_push_frac 주석) — 벽이
                #    다가온 것이 아니라 길이 휜 것이므로 세우지 않는다.
                if (cond or {}).get("curve_ahead"):
                    fwd = max(fwd, k["curve_push_frac"])
            s.v_target = k["v_max_mps"] * openness * curve * g * fwd
            if g > 0.0 and fwd > 0.0:
                s.v_target = max(s.v_target, k["v_min_move_mps"])
            # 부하 보상 배율 — 절대 상한(접촉 감지폭)으로 자른다.
            if self._boost > 1.0 and s.v_target > 0.0:
                s.v_target = min(s.v_target * self._boost,
                                 k["boost_v_cap_mps"])
            s.gates = dict(openness=round(openness, 3), curve=round(curve, 3),
                           condition=round(g, 3), forward=round(fwd, 3),
                           boost=round(self._boost, 2))

        # 가속은 완만하게, 감속은 빠르게. 정지는 늦으면 안 된다.
        rate = k["accel_mps2"] if abs(s.v_target) > abs(s.v_cmd) else k["decel_mps2"]
        step = rate * dt
        if s.v_cmd < s.v_target:
            s.v_cmd = min(s.v_cmd + step, s.v_target)
        else:
            s.v_cmd = max(s.v_cmd - step, s.v_target)
        if s.state in (IDLE, HOLD, DONE) and abs(s.v_cmd) < 1e-4:
            s.v_cmd = 0.0
        return s
