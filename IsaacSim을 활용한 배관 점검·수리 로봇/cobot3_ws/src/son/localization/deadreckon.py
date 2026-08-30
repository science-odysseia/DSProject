"""[공용] 추측 항법 — 로봇이 스스로 지나온 길을 기록한다. ROS 비의존.

**로봇은 배관 도면을 갖지 않는다.** 도면은 관리자만 가지고 있다.
그래서 로봇이 내놓는 것은 절대 좌표가 아니라 **상대 경로**다.

    "직진 1.24m → 좌 90° → 직진 0.81m → 우 90° → 직진 0.32m"

관리자 쪽이 이 열을 도면 위에 얹어 절대 위치를 만든다(monitor/blueprint.py).
진입점을 알고 있으므로 곡관을 지날 때마다 도면 꼭짓점에 스냅되어 누적 오차가
끊긴다. 설계 5.3 의 위상 매칭이 사라진 게 아니라 **관리자 쪽으로 옮겨간 것**이다.

신호 (설계 5.2)
  바퀴 엔코더 6개 평균   거리 적산 — 주 신호
  시각 오도메트리        휠이 헛도는 구간을 잡아 거리를 교정 — 설계에 없던 추가
  관절 엔코더            곡관 진입·이탈과 회전 방향 — 주 신호
  IMU 요 각속도          회전량 적분, 관절 각도와 교차 검증
  IMU 중력 절대 롤       결함의 원주 위치. 링 J1 을 어디로 돌릴지가 여기서 나온다
"""

from dataclasses import dataclass, asdict, field

import math

STRAIGHT = "STRAIGHT"
CURVE = "CURVE"

DEFAULTS = dict(
    curve_enter_deg=8.0,
    curve_exit_deg=4.0,
    curve_min_m=0.05,        # SR 곡관 호 길이가 0.157m 다.
                             # 이 값이 그보다 크면 곡관이 영영 안 끝난다
    slip_ratio_min=0.30,
    # 시각 오도메트리는 실제 이동을 직접 재고 오차가 ±0.25mm 다.
    # 휠은 '얼마나 돌렸나'이지 '얼마나 갔나'가 아니다. 둘이 어긋나면
    # 시각값이 이긴다.
    visual_trust=0.9,
    elbow_arc_mm=157.08,      # SR 곡관 R=100mm 90° — 표준 규격이라 도면 없이도 안다
    elbow_turn_deg=90.0,
    arc_snap_tol_mm=60.0,
    turn_snap_tol_deg=25.0,
)


@dataclass
class Segment:
    kind: str = STRAIGHT
    length_mm: float = 0.0
    turn_deg: float = 0.0
    direction: str = ""
    arc_start_mm: float = 0.0
    spec_gap_mm: float = 0.0
    snapped: bool = False

    def as_dict(self):
        return asdict(self)


@dataclass
class Pose:
    arc_mm: float = 0.0          # 진입점에서 잰 관 중심선 누적 거리
    heading_deg: float = 0.0     # 진입 방향 기준 누적 회전
    roll_deg: float = 0.0        # 중력 기준 절대 롤
    kind: str = STRAIGHT
    turns: int = 0
    slip_ratio: float = 1.0
    wheel_mm: float = 0.0        # 휠만으로 잰 거리 (비교용)
    corrected_mm: float = 0.0    # 시각 오도메트리 보정분
    segments: list = field(default_factory=list)

    def as_dict(self):
        d = asdict(self)
        d["segments"] = [s.as_dict() if hasattr(s, "as_dict") else s
                         for s in self.segments]
        return d


@dataclass
class DefectMark:
    """결함을 로봇 좌표로 남긴다. 절대 위치는 관리자가 도면과 맞춰 만든다."""
    defect_id: str = ""
    arc_mm: float = 0.0
    roll_deg: float = 0.0
    kind: str = ""
    size_mm: float = 0.0
    confidence: float = 0.0

    def as_dict(self):
        return asdict(self)


class DeadReckoner:
    def __init__(self, params=None):
        self.k = dict(DEFAULTS)
        if params:
            self.k.update({a: b for a, b in params.items() if a in self.k})
        self.p = Pose()
        self._cur = Segment(kind=STRAIGHT)
        self._in_curve = False
        self._curve_s = 0.0
        self._curve_yaw = 0.0
        self._curve_sign = 0.0
        self.defects = []

    # ── 거리 ─────────────────────────────────────────────────────
    def fuse_distance(self, wheel_mps, visual_mps, dt):
        """휠이 헛돌면 시각값을 믿는다.

        설계 5.2 는 휠 6개의 편차로 슬립을 잡지만 6륜이 동시에 미끄러지면
        편차가 안 생겨 못 잡는다. 시각 오도메트리가 그 경우를 메운다.
        """
        w = float(wheel_mps or 0.0)
        self.p.wheel_mm += w * dt * 1000.0
        if visual_mps is None or abs(w) < 1e-4:
            self.p.slip_ratio = 1.0
            return w * dt * 1000.0
        v = float(visual_mps)
        self.p.slip_ratio = max(0.0, min(2.0, abs(v) / max(abs(w), 1e-6)))
        if self.p.slip_ratio < self.k["slip_ratio_min"]:
            use = v                       # 헛돎 — 시각값만 믿는다
        else:
            a = self.k["visual_trust"]
            use = a * v + (1.0 - a) * w
        d = use * dt * 1000.0
        self.p.corrected_mm += (use - w) * dt * 1000.0
        return d

    # ── 곡관 ─────────────────────────────────────────────────────
    def _close_segment(self):
        seg = self._cur
        seg.length_mm = self.p.arc_mm - seg.arc_start_mm
        self.p.segments.append(seg)

    def _open_segment(self, kind):
        self._cur = Segment(kind=kind, arc_start_mm=self.p.arc_mm)

    def step(self, dt, wheel_mps=None, visual_mps=None, joint_deg=0.0,
             yaw_rate_dps=None, roll_deg=None):
        k = self.k
        d = self.fuse_distance(wheel_mps, visual_mps, dt)
        self.p.arc_mm += d
        if roll_deg is not None:
            self.p.roll_deg = float(roll_deg)

        j = float(joint_deg or 0.0)
        yaw = float(yaw_rate_dps or 0.0) * dt

        if not self._in_curve:
            if abs(j) >= k["curve_enter_deg"]:
                self._in_curve = True
                self._curve_s = 0.0
                self._curve_yaw = 0.0
                self._curve_sign = math.copysign(1.0, j)
                self._close_segment()
                self._open_segment(CURVE)
                self.p.kind = CURVE
        else:
            self._curve_s += abs(d) / 1000.0
            self._curve_yaw += yaw
            if abs(j) <= k["curve_exit_deg"] and self._curve_s >= k["curve_min_m"]:
                self._in_curve = False
                seg = self._cur
                seg.length_mm = self.p.arc_mm - seg.arc_start_mm
                # 회전 방향은 관절 엔코더 부호(주 신호),
                # 회전량은 IMU 요 적분. 표준 곡관 90° 와 교차 검증한다.
                turn = self._curve_yaw
                if abs(turn) < 1e-6:
                    turn = self._curve_sign * k["elbow_turn_deg"]
                if abs(abs(turn) - k["elbow_turn_deg"]) <= k["turn_snap_tol_deg"]:
                    turn = math.copysign(k["elbow_turn_deg"], turn)
                    seg.snapped = True
                seg.turn_deg = turn
                seg.direction = "좌" if turn > 0 else "우"
                # 곡관 호 길이는 규격으로 아는 값이다(SR R=100mm, 90°).
                # 구간 길이는 그것으로 교체한다 — 경로 재구성이 정확해진다.
                #
                # 단 **누적 거리는 건드리지 않는다.** 관절 각도 임계값 때문에
                # 곡관 진입·이탈이 안쪽에서 잡혀 구간이 짧게 측정되는데, 그
                # 차이는 이미 앞뒤 직진 구간에 들어가 있다. 누적까지 보정하면
                # 같은 거리를 두 번 세게 된다(실측 +30mm 오차의 원인).
                #
                # 슬립으로 인한 거리 오차는 시각 오도메트리가 이미 연속으로
                # 잡고 있으므로 여기서 또 손댈 필요가 없다.
                seg.spec_gap_mm = seg.length_mm - k["elbow_arc_mm"]
                if abs(seg.spec_gap_mm) <= k["arc_snap_tol_mm"]:
                    seg.length_mm = k["elbow_arc_mm"]
                self.p.segments.append(seg)
                self.p.heading_deg += turn
                self.p.turns += 1
                self._open_segment(STRAIGHT)
                self.p.kind = STRAIGHT
        return self.p

    def mark_defect(self, defect_id, kind="", size_mm=0.0, confidence=0.0):
        """지금 위치에 결함을 남긴다. 원주 위치는 절대 롤 기준이다."""
        m = DefectMark(defect_id=defect_id, arc_mm=self.p.arc_mm,
                       roll_deg=self.p.roll_deg, kind=kind,
                       size_mm=size_mm, confidence=confidence)
        self.defects.append(m)
        return m

    def path(self):
        """지금까지의 구간 열 + 진행 중인 구간."""
        cur = Segment(kind=self._cur.kind,
                      length_mm=self.p.arc_mm - self._cur.arc_start_mm,
                      arc_start_mm=self._cur.arc_start_mm)
        return list(self.p.segments) + [cur]
