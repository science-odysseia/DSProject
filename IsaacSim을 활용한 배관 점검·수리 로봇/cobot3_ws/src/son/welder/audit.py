"""[공용] 복귀 감사 — 후방 카메라로 지나온 구간을 다시 훑는다. ROS 비의존.

설계 7.2 : "임무 종료 시 정찰기 결함 목록과 수리기 처리 목록 전수 대조(감사)"
설계 시나리오 : "복귀 중 재확인 — 후방 카메라로 수리 지점 재확인 및 미탐지 결함 탐색"

3겹 검증의 ③번이다. 용접 직후의 1차 검증(전방 카메라 + 후진)은 그 지점만
본다. 복귀 감사는 **지나온 전 구간을 다른 카메라로 다시** 보므로

  - 1차 검증이 놓친 미수리
  - 정찰 때도 수리 때도 못 본 결함
  - 정답에는 있는데 아무도 손대지 않은 것

를 잡는다. 시뮬레이션이라 defect_spawner 의 정답을 알 수 있는 것이 강점이다.

카메라를 앞뒤로 하나씩 단 이유가 여기서 살아난다. 복귀는 후진으로 하므로
후방 카메라가 진행 방향을 보게 되고, 감사는 공짜로 얻는다.
"""

from dataclasses import dataclass, asdict, field

DEFAULTS = dict(
    match_tol_mm=25.0,
    roll_tol_deg=30.0,
    min_confidence=0.8,
)

REPAIRED_OK = "REPAIRED_OK"
REPAIR_FAILED = "REPAIR_FAILED"
MISSED = "MISSED"
UNTREATED = "UNTREATED"
SPURIOUS = "SPURIOUS"


@dataclass
class AuditRow:
    site_id: str = ""
    verdict: str = ""
    truth_mm: float = 0.0
    seen_mm: float = 0.0
    note: str = ""

    def as_dict(self):
        return asdict(self)


@dataclass
class AuditReport:
    rows: list = field(default_factory=list)
    truth_total: int = 0
    scouted: int = 0
    welded: int = 0
    repaired_ok: int = 0
    repair_failed: int = 0
    missed: int = 0
    untreated: int = 0
    spurious: int = 0

    @property
    def detection_rate(self):
        return self.scouted / self.truth_total if self.truth_total else 0.0

    @property
    def repair_rate(self):
        return self.repaired_ok / self.welded if self.welded else 0.0

    def as_dict(self):
        d = asdict(self)
        d["rows"] = [r.as_dict() if hasattr(r, "as_dict") else r
                     for r in self.rows]
        d["detection_rate"] = self.detection_rate
        d["repair_rate"] = self.repair_rate
        return d


def _dist(a, b):
    """축 거리와 원주 각도를 같이 본다. 관 안이라 두 값이면 위치가 정해진다."""
    dx = abs(float(a.get("x_mm", 0.0)) - float(b.get("x_mm", 0.0)))
    da = abs(float(a.get("roll_deg", 0.0)) - float(b.get("roll_deg", 0.0)))
    da = min(da, 360.0 - da)
    return dx, da


class ReturnAuditor:
    """정답 목록 · 정찰 목록 · 수리 목록 · 복귀 관측을 대조한다."""

    def __init__(self, params=None):
        self.k = dict(DEFAULTS)
        if params:
            self.k.update({a: b for a, b in params.items() if a in self.k})

    def match(self, item, pool):
        """같은 지점인가. 축 거리와 롤 각도가 둘 다 허용 안이어야 한다."""
        best, bestd = None, None
        for c in pool:
            dx, da = _dist(item, c)
            if dx <= self.k["match_tol_mm"] and da <= self.k["roll_tol_deg"]:
                if bestd is None or dx < bestd:
                    best, bestd = c, dx
        return best

    def run(self, truth, scouted, welded, rear_seen):
        """
        truth      defect_spawner 가 실제로 배치한 결함 (정답)
        scouted    정찰기가 검출해 등록한 결함
        welded     수리기가 용접을 시도한 결과 (state 필드 포함)
        rear_seen  복귀 중 후방 카메라가 검출한 결함
        """
        rep = AuditReport(truth_total=len(truth))

        for t in truth:
            sid = t.get("id", "")
            s = self.match(t, scouted)
            w = self.match(t, welded) if s else None
            r = self.match(t, rear_seen)
            row = AuditRow(site_id=sid, truth_mm=float(t.get("x_mm", 0.0)),
                           seen_mm=float(r.get("x_mm", 0.0)) if r else 0.0)

            if s:
                rep.scouted += 1
            if w:
                rep.welded += 1

            if w and w.get("state") == "SUCCESS":
                if r is None:
                    row.verdict = REPAIRED_OK
                    row.note = "용접 성공, 복귀 감사에서도 미검출"
                    rep.repaired_ok += 1
                else:
                    # 1차 검증은 통과했는데 복귀 때 다시 보인다.
                    # 냉각 후 균열이 재발했거나 1차 검증이 틀렸다.
                    row.verdict = REPAIR_FAILED
                    row.note = "1차 검증 통과했으나 복귀 감사에서 재검출"
                    rep.repair_failed += 1
            elif w:
                row.verdict = REPAIR_FAILED
                row.note = f"용접 실패 ({w.get('verdict', '')})"
                rep.repair_failed += 1
            elif s:
                row.verdict = UNTREATED
                row.note = "정찰은 했으나 수리 시도 없음"
                rep.untreated += 1
            elif r:
                row.verdict = MISSED
                row.note = "정찰이 놓쳤고 복귀 감사에서 발견"
                rep.missed += 1
            else:
                row.verdict = MISSED
                row.note = "아무도 못 봤다 — 정답에만 있음"
                rep.missed += 1
            rep.rows.append(row)

        # 정답에 없는데 검출된 것. 오검출이거나 비드를 결함으로 본 것이다.
        for r in rear_seen:
            if self.match(r, truth) is None:
                rep.spurious += 1
                rep.rows.append(AuditRow(
                    site_id=r.get("id", "?"), verdict=SPURIOUS,
                    seen_mm=float(r.get("x_mm", 0.0)),
                    note="정답에 없는 검출 — 오검출 또는 비드 오인"))
        return rep


def format_report(rep):
    lines = []
    lines.append(f"{'지점':>10} {'판정':>14} {'정답 x':>9} {'관측 x':>9}  비고")
    lines.append("-" * 78)
    for r in rep.rows:
        lines.append(f"{r.site_id:>10} {r.verdict:>14} {r.truth_mm:9.1f} "
                     f"{r.seen_mm:9.1f}  {r.note}")
    lines.append("-" * 78)
    lines.append(f"  정답 {rep.truth_total}  정찰 {rep.scouted} "
                 f"(검출률 {rep.detection_rate * 100:.1f}%)  "
                 f"용접 시도 {rep.welded}")
    lines.append(f"  수리 성공 {rep.repaired_ok} / 실패 {rep.repair_failed} "
                 f"(성공률 {rep.repair_rate * 100:.1f}%)")
    lines.append(f"  미검출 {rep.missed}  미처리 {rep.untreated}  "
                 f"오검출 {rep.spurious}")
    return "\n".join(lines)
