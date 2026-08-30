"""[오프라인] 추측 항법 시험 — 도면 없이 지나온 길을 얼마나 맞히나.

로봇은 배관 도면을 갖지 않는다. 바퀴 회전과 관절 꺾임만으로 위치를 남긴다.
정답을 아는 합성 주행을 넣고 재구성 결과를 대조한다.

핵심 질문 셋
  - 휠만 쓸 때와 시각 오도메트리를 섞을 때 거리 오차가 얼마나 다른가
  - 곡관 진입·이탈과 회전 방향을 제대로 잡는가
  - 규격 호 길이(157.08mm) 스냅이 오차를 실제로 줄이는가

실행:  python3 test_deadreckon.py
"""

import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON / "localization"))

from deadreckon import CURVE, STRAIGHT, DeadReckoner

DT = 0.05
V = 0.10
ELBOW_ARC = 157.08


def mission():
    """정답 경로. (종류, 길이mm, 회전각deg)"""
    return [("S", 1200.0, 0.0), ("C", ELBOW_ARC, +90.0),
            ("S", 800.0, 0.0), ("C", ELBOW_ARC, -90.0),
            ("S", 600.0, 0.0), ("C", ELBOW_ARC, +90.0),
            ("S", 450.0, 0.0)]


def drive(slip_zones=(), noise=0.0, seed=0):
    """합성 주행. 정답과 센서값을 함께 낸다.

    slip_zones : [(시작mm, 끝mm, 휠 과다보고 배수)]  휠이 헛도는 구간
    """
    rng = np.random.default_rng(seed)
    truth_arc = 0.0
    truth_turn = 0.0
    log = []
    for kind, length, turn in mission():
        n = int(round(length / 1000.0 / V / DT))
        for i in range(n):
            frac = (i + 1) / n
            d = length / n
            truth_arc += d
            # 관절 각도 — 곡관에서만 꺾인다. 진입·이탈이 완만하다.
            if kind == "C":
                j = 40.0 * math.sin(math.pi * frac) * (1 if turn > 0 else -1)
                yaw = turn / n / DT
            else:
                j = 0.0
                yaw = 0.0
            if kind == "C":
                truth_turn += turn / n
            v_true = d / 1000.0 / DT
            mult = 1.0
            for a, b, m in slip_zones:
                if a <= truth_arc <= b:
                    mult = m
            wheel = v_true * mult
            vis = v_true * (1.0 + rng.normal(0, noise)) if noise else v_true
            log.append(dict(wheel=wheel, visual=vis, joint=j, yaw=yaw,
                            truth_arc=truth_arc, truth_turn=truth_turn))
    return log


def run(log, use_visual=True):
    dr = DeadReckoner()
    for s in log:
        dr.step(DT, wheel_mps=s["wheel"],
                visual_mps=s["visual"] if use_visual else None,
                joint_deg=s["joint"], yaw_rate_dps=s["yaw"], roll_deg=0.0)
    return dr


def main():
    ok = []
    truth_total = sum(l for _, l, _ in mission())
    truth_turns = [(k, t) for k, _, t in mission() if k == "C"]

    print("=" * 80)
    print("정답 경로")
    print("=" * 80)
    for k, l, t in mission():
        print(f"  {'직진' if k == 'S' else '곡관':6s} {l:8.2f} mm"
              + (f"   회전 {t:+.0f}°" if t else ""))
    print(f"  {'합계':6s} {truth_total:8.2f} mm,  곡관 {len(truth_turns)}개")

    print("\n" + "=" * 80)
    print("① 슬립 없음 — 기본 정확도")
    print("=" * 80)
    log = drive()
    dr = run(log)
    err = dr.p.arc_mm - truth_total
    print(f"  거리   {dr.p.arc_mm:9.2f} mm   오차 {err:+7.2f} mm "
          f"({abs(err) / truth_total * 100:.3f}%)")
    print(f"  곡관   {dr.p.turns}개 검출 (정답 {len(truth_turns)})")
    ok.append(abs(err) < 2.0 and dr.p.turns == len(truth_turns))

    print("\n  구간 재구성")
    print(f"  {'#':>3} {'종류':>6} {'길이(mm)':>10} {'회전':>8} {'방향':>5} {'스냅':>5}")
    segs = [s for s in dr.p.segments]
    for i, s in enumerate(segs):
        print(f"  {i:3d} {'직진' if s.kind == STRAIGHT else '곡관':>6} "
              f"{s.length_mm:10.2f} "
              f"{(f'{s.turn_deg:+.1f}°' if s.kind == CURVE else '-'):>8} "
              f"{s.direction:>5} {'예' if s.snapped else '':>5}")
    dirs = [s.direction for s in segs if s.kind == CURVE]
    want = ["좌" if t > 0 else "우" for _, t in truth_turns]
    good = dirs == want
    ok.append(good)
    print(f"\n  회전 방향 {dirs} (정답 {want})  {'OK' if good else 'FAIL'}")

    print("\n" + "=" * 80)
    print("② 슬립 구간 — 휠만 쓸 때 vs 시각 오도메트리 병용")
    print("=" * 80)
    zones = [(300.0, 700.0, 1.8), (1600.0, 1900.0, 2.4)]
    print(f"  휠이 헛도는 구간: {zones[0][0]:.0f}~{zones[0][1]:.0f}mm (1.8배), "
          f"{zones[1][0]:.0f}~{zones[1][1]:.0f}mm (2.4배)")
    log = drive(slip_zones=zones)
    a = run(log, use_visual=False)
    b = run(log, use_visual=True)
    ea, eb = a.p.arc_mm - truth_total, b.p.arc_mm - truth_total
    print(f"\n  {'':22s} {'거리(mm)':>11} {'오차(mm)':>10} {'오차율':>8}")
    print(f"  {'휠만':22s} {a.p.arc_mm:11.2f} {ea:+10.2f} "
          f"{abs(ea) / truth_total * 100:7.2f}%")
    print(f"  {'휠 + 시각 오도메트리':22s} {b.p.arc_mm:11.2f} {eb:+10.2f} "
          f"{abs(eb) / truth_total * 100:7.2f}%")
    imp = (abs(ea) - abs(eb)) / max(abs(ea), 1e-9) * 100
    print(f"\n  오차 {imp:.1f}% 감소")
    ok.append(abs(eb) < abs(ea) * 0.3)
    print(f"  최저 슬립률 관측 {b.p.slip_ratio:.3f}, "
          f"시각 보정 누적 {b.p.corrected_mm:+.1f} mm")

    print("\n" + "=" * 80)
    print("③ 규격 호 길이 스냅 — 곡관 구간 오차를 끊는가")
    print("=" * 80)
    log = drive(slip_zones=[(1250.0, 1400.0, 1.6)])   # 곡관 한복판에서 헛돎
    c = run(log, use_visual=False)
    cs = [s for s in c.p.segments if s.kind == CURVE]
    print(f"  {'곡관':>4} {'재구성 길이':>12} {'규격':>10} {'측정 차이':>11} {'스냅':>6}")
    for i, s in enumerate(cs):
        print(f"  {i:4d} {s.length_mm:12.2f} {ELBOW_ARC:10.2f} "
              f"{s.spec_gap_mm:+11.2f} {'예' if s.snapped else '아니오':>6}")
    good = all(abs(s.length_mm - ELBOW_ARC) < 0.01 for s in cs)
    ok.append(good)
    print(f"\n  곡관 길이가 전부 규격값으로 교체됨  {'OK' if good else 'FAIL'}")

    print("\n" + "=" * 80)
    print("④ 결함 기록 — 위치와 원주 방향")
    print("=" * 80)
    dr = DeadReckoner()
    marks = []
    for i, s in enumerate(drive()):
        dr.step(DT, wheel_mps=s["wheel"], visual_mps=s["visual"],
                joint_deg=s["joint"], yaw_rate_dps=s["yaw"],
                roll_deg=(i * 0.7) % 360.0 - 180.0)
        if i in (120, 300, 520):
            marks.append(dr.mark_defect(f"D{len(marks) + 1}", kind="crack",
                                        size_mm=6.0, confidence=0.91))
    for m in marks:
        print(f"  {m.defect_id}  호 {m.arc_mm:8.1f} mm  롤 {m.roll_deg:+7.1f}°  "
              f"{m.kind}")
    print("\n  롤은 링 J1 을 어디로 돌릴지 정하는 값이다 — IMU 없이는 못 구한다")
    ok.append(len(marks) == 3 and all(m.arc_mm > 0 for m in marks))

    print("=" * 80)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
