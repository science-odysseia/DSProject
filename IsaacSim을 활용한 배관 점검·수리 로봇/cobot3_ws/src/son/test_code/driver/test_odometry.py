"""[오프라인] 시각 오도메트리 시험 — 알려진 전진량을 얼마나 맞히나.

배관 장면을 전진량만 바꿔 두 장 그린 뒤, 광류가 그 전진량을 복원하는지 본다.
정답을 아는 상태에서 재므로 오차를 그대로 읽을 수 있다.

실행:  python3 test_odometry.py
"""

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON / "condition"))
sys.path.insert(0, str(SON / "test_code" / "condition"))

from odometry import VisualOdometry
import make_scenes as MS

DT = 0.1
STEPS_MM = [1.0, 2.0, 5.0, 10.0, 20.0]
TOL_FRAC = 0.20
TOL_ABS_MM = 0.5


def pair(advance_mm, state="NORMAL", offset=0.0, seed=7):
    a_rgb, a_d = MS.render(state, offset, seed=seed, advance_mm=0.0)
    b_rgb, b_d = MS.render(state, offset, seed=seed, advance_mm=advance_mm)
    return (a_rgb, a_d), (b_rgb, b_d)


def measure(advance_mm, **kw):
    (r0, d0), (r1, d1) = pair(advance_mm, **kw)
    vo = VisualOdometry()
    vo.step(r0, d0, DT)
    return vo.step(r1, d1, DT)


def main():
    ok = True
    print("=" * 84)
    print("시각 오도메트리 — 알려진 전진량 복원")
    print("=" * 84)
    print(f"{'정답(mm)':>10} {'측정(mm)':>10} {'오차(mm)':>10} {'오차율':>8} "
          f"{'추적':>6} {'일치':>6}  결과")
    print("-" * 84)
    for mm in STEPS_MM:
        r = measure(mm)
        got = r.advance_m * 1000.0
        err = got - mm
        frac = abs(err) / mm
        good = r.valid and (abs(err) <= TOL_ABS_MM or frac <= TOL_FRAC)
        ok = ok and good
        print(f"{mm:10.1f} {got:10.2f} {err:+10.2f} {frac * 100:7.1f}% "
              f"{r.tracked:6d} {r.inliers:6d}  {'OK' if good else 'FAIL'}"
              + ("" if r.valid else f"  ({r.reason})"))

    print("-" * 84)
    print("정지 상태 — 0 에 가까워야 한다 (끼임 판정의 근거)")
    r = measure(0.0)
    still = abs(r.advance_m * 1000.0)
    good = still <= 0.5
    ok = ok and good
    print(f"  전진 0mm  → 측정 {r.advance_m * 1000.0:+.3f} mm  "
          f"{'OK' if good else 'FAIL'}")

    print("-" * 84)
    print("후진 — 부호가 뒤집혀야 한다")
    (r0, d0), (r1, d1) = pair(5.0)
    vo = VisualOdometry()
    vo.step(r1, d1, DT)
    rb = vo.step(r0, d0, DT)
    good = rb.valid and rb.advance_m < 0
    ok = ok and good
    print(f"  전진 -5mm → 측정 {rb.advance_m * 1000.0:+.2f} mm  "
          f"{'OK' if good else 'FAIL'}")

    print("-" * 84)
    print("어긋난 관에서도 되는가 (특징이 벽에만 있어도 무방해야 함)")
    for state, off in (("MISALIGNMENT", 5.0), ("MISALIGNMENT", 12.0)):
        r = measure(5.0, state=state, offset=off)
        got = r.advance_m * 1000.0
        good = r.valid and abs(got - 5.0) <= max(TOL_ABS_MM, 5.0 * TOL_FRAC)
        ok = ok and good
        print(f"  offset {off:4.0f}mm, 전진 5mm → {got:6.2f} mm  "
              f"{'OK' if good else 'FAIL'}")

    print("-" * 84)
    print("속도 환산 (dt = 0.1s)")
    r = measure(10.0)
    print(f"  전진 10mm / 0.1s → {r.speed_mps:.4f} m/s  (기대 0.1000)")

    print("=" * 84)
    print("전체 판정:", "통과" if ok else "실패")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
