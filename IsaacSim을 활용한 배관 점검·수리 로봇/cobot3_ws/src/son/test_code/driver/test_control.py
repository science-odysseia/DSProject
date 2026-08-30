"""[오프라인] 주행 제어 시험 — FSM 전이와 속도 법칙.

조건 스칼라를 시나리오로 만들어 넣고 상태·속도가 의도대로 가는지 본다.
로봇도 시뮬레이터도 없이 순수 로직만 돌린다.

실행:  python3 test_control.py
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON / "driver"))

from control import (CRUISE, DONE, HOLD, IDLE, RECOVER, RETURN, SLOW,
                     DriveController)

DT = 0.1


def cond(state="NORMAL", speed="full", passable=True, aperture=90000.0,
         incidence=0.0, reason=""):
    return dict(state=state, speed=speed, passable=passable,
                aperture_px=aperture, incidence_deg=incidence, reason=reason)


def run(ctl, n, dt=DT, **kw):
    """n 스텝 돌리고 마지막 상태를 돌려준다. wheel 은 지령 속도를 따른다고 가정."""
    last = None
    for _ in range(n):
        wheel = kw.pop("wheel", None)
        w = ctl.s.v_cmd if wheel is None else wheel
        vis = kw.get("visual", w)
        last = ctl.step(dt, cond=kw.get("cond"), wheel_mps=w, visual_mps=vis,
                        start=kw.get("start", False),
                        recall=kw.get("recall", False))
    return last


def check(label, got, want, ok_list):
    good = got == want
    ok_list.append(good)
    print(f"  {label:44s} {str(got):10s} (기대 {want:10s}) {'OK' if good else 'FAIL'}")
    return good


def main():
    ok = []
    print("=" * 78)
    print("FSM 전이")
    print("=" * 78)

    c = DriveController()
    check("초기", c.s.state, IDLE, ok)
    run(c, 1, cond=cond(), start=True)
    check("출발 지시 → 주행", c.s.state, CRUISE, ok)

    run(c, 20, cond=cond())
    v_cruise = c.s.v_cmd
    print(f"  {'정상 주행 속도':44s} {v_cruise:.4f} m/s (최대 0.1000)")
    ok.append(0.05 < v_cruise <= 0.1001)

    run(c, 20, cond=cond(state="MISALIGNMENT", speed="slow"))
    check("어긋남 3~8mm → 저속", c.s.state, SLOW, ok)
    print(f"  {'저속 속도':44s} {c.s.v_cmd:.4f} m/s")
    ok.append(c.s.v_cmd < v_cruise * 0.6)

    run(c, 10, cond=cond(state="DISCONNECTED", speed="stop", passable=False,
                         reason="무효 픽셀 초과"))
    check("단절 → 정지", c.s.state, HOLD, ok)
    ok.append(abs(c.s.v_cmd) < 1e-6)
    print(f"  {'정지 후 속도':44s} {c.s.v_cmd:.4f} m/s")

    run(c, 40, cond=cond())
    check("통행 회복 → 재주행", c.s.state, CRUISE, ok)

    print("-" * 78)
    print("정지 지연 — 감속이 가속보다 빨라야 한다")
    c2 = DriveController()
    run(c2, 40, cond=cond(), start=True)
    v0 = c2.s.v_cmd
    steps = 0
    while c2.s.v_cmd > 1e-4 and steps < 100:
        c2.step(DT, cond=cond(state="DISCONNECTED", speed="stop",
                              passable=False), wheel_mps=c2.s.v_cmd)
        steps += 1
    print(f"  {v0:.3f} m/s 에서 완전 정지까지 {steps * DT:.1f}s")
    ok.append(steps * DT <= 0.5)

    print("-" * 78)
    print("끼임 판정 — 바퀴는 도는데 실제로 안 나감")
    c3 = DriveController()
    run(c3, 30, cond=cond(), start=True)
    check("주행 중", c3.s.state, CRUISE, ok)
    for _ in range(30):
        c3.step(DT, cond=cond(), wheel_mps=0.08, visual_mps=0.005)
    check("휠 0.08 / 시각 0.005 → 끼임", c3.s.state, RECOVER, ok)
    print(f"  {'슬립률':44s} {c3.s.slip_ratio:.3f}")
    for _ in range(40):
        c3.step(DT, cond=cond(), wheel_mps=c3.s.v_cmd, visual_mps=c3.s.v_cmd)
    check("후진 완료 → 재시도", c3.s.state, CRUISE, ok)

    print("-" * 78)
    print("정상 슬립은 끼임이 아니다")
    c4 = DriveController()
    run(c4, 30, cond=cond(), start=True)
    for _ in range(40):
        c4.step(DT, cond=cond(), wheel_mps=0.08, visual_mps=0.065)
    check("휠 0.08 / 시각 0.065 (81%)", c4.s.state, CRUISE, ok)

    print("-" * 78)
    print("끼임 3회 초과 → 임무 중단")
    c5 = DriveController()
    run(c5, 20, cond=cond(), start=True)
    # 2026-08-07 — 거리 적산이 시각 오도메트리 기반이 되면서 제자리 끼임
    # (시각 0.0)은 거리를 만들지 않는다. 그 전에는 휠 적분이라 끼임 중에도
    # 거리가 쌓여 이 시나리오가 우연히 성립했다. "나가다 끼었다" 를 반영해
    # 실제 전진(시각=휠)을 먼저 쌓는다 — 안 그러면 4회째 포기 시점에 이미
    # 출발점이라 RETURN 이 즉시 DONE 으로 끝나 버린다(그것도 올바른 동작이다).
    for _ in range(100):
        c5.step(DT, cond=cond(), wheel_mps=c5.s.v_cmd, visual_mps=c5.s.v_cmd)
    for _ in range(4):
        for _ in range(30):
            c5.step(DT, cond=cond(), wheel_mps=0.08, visual_mps=0.0)
        for _ in range(40):
            c5.step(DT, cond=cond(), wheel_mps=c5.s.v_cmd,
                    visual_mps=c5.s.v_cmd)
    check("4회째 → 복귀", c5.s.state, RETURN, ok)

    print("-" * 78)
    print("임무 거리 도달 → 복귀")
    c6 = DriveController(dict(mission_range_m=1.0))
    run(c6, 20, cond=cond(), start=True)
    for _ in range(400):
        c6.step(DT, cond=cond(), wheel_mps=c6.s.v_cmd, visual_mps=c6.s.v_cmd)
        if c6.s.state == RETURN:
            break
    check("1m 도달 → 복귀", c6.s.state, RETURN, ok)
    # 전환 직후에는 아직 감속 중이다. 부호가 뒤집히기까지 시간이 걸린다.
    for _ in range(30):
        c6.step(DT, cond=cond(), wheel_mps=c6.s.v_cmd, visual_mps=c6.s.v_cmd)
    ok.append(c6.s.v_cmd < 0)
    print(f"  {'복귀 3s 후 속도(음수여야 함)':44s} {c6.s.v_cmd:.4f} m/s")

    print("-" * 78)
    print("판정 두절 → 정지 (타임아웃)")
    c7 = DriveController()
    run(c7, 30, cond=cond(), start=True)
    for _ in range(30):
        c7.step(DT, cond=None, wheel_mps=c7.s.v_cmd, visual_mps=c7.s.v_cmd)
    check("condition 미수신 1s 초과", c7.s.state, HOLD, ok)

    print("-" * 78)
    print("속도 법칙 — 전방 상황에 따른 감속")
    c8 = DriveController()
    print(f"  {'개구부 면적':>16} {'입사각':>8} {'개방도':>8} {'곡관':>8} {'목표속도':>10}")
    for ap, inc in ((90000, 0), (90000, 10), (90000, 25), (40000, 0),
                    (15000, 0), (15000, 20)):
        o = c8.openness(ap)
        cu = c8.curve_factor(inc)
        print(f"  {ap:16,d} {inc:7.0f}° {o:8.3f} {cu:8.3f} "
              f"{0.1 * o * cu:9.4f} m/s")
    ok.append(c8.openness(90000) > c8.openness(15000))
    ok.append(c8.curve_factor(0) > c8.curve_factor(25))

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
