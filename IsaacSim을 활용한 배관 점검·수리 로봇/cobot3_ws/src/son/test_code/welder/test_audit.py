"""[오프라인] 복귀 감사 시험 — 1차 검증이 놓친 것을 잡는가.

3겹의 ③번이다. 용접 직후 1차 검증은 그 지점만 보지만, 복귀 감사는
지나온 전 구간을 후방 카메라로 다시 훑는다.

실행:  python3 test_audit.py
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "welder"))

from audit import (MISSED, REPAIRED_OK, REPAIR_FAILED, SPURIOUS, UNTREATED,
                   ReturnAuditor, format_report)


def d(i, x, roll=0.0, **kw):
    return dict(id=i, x_mm=x, roll_deg=roll, **kw)


def main():
    ok = []
    truth = [d("D1", 500), d("D2", 1200, 90), d("D3", 2000, 180),
             d("D4", 2600, 270), d("D5", 3100)]
    scouted = [d("D1", 502), d("D2", 1198, 92), d("D3", 2005, 178),
               d("D4", 2603, 271)]                       # D5 는 놓침
    welded = [d("D1", 501, state="SUCCESS", verdict="수리 성공"),
              d("D2", 1199, 90, state="SUCCESS", verdict="수리 성공"),
              d("D3", 2003, 179, state="FAILED", verdict="정렬 초과")]
    #                                            D4 는 정찰만 하고 수리 안 함
    rear = [d("D3", 2004, 179),      # 실패했으니 여전히 보여야 함
            d("D2", 1201, 91),       # 1차 통과했는데 복귀 때 재검출 ← 감사가 잡는 것
            d("D5", 3098),           # 정찰이 놓친 것
            d("X9", 1750, 45)]       # 정답에 없는 오검출

    rep = ReturnAuditor().run(truth, scouted, welded, rear)
    print("=" * 78)
    print("복귀 감사")
    print("=" * 78)
    print(format_report(rep))

    v = {r.site_id: r.verdict for r in rep.rows}
    print()
    print("-" * 78)
    cases = [
        ("D1 용접 성공 + 복귀에도 미검출", v.get("D1"), REPAIRED_OK),
        ("D2 1차 통과했으나 복귀 재검출", v.get("D2"), REPAIR_FAILED),
        ("D3 용접 실패", v.get("D3"), REPAIR_FAILED),
        ("D4 정찰만 하고 수리 안 함", v.get("D4"), UNTREATED),
        ("D5 정찰이 놓친 것", v.get("D5"), MISSED),
        ("X9 정답에 없는 검출", v.get("X9"), SPURIOUS),
    ]
    for label, got, want in cases:
        good = got == want
        ok.append(good)
        print(f"  {label:34s} {str(got):14s} (기대 {want:14s}) "
              f"{'OK' if good else 'FAIL'}")

    print("-" * 78)
    checks = [("검출률 4/5", abs(rep.detection_rate - 0.8) < 1e-6),
              ("용접 시도 3", rep.welded == 3),
              ("수리 성공 1", rep.repaired_ok == 1),
              ("수리 실패 2", rep.repair_failed == 2),
              ("오검출 1", rep.spurious == 1)]
    for label, good in checks:
        ok.append(good)
        print(f"  {label:34s} {'OK' if good else 'FAIL'}")

    print("-" * 78)
    print("감사가 없었다면 D2 는 성공으로 남았을 것이다 —")
    print("1차 검증만 보면 3건 중 2건 성공(66.7%), 감사 후 1건(33.3%)")
    ok.append(rep.repair_rate < 0.5)

    print("=" * 78)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
