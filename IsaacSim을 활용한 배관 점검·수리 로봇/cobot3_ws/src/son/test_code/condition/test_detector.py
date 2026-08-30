"""[오프라인] 판정기 시험

실행:  python3 make_scenes.py && python3 test_detector.py
"""

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "condition"))
from detector import DEFAULTS, PipeConditionDetector, normalize_depth

HERE = Path(__file__).resolve().parent
SCENES = HERE / "scenes"
OFFSET_TOL_MM = 1.5


def expected_speed(truth, off):
    if truth in ("DISCONNECTED", "UNDETERMINED"):
        return "stop"
    if off <= DEFAULTS["offset_normal_mm"] or off <= DEFAULTS["offset_slow_mm"]:
        return "full"
    if off <= DEFAULTS["offset_max_mm"]:
        return "slow"
    return "stop"


def _mark(arr, cells, value):
    a = arr.copy()
    for i, j in cells:
        a[i, j] = value
    return a


def main():
    meta_path = SCENES / "scenes_meta.json"
    if not meta_path.is_file():
        print("[중단] 시험 장면이 없다. 먼저 만들 것:")
        print(f"    cd {SCENES.parent}  &&  python3 make_scenes.py")
        return 1
    meta = json.loads(meta_path.read_text())
    missing = [n for n in meta["scenes"]
               if not (SCENES / f"{n}_depth.npy").is_file()]
    if missing:
        print(f"[중단] 장면 {len(missing)}종의 .npy 가 없다 "
              f"(용량 때문에 배포본에서 빠진다). 먼저 만들 것:")
        print(f"    cd {SCENES.parent}  &&  python3 make_scenes.py")
        return 1
    det = PipeConditionDetector(meta)
    rows, ok = [], True

    print("=" * 104)
    print(f"{'장면':20s} {'정답':14s} {'판정':14s} {'원형도':>7s} {'조인트(m)':>10s} "
          f"{'오프셋':>10s} {'무효':>7s} {'속도':>6s}  결과")
    print("=" * 104)
    for name, info in meta["scenes"].items():
        truth, off = info["truth"], info["offset_mm"]
        depth = np.load(SCENES / f"{name}_depth.npy")
        c = det.run(depth, joint_angle_deg=0.0)
        good = c.state == truth
        if truth == "MISALIGNMENT":
            good = good and abs(c.offset_mm - off) <= OFFSET_TOL_MM
        good = good and c.speed == expected_speed(truth, off)
        ok = ok and good
        rows.append((truth, c))
        exp = f"  (기대 {off:.0f}mm)" if truth == "MISALIGNMENT" else ""
        print(f"{name:20s} {truth:14s} {c.state:14s} {c.circularity:7.3f} "
              f"{c.joint_range_m:10.3f} {c.offset_mm:8.2f}mm "
              f"{c.invalid_ratio * 100:6.2f}% {c.speed:>6s}  "
              f"{'OK' if good else 'FAIL'}{exp}")
        if not good:
            print(f"{'':20s} → {c.reason}")

    print("-" * 104)
    depth = np.load(SCENES / "disconnected_depth.npy")
    c = det.run(depth, joint_angle_deg=20.0)
    bend_ok = c.state == "NORMAL"
    ok = ok and bend_ok
    print(f"곡관 예외 (관절 20°, 단절 장면) → {c.state}  "
          f"{'OK' if bend_ok else 'FAIL'}   {c.reason}")

    print("-" * 104)
    print("임계값 분리도 — 두 무리 사이가 벌어져 있어야 임계값이 안정적이다")

    def spread(pred, field):
        v = [getattr(c, field) for t, c in rows if pred(t, c)]
        return (min(v), max(v)) if v else (float('nan'),) * 2

    inv_ok = spread(lambda t, c: t != "DISCONNECTED", "invalid_ratio")
    inv_bad = spread(lambda t, c: t == "DISCONNECTED", "invalid_ratio")
    print(f"  무효 픽셀 비율   정상군 {inv_ok[0] * 100:6.2f}~{inv_ok[1] * 100:6.2f}%"
          f"   단절군 {inv_bad[0] * 100:6.2f}~{inv_bad[1] * 100:6.2f}%"
          f"   임계 {DEFAULTS['invalid_ratio_max'] * 100:.1f}%")

    for field, key, lo_is_ok, fmt in (
            ("circularity", "circularity_min", True, "6.3f"),
            ("roughness", "roughness_max", False, "7.4f")):
        a = spread(lambda t, c: t in ("NORMAL", "MISALIGNMENT"), field)
        b = spread(lambda t, c: t == "UNDETERMINED", field)
        gap = (a[0] - b[1]) if lo_is_ok else (b[0] - a[1])
        print(f"  {field:12s}     정상군 {a[0]:{fmt}}~{a[1]:{fmt}}"
              f"   불규칙군 {b[0]:{fmt}}~{b[1]:{fmt}}"
              f"   임계 {DEFAULTS[key]}")
        print(f"  {'':12s}     → 간극 {gap:+.4f}  "
              f"({'분리됨' if gap > 0 else '겹침 — 이 지표로는 못 가른다'})")

    print("-" * 104)
    errs = [(abs(c.offset_mm - o), o, c.offset_mm)
            for (t, c), o in zip(rows, [i["offset_mm"] for i in
                                        meta["scenes"].values()])
            if t == "MISALIGNMENT"]
    if errs:
        print(f"  오프셋 실측 오차 최대 {max(e[0] for e in errs):.2f} mm  "
              f"(허용 {OFFSET_TOL_MM} mm)")

    print("=" * 104)
    print("단위 정규화 — inf 가 섞여도 안전한가 (2026-08-04 실기에서 터진 결함)")
    print("=" * 104)
    print("  Isaac Sim 은 빈 공간(관 단절 너머)을 inf 로 준다. 실측 확정.")
    print("  실제 장면은 근접 관벽(50mm)과 관 안쪽 먼 곳(1200mm)이 같이 잡힌다.")

    def scene(unit_mm=False):
        a = np.full((8, 8), 1200.0 if unit_mm else 1.200)
        a[:, :4] = 50.0 if unit_mm else 0.050
        return a

    print(f"\n  {'입력':>30} {'변환':>6} {'관벽이 얼마로 읽히나':>22} {'기대':>8}")
    cases = [
        ("정상 (m)", scene(), False),
        ("m + 단절 1픽셀 inf", _mark(scene(), [(0, 7)], np.inf), False),
        ("m + 단절 절반 inf", _mark(scene(), [(i, j) for i in range(8)
                                          for j in range(4, 8)], np.inf), False),
        ("m + NaN", _mark(scene(), [(0, 7)], np.nan), False),
        ("mm 로 들어옴", scene(True), True),
        ("mm + 단절 inf", _mark(scene(True), [(0, 7)], np.inf), True),
        ("전부 inf", np.full((8, 8), np.inf), False),
    ]
    for name, arr, want in cases:
        out, scaled = normalize_depth(arr)
        fin = out[np.isfinite(out)]
        read = f"{fin.min() * 1000:.3f} mm" if fin.size else "유효값 없음"
        good = scaled == want
        ok = ok and good
        print(f"  {name:>30} {'예' if scaled else '아니오':>6} {read:>22} "
              f"{'변환' if want else '유지':>8}  {'OK' if good else 'FAIL'}")

    bad = _mark(scene(), [(0, 7)], np.inf)
    old = bad / 1000.0 if np.nanmax(bad) > 100.0 else bad
    print(f"\n  [옛 코드] nanmax = {np.nanmax(bad)} → 100 초과로 오판 → "
          f"관벽이 {old[np.isfinite(old)].min() * 1000:.5f} mm 로 읽혔다")
    print("  [지금 ] 유한값만 보므로 변환하지 않는다 — 50.000 mm 유지")
    ok = ok and old[np.isfinite(old)].min() * 1000 < 1.0

    print("\n  [한계] 유효 픽셀이 전부 100 미만이면 mm 인지 m 인지 못 가른다.")
    flat, sc = normalize_depth(np.full((8, 8), 50.0))
    print(f"    50.0 균일 영상 → 변환 {'함' if sc else '안 함'}. 관 안에서 원거리가")
    print("    하나도 안 잡히는 상황이라 실사용엔 없지만 임계 100 은 그 가정에 기댄다.")
    ok = ok and not sc

    print("=" * 104)
    print("전체 판정:", "통과" if ok else "실패")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
