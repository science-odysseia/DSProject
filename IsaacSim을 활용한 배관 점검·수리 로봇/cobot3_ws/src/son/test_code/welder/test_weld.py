"""[오프라인] 용접 시퀀스 + 3겹 검증 시험.

핵심 질문 — **덮어서 안 보이니까 수리됨** 이 되어버리지 않는가.
그걸 막는 세 겹이 실제로 각각 작동하는지 본다.

  ① 정렬 오차   토치가 빗나가면 실패해야 한다
  ② Depth 프로파일  검출기가 놓쳐도 파여 있으면 실패여야 한다
  ③ 검출기      메워졌는데 여전히 검출되면 실패여야 한다

②가 핵심이다. 결함 있는 벽과 메워진 벽의 Depth 를 각각 합성해 넣고,
검출기 결과를 일부러 거짓으로 줘서 프로파일이 그것을 뒤집는지 확인한다.

실행:  python3 test_weld.py
"""

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SON = HERE.parents[1]
sys.path.insert(0, str(SON / "welder"))

from weld import (ALIGN, APPROACH, ARC, COOL, FAILED, RECHECK, RETRY,
                  SUCCESS, VERIFY, WeldSequencer, radial_profile)

W, H = 1280, 720
HFOV = 140.0
F = (W / 2.0) / np.radians(HFOV / 2.0)
INTR = dict(f_fish=F, ppx=W / 2.0, ppy=H / 2.0)
BORE = 0.050
DT = 0.1


def wall_depth(radius_profile_mm, site_x=0.20):
    """관 안에서 본 Depth 영상을 만든다.

    radius_profile_mm(u_mm, arc_mm) 가 그 지점의 벽 반경(mm)을 준다.
    용접 지점은 카메라 앞 site_x 에 있다고 둔다.
    """
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    du, dv = u - INTR["ppx"], v - INTR["ppy"]
    rp = np.hypot(du, dv)
    theta = np.clip(rp / F, 1e-6, np.pi / 2 * 0.99)
    phi = np.arctan2(dv, du)

    # 광선이 반경 R 의 원통과 만나는 지점: R = D sin(theta), z = D cos(theta)
    # 벽 반경이 위치에 따라 다르므로 두 번 반복해 수렴시킨다.
    R = np.full_like(theta, BORE)
    for _ in range(3):
        D = R / np.maximum(np.sin(theta), 1e-6)
        z = D * np.cos(theta)
        arc = phi * BORE * 1000.0
        R = radius_profile_mm((z - site_x) * 1000.0, arc) * 1e-3
    D = R / np.maximum(np.sin(theta), 1e-6)
    D = np.where(np.isfinite(D) & (D > 0) & (D < 2.0), D, 0.0)
    return D.astype(np.float32)


def flat(u, arc):
    return np.full_like(u, 50.0)


def cracked(u, arc, depth=2.2, half_w=0.8, half_l=9.0):
    prof = np.clip(1.0 - np.abs(arc) / half_w, 0.0, 1.0)
    along = np.clip(1.0 - (np.abs(u) / half_l) ** 6, 0.0, 1.0)
    return 50.0 + depth * prof * along


def repaired(u, arc, proud=0.8, w=2.4, l=10.6):
    prof = np.clip(1.0 - (np.abs(arc) / w) ** 2, 0.0, 1.0)
    along = np.clip(1.0 - (np.abs(u) / l) ** 8, 0.0, 1.0)
    return 50.0 - proud * prof * along


def centre_of(site_x=0.20):
    """용접 지점이 화면에서 보이는 위치(화소)."""
    th = np.arctan2(BORE, site_x)
    return (INTR["ppx"] + F * th, INTR["ppy"])


def run_to(seq, state, **kw):
    for _ in range(2000):
        seq.step(DT, **kw)
        if seq.r.state == state or seq.r.state in (SUCCESS, FAILED):
            return seq.r
    return seq.r


def scenario(name, align_mm, wall_fn, detected_after, expect, ok):
    seq = WeldSequencer()
    d = wall_depth(wall_fn)
    c = centre_of()
    kw = dict(torch_pos=(0, 0, 0), defect_pos=(align_mm, 0, 0),
              defect_size_mm=6.0, depth=d, intr=INTR, centre_px=c)
    # 접근 → 재확인(검출됨) → 정렬 → 아크 → 냉각 → 검증
    for _ in range(8000):
        det = True if seq.r.state in (APPROACH, RECHECK) else detected_after
        seq.step(DT, detected=det, **kw)
        if seq.r.state in (SUCCESS, FAILED):
            break
    r = seq.r
    good = r.state == expect
    ok.append(good)
    L = r.layers
    print(f"  {name:34s} {r.state:8s} (기대 {expect:8s}) {'OK' if good else 'FAIL'}")
    print(f"    정렬 {r.align_err_mm:5.2f}mm={str(L.get('align')):5s}  "
          f"프로파일 {r.profile_peak_mm:+5.2f}mm={str(L.get('profile')):5s}  "
          f"검출기 clear={str(L.get('detector')):5s}")
    print(f"    → {r.reason}")
    return r


def main():
    ok = []
    print("=" * 88)
    print("② Depth 프로파일 단독 확인 — 파임과 메워짐을 구분하는가")
    print("=" * 88)
    seq = WeldSequencer()
    c = centre_of()
    for label, fn, want in (("정상 벽", flat, True),
                            ("크랙 (2.2mm 파임)", cracked, False),
                            ("비드 (0.8mm 돌출)", repaired, True)):
        d = wall_depth(fn)
        v, peak, msg = seq.profile_verdict(d, INTR, c)
        good = v is want
        ok.append(good)
        print(f"  {label:22s} 프로파일 {peak:+6.2f} mm  판정 {str(v):5s} "
              f"(기대 {str(want):5s}) {'OK' if good else 'FAIL'}   {msg}")

    print()
    print("=" * 88)
    print("3겹 시나리오")
    print("=" * 88)
    scenario("정렬 OK + 메워짐 + 미검출", 0.5, repaired, False, SUCCESS, ok)
    scenario("정렬 빗나감 + 파임 + 검출", 4.0, cracked, True, FAILED, ok)

    print("-" * 88)
    print("② 가 검출기 거짓 음성을 뒤집는가  ← 이 항목이 핵심")
    scenario("파여 있는데 검출기가 놓침", 0.5, cracked, False, FAILED, ok)

    print("-" * 88)
    print("③ 검출기가 메워진 곳을 오검출하면")
    scenario("메워졌는데 여전히 검출", 0.5, repaired, True, FAILED, ok)

    print("-" * 88)
    print("① 정렬만 빗나간 경우 (프로파일·검출기는 성공처럼 보임)")
    scenario("정렬 초과 + 메워짐 + 미검출", 3.0, repaired, False, FAILED, ok)

    print("-" * 88)
    print("용접 전 미검출 → 대상 아님")
    s = WeldSequencer()
    d = wall_depth(flat)
    for _ in range(8000):
        s.step(DT, torch_pos=(0, 0, 0), defect_pos=(0.5, 0, 0),
               depth=d, intr=INTR, centre_px=centre_of(), detected=False)
        if s.r.state in (SUCCESS, FAILED):
            break
    good = s.r.state == FAILED and "용접 전" in s.r.verdict
    ok.append(good)
    print(f"  재확인에서 미검출 → {s.r.state} / {s.r.verdict}  "
          f"{'OK' if good else 'FAIL'}")

    print("-" * 88)
    print("재시도 후 성공")
    s = WeldSequencer()
    dc, dr = wall_depth(cracked), wall_depth(repaired)
    c = centre_of()
    n = 0
    for _ in range(12000):
        first = s.r.retry == 0
        s.step(DT, torch_pos=(0, 0, 0), defect_pos=(0.5, 0, 0),
               depth=dc if first else dr, intr=INTR, centre_px=c,
               detected=True if s.r.state in (APPROACH, RECHECK)
               else (True if first else False))
        n += 1
        if s.r.state in (SUCCESS, FAILED):
            break
    good = s.r.state == SUCCESS and s.r.retry == 1
    ok.append(good)
    print(f"  1차 실패 → 재시도 → {s.r.state} (재시도 {s.r.retry}회)  "
          f"{'OK' if good else 'FAIL'}")

    print("=" * 88)
    print(f"전체 판정: {'통과' if all(ok) else '실패'}  ({sum(ok)}/{len(ok)})")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
