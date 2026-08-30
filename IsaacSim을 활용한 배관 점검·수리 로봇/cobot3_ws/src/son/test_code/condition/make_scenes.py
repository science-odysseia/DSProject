"""[오프라인] 관 상태 판정용 시험 장면 생성

실기 데이터도 Isaac Sim 도 없이 판정 로직을 검증하기 위한 합성 장면.
카메라는 관 중심축에 놓고 축 방향(+Z)을 본다. 전방 D 위치에 조인트가 있고
그 너머 관이 lateral_offset 만큼 어긋난 구조로 모델링한다.

  NORMAL        offset 0,   너머 관 있음
  MISALIGNMENT  offset >0,  너머 관 있음 → 끝면이 초승달로 노출, Depth 유효
  DISCONNECTED  너머 관 없음 → 반사 없음, Depth 무효
  UNDETERMINED  너머가 불규칙(흙더미) → 반사는 오나 형태가 안 둥금

실행:  python3 make_scenes.py
"""

import argparse
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "scenes"

# 카메라는 어안 보어스코프. config/camera.yaml 과 맞춘다.
# RealSense 는 최소 측정 거리 52cm 로 관벽 25mm 에 성립하지 않아 폐기됐다.
W, H = 1280, 720
HFOV_DEG = 140.0
# 등거리 어안 r = f*theta.  화면 가로 끝에서 theta = HFOV/2
F_FISH = (W / 2.0) / np.radians(HFOV_DEG / 2.0)
PPX, PPY = W / 2.0, H / 2.0
# 판정기에 넘길 핀홀 등가 초점거리(참고용, 어안 모드에서는 쓰지 않음)
FX = FY = F_FISH

BORE_M = 0.050
JOINT_M = 0.220
FAR_END_M = 0.900
MAX_RANGE_M = 1.5


def rays():
    """등거리 어안 광선. 반환값은 단위벡터라 교점까지 거리 t 가 곧
    distance_to_camera(카메라 원점 기준 실제 거리)가 된다."""
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    du, dv = u - PPX, v - PPY
    r = np.hypot(du, dv)
    theta = np.clip(r / F_FISH, 0.0, np.pi / 2 * 0.995)
    phi = np.arctan2(dv, du)
    st = np.sin(theta)
    return np.stack([st * np.cos(phi), st * np.sin(phi), np.cos(theta)], -1)


def hit_cylinder(origin, dirs, radius, axis_xy):
    """축이 Z 인 원통 안쪽 면과의 교점까지 거리. 없으면 inf."""
    ox = origin[0] - axis_xy[0]
    oy = origin[1] - axis_xy[1]
    dx, dy = dirs[..., 0], dirs[..., 1]
    a = dx ** 2 + dy ** 2
    b = 2.0 * (ox * dx + oy * dy)
    c = ox ** 2 + oy ** 2 - radius ** 2
    disc = b ** 2 - 4 * a * c
    ok = (disc > 0) & (a > 1e-12)
    t = np.full(dirs.shape[:2], np.inf)
    sq = np.sqrt(np.where(ok, disc, 0.0))
    t1 = np.where(ok, (-b + sq) / (2 * a), np.inf)
    t[ok] = t1[ok]
    return np.where(t > 0, t, np.inf)


def render(state, offset_mm=0.0, seed=0, rough=0.0, lumpy=1.0, advance_mm=0.0):
    """advance_mm 은 카메라가 축을 따라 전진한 거리. 시각 오도메트리 시험용."""
    rng = np.random.default_rng(seed)
    d = rays()
    o = np.array([0.0, 0.0, 0.0])
    off = offset_mm * 1e-3
    adv = advance_mm * 1e-3

    joint_m = JOINT_M - adv
    t_near = hit_cylinder(o, d, BORE_M, (0.0, 0.0))
    z_at_near = d[..., 2] * t_near
    reaches_joint = z_at_near > joint_m

    t_joint = np.where(d[..., 2] > 1e-9, joint_m / np.maximum(d[..., 2], 1e-9),
                       np.inf)
    p_joint = d * t_joint[..., None]
    r_far = np.hypot(p_joint[..., 0] - off, p_joint[..., 1])
    into_far = reaches_joint & (r_far < BORE_M)
    on_endface = reaches_joint & ~into_far

    depth = np.where(reaches_joint, np.inf, t_near)
    depth = np.where(on_endface, t_joint, depth)

    if state == "DISCONNECTED":
        depth = np.where(into_far, np.nan, depth)
        depth = np.where(on_endface, np.nan, depth)
    elif state == "UNDETERMINED":
        # 끊긴 관 너머에 흙·다른 관이 있어 반사는 오지만 개구부 형태가 불규칙한 경우.
        # 개구부 테두리를 각도에 따라 들쭉날쭉하게 깎아 원형도를 떨어뜨린다.
        ang = np.arctan2(p_joint[..., 1], p_joint[..., 0] - off)
        harm = sum(rng.uniform(0.10, 0.30) * np.sin(k * ang + rng.uniform(0, 6.28))
                   for k in (2, 3, 5, 7))
        lump = BORE_M * (1.0 - 0.28 * lumpy + harm * 0.55 * lumpy)
        debris = r_far > lump
        depth = np.where(into_far & debris,
                         JOINT_M + 0.02 + 0.03 * rng.random((H, W)), depth)
        depth = np.where(into_far & ~debris,
                         np.minimum(hit_cylinder(o, d, BORE_M, (off, 0.0)),
                                    FAR_END_M), depth)
        depth = np.where(on_endface, t_joint, depth)
    else:
        t_far = hit_cylinder(o, d, BORE_M, (off, 0.0))
        depth = np.where(into_far, np.minimum(t_far, FAR_END_M - adv), depth)

    depth = np.where(np.isinf(depth), np.nan, depth)
    depth = np.where(depth > MAX_RANGE_M, np.nan, depth)
    if rough > 0:
        depth = depth + rng.normal(0, rough, depth.shape)
    depth = np.where(np.isnan(depth), 0.0, depth).astype(np.float32)
    # 센서 잡음은 프레임마다 달라야 한다. 두 프레임에 같은 잡음을 넣으면
    # 화면에 고정된 무늬가 되고 광류가 벽 텍스처 대신 그것에 달라붙어
    # 전진량이 0 으로 측정된다.
    nrng = np.random.default_rng((seed * 1000003 + int(round(advance_mm * 1000))) % 2**31)

    lit = np.where(depth > 0, 1.0 / np.maximum(depth, 0.02) ** 2, 0.0)
    lit = lit / max(lit.max(), 1e-9)

    # 관벽 텍스처. 광류가 붙잡을 무늬가 없으면 시각 오도메트리를 시험할 수 없다.
    # 광선을 따라 교점까지 간 3D 위치로 무늬를 만들면 전진 시 무늬가
    # 물리적으로 옳게 흘러간다.
    hit = d * depth[..., None]
    wz = hit[..., 2] + adv
    wa = np.arctan2(hit[..., 1], hit[..., 0])
    # 실제 강관 내벽(부식·스크래치·용접 자국)은 여러 스케일이 섞인 광대역
    # 무늬다. 이걸 제대로 흉내내지 않으면 광류 성능이 아니라 무늬의 성질을
    # 재게 된다. 실제로 두 가지 실패를 겪었다.
    #   단일 고주파  → 변위가 주기 절반을 넘는 순간 엉뚱한 주기에 달라붙음
    #   저주파만     → 코너가 약해 LK 가 변위의 17% 만 따라감
    # 옥타브를 겹쳐 1/f 진폭으로 쌓으면 굵은 층이 큰 변위를 먼저 잡고
    # 가는 층이 그것을 다듬는다. 피라미드 LK 가 원래 그렇게 동작한다.
    tex = np.zeros_like(wz)
    trng = np.random.default_rng(12345)          # 장면 간 무늬는 동일해야 한다
    fz = 45.0
    amp = 1.0
    for _ in range(8):
        fa = max(3.0, fz * 0.09)
        tex += amp * (np.sin(wz * fz + trng.uniform(0, 6.28))
                      * np.sin(wa * fa + trng.uniform(0, 6.28))
                      + 0.7 * np.sin(wz * fz * 0.63 + trng.uniform(0, 6.28)
                                     + wa * fa * 1.7))
        fz *= 1.9
        amp *= 0.72
    tex /= 3.2

    lit = np.clip(lit * (1.0 + 0.45 * tex), 0.0, None)

    rgb = np.clip(lit ** 0.45 * 235 + nrng.normal(0, 3, lit.shape), 0, 255)
    rgb = np.repeat(rgb[..., None].astype(np.uint8), 3, axis=-1)
    return rgb, depth


SCENES = [
    ("normal", "NORMAL", 0.0, 0.0, 1.0),
    ("normal_noisy", "NORMAL", 0.0, 0.002, 1.0),
    # 🚨 주행 스펙 재보정 (2026-08-07): 오프셋 문턱 1/3/8 → 6/12/25mm.
    #    실측으로 정상 주행의 이탈이 평균 5~15mm 라(전 코스), 구 문턱이면
    #    정상 주행도 감속·정지 판정을 받는다(reducer 안착 4mm → 감속
    #    12mm/s → 정지 마찰에 져 출발 정체 사망 실측). 2·5mm 는 이제
    #    주행상 정상이다 — 장면 이름은 기하(실제 이탈량)를 그대로 남긴다.
    ("misalign_2mm", "NORMAL", 2.0, 0.0, 1.0),
    ("misalign_5mm", "NORMAL", 5.0, 0.0, 1.0),
    ("misalign_12mm", "MISALIGNMENT", 12.0, 0.0, 1.0),
    ("misalign_5mm_noisy", "NORMAL", 5.0, 0.002, 1.0),
    ("disconnected", "DISCONNECTED", 0.0, 0.0, 1.0),
    ("disconnected_off", "DISCONNECTED", 4.0, 0.0, 1.0),
    ("undetermined", "UNDETERMINED", 3.0, 0.0, 1.0),
    ("undetermined_mild", "UNDETERMINED", 2.0, 0.0, 0.55),
    ("undetermined_heavy", "UNDETERMINED", 1.0, 0.0, 1.6),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--png", action="store_true", help="RGB 를 png 로도 저장")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    meta = dict(width=W, height=H, fx=FX, fy=FY, ppx=PPX, ppy=PPY,
                projection="fisheye_equidistant", f_fish=F_FISH,
                hfov_deg=HFOV_DEG,
                bore_m=BORE_M, joint_m=JOINT_M, max_range_m=MAX_RANGE_M,
                scenes={})
    print("=" * 70)
    print(f"{'장면':18s} {'정답':14s} {'offset':>8s} {'무효율':>8s} {'유효 최소~최대(m)':>20s}")
    print("=" * 70)
    for name, truth, off, rough, lumpy in SCENES:
        rgb, depth = render(truth, off, seed=abs(hash(name)) % 2 ** 31,
                            rough=rough, lumpy=lumpy)
        np.save(OUT / f"{name}_depth.npy", depth)
        np.save(OUT / f"{name}_rgb.npy", rgb)
        if args.png:
            import cv2
            cv2.imwrite(str(OUT / f"{name}_rgb.png"), rgb)
            vis = np.where(depth > 0, depth, np.nan)
            n = (np.nan_to_num((vis - np.nanmin(vis))
                               / (np.nanmax(vis) - np.nanmin(vis))) * 255)
            cv2.imwrite(str(OUT / f"{name}_depth.png"),
                        cv2.applyColorMap(n.astype(np.uint8), cv2.COLORMAP_TURBO))
        inv = float((depth == 0).mean())
        val = depth[depth > 0]
        meta["scenes"][name] = dict(truth=truth, offset_mm=off,
                                    invalid_ratio=inv)
        print(f"{name:18s} {truth:14s} {off:6.1f}mm {inv * 100:7.2f}% "
              f"{val.min():9.3f} ~ {val.max():.3f}")
    (OUT / "scenes_meta.json").write_text(json.dumps(meta, indent=2))
    print("=" * 70)
    print(f"저장: {OUT}  ({len(SCENES)}장면)")


if __name__ == "__main__":
    main()
