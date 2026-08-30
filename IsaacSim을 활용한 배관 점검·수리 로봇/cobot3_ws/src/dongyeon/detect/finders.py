"""[공용] 카메라 프레임에서 벽면 구멍(결함)·용접 비드를 찾는다 — 순수 OpenCV.

repair_demo 와 real_map_demo 에 복붙으로 갈라져 있던 검출 함수를 합쳤다
(2026-08-06). 데모는 이 모듈을 import 하고 해상도 배율만 넘긴다 — 검출
로직 수정은 **여기 한 곳**에서 하고, 오프라인 시험은 `test_code/detect/`.

⚠ `condition/` 에 넣지 않은 이유: 그 모듈은 주행 안전 판정 전용이고
  "결함 검출은 다루지 않는다" 가 명시된 경계다.

합칠 때 기준 (두 복사본이 이미 갈라져 있었다):
- **real_map 판을 기준**으로 삼았다 — 관 저 끝 거르기에 Depth 테두리
  판정(꺾인 관에서도 성립)과 창 잘라 임계(조명 몰림 대응)가 들어 있어
  repair 판의 상위호환이다. Depth 없이 부르면 repair 판 동작(중앙 덩어리
  제외)으로 물러난다.
- repair 판에만 있던 **전 화면 25% 상한**(관 저 끝을 통째로 구멍으로 잡는
  사고 방지)은 expect_px 없는 경로에 유지했다. 창을 잘랐을 때는 60%.
- **물(파랑) 화소 제외**(2026-08-06, 물결 그림자 오탐 차단)를 양쪽 경로에
  적용했다 — find_weld_bead 와 같은 "색으로 가른다" 원리다. 물결 그림자는
  어두워도 파란 기가 돌고(물 재질 diffuse B 0.85 > R 0.15), 진짜 구멍은
  무채색 검정이다.
"""

import math

import cv2
import numpy as np


def _watery_mask(a3):
    """물(파랑) 화소 — 구멍 후보에서 뺄 영역. B−R>18 이고 B≥G."""
    return ((a3[:, :, 2].astype(np.int16) - a3[:, :, 0].astype(np.int16) > 18)
            & (a3[:, :, 2] >= a3[:, :, 1]))


def find_wall_hole(rgb, expect_px=None, depth=None, area_scale=1.0,
                   near_m=0.22, win_px=70, dark_frac=0.12):
    """근접 벽면의 구멍 — 밝은 벽에 둘러싸인 어두운 덩어리 → dict 또는 None.

    🚨 **전방 개구부(관 저 끝)를 반드시 걸러야 한다.** "화면 중앙 덩어리
       제외" 는 저 끝이 화면 중앙에 있다는 전제라, 관이 꺾이면 저 끝이 화면
       옆으로 밀려 통째로 "구멍" 으로 잡힌다(실전 맵 실측 8,182px 오검).
    → **Depth 로 가른다.** 벽면 구멍은 테두리가 근접 벽(≈0.1m), 관 저 끝은
       테두리가 먼 벽(0.3~0.5m). 관이 꺾여도 성립한다. Depth 가 없으면
       중앙 덩어리 제외로 물러난다.

    🚨 **전역 이진화는 굽은 관에서 성립하지 않는다.** 조명이 한쪽으로 몰리면
       화면 절반이 통째로 어두워 결함이 배경과 이어져 버려진다. expect_px 가
       있으면 그 둘레 창만 잘라 **창의 분포로** 임계를 잡는다. (자기충족이
       아니다 — 창 안에 어두운 덩어리가 실제로 없으면 아무것도 안 잡히고,
       수리가 되면 정확히 그렇게 된다.)

    area_scale: 카메라 해상도 면적비 (1280x720 기준). 면적 임계를 함께 줄인다.
    """
    a3_full = np.asarray(rgb)[:, :, :3].astype(np.uint8)
    g_full = cv2.medianBlur(cv2.cvtColor(a3_full, cv2.COLOR_RGB2GRAY), 5)
    H, Wd = g_full.shape
    min_px = max(60, 300 * area_scale)
    if expect_px is not None:
        x0 = int(max(0, expect_px[0] - win_px))
        x1 = int(min(Wd, expect_px[0] + win_px))
        y0 = int(max(0, expect_px[1] - win_px))
        y1 = int(min(H, expect_px[1] + win_px))
    else:
        x0, y0, x1, y1 = 0, 0, Wd, H
    g = g_full[y0:y1, x0:x1]
    if g.size == 0:
        return None
    h, w = g.shape
    lo, hi = np.percentile(g, 5), np.percentile(g, 90)
    thr = lo + dark_frac * (hi - lo)
    dark = (g < thr).astype(np.uint8)
    dark[_watery_mask(a3_full[y0:y1, x0:x1])] = 0    # 물(파랑)은 제외
    n, lab, st, ce = cv2.connectedComponentsWithStats(dark, 8)
    # 중앙 원판에 걸치는 라벨(관 저 끝) 제외 — 창을 안 잘랐을 때만.
    # 한 화소가 아니라 격자로 훑는다(임계가 낮으면 중앙 화소가 배경이 된다).
    bore = set()
    rr = max(4, int(0.10 * min(h, w))) if expect_px is None else 0
    for yy in range(max(0, h // 2 - rr), min(h, h // 2 + rr), 3):
        for xx in range(max(0, w // 2 - rr), min(w, w // 2 + rr), 3):
            if lab[yy, xx]:
                bore.add(int(lab[yy, xx]))
    dz = None
    if depth is not None:
        _d = np.asarray(depth, dtype=np.float32)
        if _d.shape == g_full.shape:
            dz = _d[y0:y1, x0:x1]
    # 전 화면일 때 25% 상한 — 벽면 구멍이 화면 1/4 을 넘을 수 없다
    # (repair_demo 실측: 저 끝 130,162px = 화면 56% 가 결함으로 보고된 사고).
    big = (0.60 if expect_px is not None else 0.25) * h * w
    best = None
    for i in range(1, n):
        if i in bore:
            continue
        area = int(st[i, cv2.CC_STAT_AREA])
        if area < min_px or area > big:
            continue
        rim_m = None
        if dz is not None:
            m_i = (lab == i).astype(np.uint8)
            ring = cv2.dilate(m_i, np.ones((9, 9), np.uint8)) - m_i
            v = dz[(ring > 0) & np.isfinite(dz) & (dz > 0)]
            if v.size < 20:
                continue
            rim_m = float(np.median(v))
            if rim_m > near_m:          # 테두리가 멀다 = 관 저 끝
                continue
        cand = {"area_px": area, "cx": float(ce[i][0]) + x0,
                "cy": float(ce[i][1]) + y0, "thr": float(thr), "rim_m": rim_m,
                "r_eq_px": math.sqrt(area / math.pi)}
        # 🚨 expect 가 있으면 **가장 큰 게 아니라 가장 가까운** 덩어리를
        #    고른다 — 관 저 끝은 늘 결함보다 크다(실측 39,603px vs 수천 px).
        if expect_px is not None:
            cand["dist_px"] = math.hypot(cand["cx"] - expect_px[0],
                                         cand["cy"] - expect_px[1])
            if best is None or cand["dist_px"] < best["dist_px"]:
                best = cand
        elif best is None or area > best["area_px"]:
            best = cand
    if best and expect_px is not None:
        best["matched"] = best["dist_px"] <= max(2.0 * best["r_eq_px"],
                                                 0.06 * Wd)
    return best


def find_weld_bead(rgb, expect_px=None, area_scale=1.0,
                   sat_min=60, val_min=40):
    """비드를 **색(HSV 채도)** 으로 찾는다 → dict 또는 None.

    🔑 어둠으로 어둠을 가르지 않는다. 관벽(회색)·관 저 끝(검정)은 둘 다
    채도 0 이고 비드만 주황(채도 높음)이다. 밝기 임계에 흔들리지 않는다.
    (실물 비드도 산화막 때문에 모재와 색이 다르다 — 억지 표현이 아니다.)
    """
    hsv = cv2.cvtColor(np.asarray(rgb)[:, :, :3].astype(np.uint8),
                       cv2.COLOR_RGB2HSV)
    mask = ((hsv[:, :, 1] > sat_min)
            & (hsv[:, :, 2] > val_min)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, ce = cv2.connectedComponentsWithStats(mask, 8)
    min_px = max(20, 100 * area_scale)
    best = None
    for i in range(1, n):
        area = int(st[i, cv2.CC_STAT_AREA])
        if area < min_px:
            continue
        if best is None or area > best["area_px"]:
            best = {"area_px": area, "cx": float(ce[i][0]),
                    "cy": float(ce[i][1]), "r_eq_px": math.sqrt(area / math.pi)}
    if best and expect_px is not None:
        dd = math.hypot(best["cx"] - expect_px[0], best["cy"] - expect_px[1])
        best["dist_px"] = dd
        # 작은 덩어리는 등가반지름이 작아 반경 배수만으로는 너무 빡빡하다
        best["matched"] = dd <= max(3.0 * best["r_eq_px"],
                                    0.08 * hsv.shape[1])
    return best
