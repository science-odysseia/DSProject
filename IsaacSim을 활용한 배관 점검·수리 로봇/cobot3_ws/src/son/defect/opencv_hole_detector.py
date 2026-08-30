"""[공용] 결함(구멍·비드) OpenCV 기하 검출 — 학습 모델 불필요. ROS·Isaac 비의존.

`repair_demo.py`의 `find_wall_hole`/`find_weld_bead`(1170~1247행)를 그대로
가져와 Isaac 전역 변수(CAM_AREA_SCALE, sequencer.k 등)에 대한 의존을 없애고
함수를 하나의 클래스로 묶었다. 알고리즘은 바꾸지 않았다 — 그쪽에서 실측으로
얻은 값(HOLE_DARK_FRAC=0.12, BEAD_SAT_MIN=60 등)을 그대로 기본값으로 쓴다.

원리 (repair_demo.py 주석 원문)
  구멍  로봇 조명이 벽을 정면으로 때려 밝다. 구멍은 빛이 안 돌아와
        **주변 밝은 벽에 둘러싸인 어두운 덩어리**가 된다. 관 저 끝(전방
        개구부)도 어둡지만 화면 중앙에 걸리므로 중앙 포함 덩어리만 뺀다.
  비드  관 벽(회색)·관 저 끝(검정)은 채도 0이라, 주황(채도 높음)인 비드만
        HSV 의 S 로 가르면 절대 안 헷갈린다.

**YOLO 대비 계산량**: torch/ultralytics 로딩·추론이 없다. 프레임당
connectedComponents 두 번(그레이 임계 + HSV 채도 임계)뿐이라 CPU 만으로도
가볍다 — `pipe_vision_node`의 YOLO Seg 를 그대로 대체하려는 목적이 아니라
**별도의 가벼운 노드**로 옆에 세우기 위한 설계다.
"""

import math

import cv2
import numpy as np
from dataclasses import dataclass

REFERENCE_W, REFERENCE_H = 1280.0, 720.0


@dataclass(frozen=True)
class Detection:
    """한 결함(구멍 또는 비드) 검출 결과. mask 는 같은 해상도의 bool 배열."""

    kind: str                 # "hole" | "bead"
    area_px: int
    center_pixel: tuple        # (cx, cy)
    r_eq_px: float
    bbox_xyxy: tuple
    mask: np.ndarray
    confidence: float          # 확률이 아니라 면적 기반 근사 신뢰도(0~1)


def _area_scale(width, height):
    """기준 해상도(1280x720) 대비 면적비 — 해상도가 바뀌어도 같은 임계가 통하게."""
    return (width * height) / (REFERENCE_W * REFERENCE_H)


def _bbox_and_mask(lab, index, st):
    x, y, w, h = (int(st[index, i]) for i in
                  (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                   cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    return (float(x), float(y), float(x + w), float(y + h)), (lab == index)


def _confidence_from_area(area_px, min_px):
    """면적이 최소치의 몇 배인지로 만든 발견적 신뢰도(확률 아님, 0.5~0.97 범위)."""
    return float(min(0.97, 0.5 + 0.15 * math.log2(max(1.0, area_px / max(1, min_px)))))


class OpenCvHoleDetector:
    """구멍(HOLE_DARK_FRAC 기반 밝기 임계)과 비드(HSV 채도 임계)를 검출한다."""

    def __init__(self, hole_dark_frac=0.12, hole_min_px_ref=300,
                 bead_sat_min=60, bead_val_min=40, bead_min_px_ref=100,
                 bore_exclude_frac=0.10, bore_grid_step=3,
                 max_hole_area_frac=0.25):
        self.hole_dark_frac = float(hole_dark_frac)
        self.hole_min_px_ref = float(hole_min_px_ref)
        self.bead_sat_min = int(bead_sat_min)
        self.bead_val_min = int(bead_val_min)
        self.bead_min_px_ref = float(bead_min_px_ref)
        self.bore_exclude_frac = float(bore_exclude_frac)
        self.bore_grid_step = int(bore_grid_step)
        self.max_hole_area_frac = float(max_hole_area_frac)

    def find_wall_hole(self, rgb):
        """근접 벽면의 구멍(들)을 찾는다 → Detection 리스트(면적 큰 순).

        🚨 전방 개구부(관 저 끝)를 반드시 걸러야 한다 — 화면 중앙에 걸치는
        덩어리는 격자로 훑어 통째로 제외한다(원본 주석의 실측 버그 그대로 반영).
        """
        g = cv2.cvtColor(np.asarray(rgb)[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2GRAY)
        g = cv2.medianBlur(g, 5)
        h, w = g.shape
        min_px = max(60, self.hole_min_px_ref * _area_scale(w, h))
        lo, hi = np.percentile(g, 5), np.percentile(g, 90)
        thr = lo + self.hole_dark_frac * (hi - lo)
        dark = (g < thr).astype(np.uint8)
        n, lab, st, ce = cv2.connectedComponentsWithStats(dark, 8)

        bore = set()
        rr = max(4, int(self.bore_exclude_frac * min(h, w)))
        ys = range(max(0, h // 2 - rr), min(h, h // 2 + rr), self.bore_grid_step)
        xs = range(max(0, w // 2 - rr), min(w, w // 2 + rr), self.bore_grid_step)
        for yy in ys:
            for xx in xs:
                if lab[yy, xx]:
                    bore.add(int(lab[yy, xx]))
        big = self.max_hole_area_frac * h * w

        out = []
        for i in range(1, n):
            if i in bore:
                continue
            area = int(st[i, cv2.CC_STAT_AREA])
            if area < min_px or area > big:
                continue
            bbox, mask = _bbox_and_mask(lab, i, st)
            out.append(Detection("hole", area, (float(ce[i][0]), float(ce[i][1])),
                                  math.sqrt(area / math.pi), bbox, mask,
                                  _confidence_from_area(area, min_px)))
        out.sort(key=lambda d: d.area_px, reverse=True)
        return out

    def find_weld_bead(self, rgb):
        """용접 비드(주황, 채도로 구분)를 찾는다 → Detection 리스트(면적 큰 순)."""
        hsv = cv2.cvtColor(np.asarray(rgb)[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2HSV)
        h, w = hsv.shape[:2]
        min_px = max(20, self.bead_min_px_ref * _area_scale(w, h))
        sat, val = hsv[:, :, 1], hsv[:, :, 2]
        mask_all = ((sat > self.bead_sat_min) & (val > self.bead_val_min)).astype(np.uint8)
        mask_all = cv2.morphologyEx(mask_all, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, lab, st, ce = cv2.connectedComponentsWithStats(mask_all, 8)

        out = []
        for i in range(1, n):
            area = int(st[i, cv2.CC_STAT_AREA])
            if area < min_px:
                continue
            bbox, mask = _bbox_and_mask(lab, i, st)
            out.append(Detection("bead", area, (float(ce[i][0]), float(ce[i][1])),
                                  math.sqrt(area / math.pi), bbox, mask,
                                  _confidence_from_area(area, min_px)))
        out.sort(key=lambda d: d.area_px, reverse=True)
        return out

    def detect(self, rgb, find_holes=True, find_beads=True):
        """구멍·비드를 한 번에 찾아 하나의 리스트로 합친다."""
        out = []
        if find_holes:
            out.extend(self.find_wall_hole(rgb))
        if find_beads:
            out.extend(self.find_weld_bead(rgb))
        return out
