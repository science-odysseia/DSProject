"""[공용] 시각 오도메트리 — 관 안에서 실제로 얼마나 나아갔는지 잰다.

설계 문서 8.2 는 끼임 판정 근거를 "바퀴는 회전하나 거리 적산 정체"로 적어
놓았으나, **실제 이동을 무엇으로 재는지는 적혀 있지 않다.** 휠 엔코더만으로는
6륜이 동시에 미끄러지는 경우를 못 잡는다. 이 모듈이 그 빈칸을 메운다.

원리
  전진하면 벽면 특징이 소실점에서 바깥으로 방사상 확산한다. 단안 광류는 보통
  스케일 모호성 때문에 절대 속도를 못 구하지만, **관 안에서는 벽까지 거리를
  Depth 로 알 수 있어** 모호성이 없다.

  ρ = f·θ, R = d·sinθ, z = d·cosθ 이므로 전진 Δz 후의 화면 반경은

        ρ' = f · atan2(R, z - Δz)

  **자유도가 전진 하나뿐**이라 일반 광류(2 자유도) 대신 Δz 를 직접 탐색한다.
  후보 Δz 마다 이전 프레임 화소를 예측 위치로 옮겨 현재 프레임과 상관을 재고,
  가장 잘 맞는 Δz 를 고른다. 방사 성분만 쓰므로 로봇이 관 축을 따라 굴러도
  값이 오염되지 않는다.

  특징점 LK 로 먼저 만들었다가 갈아엎었다. 이유가 둘이다.
    - 근접 벽면은 프레임 사이에 패치가 **늘어난다**. LK 는 국소 평행이동을
      가정하므로 이 스케일 변화를 못 따라간다 (예측 변위의 17% 만 추적).
    - 변위가 커질수록 수렴이 나빠져 오차가 단조 증가했다.
  직접 정합은 두 문제가 모두 없다. 스케일 변화가 모델 안에 들어 있기 때문이다.

이미지를 다루므로 인지 노드 안에 둔다. 주행 노드로는 스칼라만 나간다.
"""

from dataclasses import dataclass

import cv2
import numpy as np

DEFAULTS = dict(
    min_radius_px=60.0,
    max_radius_px=280.0,
    max_samples=4000,
    min_samples=400,
    coarse_steps=41,
    refine_rounds=3,
    min_score=0.30,
    max_speed_mps=0.60,
)


@dataclass
class FlowResult:
    speed_mps: float = 0.0
    advance_m: float = 0.0
    tracked: int = 0
    inliers: int = 0
    score: float = 0.0
    valid: bool = False
    reason: str = ""


class VisualOdometry:
    """연속한 두 프레임의 방사 광류로 전진량을 추정한다."""

    def __init__(self, params=None, f_px=523.85):
        self.k = dict(DEFAULTS)
        if params:
            self.k.update({a: b for a, b in params.items() if a in self.k})
        self.f_px = float(f_px)
        self._rng = np.random.default_rng(0)
        self.prev_gray = None
        self.prev_depth = None

    def reset(self):
        self.prev_gray = None
        self.prev_depth = None

    @staticmethod
    def _to_gray(rgb):
        a = np.asarray(rgb)
        if a.ndim == 3:
            a = cv2.cvtColor(a[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2GRAY)
        return np.ascontiguousarray(a.astype(np.uint8))

    def _sample(self, img, xs, ys):
        """겹선형 보간."""
        h, w = img.shape
        x0 = np.clip(np.floor(xs).astype(np.int32), 0, w - 2)
        y0 = np.clip(np.floor(ys).astype(np.int32), 0, h - 2)
        fx = np.clip(xs - x0, 0.0, 1.0)
        fy = np.clip(ys - y0, 0.0, 1.0)
        a = img[y0, x0]; b = img[y0, x0 + 1]
        c = img[y0 + 1, x0]; e = img[y0 + 1, x0 + 1]
        return (a * (1 - fx) * (1 - fy) + b * fx * (1 - fy)
                + c * (1 - fx) * fy + e * fx * fy)

    def _score(self, gray, base, px, py, cx, cy, rho, R, z, dz):
        """후보 dz 에 대한 상관계수. 밝기가 거리에 따라 변하므로 정규화한다."""
        rho2 = self.f_px * np.arctan2(R, z - dz)
        k = rho2 / np.maximum(rho, 1e-6)
        xs = cx + (px - cx) * k
        ys = cy + (py - cy) * k
        h, w = gray.shape
        inside = (xs > 1) & (xs < w - 2) & (ys > 1) & (ys < h - 2)
        if inside.sum() < 64:
            return -1.0
        a = base[inside]
        b = self._sample(gray, xs[inside], ys[inside])
        a = a - a.mean(); b = b - b.mean()
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-6 or nb < 1e-6:
            return -1.0
        return float(a @ b / (na * nb))

    def step(self, rgb, depth, dt, centre=None):
        """centre 는 소실점(개구부 중심). 없으면 화면 중앙을 쓴다."""
        r = FlowResult()
        gray = self._to_gray(rgb).astype(np.float64)
        depth = np.asarray(depth, dtype=np.float64)
        h, w = gray.shape[:2]
        cx, cy = centre if centre is not None else (w / 2.0, h / 2.0)

        if self.prev_gray is None or dt <= 0:
            self.prev_gray, self.prev_depth = gray, depth
            r.reason = "첫 프레임"
            return r

        # 소실 영역 안쪽은 특징이 없고 잡음만 크다. 바깥쪽 근접 벽면은
        # 패치가 크게 늘어나 정합이 불안정하다. 중간 띠만 쓴다.
        yy, xx = np.mgrid[0:h, 0:w]
        rad = np.hypot(xx - cx, yy - cy)
        use = ((rad > self.k["min_radius_px"]) & (rad < self.k["max_radius_px"])
               & np.isfinite(self.prev_depth) & (self.prev_depth > 0))
        n_use = int(use.sum())
        if n_use < self.k["min_samples"]:
            self.prev_gray, self.prev_depth = gray, depth
            r.reason = f"유효 화소 {n_use}"
            return r

        idx = np.flatnonzero(use.reshape(-1))
        if idx.size > self.k["max_samples"]:
            idx = self._rng.choice(idx, int(self.k["max_samples"]), replace=False)
        py, px = np.divmod(idx, w)
        px = px.astype(np.float64); py = py.astype(np.float64)
        rho = np.hypot(px - cx, py - cy)
        d = self.prev_depth.reshape(-1)[idx]
        th = rho / self.f_px
        R = d * np.sin(th)
        z = d * np.cos(th)
        base = self.prev_gray.reshape(-1)[idx].astype(np.float64)
        r.tracked = int(idx.size)

        lim = self.k["max_speed_mps"] * dt
        best_dz, best_s = 0.0, -2.0
        # 성긴 격자로 훑고 두 번 좁혀 들어간다. 국소 최적에 빠지지 않으면서
        # 화소 이하 분해능까지 간다.
        lo, hi, n = -lim, lim, int(self.k["coarse_steps"])
        for _ in range(int(self.k["refine_rounds"])):
            cands = np.linspace(lo, hi, n)
            scores = [self._score(gray, base, px, py, cx, cy, rho, R, z, c)
                      for c in cands]
            j = int(np.argmax(scores))
            best_dz, best_s = float(cands[j]), float(scores[j])
            span = (hi - lo) / (n - 1) * 2.0
            lo, hi, n = best_dz - span, best_dz + span, 9

        r.advance_m = best_dz
        r.speed_mps = best_dz / dt
        r.score = best_s
        r.inliers = r.tracked
        if best_s < self.k["min_score"]:
            r.reason = f"정합 신뢰도 낮음 ({best_s:.3f})"
        elif abs(r.speed_mps) > self.k["max_speed_mps"] * 0.98:
            r.reason = f"탐색 한계 도달 {r.speed_mps:.2f} m/s"
        else:
            r.valid = True

        self.prev_gray, self.prev_depth = gray, depth
        return r
