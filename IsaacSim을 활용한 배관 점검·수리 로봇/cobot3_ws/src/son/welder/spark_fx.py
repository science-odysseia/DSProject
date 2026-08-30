"""[Isaac 3.11] 용접 아크 스파크 — **시각 전용** 파티클.

실제 아크 용접처럼 보이게 하는 것이 목적이다. 팁 끝(아크점)에서 관 안쪽으로
스패터가 원뿔 모양으로 튀고, 중력으로 포물선을 그리며 떨어지면서 색이
흰빛 → 주황 → 검붉은색으로 식는다. 아크 조명도 같이 깜빡인다.

🚨 **물리에 일절 관여하지 않는다.** 콜라이더도 강체도 없는 `PointInstancer`
   하나이고, 위치는 파이썬이 적분해서 써 넣는다. 로봇·물 입자·판정 어디에도
   영향이 없다. 카메라 판정(`find_weld_bead` 의 HSV 채도)과 겹치지 않는 이유는
   아래 「판정과의 관계」 참조.

🔑 **매질에 따라 거동이 다르다** — 이 프로젝트는 배수 조건과 만관(수중 용접)을
   둘 다 돌린다. 물속에서 스패터가 공기 중처럼 날아가면 그림이 거짓말이 된다.
   물은 밀도가 800배라 항력 시정수가 짧고(정지거리 v/k ≈ 2cm) 급냉되므로,
   `flooded=True` 면 속도·수명을 줄이고 항력을 크게 키운다. 용융 금속은
   ρ≈7,800 이라 부력으로 뜨지 않는다 — 중력은 (1 − 1000/7800) 만 깎는다.

판정과의 관계
  - 스파크는 **ARC 동안만** 존재하고 수명이 길어야 0.65초다. VERIFY 는 COOL
    (5초 이상) 과 REPOSITION 을 지난 뒤라 화면에 남아 있을 수 없다.
  - 그래도 확실히 하려고 `clear()` 를 두었다. ARC 를 빠져나올 때 부르면 그
    자리에서 전부 사라진다.
  - 헤드리스에서는 만들지 말 것(`SparkFX` 를 아예 None 으로). 렌더가 없어
    보이지도 않는데 계산만 든다.

사용
    fx = SparkFX(stage, "/World/weld_sparks", flooded=FLOODED)
    ...매 스텝...
    fx.step(dt, emitting=(state == "ARC"), origin=팁끝_월드, normal=관안쪽_단위,
            light=arc_light)
    ...ARC 끝날 때...
    fx.clear()

`origin` 은 스테이지 좌표(m)다. `path` 의 부모에 변환이 걸려 있으면 안 된다
(`/World` 처럼 항등인 곳에 둘 것).
"""

import math

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

# ── 매질별 상수 ────────────────────────────────────────────────────
# 값의 뜻: v0 초기속도(m/s) / drag 선형 항력계수(1/s, 정지거리 ≈ v0/drag) /
#          g 유효 중력(m/s²) / life 수명(s) / rate 초당 발생 개수
# 🚨 발생률은 **관벽 반사를 넣은 뒤에** 정했다. 좁은 관에서는 스패터가 금세
#    벽에 맞아 식으므로(부딪힐 때마다 수명 12% 소모) 같은 rate 라도 화면에
#    남는 개수가 절반 이하로 준다 — 실측 120/s 에서 동시 22개로 성겼다.
_AIR = dict(v0=3.2, v0_sd=1.2, drag=2.6, g=9.81,
            life=0.50, life_sd=0.20, rate=700.0, cone_deg=60.0)
_WATER = dict(v0=1.1, v0_sd=0.45, drag=45.0, g=8.55,
              life=0.20, life_sd=0.07, rate=340.0, cone_deg=45.0)

# 프로토타입 3종 = 식는 단계. (이름, 발광색, 나이 상한 비율, 발광 배율)
# 🔑 **발광 배율이 1 이면 밖에서 안 보인다.** `emissiveColor` 는 0~1 로 묶인
#    값이 아니라 HDR 이다. 1 을 넘겨야 톤매핑 뒤에도 하얗게 타고 블룸이 붙어
#    "용접 불꽃" 으로 읽힌다. 실제 아크 온도(약 3,000~6,000K)를 생각하면
#    주변 관벽 조명(4.0e5)보다 훨씬 밝은 것이 맞다.
_STAGES = (("hot", (1.00, 0.95, 0.72), 0.22, 22.0),
           ("warm", (1.00, 0.48, 0.09), 0.58, 12.0),
           ("cool", (0.72, 0.11, 0.02), 1.01, 5.0))

# 🚨 **밖에서 안 보이던 첫 번째 원인은 크기다** (사용자 지적, 2026-08-05).
#    실제 스패터는 0.1~1mm 지만, 관(ø100) 전체가 화면에 들어오는 거리에서
#    0.4mm 는 **화소 하나가 안 된다** — 물리적으로 정확해도 안 보인다.
#    시연용으로 굵기 3배·길이 3.5배로 키운다. 눈에 보이는 것이 목적이다.
_R = 0.0012      # 스패터 굵기(m)
_L = 0.0140      # 길이(m). 잔상처럼 보이도록 속도에 비례해 더 늘인다
_BALL_R = 0.0035  # 아크점 자체의 발광 구 반지름(m)
_UPDATE_HZ = 60.0   # 인스턴서 쓰기 빈도. 화면이 60fps 라 그 이상은 낭비다


class SparkFX:
    """아크 스패터 파티클 한 벌. 스테이지에 PointInstancer 하나만 만든다."""

    def __init__(self, stage, path, *, flooded=False, seed=7, max_n=460):
        self.p = dict(_WATER if flooded else _AIR)
        self.flooded = bool(flooded)
        self.rng = np.random.default_rng(seed)
        self.n = int(max_n)

        # 파티클 상태 — 고정 크기 배열. 매 스텝 새로 할당하지 않는다.
        self.pos = np.zeros((self.n, 3))
        self.vel = np.zeros((self.n, 3))
        self.age = np.zeros(self.n)
        self.life = np.ones(self.n)
        self.alive = np.zeros(self.n, dtype=bool)

        self._emit_acc = 0.0    # 발생 개수 소수점 누적
        self._dt_acc = 0.0      # 물리 스텝 → 갱신 주기 누적
        self._dirty = False     # 마지막으로 쓴 뒤 지울 것이 남았는가
        self._cyl = None        # 가둘 관 (축점, 축방향, 내반경)

        # ── USD 구성 ──
        self.inst = UsdGeom.PointInstancer.Define(stage, path)
        protos = []
        for name, rgb, _, gain in _STAGES:
            pp = f"{path}/proto_{name}"
            cap = UsdGeom.Capsule.Define(stage, pp)
            cap.CreateAxisAttr("Z")
            cap.CreateRadiusAttr(_R)
            cap.CreateHeightAttr(_L)
            cap.CreateExtentAttr([Gf.Vec3f(-_R, -_R, -(_L / 2 + _R)),
                                  Gf.Vec3f(_R, _R, _L / 2 + _R)])
            cap.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
            self._emissive(stage, f"{path}/Looks/{name}", cap.GetPrim(),
                           rgb, gain)
            protos.append(Sdf.Path(pp))
        self.inst.CreatePrototypesRel().SetTargets(protos)
        self.inst.CreateProtoIndicesAttr([])
        self.inst.CreatePositionsAttr([])
        self.inst.CreateOrientationsAttr([])
        self.inst.CreateScalesAttr([])
        self._write(np.zeros(0, dtype=bool))

        # 🔑 **아크점 자체가 가장 밝다.** 멀리서 보면 개개의 스패터보다 이
        #    덩어리가 먼저 눈에 들어온다. 크기·밝기를 매 갱신마다 흔들어
        #    아크가 요동치는 것처럼 보이게 한다.
        self.ball = UsdGeom.Sphere.Define(stage, f"{path}/arc_ball")
        self.ball.CreateRadiusAttr(_BALL_R)
        self.ball.CreateExtentAttr([Gf.Vec3f(-_BALL_R, -_BALL_R, -_BALL_R),
                                    Gf.Vec3f(_BALL_R, _BALL_R, _BALL_R)])
        self.ball.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.98, 0.92)])
        self._emissive(stage, f"{path}/Looks/arc", self.ball.GetPrim(),
                       (1.0, 0.97, 0.88), 45.0)
        self._ball_xf = UsdGeom.Xformable(self.ball).AddTransformOp()
        UsdGeom.Imageable(self.ball).MakeInvisible()

    @staticmethod
    def _emissive(stage, mat_path, prim, rgb, gain):
        """발광 재질을 만들어 붙인다.

        🚨 `displayColor` 만 두면 RTX 기본 재질로 렌더돼 **주변 조명에
           좌우된다**(기록된 함정). 스파크는 스스로 빛나는 것이라 반드시
           emissive 여야 하고, `gain > 1` 이어야 톤매핑 뒤에도 하얗게 탄다.
        """
        mat = UsdShade.Material.Define(stage, mat_path)
        sh = UsdShade.Shader.Define(stage, f"{mat_path}/surface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.02, 0.02, 0.02))
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*[c * gain for c in rgb]))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),
                                                  "surface")
        UsdShade.MaterialBindingAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(mat)

    # ── 발생 ────────────────────────────────────────────────────────
    def _spawn(self, k, origin, normal):
        """빈 슬롯 k 개에 스패터를 낸다. `normal` 은 관 안쪽 단위벡터."""
        free = np.flatnonzero(~self.alive)
        if free.size == 0 or k <= 0:
            return
        idx = free[:k]
        m = idx.size
        n = np.asarray(normal, dtype=float)
        n /= max(np.linalg.norm(n), 1e-12)
        # normal 에 수직인 기저 두 개
        a = np.array([1.0, 0.0, 0.0])
        if abs(n[0]) > 0.9:
            a = np.array([0.0, 1.0, 0.0])
        t1 = np.cross(n, a)
        t1 /= np.linalg.norm(t1)
        t2 = np.cross(n, t1)

        cone = math.radians(self.p["cone_deg"])
        # 원뿔 안에서 면적 균등하게. 25% 는 관벽을 따라 스치듯 낮게 튄다
        th = cone * np.sqrt(self.rng.random(m))
        graze = self.rng.random(m) < 0.25
        th[graze] = math.radians(80.0) + self.rng.normal(0, 0.06, graze.sum())
        az = self.rng.random(m) * 2 * math.pi
        d = (np.cos(th)[:, None] * n
             + np.sin(th)[:, None] * (np.cos(az)[:, None] * t1
                                      + np.sin(az)[:, None] * t2))
        sp = np.maximum(self.rng.normal(self.p["v0"], self.p["v0_sd"], m), 0.15)

        self.pos[idx] = np.asarray(origin, dtype=float) + self.rng.normal(
            0.0, 0.0008, (m, 3))
        self.vel[idx] = d * sp[:, None]
        self.age[idx] = 0.0
        self.life[idx] = np.maximum(
            self.rng.normal(self.p["life"], self.p["life_sd"], m), 0.04)
        self.alive[idx] = True

    # ── 관벽 반사 ───────────────────────────────────────────────────
    def _bounce(self, p, v, age):
        """관 내벽에 맞은 스패터를 튕긴다.

        반발 0.35 · 접선 감쇠 0.75 — 뜨거운 금속 방울이 강관에 부딪히면 거의
        안 튀고 벽을 따라 미끄러진다. 부딪힐 때마다 수명을 12% 깎아(열을 뺏겨
        빨리 식는다) 벽에서 무한히 튀는 것을 막는다.
        """
        if callable(self._cyl):
            r, u_all, R = self._cyl(p)
        else:
            # 직선 원기둥. 곧은 구간에서만 쓸 것 — 아래 `lo/hi` 참조.
            c, ax, R, lo, hi = self._cyl
            d = p - c
            along = d @ ax
            # 🚨 **축방향도 막아야 한다.** 원기둥은 무한히 길어서 반경만
            #    가두면 스패터가 곧은 구간을 지나 곡관 벽을 뚫고 나간다
            #    (실측 비산 772mm vs 입구 직관 343mm). 국소 직선 근사가
            #    성립하는 범위를 벗어나면 그 자리에서 식은 것으로 친다.
            if lo is not None:
                gone = (along < lo) | (along > hi)
                if gone.any():
                    age[gone] = 1.0e9
            rad = d - along[:, None] * ax           # 축에서 벗어난 성분
            r = np.linalg.norm(rad, axis=1)
            u_all = rad / np.maximum(r, 1e-12)[:, None]
        hit = r > R
        if not hit.any():
            return p, v, age
        u = u_all[hit]
        p[hit] -= u * (r[hit] - R)[:, None]         # 벽면으로 되돌린다
        vn = (v[hit] * u).sum(axis=1)               # 벽을 파고드는 성분
        vt = v[hit] - vn[:, None] * u
        v[hit] = 0.75 * vt - 0.35 * np.minimum(vn, 0.0)[:, None] * u
        age[hit] += 0.12 * self.p["life"]
        return p, v, age

    # ── 적분 + 쓰기 ─────────────────────────────────────────────────
    def step(self, dt, *, emitting=False, origin=None, normal=None,
             light=None, light_base=3.0e5, confine=None):
        """물리 스텝마다 부른다. 인스턴서 쓰기는 60Hz 로 솎는다.

        `confine` 으로 관 안에 가둔다. 두 가지를 받는다.
          - **함수** `f(점배열) -> (중심선까지 거리, 바깥향 단위벡터, 내반경)`
            — 코스가 굽어 있어도 정확하다. 중심선 함수가 있으면 이쪽을 쓸 것.
          - **튜플** `(축위의_점, 축방향, 내반경[, 축방향 하한, 상한])`
            — 곧은 구간용 근사. 곡관이 가까우면 하한·상한을 반드시 줄 것.
        🚨 **이게 없으면 스패터가 관을 뚫고 나간다.** 기중 조건은 정지거리가
           0.5m 를 넘는데 관 내반경은 0.05m 다(실측 비산 1,804mm). 실제로도
           관 안에서 용접하면 스패터는 반대쪽 벽에 맞고 튀며 흩어진다 —
           가두는 쪽이 그림도 물리도 맞다.
        """
        if confine is not None:
            if callable(confine):
                self._cyl = confine
            else:
                self._cyl = (np.asarray(confine[0], dtype=float),
                             np.asarray(confine[1], dtype=float)
                             / max(np.linalg.norm(confine[1]), 1e-12),
                             float(confine[2]),
                             None if len(confine) < 5 else float(confine[3]),
                             None if len(confine) < 5 else float(confine[4]))
        self._dt_acc += dt
        if emitting and origin is not None:
            self._emit_acc += self.p["rate"] * dt
        if self._dt_acc < 1.0 / _UPDATE_HZ:
            return
        h, self._dt_acc = self._dt_acc, 0.0

        if emitting and origin is not None and normal is not None:
            k = int(self._emit_acc)
            self._emit_acc -= k
            self._spawn(k, origin, normal)

        live = self.alive
        if live.any():
            v = self.vel[live]
            # 선형 항력 + 중력. 물에서는 항력이 지배해 금세 멎는다.
            v += (-self.p["drag"] * v + np.array([0.0, 0.0, -self.p["g"]])) * h
            p = self.pos[live] + v * h
            age = self.age[live] + h
            if self._cyl is not None:
                p, v, age = self._bounce(p, v, age)
            self.pos[live], self.vel[live], self.age[live] = p, v, age
            self.alive[live] = self.age[live] < self.life[live]

        if self.alive.any() or self._dirty:
            self._write(self.alive)

        # 아크점 발광 구 — 멀리서는 이게 가장 먼저 보인다
        if emitting and origin is not None:
            f = 0.75 + 0.6 * self.rng.random()
            m = Gf.Matrix4d().SetScale(Gf.Vec3d(f, f, f))
            m.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in origin]))
            self._ball_xf.Set(m)
            UsdGeom.Imageable(self.ball).MakeVisible()
        else:
            UsdGeom.Imageable(self.ball).MakeInvisible()

        # 아크 불빛 깜빡임 — 실제 아크는 전류가 요동쳐 밝기가 흔들린다.
        # 🔑 상한을 1 보다 크게 둔다. 용접 아크는 주변을 **날려버릴 만큼**
        #    밝은 것이 정상이다(그래서 용접공이 차광면을 쓴다).
        if light is not None:
            f = 0.7 + 1.5 * self.rng.random() if emitting else 0.0
            light.GetIntensityAttr().Set(light_base * f)

    def _write(self, live):
        n_live = int(live.sum()) if live.size else 0
        self._dirty = n_live > 0
        if n_live == 0:
            self.inst.GetProtoIndicesAttr().Set(Vt.IntArray())
            self.inst.GetPositionsAttr().Set(Vt.Vec3fArray())
            self.inst.GetOrientationsAttr().Set(Vt.QuathArray())
            self.inst.GetScalesAttr().Set(Vt.Vec3fArray())
            return
        pos = self.pos[live]
        vel = self.vel[live]
        frac = self.age[live] / self.life[live]

        # 식는 단계 → 프로토타입 번호
        pi = np.full(n_live, len(_STAGES) - 1, dtype=np.int32)
        for i, (_, _, upper, _g) in enumerate(_STAGES):
            pi[frac <= upper] = np.minimum(pi[frac <= upper], i)

        sp = np.linalg.norm(vel, axis=1)
        s = 1.15 - 0.7 * frac                      # 식으면서 작아진다
        sz = np.stack([s, s, s * (0.6 + np.minimum(sp, 4.0) / 3.0)], axis=1)

        # 캡슐 로컬 +Z 를 속도 방향으로 돌린다 → 잔상 같은 줄무늬가 된다
        d = vel / np.maximum(sp, 1e-9)[:, None]
        qs = []
        for k in range(n_live):
            dx, dy, dz = float(d[k, 0]), float(d[k, 1]), float(d[k, 2])
            if dz < -0.999999:
                qs.append(Gf.Quath(0.0, 1.0, 0.0, 0.0))
                continue
            w = 1.0 + dz
            ax, ay, az = -dy, dx, 0.0     # cross((0,0,1), d)
            nrm = math.sqrt(w * w + ax * ax + ay * ay) or 1.0
            qs.append(Gf.Quath(w / nrm, ax / nrm, ay / nrm, az / nrm))

        self.inst.GetProtoIndicesAttr().Set(Vt.IntArray.FromNumpy(pi))
        self.inst.GetPositionsAttr().Set(
            Vt.Vec3fArray.FromNumpy(pos.astype(np.float32)))
        self.inst.GetOrientationsAttr().Set(Vt.QuathArray(qs))
        self.inst.GetScalesAttr().Set(
            Vt.Vec3fArray.FromNumpy(sz.astype(np.float32)))

    def clear(self):
        """전부 지운다. ARC 를 빠져나올 때·재시작할 때 부를 것."""
        self.alive[:] = False
        self._emit_acc = 0.0
        self._dt_acc = 0.0
        self._write(np.zeros(0, dtype=bool))
        UsdGeom.Imageable(self.ball).MakeInvisible()
