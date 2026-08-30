// 3D Map — three.js 배관 맵.
//
// ━━ 데이터 소스 — 전부 동적이다. 여기 좌표를 하드코딩하지 않는다 ━━━━━
//   코스 중심선  ← `course` 토픽(latched). 시연(real_map_demo --ros)의
//                  CenterLine 이 단일 출처다. 표본 [[s,x,y,z],...] (m).
//   CAD 메시     ← GET /mesh (.webmesh — tools/usd_to_webmesh.py 로 굽는다)
//   로봇·결함    ← drive_state 의 pos_m (코스와 같은 월드 m 좌표)
//
// three.js 는 /static 벤더 사본(오프라인 동작). Z-up 으로 쓴다 — Isaac/ROS
// 규약과 좌표를 맞추기 위해 DEFAULT_UP 을 바꾼다(회전 컨트롤도 Z 축 기준).
//
// 🔑 장면은 **모듈 수준에 한 번만** 만든다. 페이지를 오갈 때마다 WebGL 컨텍스트
//    와 메시를 새로 굽는 것은 비싸고, 브라우저의 컨텍스트 개수 한도(보통 16)에
//    걸리면 조용히 렌더가 죽는다. mount 는 캔버스를 다시 붙이고 rAF 만 켠다.
//
// ━━ 로봇 여러 대 (층별 동시 주행) ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//
// 🔑 로봇 하나가 `Viz` 하나다 — 코스 튜브·지나온 초록선·결함 마커·빨간 점이
//    전부 그 안에 들어 있고, 통째로 **하나의 Group** 에 담긴다. 층이 둘이면
//    빨간 점도 둘이고 각자 자기 층에서 움직인다.
//
// 🚨 **층마다 좌표 원점이 다르다.** 시연은 활성 층의 수평망을 월드 z=0 으로
//    올려놓고 좌표를 내므로(floor2 +250 / floor1 +2740.2mm), 두 대의 `pos_m`
//    을 그대로 겹치면 1층 로봇이 2층 배관 속을 달린다. 그래서 Viz 의 Group 을
//    `dzM(r)` 만큼 z 로 밀어 **기준 프레임(= /mesh 를 준 로봇)** 에 맞춘다 —
//    Viz 안쪽은 자기 좌표 그대로 두므로 계산이 한 군데(Group.position)뿐이다.
//    z 오프셋을 모르는 로봇은 못 민다(0) — 범례에 그 사실을 적는다.

import * as THREE from 'three';
import {OrbitControls} from '/static/OrbitControls.js';
import {store, bus, dzM} from '/static/app.js';
import {mountCam} from '/static/views/camera.js';
import {mountButtons, mountAllButtons} from '/static/views/handling.js';
import {mountFloorPanels} from '/static/views/status.js';

// 카메라 역할 설명 — 조종석(handling.js)과 같은 문구를 쓴다
// (v1_3: floor1 전방 / floor2 토치 고정, rear 폐지)
const CAM_WHEN = {front: '전방 — 주행', torch: '토치 — 결함·용접부'};

// 첫 프레임용 초기 크기일 뿐이다 — 실제 크기는 mount 의 ResizeObserver 가
// 칸(#map3d, CSS 로 화면 높이를 채운다)에서 재서 VW/VH 에 넣는다.
const W3 = 960, H3 = 460;
let VW = W3, VH = H3;

const V = {
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  mul: (a, k) => [a[0] * k, a[1] * k, a[2] * k],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0]],
  norm: a => { const l = Math.hypot(a[0], a[1], a[2]) || 1;
               return [a[0] / l, a[1] / l, a[2] / l]; },
};

// ── 장면 (모듈에 한 번) ──────────────────────────────────────
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);          // Z-up (Isaac/ROS 규약)
let renderer = null, glError = null;
try {
  renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  renderer.setSize(W3, H3, false);
  renderer.setClearColor(0x000000, 0);
  renderer.autoClear = false;
  // 캔버스는 칸을 꽉 채운다. 백버퍼(setSize)는 CSS 를 안 건드리는 쪽으로
  // 부르므로(updateStyle=false) 이 두 줄이 표시 크기를 정한다.
  renderer.domElement.style.width = '100%';
  renderer.domElement.style.height = '100%';
} catch (e) {
  glError = 'WebGL 사용 불가 — 브라우저 하드웨어 가속을 켤 것';
}

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(50, W3 / H3, 0.01, 100);
camera.position.set(-1, -2, 1.5);
scene.add(new THREE.AmbientLight(0xffffff, 0.9));
{
  const d1 = new THREE.DirectionalLight(0xffffff, 1.6);
  d1.position.set(0.4, 0.3, 0.85); scene.add(d1);
  const d2 = new THREE.DirectionalLight(0xffffff, 0.5);
  d2.position.set(-0.7, 0.5, -0.2); scene.add(d2);
}
const meshGroup = new THREE.Group(); scene.add(meshGroup);      // CAD 메시
let gridHelper = null;

// 🔴 로봇 점. 스프라이트로 두되 **월드 크기(sizeAttenuation:true)** 라
//    확대하면 커지고 축소하면 작아진다. 다만 전체 맵(≈5m)까지 줌아웃하면
//    관 내반경 50mm 짜리 점이 1px 밑으로 사라지므로, 매 프레임 화면상
//    크기를 재서 **최소 픽셀로 바닥을 받친다**(fitRobotDot). 옛 2D 판의
//    `r = max(2, 0.01*k)` 와 같은 규칙이다.
//
// 🚨 `transparent: true` 가 **반드시** 있어야 한다. three 는 불투명/반투명을
//    두 목록으로 갈라 불투명을 먼저 그리고, renderOrder 는 목록 **안에서만**
//    순서를 정한다. 이게 없으면 점은 불투명 목록에서 먼저 그려지고, 그 뒤에
//    오는 반투명 CAD 메시(배관 0.80 / 벽 0.22)가 그 위를 덮어 버린다 —
//    라벨만 뜨고 점은 안 보이는 증상이 정확히 이것이었다.
function spriteMat(draw) {
  const cv = document.createElement('canvas'); cv.width = cv.height = 64;
  draw(cv.getContext('2d'));
  return new THREE.SpriteMaterial({map: new THREE.CanvasTexture(cv),
    transparent: true, depthTest: false});
}
const dotMat = spriteMat(g => {
  g.beginPath(); g.arc(32, 32, 26, 0, 7);
  g.fillStyle = '#ff3333'; g.fill();
  g.lineWidth = 6; g.strokeStyle = '#ffffff'; g.stroke();
});
const xMat = spriteMat(g => {
  g.strokeStyle = '#f96'; g.lineWidth = 10; g.lineCap = 'round';
  g.beginPath(); g.moveTo(14, 14); g.lineTo(50, 50);
  g.moveTo(50, 14); g.lineTo(14, 50); g.stroke();
});
const sqMat = spriteMat(g => {
  g.fillStyle = '#fd4'; g.fillRect(12, 12, 40, 40);
  g.strokeStyle = '#a80'; g.lineWidth = 6; g.strokeRect(12, 12, 40, 40);
});

// 점의 실제 크기 = 관 내반경 × 1.4 (내경 100mm 관이면 70mm — 관을 거의 채워
// 로봇이 거기 있다는 게 읽힌다). 최소 픽셀은 **안전망일 뿐**이다: 이 조합이면
// 카메라가 8.6m 보다 멀 때만 걸리는데, 기본 시점이 전체 맵에서도 7.2m 라
// 실사용 줌 구간에서는 항상 배율대로 커지고 작아진다.
const DOT_WORLD_K = 1.4;
const DOT_MIN_PX = 5;

const axScene = new THREE.Scene();
axScene.add(new THREE.AxesHelper(1));            // X빨강 Y초록 Z파랑
const axCam = new THREE.PerspectiveCamera(50, 1, 0.1, 10);
axCam.up.set(0, 0, 1);

// OrbitControls 는 mount 마다 새로 만든다 — r160 에는 connect/disconnect 가
// 없고 dispose 뿐이라, 안 보이는 페이지에서 드래그를 먹지 않으려면 떼는 수밖에
// 없다. 카메라는 모듈에 살아 있고 target 만 여기 받아 두면 시점이 유지된다.
let controls = null;
const camTarget = new THREE.Vector3();
let framedBy = null;      // 'mesh' > 'course'
let meshLoaded = false, meshOn = true, meshNote = '';

class ArcCurve extends THREE.Curve {   // 호길이 파라미터 곡선 (선형 보간)
  constructor(viz, sMax) { super(); this.viz = viz; this.sMax = sMax; }
  getPoint(t) { return new THREE.Vector3(...this.viz.sToXyz(t * this.sMax)); }
}

// ── 로봇 하나의 그림 ─────────────────────────────────────────
class Viz {
  constructor(r) {
    this.r = r;
    this.pts = null; this.ir = 0.05; this.sTotal = 0;
    this.courseBuilt = -1; this.progressBuilt = -1;
    this.progressMesh = null; this.entryPos = null;
    // 🔑 이 Group 하나가 층 정렬을 담당한다 — 안쪽 좌표는 시연이 준 그대로다.
    this.root = new THREE.Group();
    this.courseGroup = new THREE.Group();
    this.markerGroup = new THREE.Group();
    this.root.add(this.courseGroup, this.markerGroup);
    this.dot = new THREE.Sprite(dotMat);
    this.dot.renderOrder = 11;            // 결함 마커(9)보다도 위
    this.dot.visible = false;
    this.root.add(this.dot);
    scene.add(this.root);
    this.align();
  }

  /** 기준 프레임에 맞춘다(층 사이 z 차이를 되민다). */
  align() { this.root.position.z = dzM(this.r); }

  dispose() {
    this.clearProgress();
    for (const g of [this.courseGroup, this.markerGroup])
      for (const o of g.children)
        if (o.geometry && !o.isSprite) o.geometry.dispose();
    scene.remove(this.root);
  }

  sToXyz(s) {         // 호길이 → 3D 좌표 (표본 사이는 선형 보간)
    const P = this.pts;
    s = Math.max(P[0].s, Math.min(s, P[P.length - 1].s));
    let lo = 0, hi = P.length - 1;
    while (hi - lo > 1) { const m = (lo + hi) >> 1; P[m].s <= s ? lo = m : hi = m; }
    const A = P[lo], B = P[hi], t = (s - A.s) / Math.max(B.s - A.s, 1e-9);
    return [0, 1, 2].map(k => A.p[k] + (B.p[k] - A.p[k]) * t);
  }

  wallP(s, aRad, r) {
    // 호길이 s + 시계각(라디안, 규약: 0=천장, π=바닥) → 관 **벽면** 좌표
    if (r === undefined) r = this.ir;
    const c = this.sToXyz(s);
    const t = V.norm(V.sub(this.sToXyz(s + 0.005), this.sToXyz(s - 0.005)));
    const up = Math.abs(t[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1];
    const u = V.norm(V.cross(t, up)), v = V.norm(V.cross(u, t));  // v = 천장
    // 🔑 sin 항은 -u — Isaac 쪽 규약(atan2(Δy,Δz): +90° = +Y)과 맞춘다.
    //    u = t×up 는 입구 직관에서 -Y 라 부호를 안 뒤집으면 거울상이 된다.
    return V.add(c, V.add(V.mul(v, r * Math.cos(aRad)),
                          V.mul(u, -r * Math.sin(aRad))));
  }

  wallXyz(s, clock) {
    return clock == null ? this.sToXyz(s)
                         : this.wallP(s, clock * Math.PI / 180);
  }

  snapCourse(pos) {   // 월드 좌표 → 중심선에서 가장 가까운 {s, p}
    const P = this.pts;
    let best = null;
    for (let i = 0; i + 1 < P.length; i++) {
      const A = P[i].p, B = P[i + 1].p, AB = V.sub(B, A), AP = V.sub(pos, A);
      const L2 = AB[0] ** 2 + AB[1] ** 2 + AB[2] ** 2;
      const t = L2 > 0 ? Math.max(0, Math.min(1,
        (AP[0] * AB[0] + AP[1] * AB[1] + AP[2] * AB[2]) / L2)) : 0;
      const Q = V.add(A, V.mul(AB, t));
      const d2 = (pos[0] - Q[0]) ** 2 + (pos[1] - Q[1]) ** 2
               + (pos[2] - Q[2]) ** 2;
      if (best === null || d2 < best.d2)
        best = {d2, p: Q, s: P[i].s + (P[i + 1].s - P[i].s) * t};
    }
    return best;
  }

  /** 코스 표본의 bbox (자기 좌표계). 없으면 null. */
  bbox() {
    if (!this.pts) return null;
    const b = this.pts.reduce((m, o) => ({
      lo: m.lo.map((x, k) => Math.min(x, o.p[k])),
      hi: m.hi.map((x, k) => Math.max(x, o.p[k]))}),
      {lo: [1e9, 1e9, 1e9], hi: [-1e9, -1e9, -1e9]});
    const dz = this.root.position.z;
    return {lo: [b.lo[0] - 2 * this.ir, b.lo[1] - 2 * this.ir,
                 b.lo[2] - 2 * this.ir + dz],
            hi: [b.hi[0] + 2 * this.ir, b.hi[1] + 2 * this.ir,
                 b.hi[2] + 2 * this.ir + dz]};
  }

  buildCourse() {
    const msg = this.r.course;
    if (!msg || this.courseBuilt === msg.stamp) return false;
    this.courseBuilt = msg.stamp;
    this.pts = msg.pts.map(q => ({s: q[0], p: [q[1], q[2], q[3]]}));
    this.ir = msg.ir_m || 0.05;
    this.sTotal = msg.s_total_m || this.pts[this.pts.length - 1].s;
    for (const o of this.courseGroup.children)
      if (o.geometry) o.geometry.dispose();
    this.courseGroup.clear();
    this.courseGroup.add(new THREE.Mesh(       // 관 튜브 (반투명)
      new THREE.TubeGeometry(new ArcCurve(this, this.sTotal),
                             Math.max(96, this.pts.length * 2), this.ir, 14,
                             false),
      new THREE.MeshLambertMaterial({color: 0x4a7aa8, transparent: true,
        opacity: 0.35, side: THREE.DoubleSide, depthWrite: false})));
    this.courseGroup.add(new THREE.Line(       // 중심선
      new THREE.BufferGeometry().setFromPoints(
        this.pts.map(o => new THREE.Vector3(...o.p))),
      new THREE.LineBasicMaterial({color: 0x3a5a78})));
    this.entryPos = this.pts[0].p;
    this.progressBuilt = -1;
    this.buildProgress(); this.buildMarks();
    return true;                     // 시야를 다시 잡아야 한다
  }

  clearProgress() {   // 재시작 — 지나온 초록선을 걷어낸다
    // 🚨 `buildProgress()` 만으로는 안 지워진다 — maxS 가 0 이면 맨 위에서
    //    바로 돌아가 **옛 메시가 장면에 그대로 남는다.** 지우는 길을 따로 둔다.
    if (this.progressMesh) {
      this.root.remove(this.progressMesh);
      this.progressMesh.geometry.dispose();
      this.progressMesh = null;
    }
    this.progressBuilt = -1;
  }

  buildProgress() {   // 지나온 구간 (초록 굵은 선)
    if (!this.pts) return;
    // 🔑 답파거리가 **뒤로 갔다** = 시연 재시작(app.js 가 maxS 를 0 으로
    //    되돌린다). 3D 맵을 안 보고 있는 사이에 재시작이 나도 여기서 걸린다 —
    //    mount 가 이 함수를 다시 부르기 때문이다.
    const maxS = this.r.maxS;
    if (this.progressMesh && maxS < this.progressBuilt - 0.001)
      this.clearProgress();
    if (maxS <= 0.01) return;
    if (this.progressMesh && maxS - this.progressBuilt < 0.02) return;
    this.progressBuilt = maxS;
    if (this.progressMesh) {
      this.root.remove(this.progressMesh);
      this.progressMesh.geometry.dispose();
    }
    this.progressMesh = new THREE.Mesh(
      new THREE.TubeGeometry(new ArcCurve(this, Math.min(maxS, this.sTotal)),
                             128, 0.007, 8, false),
      // transparent 가 있어야 반투명 CAD 메시보다 **뒤에** 그려진다(위 참고)
      new THREE.MeshBasicMaterial({color: 0x2f8f5a, transparent: true,
                                   depthTest: false}));
    this.progressMesh.renderOrder = 8;
    this.root.add(this.progressMesh);
  }

  buildMarks() {
    if (!this.pts) return;
    // 🚨 Sprite 의 geometry 는 three 가 **모든 스프라이트끼리 공유**하는 하나다.
    //    여기서 dispose 하면 다른 마커까지 같이 날린다 — 스프라이트는 건너뛴다.
    for (const o of this.markerGroup.children)
      if (o.geometry && !o.isSprite) o.geometry.dispose();
    this.markerGroup.clear();
    for (const d of this.r.marks) {
      if (d.repaired && d.clock != null) {
        // 수리 스티커 = 관 벽면에 밀착한 곡면 패치 (축 ±16mm × 원주 ±25°).
        // 벽면보다 1mm 안쪽으로 띄워 튜브 표면과의 z-fight 를 피한다.
        const a0 = d.clock * Math.PI / 180, dS = 0.016,
              dA = 25 * Math.PI / 180, N = 6, pos = [], idx = [];
        for (const s of [d.s - dS, d.s + dS])
          for (let i = 0; i <= N; i++)
            pos.push(...this.wallP(s, a0 - dA + 2 * dA * i / N, this.ir - 0.001));
        for (let i = 0; i < N; i++)
          idx.push(i, i + 1, N + 1 + i, i + 1, N + 2 + i, N + 1 + i);
        const g = new THREE.BufferGeometry();
        g.setAttribute('position',
          new THREE.BufferAttribute(new Float32Array(pos), 3));
        g.setIndex(idx); g.computeVertexNormals();
        const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
          color: 0xffdd44, side: THREE.DoubleSide, transparent: true,
          opacity: 0.9, depthTest: false}));
        m.renderOrder = 9;
        this.markerGroup.add(m);
      } else {
        // ✕(미수리) / ■(시계각 미상) 은 항상 카메라를 보는 스프라이트
        const sp = new THREE.Sprite(d.repaired ? sqMat : xMat);
        sp.position.set(...this.wallXyz(d.s, d.clock));
        sp.scale.set(0.05, 0.05, 1);
        sp.renderOrder = 9;
        this.markerGroup.add(sp);
      }
    }
  }

  updateRobot() {
    // 🔴 pos_m(월드 좌표)이 오면 중심선에 스냅한다. 없으면(구판) s 로.
    const d = this.r.state;
    if (!this.pts || !d) { this.dot.visible = false; return null; }
    const pos = (Array.isArray(d.pos_m) && d.pos_m.length === 3) ? d.pos_m : null;
    let p3, sLab;
    if (pos) { const sn = this.snapCourse(pos); p3 = sn.p; sLab = sn.s; }
    else { p3 = this.sToXyz((d.s_mm || 0) / 1000); sLab = (d.s_mm || 0) / 1000; }
    this.dot.position.set(...p3);
    this.dot.visible = true;
    return `${this.r.label} s=${(sLab * 1000).toFixed(0)}mm`;
  }

  /** 점 크기를 정한다 — 관 크기에 비례하되 화면에서 사라지지는 않게.
   *
   * 원근 투영에서 월드 크기 s 인 스프라이트의 화면 높이(px)는
   *     px = P₁₁ · s / dist · H / 2      (P₁₁ = 1/tan(fov/2), H = 표시 높이)
   * 이므로, 최소 픽셀을 보장하는 월드 크기는 s = 2·px·dist / (P₁₁·H) 다. */
  /** 이 Viz 안의 좌표를 월드(기준 프레임)로. 🔑 행렬을 안 쓴다 — Group 은
   *  z 로만 밀리고, 행렬은 렌더 뒤에야 갱신돼 한 프레임 낡는다. */
  world(p3) { return [p3[0], p3[1], p3[2] + this.root.position.z]; }

  fitDot(viewH) {
    const world = this.ir * DOT_WORLD_K;
    const dist = camera.position.distanceTo(
      _tmpV.set(...this.world(this.dot.position.toArray())));
    const p11 = camera.projectionMatrix.elements[5];
    const floor = 2 * DOT_MIN_PX * dist / Math.max(p11 * viewH, 1e-6);
    this.dot.scale.setScalar(Math.max(world, floor));
    this.dot.scale.z = 1;
  }
}
const _tmpV = new THREE.Vector3();

// ns → Viz. 로봇 명패가 바뀌면 맞춰 준다(페이지를 안 보고 있어도 유지된다).
const VIZ = new Map();
function syncViz() {
  for (const r of store.robots)
    if (!VIZ.has(r.ns)) VIZ.set(r.ns, new Viz(r));
  for (const [ns, v] of [...VIZ])
    if (!store.byNs.has(ns)) { v.dispose(); VIZ.delete(ns); }
  // 명패가 갱신되면 로봇 객체 자체가 바뀔 수 있다 — 참조를 다시 건다.
  for (const r of store.robots) {
    const v = VIZ.get(r.ns);
    v.r = r;
    v.align();
  }
}

function frame(lo, hi, by) {
  if (framedBy === 'mesh' && by !== 'mesh') return;
  framedBy = by;
  const c = new THREE.Vector3((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
                              (lo[2] + hi[2]) / 2);
  const dim = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], 0.5);
  camera.position.set(c.x - 0.5 * dim, c.y - 1.1 * dim, c.z + 0.8 * dim);
  camera.near = dim / 100; camera.far = dim * 30;
  camera.updateProjectionMatrix();
  camTarget.copy(c);
  if (controls) { controls.target.copy(c); controls.update(); }
  else camera.lookAt(c);
  if (gridHelper) { scene.remove(gridHelper); gridHelper.geometry.dispose(); }
  const size = Math.ceil((dim + 0.6) * 10) / 10;      // 바닥 격자 0.1m
  gridHelper = new THREE.GridHelper(size, Math.round(size / 0.1),
                                    0x333333, 0x222222);
  gridHelper.rotation.x = Math.PI / 2;                // XZ 평면 → XY 로
  gridHelper.position.set(c.x, c.y, lo[2] - 0.08);
  scene.add(gridHelper);
}

/** 코스가 있는 로봇 **전부**를 담는 시야. 한 층만 보이면 그 층만 잡는다. */
function frameCourses() {
  let lo = null, hi = null;
  for (const v of VIZ.values()) {
    const b = v.bbox();
    if (!b) continue;
    lo = lo ? lo.map((x, k) => Math.min(x, b.lo[k])) : b.lo;
    hi = hi ? hi.map((x, k) => Math.max(x, b.hi[k])) : b.hi;
  }
  if (lo) frame(lo, hi, 'course');
}

// ── CAD 메시 — .webmesh 를 한 번만 fetch 한다 ────────────────
// 포맷·좌표계는 tools/usd_to_webmesh.py 참고. 파트별로 Mesh 를 갈라 두면
// three.js 가 반투명 정렬(뒤→앞)을 파트 단위로 해 준다. BufferAttribute 는
// 전 파트가 공유한다(업로드 1회).
// 🔑 메시에는 **두 층이 다 들어 있다** — 로봇이 두 대여도 한 번만 받는다.
//    받은 좌표계가 곧 기준 프레임이고, 각 Viz 가 거기에 맞춰 밀린다.
const GROUP_COLOR = {floor2: 0x5c9ee0, floor1: 0x54b38c,
                     aisle: 0x9999a8, etc: 0xb39966};
let meshPromise = null;
function loadMesh() {
  // 실패해도 코스 튜브로 동작하지만, 왜 메시가 안 떴는지는 화면에 반드시
  // 적는다 — 조용한 실패는 "파이프만 나온다" 문의가 된다(실화).
  if (meshPromise) return meshPromise;
  meshPromise = (async () => {
    if (!renderer) { meshNote = glError; return; }
    let buf;
    try {
      const r = await fetch('/mesh');
      if (!r.ok) { meshNote = '메시 파일 없음 — usd_to_webmesh.py 로 구울 것';
                   return; }
      buf = await r.arrayBuffer();
    } catch (e) { meshNote = '메시 fetch 실패: ' + e; return; }
    const L = new DataView(buf).getUint32(0, true);
    const hdr = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, L)));
    let off = 4 + L; off += (4 - off % 4) % 4;
    const nv = hdr.vtx_count, nt = hdr.tri_count;
    const pos = new Float32Array(buf, off, nv * 3);
    const aP = new THREE.BufferAttribute(pos, 3);
    const aN = new THREE.BufferAttribute(
      new Float32Array(buf, off + nv * 12, nv * 3), 3);
    const aI = new THREE.BufferAttribute(
      new Uint32Array(buf, off + nv * 24, nt * 3), 1);
    for (const p of hdr.parts) {
      // 벽/배관 분류 — 두 번째로 긴 축이 1m 를 넘는 파트는 방 껍데기(벽)다
      // (이 맵 실측: 벽 3개 1.5×1.8m+, 배관은 전부 0.7m 이하). 벽은 아주
      // 옅게 깔아 안의 배관·로봇이 비쳐 보이게 한다.
      const mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9];
      for (let i = p.vtx_start * 3; i < (p.vtx_start + p.vtx_count) * 3; i += 3)
        for (let k = 0; k < 3; k++) {
          const v = pos[i + k];
          if (v < mn[k]) mn[k] = v;
          if (v > mx[k]) mx[k] = v;
        }
      const ext = mx.map((x, k) => x - mn[k]).sort((a, b) => b - a);
      const shell = ext[1] > 1.0;
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', aP); g.setAttribute('normal', aN);
      g.setIndex(aI); g.setDrawRange(p.idx_start, p.idx_count);
      // 공유 어트리뷰트라 자동 bounding 계산이 전체 맵을 잡는다 — 파트 bbox 로
      // 직접 넣어 준다(반투명 뒤→앞 정렬이 이 구를 쓴다).
      g.boundingSphere = new THREE.Sphere(
        new THREE.Vector3((mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2,
                          (mn[2] + mx[2]) / 2),
        Math.hypot(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]) / 2);
      const m = new THREE.Mesh(g, new THREE.MeshLambertMaterial({
        color: shell ? 0x9fa6b8 : (GROUP_COLOR[p.group] || GROUP_COLOR.etc),
        transparent: true,
        opacity: shell ? 0.22 : (p.group === 'floor2' ? 0.80 : 0.50),
        side: THREE.DoubleSide, depthWrite: false}));
      m.renderOrder = shell ? 0 : 1;       // 벽(모든 걸 감싼다)을 먼저
      meshGroup.add(m);
    }
    meshLoaded = true;
    frame(hdr.bbox[0], hdr.bbox[1], 'mesh');
    // 🔑 **잘 됐을 때는 화면에 아무 말도 안 한다** (사용자 지시 2026-08-10 —
    //    범례에는 "CAD 메시" 체크박스만 남긴다). 파일 이름·삼각형 수는 진단용
    //    이라 콘솔로 내린다. 🚨 **실패 문구는 그대로 화면에 띄운다** — 조용한
    //    실패는 "파이프만 나온다" 문의가 된다(실화).
    meshNote = '';
    console.log(`[map3d] CAD 메시 ${hdr.source} · `
                + `${(nt / 1000).toFixed(1)}k tri · z ${hdr.z_shift_mm}mm`);
  })();
  return meshPromise;
}

// ── 페이지 ───────────────────────────────────────────────────
// 화면은 **세 칸**이다 (ref_img/web_ui.jpg):
//   왼쪽   3D 맵 (제일 넓다 — 여기가 주인공이다)
//   가운데 로봇 상태 → 결함 현황 → 임무 지령 (위에서 아래로 이 순서다:
//          지금 뭘 하는지 → 그 결과로 쌓인 숫자 → 내가 누를 것)
//   오른쪽 카메라 두 칸 (위가 2층 / 아래가 1층 — 건물 순서대로)
//
// 🔑 가운데·오른쪽 부속은 **전부 남의 모듈을 끼운다** — 두 표는
//    views/status.js, 버튼은 views/handling.js 의 `mountButtons`, 카메라는
//    views/camera.js 의 `mountCam`. 같은 것을 여기에 또 만들면 조종석과
//    이 화면이 서로 다른 말을 하게 된다.
export function mount(el) {
  el.innerHTML = `
   <div class="m3">
    <div class="m3-map">
     <div class="legend">
      <span><b style="color:#f33">●</b> 로봇</span>
      <span><b style="color:#f96">✕</b> 결함</span>
      <span><b style="color:#fd4">■</b> 수리 완료</span>
      <span class="muted">우클릭 회전 · 휠 눌러 이동 · 휠 굴려 확대</span>
      <label style="margin-left:auto"><input type="checkbox" id="m-ck" checked>
       CAD 메시</label>
      <span class="muted" id="m-note"></span>
     </div>
     <div id="map3d"><div id="labels"></div></div>
    </div>

    <div class="m3-mid">
     <div class="card">
      <h2>로봇 상태 <span class="muted">층별</span></h2>
      <div id="m-state"></div>
     </div>
     <div class="card">
      <h2>결함 현황</h2>
      <div id="m-defect"></div>
     </div>
     <div class="card">
      <h2>임무 지령 <span class="muted">층마다 따로</span></h2>
      <div id="m-cmd"></div>
      <div class="toast" id="m-toast"></div>
     </div>
    </div>

    <div class="m3-cam" id="m-cams"></div>
   </div>`;

  const wrap = el.querySelector('#map3d');
  const note = el.querySelector('#m-note');
  const ck = el.querySelector('#m-ck');
  const labels = el.querySelector('#labels');

  // ── 가운데 칸 — 로봇 상태 · 결함 현황 · 임무 지령 ─────────────
  const offPanels = mountFloorPanels(el.querySelector('#m-state'),
                                     el.querySelector('#m-defect'));
  const cmdBox = el.querySelector('#m-cmd');
  // 🔑 결과 줄(toast)은 카드 맨 아래 **하나만** 둔다 — 묶음마다 두면 좁은
  //    가운데 칸에서 그만큼 아래 버튼이 화면 밖으로 밀린다.
  const cmdToast = el.querySelector('#m-toast');
  const group = (label) => {
    const g = document.createElement('div');
    g.className = 'cmdgrp';
    g.innerHTML = `<div class="glab">${label}</div><div class="slot"></div>`;
    cmdBox.appendChild(g);
    return g.querySelector('.slot');
  };
  // 🔑 **두 대면 "전체" 를 맨 위에 둔다.** 층마다 프로세스가 따로라 기동
  //    시각이 다른데, `--wait` 로 세워 두고 이 버튼으로 같은 순간에 출발시킨다.
  //    (조종석 페이지가 꺼져 있으므로 여기가 유일한 자리다.)
  // 🔑 `mountAllButtons` 도 이제 해제 함수를 돌려준다 — 전체 비상정지 버튼이
  //    상태를 따라 ⛔↔🔓 로 바뀌느라 bus 를 잡기 때문이다. 반드시 받아 둘 것.
  const offAll = store.robots.length > 1
    ? mountAllButtons(group('전체'), {compact: true, toast: cmdToast}) : null;
  const offBtns = store.robots.map(r =>
    mountButtons(group(r.label), r, {compact: true, toast: cmdToast}));
  if (offAll) offBtns.push(offAll);
  if (!store.robots.length)
    cmdBox.innerHTML = '<div class="empty">로봇 명패 수신 대기…</div>';

  // ── 오른쪽 칸 — 카메라. 🔑 **위가 2층, 아래가 1층**이다(건물 순서 그대로).
  //    다른 화면의 왼→오른쪽 규칙(1층 왼쪽)을 세로로 옮기면 위아래가 건물과
  //    거꾸로 서서 오히려 헷갈린다.
  const camBox = el.querySelector('#m-cams');
  const offCams = [...store.robots].reverse().map(r => {
    const card = document.createElement('div');
    card.className = 'card camcard';
    card.innerHTML = `<h2>${r.label} 카메라 `
      + `<span class="muted camwhen"></span></h2><div class="slot"></div>`;
    camBox.appendChild(card);
    const when = card.querySelector('.camwhen');
    const off = mountCam(card.querySelector('.slot'), {robot: r, bare: true});
    const offW = bus.on('state', rr => {
      if (rr === r) when.textContent = CAM_WHEN[(rr.state || {}).cam] || '';
    });
    return () => { off(); offW(); };
  });

  if (!renderer) {
    wrap.innerHTML = `<div class="empty" style="padding:60px;
      text-align:center">${glError}</div>`;
    return () => {
      offPanels(); offBtns.forEach(f => f()); offCams.forEach(f => f());
    };
  }
  syncViz();

  // 층마다 라벨 두 개 (로봇 · 입구). DOM 은 mount 때만 만든다.
  const labs = store.robots.map(r => {
    const rb = document.createElement('div');
    rb.className = 'lab'; rb.style.color = '#f88'; rb.style.display = 'none';
    const en = document.createElement('div');
    en.className = 'lab'; en.style.color = '#888'; en.style.display = 'none';
    en.textContent = `${r.label} 입구`;
    labels.append(rb, en);
    return {r, viz: VIZ.get(r.ns), rb, en};
  });

  wrap.insertBefore(renderer.domElement, wrap.firstChild);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.15;
  // 마우스 배정: 좌 = **아무것도 안 함**, 휠클릭 = 이동, 우 = 회전.
  // (기본값은 좌=회전, 휠클릭=줌, 우=이동이라 셋 다 바꾼 것이다.) null 로
  // 두면 onMouseDown 의 switch 가 default 로 떨어져 state=NONE 이 된다.
  controls.mouseButtons = {LEFT: null,
                           MIDDLE: THREE.MOUSE.PAN,
                           RIGHT: THREE.MOUSE.ROTATE};
  controls.target.copy(camTarget);         // 지난 시점을 이어받는다
  controls.update();
  // 🚨 OrbitControls 의 pointerdown 은 기본 동작을 막지 않는다 — 그대로 두면
  //    휠 클릭에 브라우저 **자동 스크롤**(닻 아이콘)이 떠서 이동과 겹친다.
  //    가운데·오른쪽 버튼의 기본 동작만 여기서 막는다(왼쪽은 그대로).
  const noDefault = e => { if (e.button === 1 || e.button === 2)
                             e.preventDefault(); };
  renderer.domElement.addEventListener('mousedown', noDefault);
  renderer.domElement.addEventListener('pointerdown', noDefault);
  ck.checked = meshOn;
  ck.onchange = () => { meshOn = ck.checked; meshGroup.visible = meshOn; };
  meshGroup.visible = meshOn;

  // ── 칸 크기 추적 ────────────────────────────────────────────
  // 🚨 백버퍼를 같이 안 늘리면 960×460 짜리를 CSS 로 잡아늘인 그림이 된다
  //    (흐릿하고, 세로로 눌린다). 칸이 바뀔 때마다 setSize + aspect 를 다시
  //    잡는다. updateStyle=false 라 위에서 준 width/height 100% 는 남는다.
  function resize() {
    const w = Math.max(wrap.clientWidth | 0, 1);
    const h = Math.max(wrap.clientHeight | 0, 1);
    if (w === VW && h === VH) return;
    VW = w; VH = h;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  resize();
  // 창 크기·사이드바뿐 아니라 페이지 전환으로 칸이 다시 붙을 때도 걸린다.
  const ro = new ResizeObserver(resize);
  ro.observe(wrap);

  function placeLabel(elm, p3, dx) {
    const v = new THREE.Vector3(...p3).project(camera);
    if (v.z > 1 || Math.abs(v.x) > 1.1 || Math.abs(v.y) > 1.1) {
      elm.style.display = 'none'; return;
    }
    elm.style.display = 'block';
    elm.style.left = ((v.x * 0.5 + 0.5) * wrap.clientWidth + dx) + 'px';
    elm.style.top = ((-v.y * 0.5 + 0.5) * wrap.clientHeight) + 'px';
  }

  // 맵 아래 줄(코스 길이·답파·결함·층 정렬 경고)은 **뺐다** — 사용자 지시
  // 2026-08-10. 같은 숫자가 가운데 상태·결함 표에 이미 있어서 두 벌이었다.
  // 🚨 층 정렬을 못 하는 상황은 화면에서 사라졌지만 **서버 로그에는 남는다**
  //    (`web_panel` 의 "메시 프레임 … 과 다르다" WARN). 그림이 조용히 틀리는
  //    경우라 어딘가에는 반드시 적혀 있어야 한다.
  // 🔑 이 자리는 이제 **실패했을 때만** 글자가 뜬다(성공 시 파일 이름·삼각형
  //    수는 콘솔로 내렸다 — 사용자 지시). 그래서 눈에 띄는 색으로 적는다.
  function drawMeshNote() {
    note.textContent = meshNote ? `⚠ ${meshNote}` : '';
    note.style.color = meshNote ? 'var(--warn)' : '';
  }

  // 🚨 buildProgress 를 따로 부른다 — buildCourse 는 코스가 그대로면 바로
  //    돌아가서, 화면을 떠나 있는 동안 일어난 재시작을 못 반영한다.
  let fresh = false;
  for (const v of VIZ.values()) {
    fresh = v.buildCourse() || fresh;
    v.buildProgress(); v.buildMarks();
  }
  if (fresh) frameCourses();
  drawMeshNote();
  loadMesh().then(drawMeshNote);

  let raf = 0;
  function tick() {
    raf = requestAnimationFrame(tick);
    controls.update();
    const hideCourse = meshLoaded && meshOn;
    for (const v of VIZ.values()) {
      v.courseGroup.visible = !hideCourse;
      // 줌·회전이 매 프레임 바뀌므로 점 크기도 매 프레임 다시 잰다.
      // 화면에 보이는 높이(CSS px)로 재야 캔버스가 가로 100% 로 늘어난 배율
      // 까지 반영된다 — 캔버스 백버퍼(H3)로 재면 창 크기에 따라 어긋난다.
      if (v.dot.visible) v.fitDot(wrap.clientHeight || VH);
    }
    renderer.clear();
    renderer.render(scene, camera);
    // 좌표계 게이지 (왼쪽 아래 인셋) — Z 가 위 (Isaac/ROS 규약).
    // autoClear=false 라 본 장면의 깊이가 남아 있다 — 이 칸만 깊이를 지운다.
    const s = 76, m = 6;
    renderer.setScissorTest(true);
    renderer.setViewport(m, m, s, s); renderer.setScissor(m, m, s, s);
    renderer.clearDepth();
    axCam.position.copy(camera.position).sub(controls.target).setLength(2.8);
    axCam.lookAt(0, 0, 0);
    renderer.render(axScene, axCam);
    renderer.setScissorTest(false);
    renderer.setViewport(0, 0, VW, VH);
    for (const L of labs) {
      if (L.viz.dot.visible)
        placeLabel(L.rb, L.viz.world(L.viz.dot.position.toArray()), 10);
      else L.rb.style.display = 'none';
      if (L.viz.entryPos) placeLabel(L.en, L.viz.world(L.viz.entryPos), 8);
    }
  }
  for (const L of labs) L.rb.textContent = L.viz.updateRobot() || '';
  tick();

  const vizOf = r => VIZ.get(r.ns);
  const off = [
    bus.on('state', r => {
      const v = vizOf(r);
      if (!v) return;
      const lab = v.updateRobot();
      const L = labs.find(x => x.r.ns === r.ns);
      if (lab && L) L.rb.textContent = lab;
      v.buildProgress();
    }),
    bus.on('course', r => {
      const v = vizOf(r);
      if (v && v.buildCourse()) frameCourses();
    }),
    bus.on('event', r => { const v = vizOf(r); if (v) v.buildMarks(); }),
    bus.on('defect', r => { const v = vizOf(r); if (v) v.buildMarks(); }),
    // 명패가 갱신되면(늦게 온 z 오프셋) 층 정렬을 다시 건다.
    bus.on('roster', () => { for (const v of VIZ.values()) v.align(); }),
    // 시연 재시작 — 초록선과 마커는 지난 판의 것이다 (app.js 의 onState).
    bus.on('reset', (r, run) => {
      const v = vizOf(r);
      if (!v) return;
      v.clearProgress(); v.buildMarks();
      console.log('[map3d]', r.ns, '시연 재시작 — 답파 구간 초기화 (run',
                  run, ')');
    }),
  ];

  return () => {
    cancelAnimationFrame(raf);
    ro.disconnect();
    off.forEach(f => f());
    offPanels(); offBtns.forEach(f => f()); offCams.forEach(f => f());
    // 🔑 캔버스는 버리지 않고 떼기만 한다 — 다음 mount 가 그대로 다시 쓴다
    //    (WebGL 컨텍스트·업로드된 메시를 살려 둔다). 컨트롤만 dispose 해서
    //    전역 리스너를 떼어야 안 보이는 페이지에서 드래그를 먹지 않는다.
    camTarget.copy(controls.target);
    controls.dispose();
    controls = null;
    renderer.domElement.removeEventListener('mousedown', noDefault);
    renderer.domElement.removeEventListener('pointerdown', noDefault);
    renderer.domElement.remove();
  };
}
