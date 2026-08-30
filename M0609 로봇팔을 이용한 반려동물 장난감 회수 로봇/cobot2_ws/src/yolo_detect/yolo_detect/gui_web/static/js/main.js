// Web UI — yolo_connector_node가 호스팅하는 WebSocket에 붙어
// 1) 3D 맵(점군) 렌더링, 2) 상태 표시, 3) 정지/재개/비상정지/안전상태해제 명령 전송.
// rokey_bootbot(ws_cobot_pjt)의 MANAGER 3D 뷰어와 같은 이유로 rosbridge 대신
// 이 앱 전용 WebSocket + Three.js 조합을 그대로 재사용한다(esm.sh CDN, npm 불필요).
import * as THREE from "https://esm.sh/three@0.160.0";

const els = {
  connBadge: document.getElementById("conn-badge"),
  pointCount: document.getElementById("point-count"),
  pathInfo: document.getElementById("path-info"),
  statState: document.getElementById("stat-state"),
  statMode: document.getElementById("stat-mode"),
  statTcp: document.getElementById("stat-tcp"),
  statJoints: document.getElementById("stat-joints"),
  log: document.getElementById("log"),
  btnPause: document.getElementById("btn-pause"),
  btnResume: document.getElementById("btn-resume"),
  btnEstop: document.getElementById("btn-estop"),
  btnSafetyReset: document.getElementById("btn-safety-reset"),
};

function appendLog(text) {
  const time = new Date().toLocaleTimeString("ko-KR");
  els.log.textContent += `[${time}] ${text}\n`;
  els.log.scrollTop = els.log.scrollHeight;
}

// ---------------- Three.js 점군 뷰어 ----------------

const viewerEl = document.getElementById("viewer");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0c0d10);

const camera = new THREE.PerspectiveCamera(60, viewerEl.clientWidth / viewerEl.clientHeight, 1, 10000);
camera.position.set(500, -500, 500);
camera.up.set(0, 0, 1);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(viewerEl.clientWidth, viewerEl.clientHeight);
viewerEl.appendChild(renderer.domElement);

const axes = new THREE.AxesHelper(200);
scene.add(axes);

// ---- 표시 전용 색 보정(오토레벨 + 채도) ----
// ⚠️ 데이터를 고치는 게 아니라 "보이는 것"만 펴는 것. 필요한 이유(2026-07-25 실측):
// 원본 jpg가 이미 어둡고 무채색이다 — waypoint_1은 밝기 평균 122/채도 평균 6.8,
// waypoint_3은 200을 넘는 픽셀이 아예 0개(최대 170). 흰 박스 안에서 RealSense 컬러
// 노출이 중간 회색으로 눌려 찍힌 것으로, 그대로 그리면 회색 점만 보인다.
// 밝기는 2~98 백분위를 0.15~1.0으로 늘리고(완전 검정으로 눌리지 않게 하한을 둠),
// 색상(hue)은 유지한 채 채도만 배율로 키운다.
// ⚠️ 2026-07-25 정정: 처음엔 "색이 어둡다"고 보고 오토레벨+채도3배를 넣었는데,
// 실제 원인은 색이 아니라 **점 사이 빈 틈**이었다(점크기 1.6mm < 간격 3mm -> 영역의
// 66%가 검정 -> 평균 밝기 26). 컬러 뷰어(실사로 보였던 것)의 벽도 픽셀값이 117로
// 원본 jpg(122)와 같았고, 흰색으로 보인 건 검은 배경 대비 '연속면'이었기 때문이다.
// 틈을 메우면 보정 없이도 그 화면이 나오므로 기본을 1.0(무보정)으로 되돌린다.
// 그래도 더 선명하게 보고 싶으면 이 값만 1.5~3.0으로 올리면 된다.
const COLOR_SATURATION_GAIN = 1.0;
const AUTO_LEVEL = false;   // 밝기 스트레치(기본 off — 위 주석 참고)
function enhanceColors(colors) {
  if (!AUTO_LEVEL && COLOR_SATURATION_GAIN === 1.0) return;   // 무보정이면 통째로 건너뜀
  const n = colors.length / 3;
  if (n === 0) return;
  const lum = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    lum[i] = (colors[i * 3] + colors[i * 3 + 1] + colors[i * 3 + 2]) / 3;
  }
  const sorted = Float32Array.from(lum).sort();
  const lo = sorted[Math.floor(0.02 * (n - 1))];
  const hi = sorted[Math.floor(0.98 * (n - 1))];
  const span = Math.max(1e-4, hi - lo);
  for (let i = 0; i < n; i++) {
    const l = lum[i];
    // 오토레벨된 목표 밝기
    const target = AUTO_LEVEL ? 0.15 + 0.85 * Math.min(1, Math.max(0, (l - lo) / span)) : l;
    const scale = l > 1e-4 ? target / l : 0;
    for (let c = 0; c < 3; c++) {
      const v = colors[i * 3 + c] * scale;          // 밝기 보정(색상 유지)
      const boosted = target + (v - target) * COLOR_SATURATION_GAIN;  // 채도만 확대
      colors[i * 3 + c] = Math.min(1, Math.max(0, boosted));
    }
  }
}

let pointCloud = null;

let mapVoxelMm = 3.0;
function updatePointCloud(points, voxelMm) {
  if (voxelMm) mapVoxelMm = voxelMm;
  if (pointCloud) {
    scene.remove(pointCloud);
    pointCloud.geometry.dispose();
    pointCloud.material.dispose();
  }
  const geometry = new THREE.BufferGeometry();
  const positions = new Float32Array(points.length * 3);
  const colors = new Float32Array(points.length * 3);
  for (let i = 0; i < points.length; i++) {
    const [x, y, z, r, g, b] = points[i];
    positions[i * 3] = x;
    positions[i * 3 + 1] = y;
    positions[i * 3 + 2] = z;
    colors[i * 3] = r / 255;
    colors[i * 3 + 1] = g / 255;
    colors[i * 3 + 2] = b / 255;
  }
  enhanceColors(colors);
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  // 점 크기는 반드시 **점 간격(=voxel 크기) 이상**이어야 면으로 보인다.
  // 컬러 뷰어(실사)는 간격 1.3mm에 크기 3mm(2.3배)였고, 그 비율을 그대로 쓴다.
  // 작으면 점 사이가 검게 비어 "지도가 아니라 점점"으로 보인다(2026-07-25 실측:
  // 크기/간격 0.53배에서 영역의 66%가 검정).
  const size = Math.max(1.5, (mapVoxelMm || 3.0) * 2.3);
  const material = new THREE.PointsMaterial({ size, vertexColors: true, sizeAttenuation: true });
  pointCloud = new THREE.Points(geometry, material);
  scene.add(pointCloud);
  els.pointCount.textContent = points.length;
  fitViewToPoints(points);
}

// ---- 회수 접근 경로: 지나온 길 = 실선, 갈 길 = 점선 (둘 다 빨강) ----
// reached = 도달 완료한 웨이포인트 개수. 두 선이 끊겨 보이지 않도록 경계 지점을
// 양쪽이 공유한다(실선은 0..reached, 점선은 reached-1..끝).
let pathTraveled = null;
let pathRemaining = null;

function disposeLine(line) {
  if (!line) return;
  scene.remove(line);
  // ⚠️ 2026-07-28부터 경로는 THREE.Line이 아니라 원기둥 묶음(Group)이다 —
  //    Group에는 geometry/material이 없으므로 자식을 순회해 해제해야 한다.
  //    (예전처럼 line.geometry.dispose()를 부르면 여기서 TypeError로 죽는다)
  line.traverse((o) => {
    if (o.geometry) o.geometry.dispose();
    if (o.material) o.material.dispose();
  });
}

// ⚠️ LineBasicMaterial/LineDashedMaterial의 linewidth는 대부분의 브라우저·GPU에서
//    **무시된다**(WebGL 제약, 항상 1px). 그래서 경로가 실오라기처럼 얇게만 나왔다.
//    실제 굵기를 내려면 지오메트리로 그려야 하므로 구간마다 원기둥을 세운다.
//    (2026-07-28, 사용자 지적 — 초고밀도 뷰어와 같은 방식으로 통일)
const PATH_TUBE_R = 6.0;   // mm
const PATH_DASH_MM = 26;
const PATH_GAP_MM = 18;
const COLOR_TRAVELED = 0x00e5ff;  // 지나온 길 = 파랑
const COLOR_REMAINING = 0xff3333; // 갈 길 = 빨강 점선

function tubeSegment(a, b, radius, material) {
  const va = new THREE.Vector3(a[0], a[1], a[2]);
  const vb = new THREE.Vector3(b[0], b[1], b[2]);
  const dir = new THREE.Vector3().subVectors(vb, va);
  const len = dir.length();
  if (len < 1e-6) return null;
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, len, 10), material);
  mesh.position.copy(va).addScaledVector(dir, 0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
  mesh.renderOrder = 998;
  return mesh;
}

function makeLine(points, dashed) {
  if (points.length < 2) return null;
  const material = new THREE.MeshBasicMaterial({
    color: dashed ? COLOR_REMAINING : COLOR_TRAVELED,
  });
  material.depthTest = false;   // 점군에 파묻히지 않게
  const group = new THREE.Group();
  if (!dashed) {
    for (let i = 0; i < points.length - 1; i++) {
      const t = tubeSegment(points[i], points[i + 1], PATH_TUBE_R, material);
      if (t) group.add(t);
    }
    scene.add(group);
    return group;
  }
  // 점선: 폴리라인을 호길이로 훑으며 DASH만큼 칠하고 GAP만큼 건너뛴다.
  let carry = 0;
  let drawing = true;
  for (let i = 0; i < points.length - 1; i++) {
    const a = new THREE.Vector3(points[i][0], points[i][1], points[i][2]);
    const b = new THREE.Vector3(points[i + 1][0], points[i + 1][1], points[i + 1][2]);
    const seg = new THREE.Vector3().subVectors(b, a);
    const len = seg.length();
    if (len < 1e-6) continue;
    let s = 0;
    while (s < len) {
      const want = (drawing ? PATH_DASH_MM : PATH_GAP_MM) - carry;
      const take = Math.min(want, len - s);
      if (drawing) {
        const p0 = a.clone().addScaledVector(seg, s / len);
        const p1 = a.clone().addScaledVector(seg, (s + take) / len);
        const t = tubeSegment([p0.x, p0.y, p0.z], [p1.x, p1.y, p1.z],
                              PATH_TUBE_R * 0.85, material);
        if (t) group.add(t);
      }
      s += take;
      if (take >= want - 1e-9) { drawing = !drawing; carry = 0; } else { carry += take; }
    }
  }
  scene.add(group);
  return group;
}

// ---- 현재 이동점 = 네비게이션형 화살표(축 + 원뿔). 진행 방향을 향한다 ----
// 초고밀도 뷰어와 같은 모양·같은 규칙(2026-07-28, 사용자 요청).
// ⚠️ 위치를 로봇 TCP가 아니라 **경로의 도달 지점**에서 잡는다 — dashboard의 tcp_pose는
//    joint_states+FK 계산값이라 실측 posx와 z축이 어긋난 이력이 있어(07-24) 맵에서 뜬다.
//    경로 점은 posx 그대로라 맵과 프레임이 정확히 일치한다.
let navArrow = null;

function buildNavArrow() {
  const group = new THREE.Group();
  const mk = (color) => {
    const m = new THREE.MeshBasicMaterial({ color });
    m.depthTest = false;
    return m;
  };
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(5, 5, 46, 14), mk(COLOR_TRAVELED));
  shaft.position.y = 23;
  const head = new THREE.Mesh(new THREE.ConeGeometry(15, 40, 20), mk(COLOR_TRAVELED));
  head.position.y = 66;
  const base = new THREE.Mesh(new THREE.SphereGeometry(9, 16, 12), mk(0xffffff));
  group.add(shaft, head, base);
  group.children.forEach((o) => { o.renderOrder = 999; });
  group.visible = false;
  scene.add(group);
  return group;
}

function updateNavArrow(position, direction) {
  if (!navArrow) navArrow = buildNavArrow();
  navArrow.position.set(position[0], position[1], position[2]);
  const d = new THREE.Vector3(direction[0], direction[1], direction[2]);
  if (d.lengthSq() > 1e-9) {
    // 원뿔/원기둥의 기본 축은 +Y라 그걸 진행 방향으로 돌린다.
    navArrow.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  }
  navArrow.visible = true;
}

function updateApproachPath(points, reached) {
  disposeLine(pathTraveled);
  disposeLine(pathRemaining);
  pathTraveled = null;
  pathRemaining = null;
  if (!points || points.length === 0) {
    if (navArrow) navArrow.visible = false;
    if (els.pathInfo) els.pathInfo.textContent = "-";
    return;
  }

  const n = Math.max(0, Math.min(reached | 0, points.length));
  pathTraveled = makeLine(points.slice(0, n), false);
  pathRemaining = makeLine(points.slice(Math.max(0, n - 1)), true);

  const i = Math.min(Math.max(n - 1, 0), points.length - 1);
  const cur = points[i];
  const nxt = points[Math.min(i + 1, points.length - 1)];
  updateNavArrow(cur, [nxt[0] - cur[0], nxt[1] - cur[1], nxt[2] - cur[2]]);

  if (els.pathInfo) {
    els.pathInfo.textContent = `${n} / ${points.length}`;
  }
}

function animate() {
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}
animate();

new ResizeObserver(() => {
  const w = viewerEl.clientWidth, h = viewerEl.clientHeight;
  if (w === 0 || h === 0) return;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}).observe(viewerEl);

// 간단한 드래그 회전 (OrbitControls 없이 최소 구현)
// ⚠️ 2026-07-25: 궤도 중심이 원점(0,0,0)=로봇 베이스에 고정돼 있어서, 박스 안 맵
// (y≈-1650~-500)이 화면 구석으로 밀려 "지도로 안 보이는" 문제가 있었다. 맵을 받으면
// 그 중심(centroid)으로 target을 옮겨 맵 중심으로 열리게 한다.
let dragging = false, lastX = 0, lastY = 0, azimuth = -Math.PI / 4, elevation = Math.PI / 6, radius = 800;
const orbitTarget = new THREE.Vector3(0, 0, 0);
function updateCameraFromOrbit() {
  camera.position.set(
    orbitTarget.x + radius * Math.cos(elevation) * Math.cos(azimuth),
    orbitTarget.y + radius * Math.cos(elevation) * Math.sin(azimuth),
    orbitTarget.z + radius * Math.sin(elevation)
  );
  camera.lookAt(orbitTarget);
}

// 맵을 처음 받았을 때 한 번만 시점을 맞춘다(이후 사용자가 돌려둔 각도를 뺏지 않도록).
let viewFitted = false;
function fitViewToPoints(points) {
  if (viewFitted || points.length === 0) return;
  // ⚠️ min/max로 크기를 재면 이상점 하나가 시야를 통째로 망친다(실측: 남은 원거리
  // 노이즈 때문에 바운딩 대각이 9.5m로 잡혀 카메라가 8.6m까지 물러났다).
  // 축별 2~98 백분위로 재서 이상점에 둔감하게 만든다.
  const pct = (arr, q) => {
    const s = arr.slice().sort((a, b) => a - b);
    return s[Math.min(s.length - 1, Math.max(0, Math.floor(q * (s.length - 1))))];
  };
  const xs = points.map((p) => p[0]), ys = points.map((p) => p[1]), zs = points.map((p) => p[2]);
  const spanX = pct(xs, 0.98) - pct(xs, 0.02);
  const spanY = pct(ys, 0.98) - pct(ys, 0.02);
  const spanZ = pct(zs, 0.98) - pct(zs, 0.02);
  orbitTarget.set((pct(xs, 0.98) + pct(xs, 0.02)) / 2,
                  (pct(ys, 0.98) + pct(ys, 0.02)) / 2,
                  (pct(zs, 0.98) + pct(zs, 0.02)) / 2);
  // 대각 크기의 약 0.9배 거리면 맵 전체가 화면에 들어온다(FOV 60도 기준 경험값).
  const diag = Math.hypot(spanX, spanY, spanZ);
  radius = Math.max(300, diag * 0.9);
  viewFitted = true;
  updateCameraFromOrbit();
}
updateCameraFromOrbit();
viewerEl.addEventListener("mousedown", (e) => { dragging = true; lastX = e.clientX; lastY = e.clientY; });
window.addEventListener("mouseup", () => { dragging = false; });
window.addEventListener("mousemove", (e) => {
  if (!dragging) return;
  azimuth -= (e.clientX - lastX) * 0.005;
  elevation = Math.max(-1.4, Math.min(1.4, elevation + (e.clientY - lastY) * 0.005));
  lastX = e.clientX; lastY = e.clientY;
  updateCameraFromOrbit();
});
viewerEl.addEventListener("wheel", (e) => {
  radius = Math.max(100, Math.min(5000, radius + e.deltaY * 0.5));
  updateCameraFromOrbit();
  e.preventDefault();
}, { passive: false });

// ---------------- WebSocket ----------------

let socket = null;

function connectSocket() {
  const url = `ws://${window.__WS_HOST}:${window.__WS_PORT}`;
  socket = new WebSocket(url);

  socket.addEventListener("open", () => {
    els.connBadge.textContent = "연결됨";
    els.connBadge.className = "badge ok";
    appendLog("WebSocket 연결됨");
  });

  socket.addEventListener("close", () => {
    els.connBadge.textContent = "연결 끊김 — 재시도 중";
    els.connBadge.className = "badge danger";
    setTimeout(connectSocket, 2000);
  });

  socket.addEventListener("message", (event) => {
    const data = JSON.parse(event.data);
    if (data.type === "dashboard") {
      els.statState.textContent = data.robot_state ?? "-";
      els.statMode.textContent = data.robot_mode ?? "-";
      els.statTcp.textContent = (data.tcp_pose || []).slice(0, 3).map((v) => v.toFixed(1)).join(", ");
      els.statJoints.textContent = (data.joints_deg || []).map((v) => v.toFixed(1)).join(", ");
    } else if (data.type === "manager_log") {
      appendLog(data.text);
    } else if (data.type === "map") {
      updatePointCloud(data.points, data.voxel_mm);
    } else if (data.type === "approach_path") {
      updateApproachPath(data.points, data.reached);
    }
  });
}
connectSocket();

function sendCommand(cmd) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    appendLog(`전송 실패(연결 안 됨): ${cmd}`);
    return;
  }
  socket.send(JSON.stringify({ manager_cmd: cmd }));
  appendLog(`명령 전송: ${cmd}`);
}

// 정지/재개/비상정지/안전상태해제 — confirm() 없이 클릭 즉시 실행
// (rokey_bootbot MANAGER와 동일한 사용자 판단: 네 버튼은 빠른 대응이 우선)
els.btnPause.addEventListener("click", () => sendCommand("pause"));
els.btnResume.addEventListener("click", () => sendCommand("resume"));
els.btnEstop.addEventListener("click", () => sendCommand("emergency_stop"));
els.btnSafetyReset.addEventListener("click", () => sendCommand("release_safety"));
