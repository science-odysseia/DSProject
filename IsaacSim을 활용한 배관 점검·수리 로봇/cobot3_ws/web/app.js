// pipe_comm 관제 패널 — 셸: 라우터 + 웹소켓 + 공유 상태 + 지령.
//
// ━━ 구조 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//   app.js         이 파일 — 사이드바/라우팅, WS 수신, store, cmd()
//   views/*.js     페이지 하나당 모듈. `mount(el)` 을 export 하고,
//                  정리할 게 있으면 unmount 함수를 돌려준다.
//
// 🔑 **웹소켓은 하나이고 페이지 전환에도 끊기지 않는다.** 라우팅을 서버가
//    아니라 브라우저(History API)가 하는 이유가 이것이다 — 전환마다 재접속하면
//    카메라가 매번 끊기고 지난 결함·사건을 다시 받아야 한다. 서버는 어느
//    경로로 들어와도 같은 셸을 준다(web_panel.py 의 경로 목록과 맞출 것).
//
// 🔑 화면에 없는 페이지도 **상태는 계속 쌓인다** — store 가 받아 두고,
//    페이지는 mount 될 때 store 를 통째로 읽어 그린 뒤 bus 를 구독한다.
//
// ━━ 로봇 여러 대 (층별 동시 주행, 2026-08-10) ━━━━━━━━━━━━━━━━━━━━
//
// 🔑 **상태는 로봇마다 통째로 따로다.** `store.robots` 가 로봇 객체의 배열이고
//    (순서 = 화면 왼→오른쪽 = 서버 `-p ns:=floor1,floor2` 의 순서), 상태·코스·
//    사건·결함·마커·카메라가 전부 그 안에 들어 있다. 1층과 2층은 연결되어
//    있지 않은 별개의 임무라 합쳐 놓을 이유가 하나도 없다 — 합계를 보여 주고
//    싶은 자리에서만 두 대를 더한다.
//
// 🔑 명패(`hello`)가 로봇 목록의 단일 출처다. 서버가 접속하자마자 보내 주므로
//    **시연이 아직 안 떠서 데이터가 하나도 없어도** 칸은 제대로 선다.
//
// 🚨 층이 다르면 시연이 말하는 월드 좌표의 **원점(z)도 다르다**(floor2 +250 /
//    floor1 +2740.2mm). 명패의 `z_shift_mm`/`z_ref_mm` 으로 3D 맵이 되민다 —
//    `dzM(r)` 하나만 쓰면 되고, 좌표를 직접 만지는 다른 화면은 없다.

// 🔑 **지금은 3D Map 한 화면뿐이다** (2026-08-10 사용자 지시). 그 화면에
//    상태·결함·지령·카메라가 다 들어 있어서 나머지 페이지가 겹쳤다.
//    `views/home.js` · `handling.js` · `detect.js` · `camera.js` · `events.js`
//    는 **지우지 않았다** — 앞의 셋은 아래 ROUTES 에 한 줄 되돌리면 그대로
//    살아나고, 뒤의 둘은 3D Map 이 부속(`mountCam`/`mountButtons`)으로 쓴다.
import * as map3d from '/static/views/map3d.js';

// ── 아이콘 (인라인 SVG — 외부 폰트/CDN 없이) ──────────────────
const ico = {
  home: '<path d="M3 9.5 10 3l7 6.5"/><path d="M5 8.5V17h10V8.5"/>',
  cam: '<rect x="2" y="5.5" width="13" height="9.5" rx="2"/>'
     + '<path d="M15 9.5 18.5 7v6.5L15 11"/>',
  robot: '<rect x="4" y="7" width="12" height="9" rx="2"/>'
       + '<path d="M10 7V4"/><circle cx="7.5" cy="11" r="1"/>'
       + '<circle cx="12.5" cy="11" r="1"/>',
  map: '<path d="M2 5.5 7.3 3l5.4 2.5L18 3v11.5L12.7 17 7.3 14.5 2 17z"/>'
     + '<path d="M7.3 3v11.5M12.7 5.5V17"/>',
  list: '<path d="M7 5.5h11M7 10h11M7 14.5h11"/><circle cx="3.4" cy="5.5" r="1"/>'
      + '<circle cx="3.4" cy="10" r="1"/><circle cx="3.4" cy="14.5" r="1"/>',
  log: '<path d="M4.5 3h7l4 4v10a1 1 0 0 1-1 1h-10a1 1 0 0 1-1-1V4a1 1 0 0 1 1-1z"/>'
     + '<path d="M11 3v4.5h4.5M7 11h6M7 14h4"/>',
};

// 🔑 경로 목록의 단일 출처. 서버(web_panel.py)의 라우트 목록과 **같아야**
//    직접 주소를 치거나 새로고침해도 404 가 안 난다.
// 🚨 한 줄뿐이라 사이드바를 안 그린다(index.html 에도 없다). 페이지를 되살릴
//    때는 여기에 줄을 넣고 index.html 에 `<nav id="side">…<ul id="nav">` 를
//    돌려놓으면 된다 — app.js 는 `#nav` 가 있을 때만 사이드바를 채운다.
export const ROUTES = [
  {path: '/map', label: 'Pipe Repair Robot', title: 'Pipe Repair Robot',
   icon: ico.map, view: map3d},
];

// ── 아주 작은 이벤트 버스 ────────────────────────────────────
// 종류(로봇 단위 — 첫 인자가 그 로봇):
//   'state' 'course' 'event' 'defect' 'cmd' 'frame'(role) 'reset'(run)
// 종류(전체):  'roster'(로봇 명패가 바뀜) 'conn'(연결 여부)
function makeBus() {
  const m = new Map();
  return {
    on(evt, fn) {
      if (!m.has(evt)) m.set(evt, new Set());
      m.get(evt).add(fn);
      return () => m.get(evt).delete(fn);      // 해제 함수를 돌려준다
    },
    emit(evt, ...args) {
      const s = m.get(evt);
      if (s) for (const fn of [...s]) {
        try { fn(...args); } catch (e) { console.error(evt, e); }
      }
    },
  };
}
export const bus = makeBus();

// ── 공유 상태 ────────────────────────────────────────────────
// 🚨 카메라는 **Blob 으로만** 들고 있는다. objectURL 을 여기서 만들면 화면에
//    없는 페이지 몫까지 10Hz 로 URL 이 쌓여 새는데, Blob 은 다음 프레임이
//    덮어쓰면 그대로 회수된다. URL 은 보는 쪽이 showFrame() 으로 만든다.
export const store = {
  robots: [],             // Robot[] — 화면 왼→오른쪽 순서
  byNs: new Map(),        // ns → Robot
  byIdx: new Map(),       // 웹소켓 바이너리 머리의 번호 → Robot
  meshRobot: null,        // /mesh 가 누구의 프레임인가 (null = 서버 로컬 파일)
  zRefMm: null,           // 그 프레임의 z 오프셋(mm)
  cmds: [],               // 이 패널이 보낸 지령 이력 (서버가 기록해 흘려 준다)
  connected: false,
};

/** 로봇 하나의 상태 그릇. 서버 명패(`hello`)의 항목 하나에 대응한다. */
function makeRobot(spec) {
  return {
    ns: spec.ns, label: spec.label || spec.ns, idx: spec.idx || 1,
    zShiftMm: (spec.z_shift_mm === undefined) ? null : spec.z_shift_mm,
    stale: !!spec.stale,    // 서버가 "이 로봇 상태가 끊겼다" 고 본 상태
    state: null,            // drive_state (+roll/odom/max_s 보강)
    course: null,           // {pts:[[s,x,y,z]], ir_m, s_total_m}
    events: [],             // 사건 (오래된 것부터)
    defectRows: new Map(),  // defect_id → 리포트 dict
    marks: [],              // 3D 맵 마커 {id, s, clock, repaired}
    maxS: 0,                // 지나간 최대 호길이 (m)
    // 진행시간 — **규약에 없는 값이라 여기서 만든다.** drive_state 에는 그
    // 순간의 `stamp` 만 있고 "언제 시작했나" 가 없다.
    // 🔑 기준 시계는 **로봇이 보내온 stamp** 다(브라우저 시계가 아니라). 두
    //    시계가 어긋나도, 브라우저를 늦게 열어도 같은 값이 나온다.
    t0: null,               // 이번 판이 실제로 뜬 시각(stamp, s). null = 아직
    elapsedS: 0,            // 진행시간(s). DONE 에서 멈춘다
    run: null,              // 시연 판 번호 (서버가 센다). 바뀌면 = 재시작
    cam: {front: null, torch: null},   // {blob, t, fps} (rear 는 v1_3 에서 폐지)
  };
}

/** 이 로봇의 좌표를 **기준 프레임**으로 옮기는 z 보정(m).
 *
 * 🚨 시연은 자기 층의 수평망을 월드 z=0 으로 올려놓고 좌표를 낸다. 두 층을
 *    한 화면에 겹치려면 그 차이를 되밀어야 한다 — 안 하면 1층 로봇이 2층
 *    배관 안을 달리는 그림이 된다(2.49m 차이라 눈에 안 띄지도 않는다).
 *    한쪽이라도 z 를 모르면 0 을 준다(= 겹치지 않고 그대로 그린다). */
export function dzM(r) {
  if (store.zRefMm === null || r.zShiftMm === null
      || r.zShiftMm === undefined) return 0;
  return (store.zRefMm - r.zShiftMm) / 1000;
}

export const defectCount = () =>
  store.robots.reduce((n, r) => n + r.defectRows.size, 0);

/** 아직 명패에 없는 ns 가 오면 그 자리에서 칸을 만든다(서버가 늦게 알려 준
 *  경우의 안전망). 돌려주는 것은 언제나 Robot 이다. */
function robotOf(ns) {
  if (!ns) return store.robots[0] || addRobot({ns: '?', label: '?', idx: 1});
  return store.byNs.get(ns) || addRobot(
    {ns, label: ns, idx: store.robots.length + 1});
}

function addRobot(spec) {
  const r = makeRobot(spec);
  store.robots.push(r);
  store.byNs.set(r.ns, r);
  store.byIdx.set(r.idx, r);
  return r;
}

/** 서버 명패를 반영한다. 로봇 **목록 자체**가 바뀌면 true (페이지를 다시
 *  그려야 한다 — 칸 수가 달라진다). z 오프셋만 바뀐 것은 false. */
function setRoster(d) {
  const list = d.robots || [];
  const same = list.length === store.robots.length
    && list.every((s, i) => store.robots[i].ns === s.ns);
  store.meshRobot = d.mesh_robot || null;
  store.zRefMm = (d.z_ref_mm === undefined || d.z_ref_mm === null)
    ? null : d.z_ref_mm;
  if (!same) {
    // 🔑 이미 받아 둔 상태는 ns 로 이어 붙인다 — 명패가 늦게 갱신됐다고
    //    지난 사건·결함을 버리면 화면이 순간 비어 보인다.
    const old = store.byNs;
    store.robots = []; store.byNs = new Map(); store.byIdx = new Map();
    for (const s of list) {
      const prev = old.get(s.ns);
      const r = prev || makeRobot(s);
      r.label = s.label || r.label;
      r.idx = s.idx || r.idx;
      store.robots.push(r);
      store.byNs.set(r.ns, r);
      store.byIdx.set(r.idx, r);
    }
  }
  for (const s of list) {
    const r = store.byNs.get(s.ns);
    if (!r) continue;
    r.zShiftMm = (s.z_shift_mm === undefined) ? null : s.z_shift_mm;
    // 🚨 **끊김으로 넘어가는 순간 그 로봇 화면을 비운다.** 시연을 끄면 아무
    //    신호도 안 오므로, 서버가 침묵을 이 깃발로 바꿔 준다. 안 비우면
    //    새로고침해도 지난 판의 초록선·빨간 점·결함이 살아 있는 것처럼 남는다.
    if (s.stale && !r.stale) resetRobot(r, '끊김');
    r.stale = !!s.stale;
    if (s.run !== undefined) r.run = s.run;
  }
  return !same;
}

/** 그 로봇의 **이번 판** 자취를 지운다. 사건 로그는 남긴다 — 로그는 지난
 *  판까지 포함해 읽는 기록이다(서버도 같은 규칙으로 지운다). */
function resetRobot(r, why) {
  r.state = null;
  r.maxS = 0;
  r.t0 = null; r.elapsedS = 0;
  r.marks.length = 0;
  r.defectRows.clear();
  for (const k of Object.keys(r.cam)) r.cam[k] = null;
  console.log('[reset]', r.ns, why);
  bus.emit('reset', r, r.run);
  bus.emit('state', r, null);      // 화면이 "대기" 로 돌아가게 한 번 알린다
}

/** 지금 켜진 역할만 남기고 나머지 카메라의 Blob 을 놓아 준다.
 *
 * 🔑 시연이 한 번에 한 대만 발행하므로, 안 켜진 역할의 마지막 프레임은 아무도
 *    안 보면서 메모리만 붙들고 있다. 역할이 바뀌는 순간 버린다. */
export function dropOtherCams(r, active) {
  if (!r || !active) return;
  for (const k of Object.keys(r.cam))
    if (k !== active) r.cam[k] = null;
}

/** <img> 하나에 그 로봇의 최신 프레임을 걸어 준다. 이전 URL 은 회수한다. */
export function showFrame(img, r, role) {
  const c = r && r.cam[role];
  if (!c || !c.blob) return false;
  const url = URL.createObjectURL(c.blob);
  const old = img.dataset.url;
  // 🔑 회수를 못 한 URL 을 `prev` 에 적어 둔다 — 로드가 끝나기 전에 이 <img>
  //    를 비우면(카메라 분기 전환) onload 가 안 불려서 그 하나가 샌다.
  if (old) img.dataset.prev = old;
  img.onload = () => {
    if (old) URL.revokeObjectURL(old);
    if (img.dataset.prev === old) delete img.dataset.prev;
  };
  img.dataset.url = url;
  img.src = url;
  return true;
}

// ── 웹소켓 ───────────────────────────────────────────────────
// web_panel.py 의 CAM_CH 와 맞춘다. 채널 2(rear)는 v1_3 에서 폐지 — 번호는
// 재사용하지 않는다(들어와도 role 이 undefined 라 onFrame 이 조용히 버린다).
const CH = {1: 'front', 3: 'torch'};
const connEl = document.getElementById('conn');
function setConn(ok, msg) {
  store.connected = ok;
  connEl.className = 'conn ' + (ok ? 'on' : 'off');
  connEl.textContent = msg;
  bus.emit('conn', ok);
}

function onState(r, d) {
  // 🔑 시연 재시작(Isaac GUI 의 Stop→Play) — 서버가 run 번호를 올려서 알린다.
  //    지나온 초록선과 수리 마커는 **그 판에서만 참인 것**이라 같이 지운다.
  //    사건 로그는 남긴다 — 로그는 지난 판까지 포함해서 읽는 것이다.
  //    🚨 판은 **로봇마다 따로** 센다. 2층을 다시 돌린다고 1층 자취를 지우면
  //       안 된다 — 둘은 서로 아무 상관이 없는 임무다.
  if (d.run !== undefined && r.run !== null && d.run !== r.run) {
    r.maxS = 0;
    r.marks.length = 0;
    r.t0 = null; r.elapsedS = 0;
    bus.emit('reset', r, d.run);
  }
  if (d.run !== undefined) r.run = d.run;
  // ── 진행시간 ────────────────────────────────────────────────
  // 🔑 **안착(SETTLE)은 안 센다.** 피스톤이 관벽을 잡는 과도 구간은 임무가
  //    아니라 준비다 — 여기까지 세면 "얼마나 돌았나" 가 판마다 들쭉날쭉해진다.
  // 🔑 DONE 에서 **멈춘다.** 안 멈추면 끝난 임무의 시간이 계속 올라가서,
  //    화면만 보고는 아직 돌고 있는 줄 안다.
  // 🚨 멈추는 시점은 **DONE 을 처음 받은 그 값까지**다. `d.state !== 'DONE'`
  //    로 거르면 마지막 주행 표본에서 얼어붙어 완료 순간이 통째로 빠진다.
  //    여기서 `r.state` 는 아직 **직전** 상태라(대입은 아래에서 한다) 그것만
  //    보면 새 필드 없이 "이번 것까지 반영하고 다음부터 멈춤" 이 된다.
  if (typeof d.stamp === 'number') {
    if (r.t0 === null && d.state && d.state !== 'SETTLE') r.t0 = d.stamp;
    if (r.t0 !== null && (r.state || {}).state !== 'DONE')
      r.elapsedS = Math.max(0, d.stamp - r.t0);
  }
  // 카메라 분기가 바뀌면 지난 역할의 프레임은 바로 놓는다 — 화면(카메라 칸)이
  // 떠 있지 않아도 store 는 계속 받으므로, 여기서 버려야 실제로 안 쌓인다.
  if (d.cam && d.cam !== (r.state || {}).cam) dropOtherCams(r, d.cam);
  r.state = d;
  r.maxS = Math.max(r.maxS, (d.max_s_mm || d.s_mm || 0) / 1000);
  bus.emit('state', r, d);
}

function onEvent(r, d) {
  // 용접 사건은 3D 맵 마커로도 남는다 — 맵 페이지가 떠 있지 않아도 쌓아 둔다.
  if (d.event === 'WELD_BEGIN' && typeof d.defect_s_mm === 'number') {
    const s = d.defect_s_mm / 1000;
    if (!r.marks.some(m => Math.abs(m.s - s) < 0.02))
      r.marks.push({id: 'defect', s,
        clock: (typeof d.clock_deg === 'number') ? d.clock_deg : null});
  }
  if (d.event === 'WELD_DONE') {
    // defect_s_mm/clock_deg(결함 자체의 좌표)가 있으면 그대로 쓰고, 없으면
    // (구판 브리지) 로봇이 물러난 시점의 s 로 근사한다 — 가장 가까운 미수리
    // 결함(±100mm)에 붙이고 없으면 새로 찍는다.
    const s = (typeof d.defect_s_mm === 'number' ? d.defect_s_mm
                                                 : d.s_mm || 0) / 1000;
    const ck = (typeof d.clock_deg === 'number') ? d.clock_deg : null;
    let best = null;
    for (const m of r.marks)
      if (!m.repaired && Math.abs(m.s - s) < 0.1
          && (best === null || Math.abs(m.s - s) < Math.abs(best.s - s)))
        best = m;
    if (best) { best.repaired = true; if (ck !== null) best.clock = ck; }
    else r.marks.push({id: 'weld', s, clock: ck, repaired: true});
  }
  r.events.push(d);
  bus.emit('event', r, d);
}

function onDefect(r, d) {
  const id = d.defect_id || '?';
  r.defectRows.set(id, d);
  const s = (d.defect || {}).axial_position_from_entry_m;
  if (typeof s === 'number') {
    const ck = (typeof d.defect.clock_angle_deg === 'number')
      ? d.defect.clock_angle_deg : null;
    const m = r.marks.find(x => x.id === id);
    if (m) { m.s = s; if (ck !== null) m.clock = ck; }
    else r.marks.push({id, s, clock: ck});
  }
  bus.emit('defect', r, d);
}

function connect() {
  const ws = new WebSocket(
    (location.protocol === 'https:' ? 'wss://' : 'ws://')
    + location.host + '/ws');
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => setConn(true, '● 서버 연결됨');
  ws.onmessage = (e) => {
    if (typeof e.data === 'string') {
      const m = JSON.parse(e.data);
      if (m.type === 'hello') {
        // 로봇 목록이 바뀌면 지금 페이지를 다시 그린다 — 칸 수가 달라진다.
        const changed = setRoster(m.data || {});
        bus.emit('roster');
        if (changed) render(location.pathname);
        refreshBadges();
        return;
      }
      const r = robotOf(m.robot);
      if (m.type === 'state') onState(r, m.data);
      else if (m.type === 'event') onEvent(r, m.data);
      else if (m.type === 'defect') onDefect(r, m.data);
      else if (m.type === 'cmd') {
        // 서버가 기록해 흘려 주는 지령 이력(다른 브라우저·새로고침 전 것 포함).
        // 🚨 지금은 그리는 화면이 없다(Robot Handling 을 비웠다) — 그래도
        //    받아는 두되 **상한을 건다.** 아무도 안 보는 배열이 무한히 자라는
        //    것이 제일 조용한 누수다.
        store.cmds.push(m.data);
        if (store.cmds.length > 200) store.cmds.splice(0, 100);
        bus.emit('cmd', r, m.data);
      }
      else if (m.type === 'course') {
        r.course = m.data;
        if (typeof m.data.z_shift_mm === 'number')
          r.zShiftMm = m.data.z_shift_mm;
        bus.emit('course', r, m.data);
      }
      return;
    }
    // 바이너리 = 카메라 프레임. 머리 2 바이트가 [로봇 번호, 채널] 이다.
    const v = new Uint8Array(e.data);
    const r = store.byIdx.get(v[0]);
    const role = CH[v[1]];
    if (!r || !role) return;
    const prev = r.cam[role];
    const t = performance.now();
    r.cam[role] = {
      blob: new Blob([v.subarray(2)], {type: 'image/jpeg'}), t,
      // 프레임 간격의 지수이동평균 → 초당 장수. 튀는 값을 눌러 준다.
      fps: prev ? (prev.fps || 0) * 0.8 + 0.2 * (1000 / Math.max(t - prev.t, 1))
                : 0,
    };
    bus.emit('frame', r, role);
  };
  ws.onclose = () => {
    setConn(false, '○ 연결 끊김 — 3초 후 재접속');
    setTimeout(connect, 3000);
  };
  ws.onerror = () => ws.close();
}

// ── 지령 ─────────────────────────────────────────────────────
/** 지령을 보낸다. `robot` 은 ns 또는 'all' — 빼면 서버가 첫 로봇으로 친다.
 *  🚨 로봇이 두 대인데 대상을 안 적으면 **1층에만** 간다. 화면은 언제나
 *     명시적으로 적는다(조종석 버튼은 자기 칸의 ns 를 싣는다). */
export async function cmd(c, extra) {
  const r = await fetch('/cmd', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(Object.assign({cmd: c}, extra || {})),
  });
  return r.json();
}

// ── 로봇별 칸 만들기 (Home · Robot Handling · Detect List 공용) ─
/** el 안에 로봇 수만큼 칸을 만들고 [{r, el}] 을 돌려준다.
 *
 * 🔑 **왼쪽이 1층, 오른쪽이 2층이다** — 순서는 서버 명패(= `-p ns:=` 목록)가
 *    정하고 여기서는 뒤집지 않는다. 세 대 이상이어도 그대로 늘어선다.
 * 🔑 세 화면이 같은 헬퍼를 쓰는 이유는 칸 제목·순서가 갈리지 않게 하기
 *    위해서다. 한 대뿐이면 칸도 하나 — 예전 화면과 같아 보인다. */
export function robotCols(el, opts = {}) {
  const wrap = document.createElement('div');
  wrap.className = 'rcols' + (opts.cls ? ' ' + opts.cls : '');
  wrap.style.setProperty('--n', Math.max(store.robots.length, 1));
  el.appendChild(wrap);
  // 명패가 아직 안 왔다(접속 직후 한순간). 곧 `hello` 가 오면 페이지를 다시
  // 그리므로 여기서는 빈 말만 걸어 둔다 — 칸을 0 개로 두면 화면이 백지다.
  if (!store.robots.length) {
    wrap.innerHTML = '<div class="empty">로봇 명패 수신 대기 — 서버에 '
      + '연결되면 층별로 칸이 선다.</div>';
    return [];
  }
  return store.robots.map(r => {
    const col = document.createElement('div');
    col.className = 'rcol';
    if (opts.head !== false) {
      const h = document.createElement('div');
      h.className = 'rhead';
      h.innerHTML = `<span class="rname">${r.label}</span>`
        + `<span class="rns mono">/${r.ns}</span>`;
      col.appendChild(h);
    }
    const body = document.createElement('div');
    body.className = 'rbody';
    col.appendChild(body);
    wrap.appendChild(col);
    return {r, el: body, col};
  });
}

// ── 라우터 ───────────────────────────────────────────────────
const navEl = document.getElementById('nav');
const viewEl = document.getElementById('view');
const titleEl = document.getElementById('page-title');
let unmount = null;

// 사이드바는 **있을 때만** 채운다 — 한 화면짜리로 줄인 뒤로 index.html 에
// `#nav` 가 없다(위 ROUTES 주석 참고).
if (navEl) {
  for (const r of ROUTES) {
    const li = document.createElement('li');
    li.innerHTML = `<a href="${r.path}" data-path="${r.path}">`
      + `<svg viewBox="0 0 20 20">${r.icon}</svg><span>${r.label}</span>`
      + `<span class="badge" hidden></span></a>`;
    li.querySelector('a').onclick = (e) => {
      if (e.metaKey || e.ctrlKey || e.shiftKey) return;   // 새 탭은 그대로
      e.preventDefault();
      go(r.path);
    };
    navEl.appendChild(li);
  }
}

function refreshBadges() {
  if (!navEl) return;
  for (const r of ROUTES) {
    if (!r.badge) continue;
    const el = navEl.querySelector(`a[data-path="${r.path}"] .badge`);
    if (!el) continue;
    const n = r.badge();
    el.hidden = !n;
    el.textContent = n;
  }
}
bus.on('event', refreshBadges);
bus.on('defect', refreshBadges);

function render(path) {
  const r = ROUTES.find(x => x.path === path) || ROUTES[0];
  if (unmount) { try { unmount(); } catch (e) { console.error(e); } }
  unmount = null;
  viewEl.replaceChildren();
  titleEl.textContent = r.title;
  document.title = r.title;
  if (navEl) for (const a of navEl.querySelectorAll('a'))
    a.classList.toggle('on', a.dataset.path === r.path);
  unmount = r.view.mount(viewEl) || null;
}

export function go(path) {
  if (location.pathname !== path) history.pushState({}, '', path);
  render(path);
}
window.addEventListener('popstate', () => render(location.pathname));

// ── 머리말의 상태 한 줄 (모든 페이지 공통) ──────────────────
// 로봇이 여럿이면 **한 줄에 나란히** 적는다 — 어느 층이 서 있는지가 페이지를
// 안 옮기고도 보여야 한다.
const headEl = document.getElementById('head-state');
export function stateLine(d) {
  if (!d) return '상태 수신 대기…';
  const tot = d.s_total_mm || 0;
  const pct = tot ? (100 * d.s_mm / tot).toFixed(0) + '%' : '-';
  return `<span class="k">${d.moving ? '주행' : '정지'}</span> · `
    + `<span class="k">${d.state || '?'}</span> · `
    + `s <span class="k">${(d.s_mm || 0).toFixed(0)}mm</span> `
    + `${d.dir > 0 ? '→' : '←'} ${pct}`
    + (d.reason ? ` · ${d.reason}` : '');
}
function drawHead() {
  if (!store.robots.length) { headEl.innerHTML = '상태 수신 대기…'; return; }
  headEl.innerHTML = store.robots.map(r =>
    (store.robots.length > 1 ? `<b class="hl">${r.label}</b> ` : '')
    + (r.stale ? '<span class="k">시연 끊김</span>' : stateLine(r.state)))
    .join('<span class="hsep">|</span>');
}
bus.on('state', drawHead);
bus.on('roster', drawHead);

// ── 기동 ─────────────────────────────────────────────────────
drawHead();
setConn(false, '○ 연결 대기…');
connect();
// 어느 주소로 들어와도 이 한 화면이다 — 주소만 정본(`/map`)으로 맞춰 둔다.
// (서버는 옛 주소 `/home`·`/detect` 에도 같은 셸을 주므로 북마크가 안 깨진다.)
if (!ROUTES.some(r => r.path === location.pathname))
  history.replaceState({}, '', ROUTES[0].path);
render(location.pathname);
