// Robot Handling — 조종석. 지령 버튼 + 지금 켜진 카메라 + 상태 + 사건 로그.
//
// 🔑 이 화면 하나만 보고 시연을 몰 수 있어야 한다(사용자 요청으로 Camera·
//    Event Log 를 여기로 합쳤다). 그래서 카메라 칸과 로그 칸은 **각 페이지의
//    모듈을 그대로 끼워 쓴다** — 같은 것을 두 벌 만들면 한쪽만 고쳐진다.
//
// 🔑 로봇마다 **조종석을 통째로 하나씩** 세운다(왼쪽 1층 / 오른쪽 2층).
//    1층과 2층은 연결되어 있지 않고 따로 출발해 따로 끝나므로, 버튼도 상태도
//    섞으면 안 된다 — 버튼은 자기 칸의 ns 를 지령에 실어 보낸다.
// 🚨 그래서 **어느 칸의 버튼을 눌렀는지가 곧 대상**이다. 두 대를 같이 몰고
//    싶을 때만 맨 위의 "전체 지령" 을 쓴다(`robot: all`).
//
// 🚨 발행은 성공해도 **구독자가 0 이면 지령은 조용히 사라진다**(RELIABLE 이지
//    latch 가 아니다). 그래서 서버가 구독자 수를 세어 돌려주고, 여기서 그걸
//    그대로 띄운다 — "눌렀는데 아무 일도 없다" 를 진단으로 바꾼다.
import {store, bus, cmd, robotCols} from '/static/app.js';
import {mountCam} from '/static/views/camera.js';
import {mountLog} from '/static/views/events.js';

const CAM_WHEN = {front: '전방 — 주행', torch: '토치 — 결함·용접부'};

/** 지령 한 발. 결과를 toast 에 적는다(로봇별·전체 공용). */
async function send(toast, c, extra) {
  toast.textContent = '전송 중…';
  toast.style.color = '';
  try {
    const j = await cmd(c, extra);
    if (!j.ok) {
      toast.textContent = j.warn || j.error || '실패';
      toast.style.color = 'var(--warn)';
      return j;
    }
    // 여러 대에 보냈으면 대별 구독자 수를 그대로 적는다 — 한쪽만 안 붙어
    // 있는 상황이 제일 흔한데, 합계만 보면 그게 안 보인다.
    const det = (j.results && j.results.length > 1)
      ? j.results.map(x => `${x.label} ${x.subscribers}`).join(' · ')
      : `구독자 ${j.subscribers}`;
    toast.textContent = `✓ ${c} 전송 (${det})`;
    toast.style.color = 'var(--ok)';
    return j;
  } catch (e) {
    toast.textContent = '서버에 못 보냈다: ' + e;
    toast.style.color = 'var(--bad)';
    return null;
  }
}

/** 지령 버튼 한 벌(로봇 하나) + 결과 toast. 해제 함수를 돌려준다.
 *
 * 🔑 3D Map 페이지의 가운데 칸도 **이걸 그대로 끼워 쓴다** — 버튼을 두 벌
 *    만들면 한쪽만 고쳐져서 같은 지령이 화면마다 다르게 동작한다. */
export function mountButtons(el, r, opts = {}) {
  // opts.compact — 3D Map 의 좁은 가운데 칸용. **버튼과 지령은 그대로**이고
  // 글자만 줄인다(줄바꿈이 세 줄이 되면 아래 칸이 화면 밖으로 밀린다).
  const C = !!opts.compact;
  el.innerHTML = `
   <div class="btns">
    <button class="b-run go">▶ 시작</button>
    <button class="b-forward">${C ? '⏩ 전진' : '⏩ 전진 점검'}</button>
    <button class="b-recall">↩ 복귀</button>
    <button class="b-estop"></button>
   </div>`
   + (opts.toast ? '' : '<div class="toast"></div>');

  const $ = c => el.querySelector('.' + c);
  const toast = opts.toast || $('toast');   // 결과 줄은 바깥과 같이 쓸 수 있다
  const to = (c, extra) => send(toast, c, Object.assign({robot: r.ns},
                                                        extra || {}));

  // ── ▶ 시작 / ⏸ 정지 토글 ────────────────────────────────────
  // 🔑 버튼의 모양은 **로봇이 실제로 보내오는 상태**를 따른다(내가 뭘 눌렀는지가
  //    아니라). 시연이 지령을 미룰 수 있으므로 — 용접 중 정지는 시퀀스가 끝나야
  //    걸린다 — 누르자마자 뒤집으면 버튼이 거짓말을 한다.
  // 🚨 그래서 "보냈는데 아직 안 걸린" 구간(pend)을 따로 표시한다. 이게 없으면
  //    눌러도 글자가 안 변해서 안 먹은 줄 알고 계속 누르게 된다.
  const runBtn = $('b-run');
  let pend = null;                 // 'STOP' | 'START' — 상태가 따라오길 기다림

  function isHeld(d) {
    // HOLD = 지령 정지, DEAD = 비상정지. 둘 다 "지금 서 있다".
    return !d || d.state === 'HOLD' || d.state === 'DEAD';
  }

  function syncRun() {
    const d = r.state, held = isHeld(d);
    const estop = d && d.state === 'DEAD', done = d && d.state === 'DONE';
    // 임무가 끝나면 기다리던 지령도 없던 일이 된다 — 안 그러면 "정지 대기…"
    // 가 영영 안 풀린다(끝난 로봇은 HOLD 로 가지 않는다).
    if (done || (pend && ((pend === 'STOP' && held)
                          || (pend === 'START' && !held))))
      pend = null;
    runBtn.classList.toggle('go', held && !done);
    runBtn.disabled = !!(estop || done);
    runBtn.textContent =
      estop ? '⛔ 비상정지 중' :
      done ? '✔ 임무 완료' :
      pend === 'STOP' ? '⏸ 정지 대기…' :
      pend === 'START' ? '▶ 시작 대기…' :
      held ? '▶ 시작' : '⏸ 정지';
  }

  runBtn.onclick = () => {
    const c = isHeld(r.state) ? 'START' : 'STOP';
    pend = c;
    syncRun();
    to(c);
  };

  $('b-forward').onclick = () => to('FORWARD');
  $('b-recall').onclick = () => to('RECALL');

  // ── ⛔ 비상정지 / 🔓 해제 — **버튼 하나가 둘을 겸한다** ──────────
  // 🔑 예전에는 `b-release` 가 따로 있었다. 해제는 **비상정지 중일 때만**
  //    의미가 있으므로 늘 떠 있을 이유가 없고, 두 개가 나란히 있으면 급할 때
  //    엉뚱한 쪽을 누른다 — 같은 자리에서 얼굴만 바꾸게 했다(2026-08-11).
  // 🚨 **안 움직이는 로봇에는 비상정지를 못 누른다**(비활성). 상태가 오기
  //    전이나 서 있는 동안 눌려도 할 일이 없고, 눌린 흔적만 남아 헷갈린다.
  //    기준은 시연이 보내오는 `moving` 하나다(규약 MOVING_STATES =
  //    RUN·STUCK·RETURN) — 내가 뭘 눌렀는지가 아니라 로봇이 실제로 가는지.
  const estopBtn = $('b-estop');

  function syncEstop() {
    const d = r.state, dead = !!(d && d.state === 'DEAD');
    // 🔑 해제 얼굴일 때는 `b-release` 를 얹어 색을 바꾼다 — 규칙이 뒤에 있어
    //    같은 특이도로 `b-estop` 의 빨강을 이긴다(panel.css 의 순서).
    estopBtn.classList.toggle('b-release', dead);
    estopBtn.disabled = !dead && !(d && d.moving);
    estopBtn.textContent = dead ? (C ? '🔓 해제' : '🔓 비상정지 해제')
                                : (C ? '⛔ E-STOP' : '⛔ 비상정지');
  }

  // 🔑 **확인 팝업 없이 그 자리에서 나간다** (사용자 지시 2026-08-10).
  //    비상정지는 한 박자라도 늦으면 비상정지가 아니고, 시연 중에 뜨는 모달은
  //    그 한 박자를 먹는다. 잘못 눌렀을 때의 대가도 작다 — ESTOP 은 그 자리에
  //    서는 것뿐이고, 해제는 HOLD 로 갈 뿐이다(해제해도 **바로 안 간다** —
  //    ▶ 시작을 다시 눌러야 움직인다).
  // 🔑 해제는 **START 에 reason 을 실어** 보낸다 — 규약에 지령을 새로 만들지
  //    않으면서도, 실수로 ▶ 시작을 눌러 비상정지가 풀리는 일은 막는다.
  estopBtn.onclick = () => {
    if (r.state && r.state.state === 'DEAD')
      to('START', {reason: 'ESTOP_RELEASE'});
    else
      to('ESTOP');
  };

  syncRun(); syncEstop();            // 상태가 아직 없어도 버튼은 맞춰 둔다
  return bus.on('state', rr => { if (rr === r) { syncRun(); syncEstop(); } });
}

/** 조종석 한 벌(로봇 하나). {draw, off} 를 돌려준다. */
function mountSeat(el, r) {
  el.innerHTML = `
   <div class="card">
    <h2>임무 지령 <span class="muted mono">/${r.ns}/mission</span></h2>
    <div class="btnslot"></div>
   </div>

   <div class="card">
    <h2>카메라 <span class="muted camwhen"></span></h2>
    <div class="camslot"></div>
   </div>

   <div class="card">
    <h2>현재 상태</h2>
    <div class="rowlist">
     <div class="row"><span class="l">FSM</span>
      <span class="v k-state">—</span></div>
     <div class="row"><span class="l">주행</span>
      <span class="v k-moving">—</span></div>
     <div class="row"><span class="l">방향</span>
      <span class="v k-dir">—</span></div>
     <div class="row"><span class="l">진행</span>
      <span class="v k-s">—</span></div>
     <div class="row"><span class="l">지령 속도</span>
      <span class="v k-speed">—</span></div>
     <div class="row"><span class="l">사유</span>
      <span class="v k-reason">—</span></div>
    </div>
    <div class="bar k-bar"><i style="width:0"></i></div>
   </div>

   <div class="card">
    <h2>사건 로그 <span class="muted mono">/${r.ns}/event</span></h2>
    <div class="logslot"></div>
   </div>`;

  const $ = c => el.querySelector('.' + c);

  // ── 지령 버튼 · 카메라 · 사건 로그 (전부 남의 모듈을 그대로 끼운다) ──
  const offBtn = mountButtons($('btnslot'), r);
  const camWhen = $('camwhen');
  const offCam = mountCam($('camslot'), {robot: r, bare: true});
  const log = mountLog($('logslot'), {robot: r, limit: 60});

  function draw() {
    const d = r.state;
    if (!d) {
      // 지난 판의 숫자를 남겨 두지 않는다 (home.js 와 같은 이유)
      camWhen.textContent = '';
      for (const c of ['k-state', 'k-moving', 'k-dir', 'k-s', 'k-speed'])
        $(c).textContent = '—';
      $('k-reason').textContent = r.stale ? '시연 끊김' : '상태 수신 대기…';
      $('k-bar').firstElementChild.style.width = '0';
      return;
    }
    camWhen.textContent = CAM_WHEN[d.cam] || '';
    $('k-state').textContent = d.state || '—';
    $('k-moving').textContent = d.moving ? '주행' : '정지';
    $('k-dir').textContent = d.dir > 0 ? '전진 →' : '후진 ←';
    const tot = d.s_total_mm || 0;
    $('k-s').textContent = `${(d.s_mm || 0).toFixed(0)} / ${tot.toFixed(0)} mm`;
    $('k-speed').textContent = `${((d.speed_mps || 0) * 1000).toFixed(0)} mm/s`;
    // 🔑 지령으로 서 있으면 시연이 `HOLD(CRUISE)` 처럼 **원래 FSM 상태를
    //    괄호에 넣어** 보낸다 — 무엇을 하다 멈췄는지가 여기서만 보인다.
    $('k-reason').textContent = d.reason || '—';
    const bar = $('k-bar');
    bar.firstElementChild.style.width =
      tot ? `${Math.min(100, 100 * (d.s_mm || 0) / tot).toFixed(1)}%` : '0';
    bar.classList.toggle('done', d.state === 'DONE');
  }

  draw();
  return {draw, off: () => { offBtn(); offCam(); log.off(); }};
}

/** "전 로봇에 같이" 버튼 세 개 + toast 를 el 안에 만든다. **해제 함수를
 * 돌려준다** — 비상정지 버튼이 상태를 따라 얼굴을 바꾸느라 bus 를 잡는다.
 *
 * 🔑 3D Map 의 지령 칸도 이걸 쓴다 — 사이드바를 걷어내면서 Robot Handling
 *    페이지가 꺼졌는데, **층별 프로세스를 동시에 출발시키는 유일한 수단**이
 *    이 버튼이라 한 화면짜리에도 반드시 있어야 한다. */
export function mountAllButtons(el, opts = {}) {
  const C = !!opts.compact;
  el.innerHTML = `
   <div class="btns">
    <button class="b-run go">▶ ${C ? '전체' : '전체 시작'}</button>
    <button>⏸ ${C ? '전체' : '전체 정지'}</button>
    <button class="b-estop"></button>
   </div>`
   + (opts.toast ? '' : '<div class="toast"></div>');
  const [start, stop, estop] = el.querySelectorAll('button');
  // opts.toast — 결과 줄을 **바깥과 같이 쓴다**(좁은 칸에서 묶음마다 한 줄씩
  // 두면 그만큼 아래 버튼이 화면 밖으로 밀린다).
  const toast = opts.toast || el.querySelector('.toast');
  const all = (c, extra) => send(toast, c, Object.assign({robot: 'all'},
                                                         extra || {}));
  // 🔑 전체 버튼은 **토글이 아니다.** 두 대의 상태가 서로 다를 수 있어서
  //    (한 대는 주행, 한 대는 용접 중) 한 글자로 대표할 수가 없다 — 시작과
  //    정지를 따로 둔다. 토글이 필요하면 각 칸의 버튼을 쓴다.
  start.onclick = () => all('START');
  stop.onclick = () => all('STOP');

  // ── ⛔ 전체 비상정지 / 🔓 전체 해제 — 개별 칸과 같은 규칙 ────────
  // 🚨 **움직이는 로봇이 하나라도 있으면 ⛔ 가 이긴다.** 한 대는 비상정지
  //    중이고 다른 한 대가 달리는 섞인 상태에서 이 버튼이 "해제" 얼굴이면,
  //    정작 세워야 할 로봇을 세울 수단이 사라진다 — 멈추는 쪽이 우선이다.
  //    (그 상태에서 해제는 해당 층 버튼으로 개별로 한다.)
  function syncEstop() {
    const moving = store.robots.some(r => r.state && r.state.moving);
    const dead = store.robots.some(r => r.state && r.state.state === 'DEAD');
    const rel = dead && !moving;
    estop.classList.toggle('b-release', rel);
    estop.disabled = !moving && !dead;
    estop.textContent = rel ? (C ? '🔓 전체 해제' : '🔓 전체 비상정지 해제')
                            : (C ? '⛔ 전체' : '⛔ 전체 비상정지');
  }

  // 개별 칸과 같은 이유로 **확인 팝업 없이** 바로 나간다(위 mountButtons 참고).
  estop.onclick = () => {
    const moving = store.robots.some(r => r.state && r.state.moving);
    const dead = store.robots.some(r => r.state && r.state.state === 'DEAD');
    if (dead && !moving) all('START', {reason: 'ESTOP_RELEASE'});
    else all('ESTOP');
  };

  syncEstop();
  return bus.on('state', syncEstop);
}

/** 두 대 이상일 때만 세우는 "전체 지령" 카드 (조종석 페이지용). */
function mountAll(el) {
  const card = document.createElement('div');
  card.className = 'card wide';
  card.innerHTML = `
   <h2>전체 지령 <span class="muted">${store.robots.map(r => r.label)
     .join(' + ')} 에 같이 보낸다</span></h2>
   <div class="slot"></div>`;
  el.appendChild(card);
  return mountAllButtons(card.querySelector('.slot'));
}

export function mount(el) {
  const offAll = store.robots.length > 1 ? mountAll(el) : null;
  const seats = new Map();
  for (const {r, el: slot} of robotCols(el))
    seats.set(r, mountSeat(slot, r));
  const off = bus.on('state', r => {
    const s = seats.get(r);
    if (s) s.draw();
  });
  return () => { off(); if (offAll) offAll(); seats.forEach(s => s.off()); };
}
