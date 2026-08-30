// Event Log — 1회성 사건(`event` 토픽). alert=true 는 사람이 바로 볼 것.
//
// 🔑 관제실 화면은 **떨어져서 본다.** 그래서 한 줄에 다 밀어 넣은 12px
//    모노스페이스 대신, 사건 이름을 큼직한 배지로 세우고 시각·위치·설명을
//    아래 줄로 내렸다 — 훑을 때 눈이 배지만 따라가면 된다.
//
// 🔑 로그는 **로봇마다 따로**다. 1층·2층은 서로 상관없는 임무라 한 줄에 섞으면
//    "이 STUCK 이 어느 층이냐" 를 매번 되짚어야 한다 — 칸을 갈라 둔다.
import {store, bus, robotCols} from '/static/app.js';

// 사건 이름을 한국어로. 규약(contract.EVENTS)의 12종.
const KO = {
  START: '주행 시작', ARRIVE: '코스 끝 도달', HOME: '출발점 복귀',
  STUCK: '끼임', OFF_COURSE: '코스 이탈', DISCONNECT: '관 단절',
  BRANCH: 'T 분기', DEFECT: '결함 발견', WELD_BEGIN: '용접 시작',
  WELD_DONE: '용접 완료', ESTOP: '비상 정지', DONE: '임무 완료',
};
// 배지 색 — 규약의 alert 집합과 별개로 "좋은 일/나쁜 일" 을 나눈다.
const TONE = {
  START: 'run', HOME: 'run', ARRIVE: 'run', WELD_DONE: 'run', DONE: 'run',
  DEFECT: 'warn', BRANCH: 'warn', WELD_BEGIN: 'warn',
  STUCK: 'bad', OFF_COURSE: 'bad', DISCONNECT: 'bad', ESTOP: 'bad',
};

function line(d) {
  const t = new Date((d.stamp || 0) * 1000)
    .toLocaleTimeString('ko-KR', {hour12: false});
  const div = document.createElement('div');
  div.className = 'ev' + (d.alert ? ' alert' : '');
  const bits = [`<span class="tm">${t}</span>`,
                `s <b>${(d.s_mm || 0).toFixed(0)}</b>mm`];
  if (typeof d.defect_s_mm === 'number')
    bits.push(`결함 s <b>${d.defect_s_mm.toFixed(0)}</b>mm`
              + (d.clock_deg != null ? ` · 시계각 <b>${d.clock_deg}</b>°` : ''));
  if (d.detail) bits.push(d.detail);
  div.innerHTML =
    `<div class="ev-h"><span class="pill ${TONE[d.event] || ''}">`
    + `${d.alert ? '⚠ ' : ''}${KO[d.event] || d.event}</span>`
    + `<span class="ev-k">${d.event}</span></div>`
    + `<div class="ev-m">${bits.join('<span class="sep">·</span>')}</div>`;
  return div;
}

/** 로그 목록만 el 안에 만든다(카드·머리말 없이). 해제 함수를 돌려준다.
 *
 * 🔑 Robot Handling 화면이 이걸 그대로 끼워 쓴다 — 사건 줄을 두 벌 만들면
 *    한쪽만 고쳐져서 같은 사건이 두 화면에 다르게 보인다.
 *
 * opts.robot   어느 로봇의 로그인가 (없으면 첫 번째)
 * opts.limit   최근 N 건만 (기본 전부)
 * opts.filter  d => bool
 */
export function mountLog(el, opts = {}) {
  const R = opts.robot || store.robots[0];
  el.innerHTML = `<div class="evlog scroll" id="e-log"></div>
                  <div class="empty" id="e-empty">수신 대기…</div>`;
  const box = el.querySelector('#e-log'), empty = el.querySelector('#e-empty');

  function draw() {
    let evs = (R && R.events) || [];
    const total = evs.length;
    if (opts.filter) evs = evs.filter(opts.filter);
    empty.hidden = evs.length > 0;
    empty.textContent = total ? '조건에 맞는 사건이 없다.' : '수신 대기…';
    if (opts.limit) evs = evs.slice(-opts.limit);
    box.replaceChildren(...evs.slice().reverse().map(line));   // 최신이 위
  }
  draw();
  return {off: bus.on('event', r => { if (r === R) draw(); }), draw};
}

export function mount(el) {
  const head = document.createElement('div');
  head.className = 'legend';
  head.innerHTML = `<span class="muted">사건 로그 — <code>event</code> 토픽. `
    + `로봇마다 따로 쌓인다.</span>`
    + `<label style="margin-left:auto"><input type="checkbox" id="e-alert">`
    + ` 경고만</label>`;
  el.appendChild(head);

  let alertOnly = false;
  const logs = robotCols(el).map(({r, el: slot}) => {
    const card = document.createElement('div');
    card.className = 'card';
    slot.appendChild(card);
    return mountLog(card, {robot: r, filter: d => !alertOnly || d.alert});
  });
  head.querySelector('#e-alert').onchange = (e) => {
    alertOnly = e.target.checked;
    logs.forEach(l => l.draw());
  };
  return () => logs.forEach(l => l.off());
}
