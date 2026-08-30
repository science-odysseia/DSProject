// 층별 상태·결함 요약 — **세로가 항목, 가로가 층**인 비교표.
//
// 🔑 3D Map 페이지 가운데 칸이 쓴다(ref_img/web_ui.jpg 의 "robot state" +
//    "결함 현황/갯수"). 로봇마다 카드를 따로 세우는 Home 과 달리 여기서는
//    **한 표에 층을 나란히** 놓는다 — 관제실에서 정작 알고 싶은 것은 각
//    로봇의 절대값이 아니라 "어느 층이 처졌나" 이고, 그건 같은 줄에 놓고
//    봐야 눈으로 잡힌다. 열 순서는 언제나 왼쪽 1층 / 오른쪽 2층
//    (`store.robots` 순서 = 서버 `-p ns:=floor1,floor2`).
import {store, bus} from '/static/app.js';

const PILL = {   // FSM 상태 → 색
  RUN: 'run', SETTLE: 'warn', HOLD: 'warn', STUCK: 'bad', INSPECT: 'warn',
  REPAIR: 'warn', RETURN: 'run', DONE: 'run', DEAD: 'bad',
};

function headRow(cls) {
  return `<tr><th class="lbl"></th>`
    + store.robots.map(r => `<th class="${cls}">${r.label}</th>`).join('')
    + `</tr>`;
}

/** 진행시간 초 → `분:초` (한 시간을 넘으면 `시:분:초`). */
function hms(sec) {
  const s = Math.max(0, Math.floor(sec || 0));
  const p = n => String(n).padStart(2, '0');
  return s >= 3600
    ? `${Math.floor(s / 3600)}:${p(Math.floor(s / 60) % 60)}:${p(s % 60)}`
    : `${Math.floor(s / 60)}:${p(s % 60)}`;
}

/** 층별 주행 상태 비교표. 다시 그리는 함수를 돌려준다.
 *
 * 🔑 **네 줄뿐이다** (사용자 지시 2026-08-11: FSM · 진행률 · 진행시간 · 방향).
 *    거리·속도·중심선 이탈·롤·끼임·사유는 뺐다 — 가운데 칸에 표 둘과 지령
 *    버튼이 같이 서야 해서 한 줄이 곧 자리다. 되살릴 때는 아래 ROWS 에
 *    되돌려 넣으면 그만이다(값은 전부 `r.state` 에 그대로 들어 있다).
 *    🔑 "지금 켜진 카메라" 줄은 예전부터 없다 — 오른쪽 카메라 칸 제목이
 *       이미 그것을 적는다.
 */
export function mountStateTable(el) {
  // 항목 = 표의 줄. [키, 이름, 로봇 → 문자열]
  const ROWS = [
    ['state', 'FSM', null],          // 배지라 따로 그린다
    ['prog', '진행률', null],        // 막대라 따로 그린다
    // 🚨 진행시간은 규약에 없다 — app.js 가 `stamp` 로 만든 값이다(안착 제외,
    //    DONE 에서 멈춤). 상태가 아직 없으면 0:00 이 아니라 `—` 다.
    ['t', '진행시간', r => r.t0 === null ? '—' : hms(r.elapsedS)],
    ['dir', '방향', r => !r.state ? '—' : (r.state.dir > 0 ? '전진 →' : '후진 ←')],
  ];

  el.innerHTML = `<table class="fstat"><thead>${headRow('')}</thead><tbody>`
    + ROWS.map(([k, name]) => `<tr data-k="${k}"><th class="lbl">${name}</th>`
        + store.robots.map(() => '<td>—</td>').join('') + '</tr>').join('')
    + `</tbody></table>`;

  const cell = (k, i) =>
    el.querySelector(`tr[data-k="${k}"]`).children[i + 1];

  return function draw() {
    store.robots.forEach((r, i) => {
      const d = r.state;
      // FSM — 배지로
      const st = cell('state', i);
      // 상태가 없는 이유가 둘이다 — 아직 안 온 것(대기)과 끊긴 것. 끊김은
      // 사람이 손을 써야 하는 상황이라 색을 달리한다.
      st.innerHTML = d
        ? `<span class="pill ${PILL[d.state] || ''}">${d.state || '?'}</span>`
        : (r.stale ? '<span class="pill bad">끊김</span>'
                   : '<span class="pill">대기</span>');
      // 진행 — 숫자 + 가는 막대. 🔑 "몇 %" 만 보면 두 층 중 어느 쪽이 처졌는지
      //    한눈에 안 잡힌다. 막대를 같은 줄에 놓으면 길이로 바로 보인다.
      const tot = (d || {}).s_total_mm || 0;
      const pct = tot ? Math.max(0, Math.min(100, 100 * d.s_mm / tot)) : 0;
      cell('prog', i).innerHTML = `<div class="pw">`
        + `<span>${tot ? pct.toFixed(0) + '%' : '—'}</span>`
        + `<div class="bar${(d || {}).state === 'DONE' ? ' done' : ''}">`
        + `<i style="width:${pct}%"></i></div></div>`;
      for (const [k, , fn] of ROWS) if (fn) cell(k, i).textContent = fn(r);
    });
  };
}

/** 층별 결함 현황 — **검출·수리 완료 두 줄뿐이다.**
 *
 * 🔑 `대기`(검출−수리)와 `최근`(마지막 결함 요약)은 뺐다(사용자 지시
 *    2026-08-11). 3D Map 가운데 칸이 이 표 **하나만** 남기고 글자를 두 배로
 *    키운 판이 되면서, 떨어져서 읽을 값은 두 숫자로 족하다는 결론이다.
 */
export function mountDefectTable(el) {
  el.innerHTML = `<table class="fstat"><thead>${headRow('')}</thead><tbody>
    <tr data-k="found"><th class="lbl">검출</th>${
      store.robots.map(() => '<td>0</td>').join('')}</tr>
    <tr data-k="done"><th class="lbl">수리 완료</th>${
      store.robots.map(() => '<td>0</td>').join('')}</tr>
   </tbody></table>`;

  const cell = (k, i) =>
    el.querySelector(`tr[data-k="${k}"]`).children[i + 1];

  return function draw() {
    store.robots.forEach((r, i) => {
      const n = r.defectRows.size;
      const done = r.marks.filter(m => m.repaired).length;
      cell('found', i).textContent = `${n}건`;
      cell('done', i).innerHTML = done
        ? `<span class="pill run">${done}건</span>` : '0건';
    });
  };
}

/** 두 표를 세우고 bus 까지 걸어 준다. 해제 함수를 돌려준다.
 *
 * 🔑 화면에서의 위아래는 **로봇 상태 → 결함 현황** 이다(map3d.js 의 카드
 *    순서). 로봇이 지금 뭘 하는지가 먼저고, 그 결과로 쌓인 숫자가 다음이다.
 */
export function mountFloorPanels(stateEl, defectEl) {
  const drawState = mountStateTable(stateEl);
  const drawDefect = mountDefectTable(defectEl);
  drawState(); drawDefect();
  const off = [
    bus.on('state', drawState),
    bus.on('defect', drawDefect),
    bus.on('event', () => { drawDefect(); }),
  ];
  return () => off.forEach(f => f());
}
