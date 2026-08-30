// Detect List — dongyeon 검출 노드의 결함 리포트(`<ns>/defect/report_json`).
//
// 🔑 로봇마다 **표를 따로** 세운다 (왼쪽 1층 / 오른쪽 2층). 결함 ID 는 층
//    사이에서 겹칠 수 있고(검출 노드가 각자 1번부터 센다) 무엇보다 수리
//    담당이 다르다 — 한 표에 섞으면 어느 층 결함인지 매번 되짚어야 한다.
//
// 🚨 검출 노드도 **로봇 대수만큼** 띄워야 한다. 두 대가 이름 없는 옛 절대
//    토픽(`/defect/report_json`)에 같이 쏘면 서버는 전부 첫 번째 로봇 것으로
//    친다 — `json_topic:=/floor2/defect/report_json` 처럼 갈라 줄 것.
import {store, bus, robotCols} from '/static/app.js';

function row(r, d) {
  const def = d.defect || {}, m = d.measurement || {};
  const s = def.axial_position_from_entry_m;
  const size = (m.length_mm != null && m.width_mm != null)
    ? `${m.length_mm.toFixed(1)}×${m.width_mm.toFixed(1)}mm` : '-';
  const repaired = r.marks.some(
    x => x.id === (d.defect_id || '?') && x.repaired);
  return `<td>${d.defect_id || '?'}</td><td>${d.class || '?'}</td>`
    + `<td class="num">${((d.confidence || 0) * 100).toFixed(0)}%</td>`
    + `<td class="num">${typeof s === 'number'
        ? (s * 1000).toFixed(0) + 'mm' : '-'}</td>`
    + `<td class="num">${def.clock_angle_deg != null
        ? def.clock_angle_deg.toFixed(0) + '° (' + def.clock_hour + '시)'
        : '-'}</td>`
    + `<td class="num">${size}</td><td>${d.registration || '-'}</td>`
    + `<td>${repaired ? '<span class="pill run">수리 완료</span>'
                      : '<span class="pill warn">대기</span>'}</td>`;
}

/** 표 한 벌(로봇 하나). 다시 그리는 함수를 돌려준다. */
function mountTable(el, r) {
  el.innerHTML = `
   <div class="card">
    <h2>결함 목록 <span class="muted mono">/${r.ns}/defect/report_json</span>
     <span class="cnt" id="d-cnt"></span></h2>
    <div class="scroll">
     <table><thead><tr><th>ID</th><th>종류</th><th>신뢰도</th>
      <th>입구에서</th><th>시계각</th><th>크기(길이×폭)</th><th>등록</th>
      <th>상태</th></tr></thead>
     <tbody id="d-body"></tbody></table>
    </div>
    <div class="empty" id="d-empty">수신 대기 — 검출 노드가 떠 있는가?</div>
   </div>`;

  const body = el.querySelector('#d-body'), empty = el.querySelector('#d-empty');
  const cnt = el.querySelector('#d-cnt');

  return function draw() {
    const rows = [...r.defectRows.values()];
    const done = r.marks.filter(m => m.repaired).length;
    empty.hidden = rows.length > 0;
    cnt.textContent = rows.length
      ? `${rows.length}건 · 수리 완료 ${done}건` : '';
    // 최신이 위 — 규약에 순번이 없으니 들어온 순서를 뒤집어 쓴다
    body.replaceChildren(...rows.reverse().map(d => {
      const tr = document.createElement('tr');
      tr.innerHTML = row(r, d);
      return tr;
    }));
  };
}

export function mount(el) {
  const draws = new Map();
  for (const {r, el: slot} of robotCols(el)) {
    const draw = mountTable(slot, r);
    draw();
    draws.set(r, draw);
  }
  const redraw = (r) => { const f = draws.get(r); if (f) f(); };
  // 수리 완료 표시는 `event`(WELD_DONE)로 바뀐다 — 그때도 다시 그린다
  const off = [bus.on('defect', redraw), bus.on('event', redraw)];
  return () => off.forEach(f => f());
}
