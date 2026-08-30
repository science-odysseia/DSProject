// Home — 한눈에 보는 대시보드. 다른 페이지의 요약만 싣고 조작은 두지 않는다
// (조작은 Robot Handling, 상세는 각 페이지).
//
// 🔑 로봇마다 **세로 한 칸**이다 — 왼쪽 1층 / 오른쪽 2층. 두 층은 연결되어
//    있지 않은 별개의 임무라 진행률·결함 수를 합치지 않는다. 나란히 놓고
//    "어느 쪽이 더 갔나" 를 눈으로 재는 화면이다.
import {store, bus, showFrame, go, robotCols} from '/static/app.js';

const PILL = {   // FSM 상태 → 색
  RUN: 'run', SETTLE: 'warn', HOLD: 'warn', STUCK: 'bad', INSPECT: 'warn',
  REPAIR: 'warn', RETURN: 'run', DONE: 'run', DEAD: 'bad',
};
const CAM_NAME = {front: '전방 카메라', torch: '토치 카메라'};

/** 로봇 한 대의 요약 칸. {draw…} 를 돌려준다. */
function mountOne(el, r) {
  el.innerHTML = `
   <div class="card">
    <div class="sumhead">
     <span class="pill" id="h-pill">—</span>
     <span class="big" id="h-pct">—</span>
    </div>
    <div class="sub" id="h-reason">상태 수신 대기…</div>
    <div class="bar"><i id="h-bar" style="width:0"></i></div>
    <div class="rowlist" style="margin-top:12px">
     <div class="row"><span class="l">진행</span>
      <span class="v" id="h-s">— / — mm</span></div>
     <div class="row"><span class="l">속도</span>
      <span class="v" id="h-v">—</span></div>
     <div class="row"><span class="l">중심선 이탈</span>
      <span class="v" id="h-off">—</span></div>
     <div class="row"><span class="l">롤</span>
      <span class="v" id="h-roll">—</span></div>
     <div class="row"><span class="l">끼임 / 왕복</span>
      <span class="v" id="h-stuck">—</span></div>
     <div class="row"><span class="l">결함</span>
      <span class="v" id="h-def">0건</span></div>
    </div>
   </div>

   <div class="card">
    <h2 id="h-camtitle">전방 카메라</h2>
    <div class="cam bare"><div class="fr" style="border-radius:8px;
      overflow:hidden">
     <img id="h-img" alt="">
     <div class="nosig" id="h-nosig">프레임 대기 —
      시연을 <code>--ros</code> 로 띄웠는지 확인</div>
    </div></div>
   </div>

   <div class="card">
    <h2>최근 사건</h2>
    <div class="log" id="h-log"><div class="empty">수신 대기…</div></div>
   </div>

   <div class="card">
    <h2>코스</h2>
    <div class="rowlist" id="h-course"></div>
   </div>`;

  const $ = id => el.querySelector('#' + id);
  const img = $('h-img'), nosig = $('h-nosig');

  function drawState() {
    const d = r.state;
    if (!d) {
      // 🚨 상태가 사라졌다(시연 종료·재시작). 그냥 돌아가면 **지난 판의 숫자가
      //    화면에 그대로 남아** 살아 있는 것처럼 보인다 — 비워 준다.
      const p = $('h-pill');
      p.textContent = r.stale ? '끊김' : '대기';
      p.className = 'pill' + (r.stale ? ' bad' : '');
      $('h-reason').textContent = r.stale
        ? '시연이 멈췄다 — 상태가 오지 않는다' : '상태 수신 대기…';
      $('h-pct').textContent = '—';
      $('h-bar').style.width = '0';
      for (const id of ['h-s', 'h-v', 'h-off', 'h-roll', 'h-stuck'])
        $(id).textContent = '—';
      return;
    }
    const p = $('h-pill');
    p.textContent = d.state || '?';
    p.className = 'pill ' + (PILL[d.state] || '');
    $('h-reason').textContent = (d.moving ? '주행 중' : '정지')
      + (d.reason ? ` · ${d.reason}` : '');
    const tot = d.s_total_mm || 0;
    const pct = tot ? 100 * d.s_mm / tot : 0;
    $('h-pct').textContent = tot ? pct.toFixed(0) + '%' : '—';
    $('h-s').textContent =
      `${(d.s_mm || 0).toFixed(0)} / ${tot.toFixed(0)} mm`;
    $('h-bar').style.width = Math.max(0, Math.min(100, pct)) + '%';
    $('h-v').textContent = d.odom_v_mps !== undefined
      ? `${(d.odom_v_mps * 1000).toFixed(0)} mm/s`
      : `${((d.speed_mps || 0) * 1000).toFixed(0)} mm/s (지령)`;
    $('h-off').textContent = `${(d.off_mm || 0).toFixed(1)} mm`;
    $('h-roll').textContent = d.roll_deg !== undefined
      ? `${d.roll_deg > 0 ? '+' : ''}${d.roll_deg}°` : '—';
    $('h-stuck').textContent = `${d.stuck || 0} / ${d.lap || 0}`;
  }

  function drawDefects() {
    const done = r.marks.filter(m => m.repaired).length;
    $('h-def').textContent = `${r.defectRows.size}건 · 수리 완료 ${done}건`;
  }

  function drawLog() {
    const box = $('h-log');
    const last = r.events.slice(-6).reverse();
    if (!last.length) return;
    box.replaceChildren(...last.map(d => {
      const div = document.createElement('div');
      if (d.alert) div.className = 'alert';
      const t = new Date((d.stamp || 0) * 1000)
        .toLocaleTimeString('ko-KR', {hour12: false});
      div.innerHTML = `<span class="t">${t}</span> `
        + `${d.alert ? '⚠' : '·'} ${d.event} `
        + `<span class="t">s=${(d.s_mm || 0).toFixed(0)}mm</span>`;
      return div;
    }));
  }

  function drawCourse() {
    const c = r.course;
    const box = $('h-course');
    if (!c) {
      box.innerHTML = '<div class="empty">코스 수신 대기 — 시연을 '
        + '<code>--ros</code> 로 띄울 것</div>';
      return;
    }
    const row = (l, v) => `<div class="row"><span class="l">${l}</span>`
      + `<span class="v">${v}</span></div>`;
    box.innerHTML =
      row('중심선 길이', `${(c.s_total_m * 1000).toFixed(0)} mm`)
      + row('표본', `${c.pts.length} 점`)
      + row('관 내반경', `${(c.ir_m * 1000).toFixed(0)} mm`)
      + row('곡관 반경', c.bend_r_m ? `${(c.bend_r_m * 1000).toFixed(0)} mm`
                                    : '—')
      + row('답파', `${(r.maxS * 1000).toFixed(0)} mm`);
  }

  // 🔑 **지금 켜진 카메라**를 보여 준다 — 시연이 전진/후진/용접에 따라 한 대만
  //    발행하므로(camera.js 머리말), 전방으로 고정하면 복귀·용접 중에는 얼어
  //    붙은 마지막 그림이 남는다. 역할은 drive_state 의 `cam` 으로 온다.
  const camTitle = el.querySelector('#h-camtitle');
  function drawFrame() {
    const role = (r.state || {}).cam || 'front';
    camTitle.textContent = CAM_NAME[role] || '카메라';
    // 🚨 실패했는데 덮개를 그대로 두면 **앞 역할의 마지막 그림**이 새 제목
    //    아래 남는다 — 화면이 거짓말을 한다. 못 그리면 덮개를 다시 덮는다.
    if (showFrame(img, r, role)) {
      nosig.hidden = true;
    } else {
      nosig.textContent = `${CAM_NAME[role] || '카메라'} 프레임 대기…`;
      nosig.hidden = false;
    }
  }

  el.querySelector('.cam .fr').onclick = () => go('/camera');
  drawState(); drawDefects(); drawLog(); drawCourse(); drawFrame();
  return {drawState, drawDefects, drawLog, drawCourse, drawFrame};
}

export function mount(el) {
  const views = new Map();
  for (const {r, el: slot} of robotCols(el))
    views.set(r, mountOne(slot, r));
  const v = r => views.get(r);

  const off = [
    bus.on('state', r => {
      const x = v(r);
      if (x) { x.drawState(); x.drawCourse(); x.drawFrame(); }
    }),
    bus.on('event', r => {
      const x = v(r);
      if (x) { x.drawLog(); x.drawDefects(); }
    }),
    bus.on('defect', r => { const x = v(r); if (x) x.drawDefects(); }),
    bus.on('course', r => { const x = v(r); if (x) x.drawCourse(); }),
    bus.on('frame', r => { const x = v(r); if (x) x.drawFrame(); }),
  ];
  return () => off.forEach(f => f());
}
