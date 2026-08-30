// Camera — 로봇 한 대의 **지금 켜져 있는 카메라 한 대만** 본다.
//
// 🚨 v1_3 은 로봇당 한 대만, 항상 그 역할을 발행한다 — floor1 전방 / floor2
//    토치(후방 카메라는 폐지). 켜진 역할은 `drive_state.cam` 으로 온다. 이
//    칸은 그 값을 그대로 따라가므로 층별 매핑을 여기 하드코딩하지 않는다.
//
// 🔑 **화면도 한 칸만 만든다.** 예전에는 3칸을 나란히 띄워 놓고 둘은 늘 "대기"
//    였는데, 안 쓰는 칸이 마지막 프레임을 디코딩된 채로 붙들고 있었다(브라우저는
//    <img> 에 걸린 그림을 화면 밖이라고 버리지 않는다). 분기가 바뀌면 쓰던 칸을
//    비우고(objectURL 회수 + src 제거) 새 역할로 갈아 끼운다.
//
// 🔑 로봇이 여러 대면 **칸도 대수만큼** — 다만 한 대당 하나다(위 규칙 그대로).
//    `mountCam(el, {robot})` 로 어느 로봇의 칸인지 반드시 준다.
import {store, bus, showFrame, dropOtherCams, robotCols}
  from '/static/app.js';

// v1_3: 로봇당 카메라 1대 고정 — floor1 전방(rgb) / floor2 토치.
// floor2 토치 칸은 dongyeon opencv 오버레이(repair_robot/opencv_debug/
// compressed)를 받는다 — real_map_demo_v1_3.py 가 발행하는 실제 이름.
// rear 는 폐지됐다(더 이상 발행되지 않는다).
const CAM = {
  front: {name: '전방 카메라', topic: 'rgb/compressed'},
  torch: {name: '토치 카메라', topic: 'repair_robot/opencv_debug/compressed'},
};

/** 카메라 한 칸을 el 안에 만든다. 해제 함수를 돌려준다.
 *  opts.robot — 어느 로봇의 카메라인가 (없으면 첫 번째).
 *  opts.bare  — 카드 안에 넣을 때(테두리·배경 없이). */
export function mountCam(el, opts = {}) {
  const R = opts.robot || store.robots[0];
  el.innerHTML = `
   <div class="cam${opts.bare ? ' bare' : ''}">
    <div class="capt"><b id="cm-name">—</b>
     <span class="live" id="cm-live" hidden>● LIVE</span>
     <span class="t" id="cm-when"></span>
     <span class="fps" id="cm-fps">—</span></div>
    <div class="fr"><img id="cm-img" alt="카메라">
     <div class="nosig" id="cm-nosig">프레임 대기 —
      시연을 <code>--ros</code> 로 띄웠는지 확인</div></div>
   </div>`;

  const $ = id => el.querySelector('#' + id);
  const img = $('cm-img'), nosig = $('cm-nosig'), fps = $('cm-fps');
  const name = $('cm-name'), when = $('cm-when'), live = $('cm-live');
  let role = null;                 // 지금 이 칸이 보여 주는 역할

  /** 지금 켜진 역할. 구판 시연(cam 필드 없음)이면 전방으로 본다. */
  const active = () => ((R && R.state) || {}).cam || 'front';

  /** 칸을 비운다 — objectURL 을 회수하고 그림을 떼어 낸다.
   *  🚨 src 를 그냥 두면 역할이 바뀌어도 **앞 카메라의 마지막 그림**이 남는다
   *     (제목만 바뀌어 화면이 거짓말을 한다). 메모리도 그만큼 안 풀린다. */
  function clearFrame() {
    for (const k of ['url', 'prev']) {
      if (img.dataset[k]) {
        URL.revokeObjectURL(img.dataset[k]);
        delete img.dataset[k];
      }
    }
    img.removeAttribute('src');
    img.onload = null;
    fps.textContent = '—';
    nosig.hidden = false;
  }

  function setRole(r) {
    if (r === role) return;
    role = r;
    clearFrame();
    const c = CAM[r] || {name: r, when: ''};
    name.textContent = c.name;
    when.textContent = c.when || '';
    nosig.textContent = `${c.name} 프레임 대기…`;
    live.hidden = true;
    // 🔑 다른 역할이 붙들고 있던 프레임(Blob)을 놓아 준다 — 안 보는 카메라의
    //    그림을 들고 있을 이유가 없다.
    dropOtherCams(R, r);
    draw(r);
  }

  function draw(r) {
    if (r !== role) return;        // 지금 보는 역할의 프레임만 그린다
    if (showFrame(img, R, r)) {
      nosig.hidden = true;
      live.hidden = false;
      const c = R.cam[r];
      fps.textContent = c.fps ? `${c.fps.toFixed(1)} fps` : '';
    }
  }

  setRole(active());

  // 프레임이 끊기면 fps 가 옛날 값에 얼어붙는다 — 1초마다 늙히고, 오래 끊기면
  // 덮개를 다시 덮는다. 지금 켜진 카메라가 끊긴 것은 **정상이 아니다**.
  const timer = setInterval(() => {
    const s = R && R.cam[role];
    if (!s) return;
    const age = (performance.now() - s.t) / 1000;
    if (age > 2) fps.textContent = `${age.toFixed(0)}초 전`;
    if (age > 3) {
      nosig.textContent = `${age.toFixed(0)}초째 프레임 없음`;
      nosig.hidden = false;
      live.hidden = true;
    }
  }, 1000);

  const off = [
    bus.on('frame', (r, ro) => { if (r === R) draw(ro); }),
    bus.on('state', (r) => { if (r === R) setRole(active()); }),
    // 🚨 판이 바뀌면(시연 종료·재시작) **마지막 그림을 떼어 낸다.** 역할이
    //    그대로면 setRole 이 아무 일도 안 하므로, 지난 판의 프레임이 새 판의
    //    제목 아래 그대로 걸려 있게 된다 — 화면이 거짓말을 한다.
    bus.on('reset', (r) => {
      if (r !== R) return;
      clearFrame();
      nosig.textContent = R.stale ? '시연 끊김 — 프레임 없음'
                                  : '프레임 대기…';
      live.hidden = true;
    }),
  ];
  return () => {
    off.forEach(f => f());
    clearInterval(timer);
    clearFrame();                  // 페이지를 떠날 때도 그림을 놓는다
  };
}

export function mount(el) {
  const legend = document.createElement('div');
  legend.className = 'legend';
  legend.innerHTML = '어안 140° · 10Hz JPEG · <b>로봇 한 대당 한 칸</b> — '
    + 'floor1 전방 / floor2 토치(용접부). 각 로봇이 자기 역할을 항상 발행한다.';
  el.appendChild(legend);
  const offs = robotCols(el).map(
    ({r, el: slot}) => mountCam(slot, {robot: r}));
  return () => offs.forEach(f => f());
}
