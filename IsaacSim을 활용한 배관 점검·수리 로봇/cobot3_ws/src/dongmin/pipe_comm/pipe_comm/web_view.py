"""카메라를 **브라우저로** 본다 — X 서버도, rqt 도, 추가 설치도 없이.

이 서버에는 디스플레이가 없어서 `rqt` 가 못 뜬다(`could not connect to display`).
X11 포워딩이나 VNC 는 카메라 한 번 보자고 쓰기엔 무겁다. 그래서 브라우저로
직접 흘린다 — 브라우저면 무엇이든 열린다.

전송은 **웹소켓 하나**다. 한 연결로 RGB·Depth·상태를 같이 흘린다:

    텍스트 프레임             drive_state 요약 (사람이 읽는 한 줄)
    바이너리 [0x01 + JPEG]    전방 RGB
    바이너리 [0x02 + JPEG]    Depth 컬러맵

RFC 6455 를 stdlib(hashlib/base64/struct)로 직접 구현했다 — 이 패키지는
`src/dongmin` 만 pull 해서 바로 `ros2 run` 하는 것이 목표라 `websockets`
같은 외부 의존을 늘리지 않는다(아래 web_video_server 를 안 쓰는 이유와
같다). 서버→브라우저 단방향 방송이라 핸드셰이크와 송신 프레이밍만 있으면
되고, 클라이언트가 보내는 것(ping/close)은 읽지 않는다 — 끊김은 쓰기
실패(BrokenPipeError)로 안다.

`/rgb` `/depth` 의 MJPEG(multipart) 경로도 남겨 뒀다. JS 없이 `<img>` 태그
하나로 열리므로 웹소켓 쪽이 의심될 때 대조 진단에 쓴다.

🚨 **웹소켓으로 바꿔도 원본이 10Hz 면 10Hz 로 보인다.** 전송 방식은 지연을
   줄일 뿐 프레임을 만들어 주지 않는다. 화면이 버벅이면 먼저
   `ros2 topic hz /robot/rgb/compressed` 로 발행 쪽 실효 Hz 부터 볼 것 —
   Isaac 물리가 실시간보다 느리면 규약상 10Hz 도 3~5Hz 로 떨어진다.

🔑 **RGB 는 재인코딩하지 않는다.** 규약상 `rgb/compressed` 가 이미 JPEG 이므로
   (`contract.RELATIVE`), 받은 바이트를 그대로 실으면 끝이다.
   디코딩→인코딩을 한 번도 안 하므로 화질 손실도 CPU 부담도 없다.

🚨 **Depth 는 다르다.** 16UC1 PNG(mm) 라 그대로 보내면 브라우저가 새까만
   그림으로 그린다(값이 mm 단위 정수라 0~255 범위를 한참 넘는다). 그래서
   미터로 풀어 **정해진 범위로 정규화하고 컬러맵을 씌운다.**
   ⚠ 그 그림으로 거리를 재면 안 된다 — 보라고 만든 것이다. 실제 값은
     `camera_monitor` 가 찍는 min/중앙/max 숫자를 볼 것.

🚨 **이건 진단 도구다. 인증도 암호화도 없다.** 포트를 여는 순간 그 주소를
   아는 사람은 누구나 본다. 보안그룹에 공인망으로 열어 두지 말 것 —
   SSH 터널을 권한다:
       ssh -L 8080:localhost:8080 ubuntu@<서버>
       → 로컬 브라우저에서 http://localhost:8080

왜 `web_video_server` 를 안 쓰는가
  apt 에 있지만(`ros-humble-web-video-server`) 받는 PC 마다 설치가 하나 더
  는다. 이 패키지는 **`src/dongmin` 만 pull 해서 바로 `ros2 run`** 하는 것이
  목표라 외부 의존을 늘리지 않는다. 게다가 우리 depth 는 16UC1 PNG 라
  어차피 따로 손봐야 한다.
"""

import base64
import hashlib
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from pipe_comm import contract
from pipe_comm import image_codec as codec
from pipe_comm.contract import Topics

BOUNDARY = "pipeframe"
# 관 내부에서 보이는 거리 범위. 고정해 두어야 프레임끼리 색을 비교할 수 있다.
DEPTH_NEAR_M, DEPTH_FAR_M = 0.02, 0.30

# RFC 6455 가 정한 매직 문자열 — 핸드셰이크 응답 키를 만들 때 쓴다.
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
# 바이너리 메시지의 첫 바이트 = 채널. 브라우저 쪽 JS 와 맞춰야 한다.
CH_RGB, CH_DEPTH = b"\x01", b"\x02"

PAGE = """<!doctype html><meta charset="utf-8">
<title>pipe_comm — {ns}</title>
<style>
 body{{background:#111;color:#ddd;font:14px/1.5 monospace;margin:0;padding:16px}}
 h1{{font-size:15px;margin:0 0 12px;color:#8cf}}
 .row{{display:flex;gap:16px;flex-wrap:wrap}}
 .cell{{flex:1 1 420px}}
 .cell h2{{font-size:13px;margin:0 0 6px;color:#9a9}}
 img{{width:100%;background:#000;border:1px solid #333;display:block}}
 #state{{margin-top:14px;white-space:pre-wrap;color:#cfc}}
 .warn{{color:#fa6}}
</style>
<h1>pipe_comm web_view — {ns}</h1>
<div class="row">
  <div class="cell"><h2>rgb/compressed (원본 JPEG 그대로)</h2>
    <img id="rgb"></div>
  <div class="cell"><h2>depth/compressed — {near}~{far} m 정규화
    <span class="warn">(보기용, 거리 측정 금지)</span></h2>
    <img id="depth"></div>
</div>
<div id="state">상태 수신 대기…</div>
<script>
const imgs = {{1: document.getElementById('rgb'),
               2: document.getElementById('depth')}};
const state = document.getElementById('state');
function connect() {{
  const ws = new WebSocket(
    (location.protocol === 'https:' ? 'wss://' : 'ws://')
    + location.host + '/ws');
  ws.binaryType = 'arraybuffer';
  ws.onmessage = (e) => {{
    if (typeof e.data === 'string') {{ state.textContent = e.data; return; }}
    const v = new Uint8Array(e.data);
    const img = imgs[v[0]];
    if (!img) return;
    // Blob URL 을 쓰면 base64 변환 없이 <img> 에 바로 꽂힌다.
    // 로드가 끝난 URL 은 바로 회수한다 — 안 하면 프레임 수만큼 샌다.
    const url = URL.createObjectURL(
      new Blob([v.subarray(1)], {{type: 'image/jpeg'}}));
    img.onload = () => URL.revokeObjectURL(url);
    img.src = url;
  }};
  ws.onclose = () => {{
    state.textContent = '연결 끊김 — 3초 후 재접속';
    setTimeout(connect, 3000);
  }};
}}
connect();
</script>
"""


class Frames:
    """최신 프레임 보관소. ROS 스레드가 쓰고 HTTP 스레드들이 읽는다.

    🔑 순번(seq)을 채널별로 센다 — 하나로 합치면 depth 만 갱신돼도 rgb 를
       다시 보내게 되어 같은 그림이 대역폭만 먹는다.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.rgb = None          # JPEG 바이트 (그대로 흘린다)
        self.depth = None        # JPEG 바이트 (컬러맵 씌운 것)
        self.state = "수신 대기"
        self.seq = {"rgb": 0, "depth": 0, "state": 0}

    def put(self, rgb=None, depth=None):
        with self.lock:
            if rgb is not None:
                self.rgb = rgb
                self.seq["rgb"] += 1
            if depth is not None:
                self.depth = depth
                self.seq["depth"] += 1

    def set_state(self, text):
        with self.lock:
            self.state = text
            self.seq["state"] += 1

    def get(self, which):
        with self.lock:
            return getattr(self, which), self.seq[which]

    def snapshot(self):
        """웹소켓 송신 루프용 — 세 채널을 한 번의 잠금으로 읽는다."""
        with self.lock:
            return self.rgb, self.depth, self.state, dict(self.seq)


class ViewNode(Node):

    def __init__(self, frames):
        super().__init__("web_view")
        self.declare_parameter("ns", contract.DEFAULT_NS)
        self.declare_parameter("port", 8080)
        self.declare_parameter("depth_near_m", DEPTH_NEAR_M)
        self.declare_parameter("depth_far_m", DEPTH_FAR_M)

        self.frames = frames
        self.ns = str(self.get_parameter("ns").value)
        self.near = float(self.get_parameter("depth_near_m").value)
        self.far = float(self.get_parameter("depth_far_m").value)
        t = Topics(self.ns)
        q = contract.sensor_qos()

        self.create_subscription(CompressedImage, t.rgb, self._on_rgb, q)
        self.create_subscription(CompressedImage, t.depth, self._on_depth, q)
        self.create_subscription(String, t.drive_state, self._on_state,
                                 contract.command_qos())
        contract.log_env(self.get_logger())

    def _on_rgb(self, msg):
        # 🔑 디코딩하지 않는다 — 이미 JPEG 다.
        self.frames.put(rgb=bytes(msg.data))

    def _on_depth(self, msg):
        import cv2
        try:
            d = codec.png16_to_depth_m(msg)
        except ValueError:
            return
        span = max(self.far - self.near, 1e-6)
        v = np.clip((d - self.near) / span, 0.0, 1.0)
        v = np.nan_to_num(v, nan=0.0)             # 무효는 검게
        img = cv2.applyColorMap((v * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        img[~np.isfinite(d)] = 0
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.frames.put(depth=buf.tobytes())

    def _on_state(self, msg):
        d = contract.parse(msg.data)
        if d is None:
            return
        arrow = "→" if d.get("dir", 1) > 0 else "←"
        tot = d.get("s_total_mm") or 0.0
        pct = f"{100.0 * d.get('s_mm', 0.0) / tot:.0f}%" if tot else "-"
        self.frames.set_state(
            f"{'주행' if d.get('moving') else '정지'}  "
            f"{d.get('state', '?')}   "
            f"s {d.get('s_mm', 0.0):.0f}mm {arrow} {pct}   "
            f"이탈 {d.get('off_mm', 0.0):.1f}mm   "
            f"왕복 {d.get('lap', 0)}  끼임 {d.get('stuck', 0)}"
            + (f"   — {d['reason']}" if d.get("reason") else ""))


def make_handler(frames, ns, near, far):

    class Handler(BaseHTTPRequestHandler):

        # 🚨 웹소켓 핸드셰이크는 HTTP/1.1 이어야 한다. 기본값(HTTP/1.0)으로
        #    101 을 보내면 브라우저가 업그레이드를 거부한다.
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass                       # 요청마다 찍으면 콘솔이 못 쓰게 된다

        # ── 웹소켓 ──────────────────────────────────────────────
        def _websocket(self):
            key = self.headers.get("Sec-WebSocket-Key")
            if not key or \
                    "websocket" not in self.headers.get("Upgrade", "").lower():
                self.send_error(400, "websocket upgrade 요청이 아니다")
                return
            accept = base64.b64encode(
                hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
            self.send_response(101)
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            self.send_header("Sec-WebSocket-Accept", accept)
            self.end_headers()

            # 접속 즉시 현재 상태부터 보낸다(-1 이 그 역할이다). 프레임은
            # 순번이 바뀐 채널만 보낸다 — 새 것이 없으면 30ms 쉰다.
            sent = {"rgb": -1, "depth": -1, "state": -1}
            try:
                while True:
                    rgb, depth, state, seq = frames.snapshot()
                    fresh = False
                    if seq["state"] != sent["state"]:
                        self._ws_send(0x1, state.encode())
                        sent["state"] = seq["state"]
                        fresh = True
                    if rgb is not None and seq["rgb"] != sent["rgb"]:
                        self._ws_send(0x2, CH_RGB + rgb)
                        sent["rgb"] = seq["rgb"]
                        fresh = True
                    if depth is not None and seq["depth"] != sent["depth"]:
                        self._ws_send(0x2, CH_DEPTH + depth)
                        sent["depth"] = seq["depth"]
                        fresh = True
                    if not fresh:
                        time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                pass                      # 탭을 닫은 것. 정상이다

        def _ws_send(self, opcode, payload):
            """서버→클라이언트 프레임 하나 (RFC 6455 §5.2, 마스킹 없음)."""
            n = len(payload)
            head = bytearray([0x80 | opcode])         # FIN + opcode
            if n < 126:
                head.append(n)
            elif n < 1 << 16:
                head += struct.pack("!BH", 126, n)
            else:
                head += struct.pack("!BQ", 127, n)
            self.wfile.write(bytes(head) + payload)

        # ── MJPEG (진단용 대조 경로) ────────────────────────────
        def _stream(self, which):
            self.send_response(200)
            self.send_header("Content-Type",
                             f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            try:
                while True:
                    data, seq = frames.get(which)
                    if data is None or seq == last:
                        time.sleep(0.03)      # 새 프레임이 없으면 쉰다
                        continue
                    last = seq
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass                          # 탭을 닫은 것. 정상이다

        def do_GET(self):
            path = self.path.split("?")[0].strip("/")
            if path in ("", "index.html"):
                body = PAGE.format(ns=ns, near=near, far=far).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "ws":
                self._websocket()
            elif path == "rgb":
                self._stream("rgb")
            elif path == "depth":
                self._stream("depth")
            elif path == "state":
                with frames.lock:
                    body = frames.state.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

    return Handler


def main(args=None):
    rclpy.init(args=args)
    frames = Frames()
    node = ViewNode(frames)
    port = int(node.get_parameter("port").value)

    srv = ThreadingHTTPServer(
        ("0.0.0.0", port),
        make_handler(frames, node.ns, node.near, node.far))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    node.get_logger().info(
        f"웹 뷰 시작 [{node.ns}] — http://<서버IP>:{port}  (웹소켓 /ws)\n"
        f"  🚨 인증이 없다. 공인망에 열지 말고 SSH 터널을 쓸 것:\n"
        f"     ssh -L {port}:localhost:{port} ubuntu@<서버>  "
        f"→ http://localhost:{port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
