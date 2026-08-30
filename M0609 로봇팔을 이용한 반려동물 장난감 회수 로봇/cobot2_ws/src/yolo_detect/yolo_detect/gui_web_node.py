"""Web UI."""
import asyncio
import glob
import json
import os
import socket
import struct
import threading
import time

import rclpy
import websockets
from ament_index_python.packages import get_package_share_directory
from flask import Flask, Response, abort, render_template, send_from_directory
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Int32

from dsr_msgs2.srv import (
    MovePause, MoveResume, ServoOff, SetRobotControl,
    GetRobotState, GetRobotMode,
)
from yolo_detect import config, transforms

ROBOT_STATE_LABELS = {
    0: "INITIALIZING", 1: "STANDBY", 2: "MOVING", 3: "SAFE_OFF", 4: "TEACHING",
    5: "SAFE_STOP", 6: "EMERGENCY_STOP", 7: "HOMMING", 8: "RECOVERY",
    9: "SAFE_STOP2", 10: "SAFE_OFF2", 15: "NOT_READY",
}
ROBOT_MODE_LABELS = {0: "MANUAL", 1: "AUTONOMOUS", 2: "MEASURE"}

SAFETY_RECOVERY_CONTROL_CODE = {5: 2, 3: 3, 9: 4, 10: 5}

STOP_TYPE_EMERGENCY = 3

class GuiWebNode(Node):
    """브라우저와 ROS 사이의 유일한 연결."""

    def __init__(self):
        super().__init__('gui_web_node')

        ns = config.ROBOT_ID

        self._move_pause_client = self.create_client(MovePause, f'/{ns}/motion/move_pause')
        self._move_resume_client = self.create_client(MoveResume, f'/{ns}/motion/move_resume')
        self._servo_off_client = self.create_client(ServoOff, f'/{ns}/system/servo_off')
        self._set_robot_control_client = self.create_client(SetRobotControl, f'/{ns}/system/set_robot_control')

        self._get_robot_state_client = self.create_client(GetRobotState, f'/{ns}/system/get_robot_state')
        self._get_robot_mode_client = self.create_client(GetRobotMode, f'/{ns}/system/get_robot_mode')

        self._latest_robot_state_code = None
        self._latest_robot_mode_code = None
        self._latest_tcp_pose = [0.0] * 6
        self._latest_joints_deg = [0.0] * 6
        self._state_req_in_flight = False
        self._mode_req_in_flight = False

        self._latest_joint_state = None
        self._joint_state_received_time = None
        self.create_subscription(JointState, f'/{ns}/joint_states', self._joint_state_cb, 10)

        self.create_timer(config.STATUS_POLL_PERIOD_SEC, self._poll_status)

        self._latest_map_msg = None
        map_qos = QoSProfile(
            depth=config.MAP_QOS_DEPTH,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(PointCloud2, config.TOPIC_MAP_POINTS, self._on_map_points, map_qos)
        self.create_timer(config.MAP_PUBLISH_PERIOD_SEC, self._broadcast_cached_views)

        self._latest_path_msg = None
        self._approach_progress = 0
        self._push_map_to_new_client = False
        self.create_subscription(
            PoseArray, config.TOPIC_APPROACH_PATH, self._on_approach_path, map_qos)
        self.create_subscription(
            Int32, config.TOPIC_APPROACH_PROGRESS, self._on_approach_progress, map_qos)

        self._ws_clients = set()
        self._loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(
            target=self._run_ws_server, args=(config.WS_HOST, config.WS_PORT), daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f"GuiWebNode initialized. WS server on ws://{config.WS_HOST}:{config.WS_PORT}")

    def _joint_state_cb(self, msg):
        self._latest_joint_state = {'name': list(msg.name), 'position': list(msg.position)}
        self._joint_state_received_time = time.time()

    def _is_joint_state_stale(self):
        if self._joint_state_received_time is None:
            return True
        return (time.time() - self._joint_state_received_time) > config.JOINT_STATE_STALE_TIMEOUT_SEC

    def _poll_status(self):
        """robot_state/robot_mode는 non-blocking 서비스 조회."""
        if not self._state_req_in_flight and self._get_robot_state_client.service_is_ready():
            self._state_req_in_flight = True
            self._get_robot_state_client.call_async(GetRobotState.Request()).add_done_callback(
                self._on_robot_state_response)

        if not self._mode_req_in_flight and self._get_robot_mode_client.service_is_ready():
            self._mode_req_in_flight = True
            self._get_robot_mode_client.call_async(GetRobotMode.Request()).add_done_callback(
                self._on_robot_mode_response)

        if self._latest_joint_state is not None and not self._is_joint_state_stale():
            joint_dict = dict(zip(
                self._latest_joint_state['name'], self._latest_joint_state['position']))
            self._latest_tcp_pose = transforms.forward_kinematics_posx_mm_deg(joint_dict)
            self._latest_joints_deg = transforms.ordered_joint_degrees(joint_dict)

        payload = {
            'type': 'dashboard',
            'robot_state': ROBOT_STATE_LABELS.get(self._latest_robot_state_code, str(self._latest_robot_state_code)),
            'robot_mode': ROBOT_MODE_LABELS.get(self._latest_robot_mode_code, str(self._latest_robot_mode_code)),
            'tcp_pose': [float(v) for v in self._latest_tcp_pose],
            'joints_deg': [float(v) for v in self._latest_joints_deg],
            'ws_clients': len(self._ws_clients),
        }
        self._broadcast_threadsafe(json.dumps(payload, ensure_ascii=False))

    def _on_robot_state_response(self, future):
        self._state_req_in_flight = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f"get_robot_state failed: {e}")
            return
        if result.success:
            self._latest_robot_state_code = result.robot_state

    def _on_robot_mode_response(self, future):
        self._mode_req_in_flight = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f"get_robot_mode failed: {e}")
            return
        if result.success:
            self._latest_robot_mode_code = result.robot_mode

    def _on_manager_command(self, data):
        """브라우저 명령을 받아 dsr_msgs2 서비스로 그대로 옮김."""
        cmd = data.get('manager_cmd')
        if cmd == config.CMD_PAUSE:
            self._call_move_pause()
        elif cmd == config.CMD_RESUME:
            self._call_move_resume()
        elif cmd == config.CMD_EMERGENCY_STOP:
            self._call_servo_off_emergency()
        elif cmd == config.CMD_RELEASE_SAFETY:
            self._call_release_safety()
        else:
            self.get_logger().error(f"unknown manager_cmd: {cmd}")

    def _call_move_pause(self):
        if not self._move_pause_client.service_is_ready():
            self._manager_log("정지 실패: move_pause 서비스 준비 안 됨")
            return
        future = self._move_pause_client.call_async(MovePause.Request())
        future.add_done_callback(lambda f: self._on_manager_service_response("정지(move_pause)", f))

    def _call_move_resume(self):
        if not self._move_resume_client.service_is_ready():
            self._manager_log("재개 실패: move_resume 서비스 준비 안 됨")
            return
        future = self._move_resume_client.call_async(MoveResume.Request())
        future.add_done_callback(lambda f: self._on_manager_service_response("재개(move_resume)", f))

    def _call_servo_off_emergency(self):
        if not self._servo_off_client.service_is_ready():
            self._manager_log("비상정지 실패: servo_off 서비스 준비 안 됨")
            return
        req = ServoOff.Request()
        req.stop_type = STOP_TYPE_EMERGENCY
        future = self._servo_off_client.call_async(req)
        future.add_done_callback(lambda f: self._on_manager_service_response("비상정지(servo_off)", f))

    def _call_release_safety(self):
        """SAFE_STOP/SAFE_OFF류 안전 상태에서 수동으로 빠져나옴."""
        state_code = self._latest_robot_state_code
        control_code = SAFETY_RECOVERY_CONTROL_CODE.get(state_code)
        if control_code is None:
            self._manager_log(f"안전상태 해제 불필요/불가: 현재 robot_state={state_code}")
            return
        if not self._set_robot_control_client.service_is_ready():
            self._manager_log("안전상태 해제 실패: set_robot_control 서비스 준비 안 됨")
            return
        req = SetRobotControl.Request()
        req.robot_control = control_code
        future = self._set_robot_control_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_manager_service_response("안전상태 해제(set_robot_control)", f))

    def _on_manager_service_response(self, label, future):
        try:
            result = future.result()
        except Exception as e:
            self._manager_log(f"{label} 호출 실패: {e}")
            return
        self._manager_log(f"{label} {'성공' if result.success else '실패'}")

    def _manager_log(self, text):
        self.get_logger().info(f"[web] {text}")
        self._broadcast_threadsafe(json.dumps({'type': 'manager_log', 'text': text}, ensure_ascii=False))

    def _broadcast_cached_views(self):
        """드물게 갱신되고 늦게 접속한 브라우저도 봐야 하는 데이터를 주기적으로 재전송함."""
        if self._push_map_to_new_client:
            self._push_map_to_new_client = False
            self._broadcast_map()
        self._broadcast_approach_path()

    def _on_map_points(self, msg):
        self._latest_map_msg = msg
        self._broadcast_map()

    def _broadcast_map(self):
        if self._latest_map_msg is None or not self._ws_clients:
            return
        points = list(pc2.read_points(
            self._latest_map_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        flat = []
        for x, y, z, rgb in points:
            packed = struct.unpack('I', struct.pack('f', rgb))[0]
            r, g, b = (packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF
            flat.append([round(float(x), 1), round(float(y), 1), round(float(z), 1),
                         int(r), int(g), int(b)])
        payload = json.dumps({'type': 'map', 'points': flat,
                              'voxel_mm': config.MAP_WEB_VOXEL_SIZE_MM})
        size_mb = len(payload) / 1024 / 1024
        if size_mb > 15.0:
            self.get_logger().error(
                f"맵 JSON {size_mb:.1f}MB — WebSocket 한계(16MB)에 근접/초과해 전송을 건너뜁니다. "
                f"config.MAP_WEB_VOXEL_SIZE_MM({config.MAP_WEB_VOXEL_SIZE_MM})을 키우거나 "
                f"MAP_BROADCAST_MAX_POINTS({config.MAP_BROADCAST_MAX_POINTS})를 줄이세요")
            return
        self.get_logger().info(f"맵 브로드캐스트: {len(flat):,}점 / {size_mb:.1f}MB")
        self._broadcast_threadsafe(payload)

    def _on_approach_path(self, msg):
        self._latest_path_msg = msg
        self._broadcast_approach_path()

    def _on_approach_progress(self, msg):
        self._approach_progress = int(msg.data)
        self._broadcast_approach_path()

    def _broadcast_approach_path(self):
        """경로와 진행도를 한 메시지로 보냄."""
        if self._latest_path_msg is None or not self._ws_clients:
            return
        points = [[float(p.position.x), float(p.position.y), float(p.position.z)]
                  for p in self._latest_path_msg.poses]
        self._broadcast_threadsafe(json.dumps({
            'type': 'approach_path',
            'points': points,
            'reached': self._approach_progress,
        }))

    def _run_ws_server(self, host, port):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve_forever(host, port))

    async def _serve_forever(self, host, port):
        async with websockets.serve(self._on_client_connected, host, port, max_size=16 * 1024 * 1024):
            await asyncio.Future()

    async def _on_client_connected(self, websocket):
        self._ws_clients.add(websocket)
        self.get_logger().info(f"web client connected ({len(self._ws_clients)} total)")
        self._push_map_to_new_client = True
        try:
            async for message in websocket:
                self._on_ws_message(message)
        finally:
            self._ws_clients.discard(websocket)
            self.get_logger().info(f"web client disconnected ({len(self._ws_clients)} total)")

    async def _broadcast(self, data):
        if not self._ws_clients:
            return
        await asyncio.gather(*(c.send(data) for c in self._ws_clients), return_exceptions=True)

    def _broadcast_threadsafe(self, data):
        asyncio.run_coroutine_threadsafe(self._broadcast(data), self._loop)

    def _on_ws_message(self, raw_message):
        try:
            data = json.loads(raw_message)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"WS message parse failed: {e}")
            return
        if 'manager_cmd' in data:
            self._on_manager_command(data)
        else:
            self.get_logger().error(f"unknown WS message: {raw_message[:200]}")

def create_app():
    share_dir = get_package_share_directory('yolo_detect')
    gui_web_dir = os.path.join(share_dir, 'gui_web')

    app = Flask(
        __name__,
        template_folder=os.path.join(gui_web_dir, 'templates'),
        static_folder=os.path.join(gui_web_dir, 'static'),
    )

    ws_host = os.environ.get('WS_HOST', config.WS_HOST if config.WS_HOST != '0.0.0.0' else 'localhost')
    ws_port = os.environ.get('WS_PORT', str(config.WS_PORT))

    @app.route('/')
    def index():
        return render_template('index.html', ws_host=ws_host, ws_port=ws_port)

    def _latest_photo_map():
        """가장 최근에 생성된 *_color_viewer.html 파일명."""
        pattern = os.path.join(config.PHOTO_MAP_OUTPUT_DIR, '*_color_viewer.html')
        files = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)
        return os.path.basename(files[0]) if files else None

    @app.route('/photo_map')
    def photo_map():
        name = _latest_photo_map()
        if name is None:
            return (
                "<body style='background:#111;color:#ddd;font-family:sans-serif;padding:40px'>"
                "<h2>실사맵이 아직 없습니다</h2>"
                "<p>정찰(RunRecon)을 한 번 완료하면 object_detect_node가 배경에서 자동 생성합니다"
                " (10~30초 소요).</p>"
                f"<p>수동 생성: <code>python3 {config.PHOTO_MAP_SCRIPT} "
                "&lt;스냅샷폴더&gt; --color-viewer</code></p>"
                "</body>", 404)
        path = os.path.join(config.PHOTO_MAP_OUTPUT_DIR, name)
        with open(path, encoding='utf-8') as f:
            html = f.read()
        base_tag = '<base href="/photo_map/">'
        if '<head>' in html:
            html = html.replace('<head>', '<head>' + base_tag, 1)
        else:
            html = base_tag + html
        resp = Response(html, mimetype='text/html')
        resp.headers['Cache-Control'] = 'no-store, must-revalidate'
        resp.headers['X-Photo-Map-File'] = name
        return resp

    @app.route('/photo_map/<path:filename>')
    def photo_map_asset(filename):
        """뷰어 HTML이 참조하는 .ply 파일들."""
        if not filename.endswith(('.ply', '.html')):
            abort(404)
        return send_from_directory(config.PHOTO_MAP_OUTPUT_DIR, filename)

    return app

def _ws_port_already_taken():
    """WS 포트에 실제로 듣고 있는 서버가 있는지 접속을 시도해서 판정함."""
    try:
        with socket.create_connection(("127.0.0.1", config.WS_PORT), timeout=0.4):
            return True
    except OSError:
        return False

def _start_bridge_in_process():
    """ROS 브리지 노드를 이 프로세스 안에서 함께 돌림."""
    if _ws_port_already_taken():
        print(f"[gui_web] WS 포트 {config.WS_PORT} 사용 중 — 이미 떠 있다고 보고 "
              f"연결하지 않습니다(정적 서버로만 동작).")
        return None
    try:
        rclpy.init()
        node = GuiWebNode()
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        print(f"[gui_web] ROS 브리지 내장 실행 — WS {config.WS_HOST}:{config.WS_PORT}")
        return node
    except Exception as e:
        print(f"[gui_web] ⚠️ 브리지 시작 실패 — 실시간 표시(로봇 연결/3D 맵/정지 버튼)가 "
              f"동작하지 않습니다: {e}")
        return None

def main():
    _start_bridge_in_process()
    app = create_app()
    print(f"[gui_web] http://localhost:{config.FLASK_PORT}  (실사맵: /photo_map)")
    app.run(host=config.FLASK_HOST, port=config.FLASK_PORT, use_reloader=False)

if __name__ == '__main__':
    main()
