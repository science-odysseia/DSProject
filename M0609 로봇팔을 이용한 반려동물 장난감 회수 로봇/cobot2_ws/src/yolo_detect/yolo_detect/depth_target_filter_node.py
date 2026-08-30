"""작업영역 밖과 타겟 반경의 depth를 마스킹해 내보내는 중계 노드."""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import tf2_ros

from yolo_detect import config

class DepthTargetFilterNode(Node):
    def __init__(self):
        super().__init__('depth_target_filter_node')
        self.bridge = CvBridge()
        self.intrinsics = None

        p1 = np.array(config.BOX_CORNER_1_MM, dtype=np.float64)
        p2 = np.array(config.BOX_CORNER_2_MM, dtype=np.float64)
        p3 = np.array(config.BOX_CORNER_3_MM, dtype=np.float64)
        box_ex = p2 - p1
        self._box_width = np.linalg.norm(box_ex)
        box_ex = box_ex / self._box_width
        box_ey = p3 - p1
        self._box_height = np.linalg.norm(box_ey)
        box_ey = box_ey / self._box_height
        box_ez = np.cross(box_ex, box_ey)
        box_ez = box_ez / np.linalg.norm(box_ez)
        self._box_corner1 = p1
        self._box_axes = np.column_stack([box_ex, box_ey, box_ez])
        self._box_exterior_depth = config.BOX_DEPTH_MM + config.BOX_WALL_THICKNESS_MM
        self._is_moving = True
        self._stopped_since = None
        self._dropped_count = 0
        self._passed_count = 0

        self.declare_parameter('enabled', False)
        self.declare_parameter('target_x_mm', 0.0)
        self.declare_parameter('target_y_mm', 0.0)
        self.declare_parameter('target_z_mm', 0.0)
        self.declare_parameter('radius_mm', config.DEPTH_TARGET_MASK_RADIUS_MM)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self._camera_info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, config.TOPIC_DEPTH_RAW, self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(
            JointState, f'/{config.ROBOT_ID}/joint_states', self._joint_state_cb, 10)
        self._pub = self.create_publisher(
            Image, config.TOPIC_DEPTH_FILTERED, qos_profile_sensor_data)
        self._camera_info_pub = self.create_publisher(
            CameraInfo, config.TOPIC_CAMERA_INFO_FILTERED, qos_profile_sensor_data)

        self.get_logger().info("DepthTargetFilterNode initialized.")

    def _camera_info_cb(self, msg):
        self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}
        self._camera_info_pub.publish(msg)

    def _joint_state_cb(self, msg):
        was_moving = self._is_moving
        self._is_moving = any(
            abs(v) > config.JOINT_MOVING_VELOCITY_THRESHOLD_RAD_S for v in msg.velocity)
        if was_moving and not self._is_moving:
            self._stopped_since = time.monotonic()
            self.get_logger().info(
                "[게이팅] 정지 감지 — settle 타이머 시작", throttle_duration_sec=0.0)
        elif self._is_moving and not was_moving:
            self._stopped_since = None
            self.get_logger().info(
                f"[게이팅] 움직임 감지(다시 이동 시작) — 통과 {self._passed_count}건, "
                f"차단 {self._dropped_count}건(누적)", throttle_duration_sec=0.0)

    def _depth_cb(self, msg):
        settled = (
            not self._is_moving
            and self._stopped_since is not None
            and (time.monotonic() - self._stopped_since) >= config.JOINT_STOPPED_SETTLE_DELAY_SEC)
        if not settled:
            self._dropped_count += 1
            self.get_logger().info(
                f"[게이팅] depth 프레임 차단(is_moving={self._is_moving}, "
                f"stopped_since={'있음' if self._stopped_since else '없음'}) — "
                f"누적 차단 {self._dropped_count}/통과 {self._passed_count}",
                throttle_duration_sec=2.0)
            return
        self._passed_count += 1
        self.get_logger().info(
            f"[게이팅] depth 프레임 통과 — 누적 차단 {self._dropped_count}/통과 {self._passed_count}",
            throttle_duration_sec=2.0)
        if self.intrinsics is None:
            self._pub.publish(msg)
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                msg.header.frame_id, 'base_link', Time())
        except Exception as e:
            self.get_logger().warn(
                f"TF 조회 실패({msg.header.frame_id} <- base_link): {e}",
                throttle_duration_sec=5.0)
            self._pub.publish(msg)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        t_vec = np.array([t.x, t.y, t.z]) * 1000.0

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = depth.shape
        fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
        ppx, ppy = self.intrinsics['ppx'], self.intrinsics['ppy']

        us, vs = np.meshgrid(np.arange(w), np.arange(h))
        d = depth.astype(np.float32)
        valid = d > 0
        xs = (us - ppx) * d / fx
        ys = (vs - ppy) * d / fy

        mask = np.zeros_like(valid)

        corner1_cam = R @ self._box_corner1 + t_vec
        axes_cam = R @ self._box_axes
        rel_x = xs - corner1_cam[0]
        rel_y = ys - corner1_cam[1]
        rel_z = d - corner1_cam[2]
        lx = rel_x * axes_cam[0, 0] + rel_y * axes_cam[1, 0] + rel_z * axes_cam[2, 0]
        ly = rel_x * axes_cam[0, 1] + rel_y * axes_cam[1, 1] + rel_z * axes_cam[2, 1]
        lz = rel_x * axes_cam[0, 2] + rel_y * axes_cam[1, 2] + rel_z * axes_cam[2, 2]

        mx, mz = config.BOX_MARGIN_XY_MM, config.BOX_MARGIN_Z_MM
        in_box = (
            (lx >= -mx) & (lx <= self._box_width + mx)
            & (ly >= -mx) & (ly <= self._box_height + mx)
            & (lz >= -mz) & (lz <= self._box_exterior_depth + mz))
        mask |= valid & ~in_box

        enabled = self.get_parameter('enabled').value
        if enabled:
            target_mm = np.array([
                self.get_parameter('target_x_mm').value,
                self.get_parameter('target_y_mm').value,
                self.get_parameter('target_z_mm').value,
            ])
            radius_mm = self.get_parameter('radius_mm').value
            target_cam_mm = R @ target_mm + t_vec
            dist = np.sqrt(
                (xs - target_cam_mm[0]) ** 2 + (ys - target_cam_mm[1]) ** 2
                + (d - target_cam_mm[2]) ** 2)
            mask |= valid & (dist <= radius_mm)

        filtered = depth.copy()
        filtered[mask] = config.DEPTH_TARGET_MASK_FAR_VALUE_MM

        out_msg = self.bridge.cv2_to_imgmsg(filtered, encoding=msg.encoding)
        out_msg.header = msg.header
        self._pub.publish(out_msg)

def main():
    rclpy.init()
    node = DepthTargetFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
