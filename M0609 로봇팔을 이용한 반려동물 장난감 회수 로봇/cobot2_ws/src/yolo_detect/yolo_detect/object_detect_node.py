"""정찰 노드."""
import collections
import json
import os
import subprocess
import sys
import threading
import time
import traceback
import warnings

import numpy as np
import cv2
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, JointState
from std_msgs.msg import Header, Int32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from yolo_detect_msgs.action import RunRecon
from yolo_detect_msgs.msg import DetectedObject, RawDetection
from dsr_msgs2.srv import GetCurrentPosx

from yolo_detect import transforms, config

class ObjectDetectNode(Node):
    def __init__(self):
        super().__init__('object_detect_node')
        self.bridge = CvBridge()
        self.intrinsics = None
        self.latest_color = None
        self.latest_depth = None

        self.T_gripper2camera = self._load_gripper2camera()

        self._recon_active = False
        self._goal_waypoints = []
        self._visited = []
        self._dwell_counts = []
        self._arrival_times = []
        self._pending_since = []
        self._snapshot_taken = []
        self._snapshots = []
        self._snapshot_boxes = {}

        cb_group = ReentrantCallbackGroup()

        ns = config.ROBOT_ID
        self._get_current_posx_client = self.create_client(
            GetCurrentPosx, f'/{ns}/aux_control/get_current_posx', callback_group=cb_group)
        self._latest_posx = None
        self._posx_req_in_flight = False
        self._posx_req_sent_time = None

        self._depth_buffer = collections.deque(maxlen=config.DEPTH_MEDIAN_FRAMES)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self._color_cb, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._camera_info_cb, 10)

        joint_state_group = MutuallyExclusiveCallbackGroup()
        self._latest_joint_state = None
        self._joint_state_received_time = None
        self.create_subscription(
            JointState, f'/{config.ROBOT_ID}/joint_states', self._joint_state_cb, 10,
            callback_group=joint_state_group)

        self._yolo_model = None
        self._mask_model = None
        self._zero_shot = None

        self._action_server = ActionServer(
            self, RunRecon, config.ACTION_RUN_RECON,
            execute_callback=self._execute_run_recon,
            callback_group=cb_group,
        )

        pose_tick_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / config.POSE_POLL_HZ, self._pose_poll_tick, callback_group=pose_tick_group)

        self._snapshot_done_pub = self.create_publisher(Int32, config.TOPIC_SNAPSHOT_TAKEN, 10)

        map_qos = QoSProfile(
            depth=config.MAP_QOS_DEPTH,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._map_pub = self.create_publisher(PointCloud2, config.TOPIC_MAP_POINTS, map_qos)

        self.get_logger().info("ObjectDetectNode initialized.")

    def _load_gripper2camera(self):
        if not os.path.exists(config.T_GRIPPER2CAMERA_PATH):
            self.get_logger().error(f"T_gripper2camera.npy not found: {config.T_GRIPPER2CAMERA_PATH}")
            return np.eye(4)
        return np.load(config.T_GRIPPER2CAMERA_PATH)

    def _camera_info_cb(self, msg):
        self.intrinsics = {"fx": msg.k[0], "fy": msg.k[4], "ppx": msg.k[2], "ppy": msg.k[5]}

    def _color_cb(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self._depth_buffer.append((time.time(), self.latest_depth))

    def _joint_state_cb(self, msg):
        self._latest_joint_state = {'name': list(msg.name), 'position': list(msg.position)}
        self._joint_state_received_time = time.time()

    def _is_joint_state_stale(self):
        """joint_states가 마지막으로 갱신된 뒤 얼마나 지났는지 봄."""
        if self._joint_state_received_time is None:
            return True
        return (time.time() - self._joint_state_received_time) > config.JOINT_STATE_STALE_TIMEOUT_SEC

    def _poll_posx(self):
        """non-blocking으로 자세 조회를 계속 발사, 결과는 콜백에서 캐시에 반영."""
        ready = self._get_current_posx_client.service_is_ready()
        if self._posx_req_in_flight:
            elapsed = time.time() - self._posx_req_sent_time
            if elapsed < config.POSX_REQUEST_TIMEOUT_SEC:
                return
            self.get_logger().warn(
                f"get_current_posx 응답 {elapsed:.1f}s 동안 없음 — 이 요청 포기하고 재시도")
            self._posx_req_in_flight = False
        if not ready:
            return
        self._posx_req_in_flight = True
        self._posx_req_sent_time = time.time()
        future = self._get_current_posx_client.call_async(GetCurrentPosx.Request())
        future.add_done_callback(self._on_posx_response)

    def _on_posx_response(self, future):
        self._posx_req_in_flight = False
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f"get_current_posx failed: {e}")
            return
        if result.success and result.task_pos_info:
            self._latest_posx = list(result.task_pos_info[0].data[:6])

    def _compute_posx(self):
        """get_current_posx 서비스의 캐시값을 반환함."""
        self._poll_posx()
        return self._latest_posx

    def _pose_poll_tick(self):
        """웨이포인트 도착을 판정하고 스냅샷을 저장함."""
        if not self._recon_active:
            return
        posx = self._compute_posx()
        if posx is None:
            self.get_logger().warn(
                "_compute_posx 실패 — get_current_posx 응답 아직 없음", throttle_duration_sec=1.0)
            return

        for i, wp in enumerate(self._goal_waypoints):
            if self._visited[i]:
                continue

            if self._arrival_times[i] is None:
                if self._pending_since[i] is None:
                    self._pending_since[i] = time.time()
                pending_elapsed = time.time() - self._pending_since[i]
                if pending_elapsed >= config.WAYPOINT_ARRIVAL_TIMEOUT_SEC:
                    self.get_logger().warn(
                        f"waypoint {i} 도착 확인 {config.WAYPOINT_ARRIVAL_TIMEOUT_SEC}s 안에 못 함 "
                        f"(mover가 이미 다음 웨이포인트로 넘어갔을 가능성) — 스냅샷 없이 건너뜀")
                    self._visited[i] = True
                    break
                pos_err = np.linalg.norm(np.array(posx[:3]) - np.array([wp.x, wp.y, wp.z]))
                ok = pos_err <= config.POSITION_TOLERANCE_MM
                ori_err = None
                if wp.has_orientation:
                    ori_err = transforms.rotation_geodesic_error_deg(
                        posx[3], posx[4], posx[5], wp.rx, wp.ry, wp.rz)
                    ok = ok and ori_err <= config.ORIENTATION_TOLERANCE_DEG
                if pending_elapsed >= 2.0:
                    self.get_logger().info(
                        f"waypoint {i} pending {pending_elapsed:.1f}s: pos_err={pos_err:.1f}mm "
                        f"ori_err={f'{ori_err:.1f}deg' if ori_err is not None else 'n/a'} ok={ok}",
                        throttle_duration_sec=1.0)
                self._dwell_counts[i] = self._dwell_counts[i] + 1 if ok else 0
                if self._dwell_counts[i] >= config.DWELL_CHECKS:
                    self._arrival_times[i] = time.time()
                    self.get_logger().info(
                        f"waypoint {i} 근접 확인(pos_err={pos_err:.1f}mm) — "
                        f"{config.WAYPOINT_DWELL_SEC}s 정착 대기 시작")
            else:
                elapsed = time.time() - self._arrival_times[i]
                if not self._snapshot_taken[i] and elapsed >= config.SNAPSHOT_DELAY_SEC:
                    self._take_snapshot(i, posx, arrival_ts=self._arrival_times[i])
                    self._snapshot_taken[i] = True
                if elapsed >= config.WAYPOINT_DWELL_SEC:
                    self._visited[i] = True
            break

    def _start_photo_map_build(self):
        """정찰 완료 직후 실사맵."""
        if not config.PHOTO_MAP_ENABLED or not self._snapshot_run_dir:
            return
        folder_name = os.path.basename(self._snapshot_run_dir)

        def _worker():
            cmd = [sys.executable, config.PHOTO_MAP_SCRIPT, folder_name,
                   "--color-viewer", "--wps", config.PHOTO_MAP_WPS]
            try:
                self.get_logger().info(f"실사맵 생성 시작(배경): {folder_name} — 10~30초 소요")
                r = subprocess.run(cmd, capture_output=True, text=True,
                                   timeout=config.PHOTO_MAP_TIMEOUT_SEC)
                if r.returncode == 0:
                    self.get_logger().info(f"실사맵 생성 완료 — 웹 '실사맵 보기'로 확인 가능 ({folder_name})")
                else:
                    self.get_logger().warn(
                        f"실사맵 생성 실패(rc={r.returncode}): {r.stderr.strip()[-300:]}")
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"실사맵 생성이 {config.PHOTO_MAP_TIMEOUT_SEC:.0f}초 초과 — 포기")
            except Exception:
                self.get_logger().warn(f"실사맵 생성 중 예외:\n{traceback.format_exc()}")

        threading.Thread(target=_worker, daemon=True).start()

    def _median_depth(self, since_ts=None):
        """롤링 버퍼의 depth 프레임들을 픽셀별 중앙값으로 합침."""
        frames = [d for ts, d in self._depth_buffer if since_ts is None or ts >= since_ts]
        if len(frames) < config.DEPTH_MEDIAN_MIN_FRAMES:
            self.get_logger().warn(
                f"depth 중앙값 포기 — 정지 후 프레임 {len(frames)}장뿐"
                f"(최소 {config.DEPTH_MEDIAN_MIN_FRAMES}장 필요). 최신 1장으로 진행")
            return self.latest_depth.copy(), 1

        med = transforms.pixelwise_median_depth(frames)
        return med.astype(self.latest_depth.dtype), len(frames)

    def _take_snapshot(self, waypoint_idx, posx, arrival_ts=None):
        if self.latest_color is None or self.latest_depth is None:
            self.get_logger().warn("snapshot skipped: no frame yet")
            return
        checker_T = self._try_checkerboard(self.latest_color, posx)
        depth_med, n_frames = self._median_depth(arrival_ts)
        self._snapshots.append({
            'waypoint_idx': waypoint_idx,
            'color': self.latest_color.copy(),
            'depth': depth_med,
            'posx': list(posx),
            'joint_state': dict(self._latest_joint_state) if self._latest_joint_state else None,
            'checker_T_checker2base': checker_T,
        })
        image_path = os.path.join(self._snapshot_run_dir, f"waypoint_{waypoint_idx}.jpg")
        cv2.imwrite(image_path, self.latest_color)
        depth_path = os.path.join(self._snapshot_run_dir, f"waypoint_{waypoint_idx}_depth.npy")
        np.save(depth_path, depth_med)
        meta_path = os.path.join(self._snapshot_run_dir, f"waypoint_{waypoint_idx}_meta.json")
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump({
                'waypoint_idx': waypoint_idx,
                'posx_mm_deg': list(posx[:6]),
                'intrinsics': self.intrinsics,
                'joint_state': dict(self._latest_joint_state) if self._latest_joint_state else None,
                'depth_median_frames': n_frames,
            }, f, ensure_ascii=False, indent=2)
        self.get_logger().info(
            f"snapshot taken at waypoint {waypoint_idx} ({len(self._snapshots)} total) — saved to {image_path}"
            f" (+depth npy [{n_frames}장 중앙값], +meta json)")
        js = self._snapshots[-1]['joint_state']
        self.get_logger().info(
            f"SNAPSHOT_POSE waypoint={waypoint_idx} (get_current_posx 서비스값, "
            f"joint_rad는 FK 대조/디버깅 참고용) "
            f"posx_mm_deg={[round(v, 3) for v in posx[:6]]} "
            f"joint_names={js['name'] if js else None} "
            f"joint_rad={[round(v, 5) for v in js['position']] if js else None}")
        self._snapshot_done_pub.publish(Int32(data=waypoint_idx))

    def _try_checkerboard(self, color_bgr, posx):
        if self.intrinsics is None:
            return None
        camera_matrix = np.array([
            [self.intrinsics['fx'], 0, self.intrinsics['ppx']],
            [0, self.intrinsics['fy'], self.intrinsics['ppy']],
            [0, 0, 1],
        ])
        dist_coeffs = np.zeros(5)
        R, t = transforms.find_checkerboard_pose(color_bgr, camera_matrix, dist_coeffs)
        if R is None:
            return None
        return transforms.checker_to_base(R, t, self.T_gripper2camera, posx)

    async def _execute_run_recon(self, goal_handle):
        try:
            return await self._run_recon_body(goal_handle)
        except Exception:
            self._recon_active = False
            self.get_logger().error("RunRecon 중 예외 발생:\n" + traceback.format_exc())
            goal_handle.abort()
            result = RunRecon.Result()
            result.success = False
            result.objects = []
            return result

    async def _run_recon_body(self, goal_handle):
        self.get_logger().info("RunRecon goal received — starting recon")
        self._goal_waypoints = goal_handle.request.waypoints
        self._visited = [False] * len(self._goal_waypoints)
        self._dwell_counts = [0] * len(self._goal_waypoints)
        self._arrival_times = [None] * len(self._goal_waypoints)
        self._pending_since = [None] * len(self._goal_waypoints)
        self._snapshot_taken = [False] * len(self._goal_waypoints)
        self._snapshots = []
        self._recon_active = True

        self._snapshot_run_dir = os.path.join(
            config.SNAPSHOT_SAVE_DIR, time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self._snapshot_run_dir, exist_ok=True)

        feedback = RunRecon.Feedback()
        while not all(self._visited):
            feedback.stage = "accumulating"
            feedback.snapshots_taken = len(self._snapshots)
            feedback.snapshots_total = len(self._goal_waypoints)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)

        feedback.stage = "accumulating"
        feedback.snapshots_taken = len(self._snapshots)
        feedback.snapshots_total = len(self._goal_waypoints)
        goal_handle.publish_feedback(feedback)

        self._recon_active = False
        self.get_logger().info(f"all {len(self._goal_waypoints)} waypoints visited — running YOLO batch")

        feedback.stage = "yolo_batch"
        goal_handle.publish_feedback(feedback)
        detections = self._run_yolo_batch()
        static_obstacle_points = self._extract_static_obstacle_points_mm()

        feedback.stage = "scene_map"
        goal_handle.publish_feedback(feedback)
        self._build_and_publish_scene_map()
        self._start_photo_map_build()

        feedback.stage = "clustering"
        goal_handle.publish_feedback(feedback)
        objects = self._cluster_and_fuse(detections)

        feedback.stage = "mask_refine"
        goal_handle.publish_feedback(feedback)
        refined_objects = self._refine_coords_with_masks(detections, objects)
        if refined_objects is not None:
            self.get_logger().info(
                f"bbox 기반 좌표(참고용, 그대로 둠): "
                f"{[(o.object_id, [round(v, 1) for v in o.coords_base]) for o in objects]}")
            objects = refined_objects

        feedback.stage = "done"
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        result = RunRecon.Result()
        result.success = True
        result.objects = objects
        result.raw_detections = self._to_raw_detection_msgs(detections)
        result.static_obstacle_points = [
            Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in static_obstacle_points
        ]
        self.get_logger().info(
            f"recon finalized: {len(objects)} objects, "
            f"{len(result.static_obstacle_points)} static obstacle points")
        return result

    def _to_raw_detection_msgs(self, detections):
        """클러스터링 전 원시 탐지를 그대로 Result에 실어 보냄."""
        msgs = []
        for d in detections:
            msg = RawDetection()
            msg.waypoint_idx = int(d['waypoint_idx'])
            msg.object_class = d['class']
            msg.confidence = float(d['confidence'])
            msg.coords_base = [float(v) for v in d['coords_base']]
            msgs.append(msg)
        return msgs

    def _ensure_yolo_loaded(self):
        if self._yolo_model is not None:
            return
        if not os.path.exists(config.YOLO_MODEL_PATH):
            self.get_logger().error(
                f"YOLO model not found at {config.YOLO_MODEL_PATH} — 학습 코드/가중치는 별도 준비 예정")
            return
        from ultralytics import YOLO
        self._yolo_model = YOLO(config.YOLO_MODEL_PATH)
        self._yolo_model.to(config.YOLO_DEVICE)

    def _run_yolo_batch(self):
        """정찰 완료 후 딱 한 번."""
        self._ensure_yolo_loaded()
        detections = []
        self._snapshot_boxes = {}
        if self._yolo_model is None or not self._snapshots:
            return detections

        images = [s['color'] for s in self._snapshots]
        results = self._yolo_model(images, verbose=False, device=config.YOLO_DEVICE)

        for snap, res in zip(self._snapshots, results):
            annotated = snap['color'].copy()
            boxes_this_snap = []
            for box, score, cls_idx in zip(
                res.boxes.xyxy.tolist(), res.boxes.conf.tolist(), res.boxes.cls.tolist()
            ):
                if score < config.YOLO_CONF_THRESHOLD:
                    continue
                boxes_this_snap.append(box)
                x1, y1, x2, y2 = [int(round(v)) for v in box]
                depth = snap['depth']
                frame_h, frame_w = depth.shape[:2]
                m = config.BBOX_EDGE_MARGIN_PX
                clipped = x1 <= m or y1 <= m or x2 >= frame_w - m or y2 >= frame_h - m
                box_color = (0, 165, 255) if clipped else (0, 255, 0)
                label = f"{res.names[int(cls_idx)]} {score:.2f}" + (" [clipped-excluded]" if clipped else "")
                cv2.rectangle(annotated, (x1, y1), (x2, y2), box_color, 2)
                cv2.putText(
                    annotated, label, (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                if clipped:
                    self.get_logger().warn(
                        f"waypoint={snap['waypoint_idx']} class={res.names[int(cls_idx)]} "
                        f"bbox가 프레임 경계에 닿음(box={[round(v,1) for v in box]}, frame={frame_w}x{frame_h}) "
                        f"— 좌표 계산에서 제외")
                    continue
                cam_pt = transforms.backproject_bbox_median(depth, box, self.intrinsics)
                if cam_pt is None:
                    continue
                base_pt = transforms.camera_to_base(cam_pt, self.T_gripper2camera, snap['posx'])
                class_name = res.names[int(cls_idx)]
                detections.append({
                    'class': class_name,
                    'confidence': float(score),
                    'coords_base': base_pt,
                    'box': box,
                    'waypoint_idx': snap['waypoint_idx'],
                })
                self.get_logger().info(
                    f"RAW_DETECTION waypoint={snap['waypoint_idx']} class={class_name} "
                    f"confidence={score:.3f} coords_cam_mm=[{cam_pt[0]:.1f}, {cam_pt[1]:.1f}, {cam_pt[2]:.1f}] "
                    f"coords_base_mm=[{base_pt[0]:.1f}, {base_pt[1]:.1f}, {base_pt[2]:.1f}] "
                    f"posx_mm_deg={[round(v, 2) for v in snap['posx'][:6]]}")
            annotated_path = os.path.join(
                self._snapshot_run_dir, f"waypoint_{snap['waypoint_idx']}_yolo.jpg")
            cv2.imwrite(annotated_path, annotated)
            self._snapshot_boxes[snap['waypoint_idx']] = boxes_this_snap
        return detections

    def _ensure_mask_model_loaded(self):
        if self._mask_model is not None:
            return
        if not os.path.exists(config.MASK_MODEL_PATH):
            self.get_logger().error(f"Mask R-CNN model not found at {config.MASK_MODEL_PATH}")
            return
        import torch
        from detectron2 import model_zoo
        from detectron2.config import get_cfg
        from detectron2.modeling import build_model
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml'))
        cfg.MODEL.ROI_HEADS.NUM_CLASSES = config.MASK_MODEL_NUM_CLASSES
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = config.MASK_CONF_THRESHOLD
        cfg.MODEL.DEVICE = config.MASK_DEVICE
        model = build_model(cfg)
        ckpt = torch.load(config.MASK_MODEL_PATH, map_location=config.MASK_DEVICE, weights_only=False)
        model.load_state_dict(ckpt['model'])
        model.eval()
        self._mask_model = model

    def _run_mask_batch(self):
        """스냅샷마다 Mask R-CNN 부위 분할 실행."""
        self._ensure_mask_model_loaded()
        results = {}
        if self._mask_model is None:
            return results
        import torch
        with torch.no_grad():
            for snap in self._snapshots:
                img = snap['color']
                h, w = img.shape[:2]
                image_tensor = torch.as_tensor(img.astype('float32').transpose(2, 0, 1))
                outputs = self._mask_model([{"image": image_tensor, "height": h, "width": w}])[0]
                instances = outputs["instances"].to("cpu")
                boxes = (instances.pred_boxes.tensor.numpy()
                         if instances.has("pred_boxes") else None)
                parts = []
                for i in range(len(instances)):
                    mask = instances.pred_masks[i].numpy()
                    score = float(instances.scores[i])
                    box = tuple(float(v) for v in boxes[i]) if boxes is not None else None
                    sam2_applied = False
                    if (config.ZERO_SHOT_FALLBACK_ENABLED and box is not None
                            and score < config.TWO_STAGE_SAM2_CONF_THR):
                        refined = self._refine_mask_with_sam2(img, box)
                        if refined is not None:
                            mask, sam2_applied = refined, True
                    parts.append({
                        'class_idx': int(instances.pred_classes[i]),
                        'mask': mask,
                        'score': score,
                        'box': box,
                        'sam2_applied': sam2_applied,
                    })
                results[snap['waypoint_idx']] = parts
        return results

    def _ensure_zero_shot(self):
        """Zero-shot 분할기를 처음 필요한 순간에만 만든다."""
        if self._zero_shot is None:
            from yolo_detect.zero_shot import ZeroShotSegmenter
            self._zero_shot = ZeroShotSegmenter(log=self.get_logger().info)
        return self._zero_shot

    def _refine_mask_with_sam2(self, frame_bgr, box):
        """Mask R-CNN confidence가 낮은 인스턴스의 경계를 SAM2로 보정한다."""
        try:
            mask, _iou = self._ensure_zero_shot().mask_from_box(frame_bgr, box)
            return mask
        except Exception:
            self.get_logger().warn("SAM2 경계 보정 실패 — RCNN 마스크를 그대로 쓴다:\n"
                                   + traceback.format_exc())
            return None

    def _zero_shot_parts(self, obj, detections, snap_by_wp):
        """학습 경로가 부위를 못 찾은 물체에만 GDINO+SAM2로 부위를 찾는다."""
        if not config.ZERO_SHOT_FALLBACK_ENABLED or not detections:
            return []
        best_det = max(detections, key=lambda d: d['confidence'])
        snap = snap_by_wp.get(best_det['waypoint_idx'])
        if snap is None:
            return []
        self.get_logger().warn(
            f"{obj.object_id}({obj.object_class}): 학습 경로가 부위를 못 찾음 — "
            f"zero-shot 폴백(waypoint {best_det['waypoint_idx']})")
        try:
            segmenter = self._ensure_zero_shot()
        except Exception:
            self.get_logger().error("zero-shot 분할기 초기화 실패:\n" + traceback.format_exc())
            return []

        found = []
        for part in sorted(config.TWO_STAGE_PART_CLASSES):
            try:
                got = segmenter.detect_part(snap['color'], part, obj.object_class)
            except Exception:
                self.get_logger().warn(f"zero-shot '{part}' 실패:\n" + traceback.format_exc())
                continue
            if got is None:
                continue
            found.append((best_det['waypoint_idx'], config.TWO_STAGE_CLASS_NAMES.index(part),
                          got['mask'], got['score'], 1.0))
        self.get_logger().info(
            f"{obj.object_id}: zero-shot 부위 {len(found)}개 확보 "
            f"({[config.TWO_STAGE_CLASS_NAMES[f[1]] for f in found]})")
        return found

    def _refine_coords_with_masks(self, detections, objects):
        """YOLO bbox와 겹치는 Mask R-CNN 부위 인스턴스를 찾아 매칭한 뒤, 가장 자주 검출된."""
        if not detections or not objects:
            return None
        if config.RECON_TARGET_CLASS:
            want = str(config.RECON_TARGET_CLASS).strip().lower()
            kept = [d for d in detections if str(d['class']).strip().lower() == want]
            dropped = len(detections) - len(kept)
            if not kept:
                self.get_logger().warn(
                    f"회수 대상 클래스 '{want}' 검출 0건 — 있는 클래스: "
                    f"{sorted({d['class'] for d in detections})}. bbox 기반 좌표로 폴백")
                return None
            if dropped:
                self.get_logger().info(
                    f"회수 대상 '{want}'만 사용 — 다른 클래스 검출 {dropped}건 제외"
                    f"(그 물체들은 옥토맵에 남아 장애물로 회피된다)")
            detections = kept
        mask_results = self._run_mask_batch()
        if not any(mask_results.values()):
            self.get_logger().warn("Mask R-CNN 결과 없음 — bbox 기반 좌표로 폴백")
            return None
        snap_by_wp = {s['waypoint_idx']: s for s in self._snapshots}

        by_object = self._assign_detections_to_objects(detections, objects)
        refined_objects, any_refined = [], False
        for obj in objects:
            dets = by_object.get(obj.object_id, [])
            refined = (self._refine_one_object(obj, dets, mask_results, snap_by_wp)
                       if dets else None)
            if refined is None:
                refined_objects.append(obj)
            else:
                refined_objects.append(refined)
                any_refined = True
        if not any_refined:
            self.get_logger().warn("어느 물체도 부위 마스크와 매칭되지 않음 — bbox 기반 좌표로 폴백")
            return None
        return refined_objects

    def _assign_detections_to_objects(self, detections, objects):
        """검출 하나하나를 어느 물체의 것인지로 되돌림."""
        by_object = {o.object_id: [] for o in objects}
        for d in detections:
            same_class = [o for o in objects if o.object_class == d['class']]
            if not same_class:
                continue
            det_pt = np.asarray(d['coords_base'], dtype=np.float64)
            nearest = min(same_class, key=lambda o: float(np.linalg.norm(
                det_pt - np.asarray(o.coords_base, dtype=np.float64))))
            by_object[nearest.object_id].append(d)
        summary = {oid: len(v) for oid, v in by_object.items() if v}
        self.get_logger().info(f"검출을 물체별로 분리: {summary}")
        return by_object

    def _refine_one_object(self, obj, detections, mask_results, snap_by_wp):
        """물체 하나에 대해 부위 마스크로 좌표를 다시 계산하고 부위 점군을 저장함."""
        matches = []
        overlapping = []
        for d in detections:
            parts = mask_results.get(d['waypoint_idx'], [])
            x1, y1, x2, y2 = d['box']
            best_part, best_overlap = None, 0.0
            for part in parts:
                mask = part['mask']
                mask_pixel_count = mask.sum()
                if mask_pixel_count == 0:
                    continue
                box_mask = np.zeros_like(mask)
                bx1, by1 = max(int(x1), 0), max(int(y1), 0)
                bx2, by2 = min(int(x2), mask.shape[1]), min(int(y2), mask.shape[0])
                box_mask[by1:by2, bx1:bx2] = True
                overlap = float((mask & box_mask).sum()) / float(mask_pixel_count)
                if overlap >= 0.3:
                    overlapping.append((d['waypoint_idx'], part['class_idx'], mask,
                                        part['score'], overlap))
                if overlap > best_overlap:
                    best_part, best_overlap = part, overlap
            if best_part is not None and best_overlap >= 0.3:
                matches.append((d['waypoint_idx'], best_part['class_idx'],
                                best_part['mask'], float(best_part['score'])))

        if not matches:
            zero_shot = self._zero_shot_parts(obj, detections, snap_by_wp)
            if not zero_shot:
                self.get_logger().warn(
                    f"{obj.object_id}({obj.object_class}): 겹치는 부위 마스크 없음 — bbox 좌표 유지")
                return None
            overlapping.extend(zero_shot)
            matches = [(wp, cls, mask, score) for wp, cls, mask, score, _ov in zero_shot]

        class_counts, class_scores = {}, {}
        for _, class_idx, _, score in matches:
            class_counts[class_idx] = class_counts.get(class_idx, 0) + 1
            class_scores[class_idx] = class_scores.get(class_idx, 0.0) + score
        majority_class = max(class_counts,
                             key=lambda k: (class_counts[k], class_scores[k]))
        self.get_logger().info(
            f"{obj.object_id}({obj.object_class}) 부위 클래스별 검출 횟수: {class_counts} "
            f"— 최다 클래스 {majority_class} 채택")

        refined_points, refined_confidences = [], []
        for waypoint_idx, class_idx, mask, _score in matches:
            if class_idx != majority_class:
                continue
            snap = snap_by_wp[waypoint_idx]
            cam_pt = transforms.backproject_mask_median(snap['depth'], mask, self.intrinsics)
            if cam_pt is None:
                continue
            base_pt = transforms.camera_to_base(cam_pt, self.T_gripper2camera, snap['posx'])
            refined_points.append(base_pt)
            self.get_logger().info(
                f"MASK_REFINED {obj.object_id} waypoint={waypoint_idx} class_idx={class_idx} "
                f"coords_base_mm=[{base_pt[0]:.1f}, {base_pt[1]:.1f}, {base_pt[2]:.1f}]")

        if not refined_points:
            return None

        pts = np.array(refined_points)
        centroid = pts.mean(axis=0)
        spread = np.linalg.norm(pts - centroid, axis=1)
        self.get_logger().info(
            f"부위 기반 재계산 {len(pts)}개 — 편차(mm) min={spread.min():.1f} "
            f"max={spread.max():.1f} RMS={np.sqrt((spread**2).mean()):.1f} "
            f"(참고: bbox 기반은 웨이포인트 간 최대 수백mm 산포였음)")

        refined = DetectedObject()
        refined.object_id = obj.object_id
        refined.object_class = obj.object_class
        refined.coords_base = [float(v) for v in centroid]
        refined.confidence = float(np.mean([d['confidence'] for d in detections]))
        obj = refined

        try:
            self._save_part_clouds(overlapping, snap_by_wp, obj.object_id, obj.object_class)
        except Exception:
            self.get_logger().warn("부위 점군 저장 실패(정찰 결과에는 영향 없음):\n"
                                   + traceback.format_exc())
        return obj

    def _save_part_clouds(self, overlapping, snap_by_wp, object_id, object_class):
        """부위별 점군을 정찰 시점에 확보해 저장함."""
        if not overlapping or not self._snapshot_run_dir:
            return None
        best = {}
        for wp_idx, class_idx, mask, score, overlap in overlapping:
            name = (config.TWO_STAGE_CLASS_NAMES[class_idx]
                    if 0 <= class_idx < len(config.TWO_STAGE_CLASS_NAMES) else str(class_idx))
            if name not in config.TWO_STAGE_PART_CLASSES:
                continue
            if name not in best or score > best[name][3]:
                best[name] = (wp_idx, class_idx, mask, score, overlap)

        saved, arrays = {}, {}
        for name, (wp_idx, _class_idx, mask, score, overlap) in sorted(best.items()):
            snap = snap_by_wp.get(wp_idx)
            if snap is None:
                continue
            cam_pts = transforms.backproject_mask_points(
                snap['depth'], mask, self.intrinsics,
                stride=config.TWO_STAGE_POINT_STRIDE,
                z_min=config.TWO_STAGE_DEPTH_Z_MIN_MM, z_max=config.TWO_STAGE_DEPTH_Z_MAX_MM)
            if len(cam_pts) < 10:
                self.get_logger().warn(f"부위 '{name}' 유효 depth {len(cam_pts)}점 — 저장 생략")
                continue
            base_pts = transforms.camera_to_base_batch(
                cam_pts, self.T_gripper2camera, snap['posx'])
            arrays[f"points_{name}"] = base_pts.astype(np.float32)
            saved[name] = {"waypoint_idx": int(wp_idx), "score": float(score),
                           "overlap": float(overlap), "num_points": int(len(base_pts)),
                           "posx_mm_deg": [float(v) for v in snap['posx'][:6]],
                           "centroid_mm": [float(v) for v in base_pts.mean(axis=0)]}
            self.get_logger().info(
                f"PART_CLOUD '{name}' wp={wp_idx} score={score:.2f} {len(base_pts)}점 "
                f"중심=[{saved[name]['centroid_mm'][0]:.1f}, "
                f"{saved[name]['centroid_mm'][1]:.1f}, {saved[name]['centroid_mm'][2]:.1f}]")
        if not saved:
            return None
        path = os.path.join(self._snapshot_run_dir, f"parts_{object_id}.npz")
        np.savez_compressed(path, meta=json.dumps(
            {"object_id": object_id, "object_class": object_class,
             "sam2_applied": False, "parts": saved}), **arrays)
        self.get_logger().info(f"부위 점군 저장: {path} (부위 {len(saved)}개: {list(saved)})")
        return path

    def _extract_static_obstacle_points_mm(self):
        """스냅샷 depth에서 YOLO bbox 바깥 픽셀만 역투영해 정적 장애물 후보를 만듦."""
        if self.intrinsics is None:
            return []
        fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
        ppx, ppy = self.intrinsics['ppx'], self.intrinsics['ppy']
        stride = config.MAP_OBSTACLE_PIXEL_STRIDE

        all_base_pts = []
        for snap in self._snapshots:
            depth = snap['depth']
            h, w = depth.shape
            boxes = self._snapshot_boxes.get(snap['waypoint_idx'], [])

            grid_x, grid_y = np.meshgrid(
                np.arange(0, w, stride), np.arange(0, h, stride))
            keep = np.ones(grid_x.shape, dtype=bool)
            for (x1, y1, x2, y2) in boxes:
                inside = (grid_x >= x1) & (grid_x <= x2) & (grid_y >= y1) & (grid_y <= y2)
                keep &= ~inside

            d = depth[grid_y, grid_x].astype(np.float32)
            valid = keep & (d > config.MAP_OBSTACLE_NEAR_CLIP_MM)
            if not np.any(valid):
                continue
            u, v, dv = grid_x[valid].astype(np.float32), grid_y[valid].astype(np.float32), d[valid]
            cam_pts = np.stack([(u - ppx) * dv / fx, (v - ppy) * dv / fy, dv], axis=1)

            base2gripper = transforms.robot_pose_to_matrix(*snap['posx'][:6])
            base2cam = base2gripper @ self.T_gripper2camera
            cam_pts_h = np.hstack([cam_pts, np.ones((len(cam_pts), 1))])
            base_pts = (base2cam @ cam_pts_h.T).T[:, :3]
            all_base_pts.append(base_pts)

        if not all_base_pts:
            return []
        merged = np.concatenate(all_base_pts, axis=0)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(merged)
        pcd = pcd.voxel_down_sample(voxel_size=config.MAP_OBSTACLE_VOXEL_SIZE_MM)
        points = np.asarray(pcd.points)
        self.get_logger().info(f"static obstacle points extracted: {len(points)}")
        return points.tolist()

    def _build_and_publish_scene_map(self):
        """정찰 완료 후 한 번만, 스냅샷들로 3D 씬을 재구성해 웹으로 발행함."""
        try:
            if self.intrinsics is None:
                return
            fx, fy = self.intrinsics['fx'], self.intrinsics['fy']
            ppx, ppy = self.intrinsics['ppx'], self.intrinsics['ppy']
            stride = config.MAP_WEB_PIXEL_STRIDE

            all_pts, all_colors = [], []
            for snap in self._snapshots:
                depth = snap['depth']
                color_bgr = snap['color']
                h, w = depth.shape

                grid_x, grid_y = np.meshgrid(
                    np.arange(0, w, stride), np.arange(0, h, stride))
                d = depth[grid_y, grid_x].astype(np.float32)
                valid = ((d > config.MAP_OBSTACLE_NEAR_CLIP_MM)
                         & (d < config.MAP_WEB_MAX_DEPTH_MM))
                if not np.any(valid):
                    continue
                u, v, dv = (grid_x[valid].astype(np.float32),
                            grid_y[valid].astype(np.float32), d[valid])
                cam_pts = np.stack([(u - ppx) * dv / fx, (v - ppy) * dv / fy, dv], axis=1)

                base2gripper = transforms.robot_pose_to_matrix(*snap['posx'][:6])
                base2cam = base2gripper @ self.T_gripper2camera
                cam_pts_h = np.hstack([cam_pts, np.ones((len(cam_pts), 1))])
                base_pts = (base2cam @ cam_pts_h.T).T[:, :3]

                bgr01 = color_bgr[grid_y[valid], grid_x[valid]].astype(np.float32) / 255.0
                rgb01 = bgr01[:, ::-1]

                all_pts.append(base_pts)
                all_colors.append(rgb01)

            if not all_pts:
                self.get_logger().warn("scene map: 유효 점 없음 — 발행 건너뜀")
                return

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.concatenate(all_pts, axis=0))
            pcd.colors = o3d.utility.Vector3dVector(np.concatenate(all_colors, axis=0))
            pcd = pcd.voxel_down_sample(voxel_size=config.MAP_WEB_VOXEL_SIZE_MM)
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)

            if len(points) > config.MAP_BROADCAST_MAX_POINTS:
                idx = np.random.choice(len(points), config.MAP_BROADCAST_MAX_POINTS, replace=False)
                points, colors = points[idx], colors[idx]

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "base_link"
            msg = transforms.build_colored_pointcloud2(header, points, colors)
            self._map_pub.publish(msg)
            self.get_logger().info(f"scene map published: {len(points)}점 (정찰 완료 후 1회)")
        except Exception:
            self.get_logger().error(
                f"scene map 생성/발행 실패(시각화 부가기능이라 정찰 결과엔 영향 없음): "
                f"{traceback.format_exc()}")

    def _cluster_and_fuse(self, detections):
        """3D 거리로 검출을 묶고 YOLO confidence 가중평균으로 좌표를 냄."""
        objects = []
        if not detections:
            return objects

        by_class = {}
        for d in detections:
            by_class.setdefault(d['class'], []).append(d)

        obj_idx = 1
        for cls_name, dets in by_class.items():
            points = [d['coords_base'] for d in dets]
            weights = [d['confidence'] for d in dets]
            clusters = transforms.cluster_points(points, weights, threshold_mm=config.CLUSTER_THRESHOLD_MM)
            for centroid, confidence in clusters:
                obj = DetectedObject()
                obj.object_id = f"obj_{obj_idx:03d}"
                obj.object_class = cls_name
                obj.coords_base = [float(v) for v in centroid]
                obj.confidence = confidence
                objects.append(obj)
                obj_idx += 1
        return objects

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
