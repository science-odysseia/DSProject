"""정찰 스냅샷 폴더로부터 3D 맵을 재구성함."""
import argparse
import glob
import json
import os
import re
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np

_KOREAN_FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if os.path.exists(_KOREAN_FONT_PATH):
    fm.fontManager.addfont(_KOREAN_FONT_PATH)
    matplotlib.rcParams["font.family"] = fm.FontProperties(fname=_KOREAN_FONT_PATH).get_name()

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from yolo_detect import config, transforms

SNAPSHOTS_DIR = os.path.join(os.path.dirname(__file__), "recon_results", "snapshots")
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "recon_results", "octree_preview")
COLOR_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "recon_results", "color_viewer")

COLOR_STRIDE = 2
COLOR_MAX_DEPTH_MM = 1500.0
COLOR_POINT_SIZE = 3
ICP_VOXEL_MM = 5.0
ICP_MAX_CORR_MM = 25.0

SYNTHETIC_DEPTH_MM = 800.0
SYNTHETIC_INTRINSICS = {"fx": 915.0, "fy": 915.0, "ppx": 640.0, "ppy": 360.0}
SYNTHETIC_IMAGE_SHAPE = (720, 1280)

def _load_gripper2camera():
    return np.load(config.T_GRIPPER2CAMERA_PATH)

def _load_waypoint_data(folder, waypoint_idx):
    """웨이포인트 하나의 depth, posx, intrinsics를 읽음."""
    depth_path = os.path.join(folder, f"waypoint_{waypoint_idx}_depth.npy")
    meta_path = os.path.join(folder, f"waypoint_{waypoint_idx}_meta.json")

    if os.path.exists(depth_path) and os.path.exists(meta_path):
        depth = np.load(depth_path)
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        return depth.astype(np.float32), meta["posx_mm_deg"], meta["intrinsics"], False

    if waypoint_idx >= len(config.RECON_WAYPOINTS):
        return None, None, None, True
    wp = config.RECON_WAYPOINTS[waypoint_idx]
    posx = [wp["x"], wp["y"], wp["z"], wp["rx"], wp["ry"], wp["rz"]]
    depth = np.full(SYNTHETIC_IMAGE_SHAPE, SYNTHETIC_DEPTH_MM, dtype=np.float32)
    return depth, posx, SYNTHETIC_INTRINSICS, True

def _available_waypoints(folder):
    """폴더에 실제로 있는 웨이포인트 번호를 정렬해 돌려줌."""
    idxs = []
    for path in glob.glob(os.path.join(folder, "waypoint_*.jpg")):
        m = re.fullmatch(r"waypoint_(\d+)\.jpg", os.path.basename(path))
        if m:
            idxs.append(int(m.group(1)))
    return sorted(idxs)

def _unproject_to_base(depth_mm, posx, intrinsics, T_gripper2camera, stride=4,
                       max_depth_mm=None):
    """depth 이미지 전체를 카메라 프레임 3D 점으로 역투영한 뒤 base_link로 변환."""
    h, w = depth_mm.shape
    us, vs = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
    d = depth_mm[vs, us].astype(np.float32)
    valid = d > 0
    if max_depth_mm is not None:
        valid = valid & (d < max_depth_mm)
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    ppx, ppy = intrinsics["ppx"], intrinsics["ppy"]
    xs = (us - ppx) * d / fx
    ys = (vs - ppy) * d / fy
    cam_pts = np.stack([xs[valid], ys[valid], d[valid]], axis=1)

    base2gripper = transforms.robot_pose_to_matrix(*posx)
    base2cam = base2gripper @ T_gripper2camera
    cam_pts_h = np.concatenate([cam_pts, np.ones((len(cam_pts), 1))], axis=1)
    base_pts = (base2cam @ cam_pts_h.T).T[:, :3]
    return base_pts

def _voxelize(points_mm, voxel_mm):
    """단순 voxel-grid 다운샘플."""
    idx = np.floor(points_mm / voxel_mm).astype(np.int64)
    unique_idx = np.unique(idx, axis=0)
    return (unique_idx.astype(np.float64) + 0.5) * voxel_mm

def _generate_obstacle_points(points_mm, target_obj_mm, voxel_mm, mask_radius_mm):
    """저장된 정찰 점군 전체를 접근계획용 장애물점으로 변환."""
    voxels = _voxelize(points_mm, voxel_mm)
    d = np.linalg.norm(voxels - np.asarray(target_obj_mm, dtype=np.float64), axis=1)
    keep = d > mask_radius_mm
    return voxels[keep], int((~keep).sum())

def _seed_joints_from_snapshot(folder):
    """스냅샷 meta의 joint_state를 수치 IK seed용 dict."""
    for idx in range(len(config.RECON_WAYPOINTS) - 1, -1, -1):
        meta_path = os.path.join(folder, f"waypoint_{idx}_meta.json")
        if not os.path.exists(meta_path):
            continue
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
        js = meta.get("joint_state")
        if js and js.get("name") and js.get("position"):
            return dict(zip(js["name"], js["position"])), idx
    return None, None

def _verify_reorientation(planner, folder, arrival_posx, target_obj_mm, T_gripper2camera):
    """도착 후 제자리 재조정의 사전 충돌검사를 오프라인에서 재현함."""
    try:
        rx, ry, rz = transforms.look_at_orientation(arrival_posx[:3], target_obj_mm, T_gripper2camera)
    except ValueError as e:
        print("-" * 64)
        print(f"②단계 제자리 재조정 사전검사 생략 — 카메라-지향 자세가 존재하지 않음: {e}")
        return
    target_reorient = [*arrival_posx[:3], rx, ry, rz]
    total_deg = transforms.rotation_geodesic_error_deg(*arrival_posx[3:6], rx, ry, rz)
    print("-" * 64)
    n_samples = planner._resolve_sample_count(
        config.REORIENT_VALIDATION_SAMPLES, rot_deg=total_deg)
    print(f"②단계 제자리 재조정 사전검사 (회전량 {total_deg:.1f}°, 샘플 {n_samples}개)")
    print(f"  목표 posx: [{target_reorient[0]:.1f}, {target_reorient[1]:.1f}, "
          f"{target_reorient[2]:.1f}, {rx:.1f}, {ry:.1f}, {rz:.1f}]")

    seed, seed_idx = _seed_joints_from_snapshot(folder)
    if seed is None:
        print("  ⚠️ 스냅샷에 joint_state가 없어 IK seed를 못 만듦 — 검사 생략")
        return
    print(f"  IK seed: waypoint_{seed_idx}의 joint_state")

    fraction, why = planner.validate_in_place_rotation(arrival_posx, target_reorient, seed)
    if fraction is None:
        print(f"  결과: ⚠️ 검사 불가 — {why}")
    elif fraction >= 1.0:
        print(f"  결과: ✅ 전구간 안전 — {why}")
    elif fraction <= 0.0:
        print(f"  결과: ❌ 처음부터 충돌 — {why}")
        print("        => 실기에서는 재조정을 건너뛰고 도착 자세를 유지함")
    else:
        partial = transforms.slerp_posx(arrival_posx, target_reorient, fraction)
        print(f"  결과: ⚠️ 부분만 안전({fraction * 100:.0f}%) — {why}")
        print(f"        => 실기에서는 여기까지만 회전: rx,ry,rz = "
              f"{partial[3]:.1f}, {partial[4]:.1f}, {partial[5]:.1f}")

def _verify_approach(points_mm, target_obj_mm, other_objs_mm, mask_radius_mm, folder=None):
    """저장된 정찰 데이터로 접근 경로 계획 단계만 재현함."""
    import rclpy
    from yolo_detect.motion_planning import ApproachPathPlanner

    obst_voxels, masked = _generate_obstacle_points(
        points_mm, target_obj_mm, config.MAP_OBSTACLE_VOXEL_SIZE_MM, mask_radius_mm)
    obstacle_points = (
        [(o[0], o[1], o[2], config.OBSTACLE_SAFETY_RADIUS_MM) for o in other_objs_mm]
        + [(p[0], p[1], p[2], config.MAP_OBSTACLE_SAFETY_RADIUS_MM) for p in obst_voxels]
    )
    x, y, z = target_obj_mm
    target_xyz = (x, y, z + config.OBJECT_APPROACH_OFFSET_MM)
    T_gripper2camera = _load_gripper2camera()

    print("\n" + "=" * 64)
    print("접근경로 계획 강제 재현 (자체 경로계획, 로봇 안 움직임)")
    print("=" * 64)
    print(f"타겟 객체(박스 내부): [{x:.1f}, {y:.1f}, {z:.1f}]mm  -> 접근목표 z+{config.OBJECT_APPROACH_OFFSET_MM:.0f}mm")
    print(f"다른 객체(회피 대상): {len(other_objs_mm)}개  (각 반경 {config.OBSTACLE_SAFETY_RADIUS_MM:.0f}mm)")
    print(f"정적 장애물점(박스벽/바닥/배경, {config.MAP_OBSTACLE_VOXEL_SIZE_MM:.0f}mm voxel): "
          f"{len(obst_voxels)}개 (각 반경 {config.MAP_OBSTACLE_SAFETY_RADIUS_MM:.0f}mm), "
          f"타겟 마스킹({mask_radius_mm:.0f}mm)으로 {masked}개 제외됨")

    rclpy.init()
    node = rclpy.create_node("build_octree_preview_verify")
    try:
        planner = ApproachPathPlanner(node)
        ok_walls = planner.register_box_walls(
            config.BOX_CORNER_1_MM, config.BOX_CORNER_2_MM, config.BOX_CORNER_3_MM,
            config.BOX_DEPTH_MM, config.BOX_WALL_THICKNESS_MM)
        print(f"박스 5벽 등록: {'성공' if ok_walls else '실패'}")

        points = planner.plan_dense_waypoints_mm(
            target_xyz, obstacle_points, config.GOAL_TOLERANCE_MM,
            look_at_xyz_mm=target_obj_mm, T_gripper2camera=T_gripper2camera,
            orientation_tolerance_deg=config.APPROACH_PLANNING_ORIENTATION_TOLERANCE_DEG)

        print("-" * 64)
        if points is None:
            print("결과: ❌ 접근경로 계획 실패 (자체 계획기가 충돌회피 경로를 못 찾음)")
            return
        print(f"결과: ✅ 접근경로 계획 성공 — dense waypoint {len(points)}개 (base_link mm)")
        print(f"  시작점: {[round(v, 1) for v in points[0]]}")
        print(f"  끝점:   {[round(v, 1) for v in points[-1]]}")
        if folder is not None:
            _verify_reorientation(planner, folder, list(points[-1]), target_obj_mm, T_gripper2camera)
    finally:
        node.destroy_node()
        rclpy.shutdown()

def _unproject_to_base_color(folder, waypoint_idx, T_gripper2camera, stride, max_depth_mm):
    """스냅샷 한 장을 컬러가 입혀진 base 좌표 점군으로 만듦."""
    depth_path = os.path.join(folder, f"waypoint_{waypoint_idx}_depth.npy")
    meta_path = os.path.join(folder, f"waypoint_{waypoint_idx}_meta.json")
    jpg_path = os.path.join(folder, f"waypoint_{waypoint_idx}.jpg")
    if not (os.path.exists(depth_path) and os.path.exists(meta_path) and os.path.exists(jpg_path)):
        return None
    depth = np.load(depth_path).astype(np.float32)
    with open(meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)
    posx, intr = meta["posx_mm_deg"], meta["intrinsics"]
    bgr = cv2.imread(jpg_path)
    h, w = depth.shape
    us, vs = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
    d = depth[vs, us]
    valid = (d > 0) & (d < max_depth_mm)
    us_v, vs_v, d_v = us[valid], vs[valid], d[valid]
    xs = (us_v - intr["ppx"]) * d_v / intr["fx"]
    ys = (vs_v - intr["ppy"]) * d_v / intr["fy"]
    cam = np.stack([xs, ys, d_v], axis=1)
    b2c = transforms.robot_pose_to_matrix(*posx) @ T_gripper2camera
    base = (b2c @ np.concatenate([cam, np.ones((len(cam), 1))], axis=1).T).T[:, :3]
    ch, cw = bgr.shape[:2]
    rgb = bgr[np.clip(vs_v, 0, ch - 1), np.clip(us_v, 0, cw - 1)][:, ::-1]
    return base.astype(np.float64), rgb.astype(np.uint8)

def _icp_align(xyz_list):
    """xyz_list."""
    import open3d as o3d

    def to_down(xyz):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(xyz)
        pc = pc.voxel_down_sample(ICP_VOXEL_MM)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=ICP_VOXEL_MM * 3, max_nn=30))
        return pc

    downs = [to_down(x) for x in xyz_list]
    Ts = [np.eye(4)]
    accum = o3d.geometry.PointCloud(downs[0])
    print("시점별 ICP 보정량 (FK+핸드아이 초기정합 대비 — 크면 캘리브레이션 오차):")
    print(f"{'WP':>3} {'회전(도)':>9} {'평균점이동(mm)':>13} {'정합후RMSE(mm)':>14} {'fitness':>9}")
    print(f"{0:>3} {'기준':>9} {'기준':>13} {'-':>14} {'-':>9}")
    for i in range(1, len(downs)):
        reg = o3d.pipelines.registration.registration_icp(
            downs[i], accum, ICP_MAX_CORR_MM, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        Ti = np.asarray(reg.transformation)
        Ts.append(Ti)
        pts = np.asarray(downs[i].points)
        moved = (Ti @ np.concatenate([pts, np.ones((len(pts), 1))], axis=1).T).T[:, :3]
        disp = float(np.linalg.norm(moved - pts, axis=1).mean())
        ang = np.degrees(np.arccos(np.clip((np.trace(Ti[:3, :3]) - 1) / 2, -1, 1)))
        print(f"{i:>3} {ang:>9.2f} {disp:>13.1f} {reg.inlier_rmse:>14.2f} {reg.fitness:>9.3f}")
        accum += o3d.geometry.PointCloud(downs[i]).transform(Ti)
    return Ts

def _write_ply(path, xyz_list, rgb_list, transforms_list=None):
    """옥트리 미리보기 이미지를 만듦."""
    xs, cs = [], []
    for i in range(len(xyz_list)):
        p = xyz_list[i]
        if transforms_list is not None:
            p = (transforms_list[i] @ np.concatenate(
                [p, np.ones((len(p), 1))], axis=1).T).T[:, :3]
        xs.append(p)
        cs.append(rgb_list[i])
    xyz = np.concatenate(xs).astype(np.float32)
    rgb = np.concatenate(cs).astype(np.uint8)
    centroid = xyz.mean(axis=0)
    n = len(xyz)
    v = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                            ("r", "u1"), ("g", "u1"), ("b", "u1")])
    v["x"], v["y"], v["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    v["r"], v["g"], v["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    hdr = (f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\n"
           "property float x\nproperty float y\nproperty float z\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    with open(path, "wb") as f:
        f.write(hdr.encode("ascii"))
        f.write(v.tobytes())
    return n, centroid

def _write_color_viewer_html(path, before_ply, after_ply, folder_name, n, wps,
                             centroid=(0.0, 0.0, 0.0)):
    """고밀도 컬러 뷰어 HTML 생성."""
    ws_port = config.WS_PORT
    cx, cy, cz = (float(centroid[0]), float(centroid[1]), float(centroid[2]))
    html = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>{folder_name} color 3D</title>
<style>html,body{{margin:0;height:100%;background:#000;overflow:hidden;font-family:monospace}}
#info{{position:fixed;top:8px;left:8px;color:#ccc;font-size:13px;line-height:1.6;z-index:10}}
#live{{position:fixed;top:8px;right:10px;color:#ccc;font-size:12.5px;line-height:1.7;z-index:10;text-align:right}}
b{{color:#6cf}} .on{{color:#5f5}} .off{{color:#f66}}</style></head><body>
<div id="info">{folder_name} 고밀도 컬러 3D (WP{wps}, {n:,}점) &nbsp; <b id="mode">BEFORE (raw)</b><br>
<b>스페이스바</b>=before/after(ICP) 토글 &nbsp;|&nbsp; 좌드래그=회전 휠=확대 우드래그=이동</div>
<div id="live"><span id="conn" class="off">● 실시간 연결 안 됨</span><br><span id="pathinfo">경로 없음</span></div>
<script src="https://unpkg.com/three@0.128.0/build/three.min.js"></script>
<script src="https://unpkg.com/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script src="https://unpkg.com/three@0.128.0/examples/js/loaders/PLYLoader.js"></script>
<script>
const scene=new THREE.Scene(); scene.background=new THREE.Color(0x000000);
const cam=new THREE.PerspectiveCamera(55,innerWidth/innerHeight,1,50000);
// PLY가 base_link mm 그대로라 박스는 원점에서 수백 mm 떨어져 있다.
//    좌표를 옮기는 대신 카메라를 점군 중심으로 보내서 화면을 맞춘다.
const CENTER=new THREE.Vector3({cx:.1f},{cy:.1f},{cz:.1f});
cam.up.set(0,0,1);
cam.position.set(CENTER.x+600,CENTER.y-800,CENTER.z+600);
const rndr=new THREE.WebGLRenderer({{antialias:true}}); rndr.setSize(innerWidth,innerHeight);
document.body.appendChild(rndr.domElement);
const ctrl=new THREE.OrbitControls(cam,rndr.domElement);
ctrl.target.copy(CENTER); ctrl.update();
let before=null,after=null,showAfter=false;
const mat=()=>new THREE.PointsMaterial({{size:{COLOR_POINT_SIZE},vertexColors:true}});
new THREE.PLYLoader().load('{before_ply}',g=>{{before=new THREE.Points(g,mat());scene.add(before);}});
new THREE.PLYLoader().load('{after_ply}', g=>{{after=new THREE.Points(g,mat());after.visible=false;scene.add(after);}});
addEventListener('keydown',e=>{{if(e.code==='Space'){{e.preventDefault();showAfter=!showAfter;
 if(before)before.visible=!showAfter;if(after)after.visible=showAfter;
 document.getElementById('mode').textContent=showAfter?'AFTER (ICP)':'BEFORE (raw)';}}}});
scene.add(new THREE.AxesHelper(200));

// ───────── 실시간 경로 + 네비게이션 화살표 ─────────
let doneLine=null, todoLine=null;
const arrow=new THREE.Group(); arrow.visible=false; scene.add(arrow);
(function buildArrow(){{
  // 점군에 파묻히지 않도록 depthTest를 끄고 마지막에 그린다.
  const m=c=>{{const x=new THREE.MeshBasicMaterial({{color:c}});x.depthTest=false;return x;}};
  const shaft=new THREE.Mesh(new THREE.CylinderGeometry(5,5,46,14), m(0x00e5ff));
  shaft.position.y=23;
  const head=new THREE.Mesh(new THREE.ConeGeometry(15,40,20), m(0x00e5ff));
  head.position.y=66;
  const base=new THREE.Mesh(new THREE.SphereGeometry(9,16,12), m(0xffffff));
  arrow.add(shaft); arrow.add(head); arrow.add(base);
  arrow.children.forEach(o=>{{o.renderOrder=999;}});
}})();
function orientArrow(pos,dir){{
  arrow.position.set(pos[0],pos[1],pos[2]);
  const d=new THREE.Vector3(dir[0],dir[1],dir[2]);
  if(d.lengthSq()>1e-9){{
    d.normalize();
    // 원뿔/원기둥의 기본 축은 +Y라 그걸 진행 방향으로 돌린다.
    arrow.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), d);
  }}
  arrow.visible=true;
}}
// ⚠️ THREE.LineBasicMaterial/LineDashedMaterial의 linewidth는 대부분의 브라우저·GPU에서
//    무시된다(WebGL 제약, 항상 1px). 실제 굵기를 내려면
//    지오메트리로 그려야 하므로 구간마다 원기둥을 세운다.
const TUBE_R=6.0, DASH_MM=26, GAP_MM=18;
function tube(a,b,r,m){{
  const va=new THREE.Vector3(a[0],a[1],a[2]), vb=new THREE.Vector3(b[0],b[1],b[2]);
  const d=new THREE.Vector3().subVectors(vb,va), L=d.length();
  if(L<1e-6) return null;
  const mesh=new THREE.Mesh(new THREE.CylinderGeometry(r,r,L,10), m);
  mesh.position.copy(va).addScaledVector(d,0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), d.clone().normalize());
  mesh.renderOrder=998;
  return mesh;
}}
function makeLine(pts,dashed){{
  if(pts.length<2) return null;
  const m=new THREE.MeshBasicMaterial({{color:dashed?0xff3333:0x00e5ff}});
  m.depthTest=false;
  const grp=new THREE.Group();
  if(!dashed){{
    for(let i=0;i<pts.length-1;i++){{const t=tube(pts[i],pts[i+1],TUBE_R,m); if(t) grp.add(t);}}
    return grp;
  }}
  // 점선: 폴리라인을 호길이로 훑으며 DASH_MM 칠하고 GAP_MM 건너뛴다.
  let carry=0, drawing=true;
  for(let i=0;i<pts.length-1;i++){{
    const a=new THREE.Vector3(pts[i][0],pts[i][1],pts[i][2]);
    const b=new THREE.Vector3(pts[i+1][0],pts[i+1][1],pts[i+1][2]);
    const seg=new THREE.Vector3().subVectors(b,a), L=seg.length();
    if(L<1e-6) continue;
    let s=0;
    while(s<L){{
      const want=(drawing?DASH_MM:GAP_MM)-carry;
      const take=Math.min(want,L-s);
      if(drawing){{
        const p0=a.clone().addScaledVector(seg,s/L), p1=a.clone().addScaledVector(seg,(s+take)/L);
        const t=tube([p0.x,p0.y,p0.z],[p1.x,p1.y,p1.z],TUBE_R*0.85,m); if(t) grp.add(t);
      }}
      s+=take;
      if(take>=want-1e-9){{drawing=!drawing;carry=0;}} else {{carry+=take;}}
    }}
  }}
  return grp;
}}
function updatePath(points,reached){{
  [doneLine,todoLine].forEach(l=>{{if(l) scene.remove(l);}});
  doneLine=todoLine=null;
  if(!points||points.length===0){{arrow.visible=false;
    document.getElementById('pathinfo').textContent='경로 없음'; return;}}
  const n=Math.max(0,Math.min(reached|0,points.length));
  // 경계 지점을 양쪽이 공유해야 두 선이 끊겨 보이지 않는다(대시보드와 같은 규칙).
  doneLine=makeLine(points.slice(0,n),false);
  todoLine=makeLine(points.slice(Math.max(0,n-1)),true);
  if(doneLine) scene.add(doneLine);
  if(todoLine) scene.add(todoLine);
  const i=Math.min(Math.max(n-1,0),points.length-1);
  const nxt=points[Math.min(i+1,points.length-1)];
  const cur=points[i];
  orientArrow(cur,[nxt[0]-cur[0],nxt[1]-cur[1],nxt[2]-cur[2]]);
  document.getElementById('pathinfo').textContent=
    '경로 '+n+' / '+points.length+' 지점 통과';
}}
(function connect(){{
  const ws=new WebSocket('ws://'+location.hostname+':{ws_port}');
  const conn=document.getElementById('conn');
  ws.onopen=()=>{{conn.textContent='● 실시간 연결됨';conn.className='on';}};
  ws.onclose=()=>{{conn.textContent='● 연결 끊김 — 5초 후 재시도';conn.className='off';
                  setTimeout(connect,5000);}};
  ws.onerror=()=>ws.close();
  ws.onmessage=e=>{{
    let d; try{{d=JSON.parse(e.data);}}catch(_){{return;}}
    if(d.type==='approach_path') updatePath(d.points,d.reached);
  }};
}})();

addEventListener('resize',()=>{{cam.aspect=innerWidth/innerHeight;cam.updateProjectionMatrix();rndr.setSize(innerWidth,innerHeight);}});
(function loop(){{requestAnimationFrame(loop);ctrl.update();rndr.render(scene,cam);}})();
</script></body></html>"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)

def _serve_dir(directory, port, html_name):
    import functools
    import http.server
    import socketserver
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=directory)
    with socketserver.TCPServer(("127.0.0.1", port), handler) as httpd:
        print(f"\n브라우저에서 조회: http://127.0.0.1:{port}/{html_name}")
        print("(Ctrl-C 로 서버 종료)")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n서버 종료")

def _color_icp_viewer(folder, folder_name, T_gripper2camera, wps, stride, max_depth_mm, serve_port):
    """고밀도 컬러 점군 + ICP 정합 + before/after 브라우저 뷰어 생성."""
    if wps is None:
        wps = _available_waypoints(folder)
    loaded_wps, xyz_list, rgb_list = [], [], []
    for i in wps:
        res = _unproject_to_base_color(folder, i, T_gripper2camera, stride, max_depth_mm)
        if res is None:
            print(f"  WP{i}: 실측 depth/컬러 없음 — 건너뜀(컬러 뷰어는 실측 전용)")
            continue
        xyz, rgb = res
        loaded_wps.append(i)
        xyz_list.append(xyz)
        rgb_list.append(rgb)
        print(f"  WP{i}: {len(xyz)} pts")
    if len(xyz_list) < 2:
        print("컬러 뷰어에는 실측 depth를 가진 WP가 2개 이상 필요합니다(ICP 정합용).")
        sys.exit(1)

    Ts = _icp_align(xyz_list)

    os.makedirs(COLOR_OUTPUT_DIR, exist_ok=True)
    tag = "_".join(str(i) for i in loaded_wps)
    before_name = f"{folder_name}_wp{tag}_before.ply"
    after_name = f"{folder_name}_wp{tag}_after.ply"
    html_name = f"{folder_name}_wp{tag}_color_viewer.html"
    n, centroid = _write_ply(os.path.join(COLOR_OUTPUT_DIR, before_name),
                             xyz_list, rgb_list, None)
    _write_ply(os.path.join(COLOR_OUTPUT_DIR, after_name), xyz_list, rgb_list, Ts)
    _write_color_viewer_html(os.path.join(COLOR_OUTPUT_DIR, html_name),
                             before_name, after_name, folder_name, n, loaded_wps,
                             centroid)
    print(f"\n저장됨({COLOR_OUTPUT_DIR}):")
    print(f"  {before_name} / {after_name} ({n:,}점)")
    print(f"  {html_name}")
    if serve_port:
        _serve_dir(COLOR_OUTPUT_DIR, serve_port, html_name)
    else:
        print(f"\n조회하려면: cd {COLOR_OUTPUT_DIR} && python3 -m http.server 8899 --bind 127.0.0.1")
        print(f"          그 뒤 http://127.0.0.1:8899/{html_name}")
        print("또는 이 스크립트에 --serve 8899 를 붙여 바로 서버까지 띄우세요.")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", help="recon_results/snapshots 아래 폴더명(예: 20260723_214720)")
    parser.add_argument("--voxel-mm", type=float, default=20.0,
                         help="voxel 크기(mm)")
    parser.add_argument("--stride", type=int, default=4, help="depth 픽셀 서브샘플링 간격")
    parser.add_argument("--verify-approach", action="store_true",
                         help="자체 경로계획으로 접근경로 계획 단계를 재현(로봇 불필요)")
    parser.add_argument("--target-mm", type=float, nargs=3, metavar=("X", "Y", "Z"),
                         help="탐지된 타겟 객체 좌표(base mm) — --verify-approach 시 필수")
    parser.add_argument("--other-obj-mm", type=float, nargs=3, action="append", metavar=("X", "Y", "Z"),
                         default=[], help="타겟 외 회피할 다른 객체 좌표(base mm), 여러 번 지정 가능")
    parser.add_argument("--target-mask-mm", type=float, default=config.DEPTH_TARGET_MASK_RADIUS_MM,
                         help="타겟 중심 이 반경 이내 정적 장애물점 제외(객체가 스스로를 막지 않게)")
    parser.add_argument("--color-viewer", action="store_true",
                         help="각 WP 컬러 jpg를 입힌 고밀도 실사 3D 맵 + ICP 정합 + 브라우저 뷰어 생성")
    parser.add_argument("--wps", type=str, default="0,1,2",
                         help="컬러 뷰어에 쓸 WP 인덱스(쉼표구분). 기본값 0,1,2(입구쪽 3장, "
                              "유령 겹침이 적어 더 깨끗하다. 정합오차 자체는 5장 전부일 때와 "
                              "같다). 전체를 쓰려면 --wps '' (빈 문자열)")
    parser.add_argument("--color-stride", type=int, default=COLOR_STRIDE,
                         help=f"컬러 뷰어 depth 서브샘플 간격(기본 {COLOR_STRIDE}=고밀도)")
    parser.add_argument("--max-depth-mm", type=float, default=COLOR_MAX_DEPTH_MM,
                         help=f"이보다 먼 점 제외(기본 {COLOR_MAX_DEPTH_MM:.0f}mm). 컬러 뷰어뿐 "
                              f"아니라 기본 octree와 --verify-approach 경로에도 적용된다. "
                              f"원거리 노이즈가 장애물로 들어가면 장애물 집합이 실제와 달라진다")
    parser.add_argument("--serve", type=int, default=None, metavar="PORT",
                         help="컬러 뷰어 생성 후 그 자리에서 로컬 웹서버까지 띄움(예: --serve 8899)")
    args = parser.parse_args()

    if args.verify_approach and args.target_mm is None:
        parser.error("--verify-approach 에는 --target-mm X Y Z 가 필요합니다 "
                     "(예: recon txt의 obj_001 좌표)")

    folder = os.path.join(SNAPSHOTS_DIR, args.folder)
    if not os.path.isdir(folder):
        print(f"폴더 없음: {folder}")
        sys.exit(1)

    T_gripper2camera = _load_gripper2camera()

    if args.color_viewer:
        wps = None
        if args.wps:
            wps = [int(s) for s in args.wps.split(",") if s.strip() != ""]
        _color_icp_viewer(folder, args.folder, T_gripper2camera, wps,
                          args.color_stride, args.max_depth_mm, args.serve)
        return

    all_points = []
    any_synthetic = False

    available = _available_waypoints(folder)
    print(f"사용 가능 웨이포인트 : {available}")
    for waypoint_idx in available:
        depth, posx, intrinsics, is_synthetic = _load_waypoint_data(folder, waypoint_idx)
        if depth is None:
            print(f"       waypoint {waypoint_idx}: depth/meta 없음 — 건너뜀")
            continue
        any_synthetic = any_synthetic or is_synthetic
        tag = "[합성]" if is_synthetic else "[실측]"
        pts = _unproject_to_base(depth, posx, intrinsics, T_gripper2camera, stride=args.stride,
                                 max_depth_mm=args.max_depth_mm)
        print(f"{tag} waypoint {waypoint_idx}: posx={[round(v, 1) for v in posx]}  점 {len(pts)}개")
        all_points.append(pts)

    if not all_points:
        print("사용 가능한 waypoint 데이터를 찾지 못함")
        sys.exit(1)

    points_mm = np.concatenate(all_points, axis=0)
    voxels_mm = _voxelize(points_mm, args.voxel_mm)
    print(f"\n원본 점 {len(points_mm)}개 -> voxel({args.voxel_mm}mm) {len(voxels_mm)}개")
    print("bounding box(mm): "
          f"x=[{voxels_mm[:, 0].min():.0f}, {voxels_mm[:, 0].max():.0f}]  "
          f"y=[{voxels_mm[:, 1].min():.0f}, {voxels_mm[:, 1].max():.0f}]  "
          f"z=[{voxels_mm[:, 2].min():.0f}, {voxels_mm[:, 2].max():.0f}]")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    npz_path = os.path.join(OUTPUT_DIR, f"{args.folder}_voxels.npz")
    np.savez(npz_path, voxels_mm=voxels_mm, voxel_size_mm=args.voxel_mm, synthetic=any_synthetic)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    z = voxels_mm[:, 2]
    titles = [
        ("위에서 본 뷰 (top, X-Y)", voxels_mm[:, 0], voxels_mm[:, 1], "X (mm)", "Y (mm)"),
        ("정면 뷰 (front, X-Z)", voxels_mm[:, 0], voxels_mm[:, 2], "X (mm)", "Z (mm)"),
        ("측면 뷰 (side, Y-Z)", voxels_mm[:, 1], voxels_mm[:, 2], "Y (mm)", "Z (mm)"),
    ]
    for ax, (title, xs, ys, xl, yl) in zip(axes, titles):
        sc = ax.scatter(xs, ys, c=z, cmap="viridis", s=2)
        ax.set_title(title)
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.set_aspect("equal", adjustable="datalim")
    fig.colorbar(sc, ax=axes, label="Z (mm, base_link)", shrink=0.7)
    synthetic_note = " — ⚠️ 합성(가짜) depth 포함, 실제 형상 아님" if any_synthetic else ""
    fig.suptitle(f"{args.folder} 옥트리 미리보기 (voxel={args.voxel_mm}mm){synthetic_note}")

    png_path = os.path.join(OUTPUT_DIR, f"{args.folder}_octree.png")
    fig.savefig(png_path, dpi=120, bbox_inches="tight")
    print(f"\n저장됨: {png_path}")
    print(f"저장됨: {npz_path}")
    if any_synthetic:
        print("⚠️ 이 결과는 합성(가짜) depth를 포함합니다 — 실제 박스/장애물 형상 검증 근거로 쓰지 마세요.")

    if args.verify_approach:
        if any_synthetic:
            print("⚠️ 합성 depth로는 접근계획 재현이 무의미 — 실측 depth 폴더에서만 --verify-approach 쓰세요.")
            return
        _verify_approach(points_mm, tuple(args.target_mm), args.other_obj_mm, args.target_mask_mm,
                          folder=folder)

if __name__ == "__main__":
    main()
