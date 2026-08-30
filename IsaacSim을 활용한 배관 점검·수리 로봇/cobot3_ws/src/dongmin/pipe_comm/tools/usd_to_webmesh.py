"""맵 USD → 웹 3D 맵용 메시(.webmesh) 변환 — web_panel 2단계(CAD 메시)용.

web_panel 의 3D 맵은 1단계에서 중심선 기반 와이어프레임만 그렸다. 이 도구는
실전 맵 USD 의 **메시 전체**(배관 + 건물)를 꺼내 브라우저가 한 번에 fetch 해
WebGL 로 그릴 수 있는 단일 바이너리로 굽는다. glTF 를 안 쓰는 이유: 로더
(three.js 등) 를 페이지에 들여와야 하는데, web_panel 은 한 파일·무외부의존
원칙이라 포맷을 직접 정하는 쪽이 전체 비용이 싸다 (파서가 JS 30줄이다).

━━ .webmesh 포맷 (little-endian) ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  [0:4]   uint32   JSON 헤더 바이트 길이 L
  [4:4+L] utf-8    JSON  {version, vtx_count, tri_count, bbox, parts:[
                          {name, group, vtx_start, vtx_count,
                           idx_start, idx_count}]}
  (4바이트 정렬 패딩)
  float32 × 3·vtx_count   위치 (월드 m — 아래 좌표계 참고)
  float32 × 3·vtx_count   법선 (스무스, 월드)
  uint32  × 3·tri_count   삼각형 인덱스 (전역 정점 번호)

━━ 좌표계 — real_map_demo / web_panel JS 와 동일하게 굽는다 ━━━━━━━
  맵 USD 는 map mm (metersPerUnit 0.001, upAxis Z). real_map_demo 는 스테이지에
  얹을 때 scale 0.001 + z 오프셋으로 **활성 층 수평망을 월드 z=0** 에 놓는다
  (floor2: +250mm). 같은 변환을 여기서 미리 구워 두면 JS 는 받은 좌표를
  그대로 로봇 pos_m·중심선과 한 좌표계에서 그린다.
  → 파이프가 아니라 맵 배치가 바뀌면 --z-shift-mm 만 맞춰 다시 구우면 된다.

🚨 법선은 USD 의 authored normal 을 **안 쓴다** — 이 맵의 Sweep5 가 법선
   primvar 버퍼가 깨져 있다(Isaac 로그: buffer size 64 ≠ expected 9499).
   월드 좌표로 변환한 삼각형에서 직접 누적(스무스)한다. 조명은 페이지 쪽이
   양면(two-sided)으로 계산하므로 와인딩/부호 뒤집힘에도 안전하다.

실행 (pxr 는 Isaac 번들에만 있다 — ROS 3.10 파이썬으로는 못 돌린다):
  U=$HOME/isaacsim_venv/lib/python3.11/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
  PYTHONPATH=$U LD_LIBRARY_PATH=$U/bin $HOME/isaacsim_venv/bin/python \\
      src/dongmin/pipe_comm/tools/usd_to_webmesh.py \\
      src/dongyeon/integration_test/maps/restroom_pipe150_final_fixed.usd
  → 같은 자리에 restroom_pipe150_final_fixed.webmesh
"""

import argparse
import json
import struct
import sys
from pathlib import Path

import numpy as np

try:
    from pxr import Usd, UsdGeom
except ImportError:
    raise SystemExit("pxr 가 없다 — 모듈 docstring 의 실행 예처럼 Isaac 번들 "
                     "omni.usd.libs 를 PYTHONPATH/LD_LIBRARY_PATH 에 넣고 "
                     "isaacsim_venv 파이썬으로 돌릴 것")


def _group_of(path: str) -> str:
    """프림 경로에서 층 그룹을 뽑는다 — 페이지가 그룹별 색을 입힌다."""
    for g in ("floor2", "floor1", "aisle"):
        if f"/{g}/" in path:
            return g
    return "etc"


def convert(usd_path: Path, z_shift_mm: float) -> Path:
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise SystemExit(f"USD 를 못 열었다: {usd_path}")
    mpu = UsdGeom.GetStageMetersPerUnit(stage)   # 이 맵은 0.001 (mm)

    all_pos, all_nrm, all_idx, parts = [], [], [], []
    v_base = i_base = 0
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())

    # instanceable 프로토타입 안의 메시도 잡아야 하므로 인스턴스 프록시 순회
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        cnts = mesh.GetFaceVertexCountsAttr().Get()
        idxs = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not cnts or not idxs:
            continue

        # 스테이지 단위(mm) 월드 변환 → 시연과 같은 배치(mm→m, z 오프셋)
        m = np.array(xcache.GetLocalToWorldTransform(prim))       # 4×4, 행벡터
        p = np.asarray(pts, np.float64)
        p = p @ m[:3, :3] + m[3, :3]
        p[:, 2] += z_shift_mm
        p *= mpu

        # n각형 → 부채꼴 삼각화. 좌수 와인딩·음수 행렬식이면 뒤집는다
        flip = (np.linalg.det(m[:3, :3]) < 0)
        if mesh.GetOrientationAttr().Get() == UsdGeom.Tokens.leftHanded:
            flip = not flip
        tri = []
        k = 0
        fv = np.asarray(idxs, np.int64)
        for c in cnts:
            for j in range(1, c - 1):
                t = (fv[k], fv[k + j], fv[k + j + 1])
                tri.append((t[0], t[2], t[1]) if flip else t)
            k += c
        tri = np.asarray(tri, np.int64)

        # 스무스 법선 — 월드 삼각형의 면법선을 정점에 면적 가중 누적
        e1 = p[tri[:, 1]] - p[tri[:, 0]]
        e2 = p[tri[:, 2]] - p[tri[:, 0]]
        fn = np.cross(e1, e2)
        n = np.zeros_like(p)
        for col in range(3):
            np.add.at(n, tri[:, col], fn)
        ln = np.linalg.norm(n, axis=1, keepdims=True)
        n /= np.maximum(ln, 1e-12)

        parts.append({
            "name": prim.GetPath().pathString.rsplit("/", 2)[-2],
            "group": _group_of(prim.GetPath().pathString),
            "vtx_start": v_base, "vtx_count": len(p),
            "idx_start": i_base, "idx_count": tri.shape[0] * 3,
        })
        all_pos.append(p.astype(np.float32))
        all_nrm.append(n.astype(np.float32))
        all_idx.append((tri + v_base).astype(np.uint32))
        v_base += len(p)
        i_base += tri.shape[0] * 3

    if not parts:
        raise SystemExit("메시가 하나도 없다 — 맵 파일이 맞는지 확인할 것")

    pos = np.concatenate(all_pos)
    nrm = np.concatenate(all_nrm)
    idx = np.concatenate(all_idx)
    header = {
        "version": 1, "source": usd_path.name,
        "z_shift_mm": z_shift_mm,
        "vtx_count": int(len(pos)), "tri_count": int(len(idx)),
        "bbox": [pos.min(0).tolist(), pos.max(0).tolist()],
        "parts": parts,
    }
    hj = json.dumps(header, ensure_ascii=False).encode()
    pad = (-(4 + len(hj))) % 4

    out = usd_path.with_suffix(".webmesh")
    with open(out, "wb") as f:
        f.write(struct.pack("<I", len(hj)))
        f.write(hj)
        f.write(b"\x00" * pad)
        f.write(pos.tobytes())
        f.write(nrm.tobytes())
        f.write(idx.tobytes())
    print(f"[완료] {out.name}  메시 {len(parts)}개  정점 {len(pos):,}  "
          f"삼각형 {len(idx):,}  {out.stat().st_size / 1e6:.2f} MB")
    for pt in parts:
        print(f"    {pt['group']:6s} {pt['name']:20s} tri {pt['idx_count'] // 3}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("usd", type=Path)
    ap.add_argument("--z-shift-mm", type=float, default=250.0,
                    help="맵 z 에 더할 오프셋(mm). real_map_demo floor2 와 같은 "
                         "기본 250 → 수평망이 월드 z=0 (기본: %(default)s)")
    a = ap.parse_args()
    if not a.usd.is_file():
        raise SystemExit(f"파일이 없다: {a.usd}")
    convert(a.usd, a.z_shift_mm)


if __name__ == "__main__":
    main()
