"""ROS 영상 메시지 ↔ numpy.  **cv_bridge 를 쓰지 않는다.**

cv_bridge 를 뺀 이유 — 발행하는 쪽이 Isaac Sim 의 **Python 3.11** 이라 우리
3.10 용 cv_bridge 를 못 쓴다. 양쪽이 같은 코드를 쓰려면 stdlib + numpy + cv2
만으로 끝나야 한다. 어차피 하는 일은 `frombuffer` 와 `imencode` 둘뿐이다.

🚨 **Depth 압축 규약 — 16UC1 PNG, 단위 mm, 0 = 무효.**
   JPEG 는 손실 압축이라 깊이값을 훼손하므로 절대 쓰지 않는다. dongyeon 쪽
   `pipe_vision_node` 가 이 형식만 디코딩한다(`compressed_to_depth_m`).
   원본 정밀도가 필요하면 `Topics.DEPTH_RAW` 로 32FC1 을 따로 받는다.

🚨 **무효 픽셀을 0 으로 보내야 한다.** Isaac 의 `distance_to_camera` 는 빈
   공간을 `inf` / `NaN` / 최대거리 중 무엇으로도 돌려줄 수 있다(카메라 진단
   `camera/depth_probe.py` 가 가리는 값). uint16 으로 캐스팅하면 그것들이
   쓰레기 정수가 되므로 **캐스팅 전에** 0 으로 눌러야 한다. 안 그러면 관
   단절이 "가까운 벽"으로 둔갑해 판정이 조용히 뒤집힌다.

🚨 **mm × 65535 = 65.5m 에서 넘친다.** 관 안에서는 안 닿지만 관 단절로 시야가
   뻥 뚫리면 닿는다 — 넘친 값은 wrap 되어 **먼 곳이 코앞으로 보인다.** 상한을
   넘는 것도 0(무효)으로 눌러 둔다.
"""

import numpy as np

# 16UC1 mm 규약의 표현 한계. 이 위는 전부 무효로 본다.
DEPTH_MAX_M = 65.0
DEFAULT_JPEG_QUALITY = 85


def _cv2():
    """cv2 를 늦게 import 한다 — 규약만 읽는 쪽에 opencv 를 강요하지 않는다."""
    import cv2
    return cv2


# ── 인코딩 (Isaac 쪽이 쓴다) ──────────────────────────────────

def rgb_to_jpeg(rgb, quality=DEFAULT_JPEG_QUALITY):
    """RGB(또는 RGBA) uint8 배열 → JPEG 바이트.

    🚨 Isaac annotator 의 `rgb` 는 **RGBA(HxWx4)** 다. 알파를 안 떼고 cv2 에
       넘기면 채널 수가 안 맞는다. 여기서 잘라 준다.
    🚨 cv2 는 **BGR** 순서를 기대한다. 안 뒤집으면 빨강과 파랑이 바뀐 채로
       YOLO 에 들어간다 — 에러가 안 나고 검출률만 떨어져 원인을 못 찾는다.
    """
    cv2 = _cv2()
    a = np.asarray(rgb)
    if a.ndim == 3 and a.shape[2] == 4:
        a = a[:, :, :3]
    a = np.ascontiguousarray(a, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", a[:, :, ::-1],
                           [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise RuntimeError("RGB JPEG 인코딩 실패")
    return buf.tobytes()


def depth_to_png16(depth_m):
    """float32 깊이(m) → 16UC1 PNG 바이트. 무효 픽셀은 0.

    반환값은 (바이트, 유효픽셀비율) 이다. 비율이 0 에 가까우면 카메라가 아무
    것도 못 보고 있다는 뜻이라 발행 쪽에서 경고를 띄우는 데 쓴다.
    """
    cv2 = _cv2()
    d = np.asarray(depth_m, dtype=np.float32)
    # 🚨 순서가 중요하다 — 캐스팅 **전에** 무효를 걸러야 한다.
    good = np.isfinite(d) & (d > 0.0) & (d < DEPTH_MAX_M)
    mm = np.where(good, d * 1000.0, 0.0)
    mm = np.ascontiguousarray(mm.astype(np.uint16))
    ok, buf = cv2.imencode(".png", mm, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        raise RuntimeError("Depth PNG16 인코딩 실패")
    ratio = float(good.mean()) if good.size else 0.0
    return buf.tobytes(), ratio


# ── 디코딩 (ROS 노드 쪽이 쓴다) ─────────────────────────────────────────

def jpeg_to_rgb(msg):
    """sensor_msgs/CompressedImage(jpeg) → RGB uint8 배열."""
    cv2 = _cv2()
    bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("압축 RGB 를 디코딩할 수 없다")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def png16_to_depth_m(msg):
    """sensor_msgs/CompressedImage(16UC1 png) → 미터 배열. 무효는 NaN.

    🔑 무효를 0 이 아니라 **NaN** 으로 돌려준다. 0 으로 두면 `min()` 이 항상
       0 이 되어 "코앞에 벽이 있다"는 오판이 난다.
    """
    cv2 = _cv2()
    raw = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
    if raw is None or raw.dtype != np.uint16 or raw.ndim != 2:
        raise ValueError("압축 Depth 는 16UC1 PNG 여야 한다 "
                         f"(받은 것: {None if raw is None else raw.dtype})")
    d = raw.astype(np.float32) * 0.001
    d[raw == 0] = np.nan
    return d


def image_to_rgb(msg):
    """sensor_msgs/Image → RGB uint8 배열.  step 패딩을 존중한다.

    🚨 `width*채널` 이 아니라 **`step`** 으로 행을 끊어야 한다. 발행자가 행을
       정렬해 패딩을 넣으면 width 로 끊은 영상은 비스듬히 밀려 보인다.
    """
    cv2 = _cv2()
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(
        msg.encoding.lower())
    if ch is None:
        raise ValueError(f"지원하지 않는 RGB 인코딩 {msg.encoding!r}")
    row = int(msg.width) * ch
    raw = np.frombuffer(msg.data, np.uint8)
    need = int(msg.height) * int(msg.step)
    if msg.step < row or raw.size < need:
        raise ValueError(f"Image 크기가 안 맞는다 step={msg.step} row={row} "
                         f"data={raw.size} 필요={need}")
    img = raw[:need].reshape(msg.height, msg.step)[:, :row].reshape(
        msg.height, msg.width, ch)
    enc = msg.encoding.lower()
    if enc == "rgb8":
        return img.copy()
    if enc == "bgr8":
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if enc == "rgba8":
        return cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
    if enc == "bgra8":
        return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    return cv2.cvtColor(img[:, :, 0], cv2.COLOR_GRAY2RGB)


def image_to_depth_m(msg):
    """sensor_msgs/Image(32FC1 또는 16UC1) → 미터 배열. 무효는 NaN."""
    dt = {"32fc1": np.dtype(np.float32),
          "16uc1": np.dtype(np.uint16)}.get(msg.encoding.lower())
    if dt is None:
        raise ValueError(f"지원하지 않는 Depth 인코딩 {msg.encoding!r}")
    row = int(msg.width) * dt.itemsize
    raw = np.frombuffer(msg.data, np.uint8)
    need = int(msg.height) * int(msg.step)
    if msg.step < row or raw.size < need:
        raise ValueError(f"Depth Image 크기가 안 맞는다 step={msg.step} "
                         f"row={row} data={raw.size} 필요={need}")
    packed = np.ascontiguousarray(
        raw[:need].reshape(msg.height, msg.step)[:, :row])
    src = dt.newbyteorder(">" if bool(msg.is_bigendian) else "<")
    d = packed.view(src).reshape(msg.height, msg.width).astype(np.float32)
    if msg.encoding.lower() == "16uc1":
        d = d * 0.001
    d[~np.isfinite(d) | (d <= 0.0)] = np.nan
    return d
