"""영상 코덱 검증 — 특히 **무효 픽셀 처리**. 여기가 조용히 틀리는 자리다."""

import numpy as np
import pytest

from pipe_comm import image_codec as codec


class FakeMsg:
    """CompressedImage 흉내. rclpy 없이 코덱만 시험한다."""

    def __init__(self, data):
        self.data = data


def test_rgb_jpeg_roundtrip_keeps_channel_order():
    # 빨강 화면. BGR 로 뒤집혀 나가면 파랑으로 돌아온다.
    rgb = np.zeros((32, 48, 3), np.uint8)
    rgb[:, :, 0] = 200
    back = codec.jpeg_to_rgb(FakeMsg(codec.rgb_to_jpeg(rgb)))
    assert back.shape == (32, 48, 3)
    assert back[:, :, 0].mean() > 150      # R 이 살아 있어야 한다
    assert back[:, :, 2].mean() < 50       # B 로 새면 안 된다


def test_rgba_alpha_is_dropped():
    # Isaac annotator 는 RGBA 를 준다 — 알파를 안 떼면 인코딩이 깨진다.
    rgba = np.zeros((16, 16, 4), np.uint8)
    rgba[:, :, 1] = 180
    rgba[:, :, 3] = 255
    back = codec.jpeg_to_rgb(FakeMsg(codec.rgb_to_jpeg(rgba)))
    assert back.shape == (16, 16, 3)
    assert back[:, :, 1].mean() > 140


def test_depth_png16_is_lossless_in_mm():
    d = np.array([[0.025, 0.050], [0.500, 1.234]], np.float32)
    buf, ratio = codec.depth_to_png16(d)
    assert ratio == 1.0
    back = codec.png16_to_depth_m(FakeMsg(buf))
    # mm 로 반올림되므로 0.5mm 안에 들어와야 한다.
    assert np.allclose(back, d, atol=0.0005)


def test_invalid_depth_becomes_nan_not_garbage():
    # 🚨 inf/NaN/0 을 uint16 으로 그냥 캐스팅하면 쓰레기 정수가 되어
    #    "관 단절"이 "코앞의 벽"으로 둔갑한다.
    d = np.array([[np.inf, np.nan], [0.0, -1.0]], np.float32)
    buf, ratio = codec.depth_to_png16(d)
    assert ratio == 0.0
    back = codec.png16_to_depth_m(FakeMsg(buf))
    assert np.isnan(back).all()


def test_depth_beyond_uint16_range_is_invalid_not_wrapped():
    # 65.535m 를 넘으면 wrap 되어 먼 곳이 코앞으로 보인다.
    d = np.array([[0.5, 100.0]], np.float32)
    back = codec.png16_to_depth_m(FakeMsg(codec.depth_to_png16(d)[0]))
    assert abs(back[0, 0] - 0.5) < 0.0005
    assert np.isnan(back[0, 1])


def test_valid_ratio_reported():
    d = np.array([[1.0, np.nan], [2.0, 0.0]], np.float32)
    _, ratio = codec.depth_to_png16(d)
    assert ratio == 0.5


def test_png16_rejects_jpeg():
    rgb = np.zeros((8, 8, 3), np.uint8)
    with pytest.raises(ValueError):
        codec.png16_to_depth_m(FakeMsg(codec.rgb_to_jpeg(rgb)))
