"""규약 자체 검증 — ROS 없이 `python3 -m pytest` 로 돈다.

🔑 규약이 깨지는 것은 **런타임에 침묵으로** 나타난다(토픽 이름 오타 → 아무도
   못 받음). 그래서 이름·스키마는 여기서 미리 걸러야 한다.
"""

import json

import pytest

from pipe_comm import contract
from pipe_comm.contract import Topics


def test_namespace_applied():
    t = Topics("elbow_v")
    assert t.rgb == "/elbow_v/rgb/compressed"
    assert t.depth == "/elbow_v/depth/compressed"
    assert t.cmd_vel == "/elbow_v/cmd_vel"
    assert Topics().drive_state == "/robot/drive_state"


def test_unknown_topic_raises():
    # 오타는 침묵이 아니라 예외여야 한다.
    with pytest.raises(AttributeError):
        Topics("tee").rgb_compressed


def test_all_topics_absolute_and_unique():
    got = Topics("tee").all()
    assert len(got) == len(contract.RELATIVE)
    assert all(v.startswith("/tee/") for v in got.values())
    assert len(set(got.values())) == len(got)


def test_drive_state_roundtrip():
    d = contract.drive_state("tee", contract.STATE_RUN, direction=-1,
                             speed_mps=0.1, s_mm=431.25, s_total_mm=935.6,
                             off_mm=3.44, lap=2, stuck=1, step=2963)
    assert d["moving"] is True and d["dir"] == -1
    assert d["s_mm"] == 431.2 and d["off_mm"] == 3.44
    back = contract.parse(contract.dumps(d))
    assert back == d


def test_moving_follows_state():
    for s in contract.STATES:
        d = contract.drive_state("robot", s)
        assert d["moving"] is (s in contract.MOVING_STATES)
    # 명시로 덮어쓸 수 있어야 한다 (지령 정지 중인 RUN 등)
    assert contract.drive_state("robot", contract.STATE_RUN,
                                moving=False)["moving"] is False


def test_unknown_state_and_event_raise():
    with pytest.raises(ValueError):
        contract.drive_state("robot", "CRUISING")
    with pytest.raises(ValueError):
        contract.event("robot", "EXPLODED")
    with pytest.raises(ValueError):
        contract.mission("GO")


def test_event_alert_flag():
    assert contract.event("robot", contract.EV_OFF_COURSE)["alert"] is True
    assert contract.event("robot", contract.EV_HOME)["alert"] is False


def test_speed_command_needs_mps():
    with pytest.raises(ValueError):
        contract.mission(contract.CMD_SPEED)
    assert contract.mission(contract.CMD_SPEED, mps=0.05)["mps"] == 0.05


def test_repair_target_clock_wraps():
    # 시계각은 0~360 으로 접힌다 — 부호가 뒤집히면 토치를 정반대로 돌린다.
    assert contract.repair_target("d1", s_mm=1130, clock_deg=-180.0
                                  )["clock_deg"] == 180.0
    assert contract.repair_target("d1", s_mm=1130, clock_deg=540.0
                                  )["clock_deg"] == 180.0


def test_parse_survives_garbage():
    assert contract.parse("{not json") is None
    assert contract.parse("[1,2,3]") is None      # dict 가 아니면 거절
    assert contract.parse("") is None


def test_dumps_keeps_korean():
    s = contract.dumps(contract.event("tee", contract.EV_STUCK,
                                      detail="방향 전환"))
    assert "방향 전환" in s
    assert json.loads(s)["detail"] == "방향 전환"
