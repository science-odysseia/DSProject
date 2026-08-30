"""결함 보고 저장소의 이력 보존과 요약 갱신을 검증한다."""

import json
from datetime import datetime, timezone

import pytest

from pipe_inspect_demo.defect_report_store import DefectReportStore


def make_report(defect_id, confidence, registration="new"):
    """테스트에 필요한 최소 결함 보고 객체를 만든다."""
    return {"schema_version": "1.5", "defect_id": defect_id, "confidence": confidence, "registration": registration}


def test_store_preserves_events_and_summarizes_latest(tmp_path):
    """모든 이벤트가 남고 최신 보고와 최고 신뢰도 보고가 각각 선택되는지 확인한다."""
    store = DefectReportStore(tmp_path, datetime(2026, 8, 4, tzinfo=timezone.utc))
    store.add_event(make_report("crack_0001", 0.90))
    store.add_event(make_report("crack_0001", 0.80, "measurement_updated"))
    store.add_event(make_report("crack_0002", 0.95))
    store.close()
    events = [json.loads(line) for line in store.events_path.read_text(encoding="utf-8").splitlines()]
    summary = json.loads(store.summary_path.read_text(encoding="utf-8"))
    assert len(events) == 3
    assert summary["mission"]["status"] == "completed"
    assert summary["mission"]["event_count"] == 3
    assert summary["mission"]["defect_count"] == 2
    assert summary["defects"][0]["latest"]["confidence"] == 0.80
    assert summary["defects"][0]["highest_confidence"]["confidence"] == 0.90


def test_store_rejects_report_without_defect_id(tmp_path):
    """결함 ID가 없는 잘못된 보고를 파일에 기록하지 않는지 확인한다."""
    store = DefectReportStore(tmp_path)
    with pytest.raises(ValueError):
        store.add_event({"confidence": 0.9})
    assert not store.events_path.exists()


def test_summary_contains_mission_repair_plan(tmp_path):
    """결함 요약에 최신 결함 기준 용접봉 사전 계획이 함께 기록되는지 확인한다."""
    store = DefectReportStore(tmp_path)
    report = make_report("crack_0001", 0.9)
    report["class"] = "crack"
    report["measurement"] = {"length_mm": 40.0, "width_mm": 1.5, "defect_depth_mm": 2.0}
    store.add_event(report)
    summary = json.loads(store.summary_path.read_text(encoding="utf-8"))
    assert summary["repair_plan"]["planning_complete"] is True
    assert summary["repair_plan"]["recommended_initial_load_mm"] == pytest.approx(52.713, abs=0.001)


def test_summary_separates_valid_aligned_repair_targets(tmp_path):
    """좌표가 유효한 정렬 보고만 수리 목표 목록에 포함되는지 확인한다."""
    store = DefectReportStore(tmp_path)
    aligned = make_report("crack_0001", 0.9, "aligned")
    aligned["repair_target"] = {"pose_transform_valid": True, "navigation_goal_xyz_m": [0.0, 0.7, 0.02], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]}
    store.add_event(aligned)
    summary = json.loads(store.summary_path.read_text(encoding="utf-8"))
    assert [target["defect_id"] for target in summary["repair_targets"]] == ["crack_0001"]
