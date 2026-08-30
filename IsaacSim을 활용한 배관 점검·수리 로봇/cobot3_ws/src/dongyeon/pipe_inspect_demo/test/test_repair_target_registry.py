"""정렬 수리 목표의 저장·요청·상태 갱신을 검증한다."""

import json

from pipe_inspect_demo.repair_target_registry import RepairTargetRegistry


def make_aligned_report(defect_id="crack_0001", valid=True):
    """단위 테스트용 정렬 완료 결함 보고를 만든다."""
    return {"schema_version": "1.7", "defect_id": defect_id, "registration": "aligned", "repair_target": {"frame_id": "pipe_entry", "pose_transform_valid": valid, "navigation_goal_xyz_m": [0.0, 0.7, 0.02], "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]}, "measurement": {"length_mm": 20.0}}


def test_valid_target_is_stored_and_requested():
    """유효한 정렬 목표가 ID로 조회되는지 확인한다."""
    registry = RepairTargetRegistry()
    assert registry.add_aligned_report(make_aligned_report()) == (True, "stored")
    target, status = registry.get_target("crack_0001")
    assert status == "pending"
    assert target["repair_target"]["frame_id"] == "pipe_entry"


def test_invalid_pose_is_not_stored():
    """좌표 변환이 무효인 결함을 수리 목표에서 제외하는지 확인한다."""
    registry = RepairTargetRegistry()
    assert registry.add_aligned_report(make_aligned_report(valid=False)) == (False, "pose_transform_invalid")
    assert registry.get_target("crack_0001") == (None, "target_not_found")


def test_completed_target_is_not_reissued():
    """완료된 결함 목표가 다시 발행 대상으로 선택되지 않는지 확인한다."""
    registry = RepairTargetRegistry()
    registry.add_aligned_report(make_aligned_report())
    assert registry.update_result("crack_0001", "completed") == (True, "completed")
    assert registry.get_target("crack_0001") == (None, "repair_already_completed")


def test_targets_load_from_mission_summary(tmp_path):
    """늦게 시작한 coordinator가 임무 요약에서 목표를 복원하는지 확인한다."""
    summary_path = tmp_path / "defect_summary.json"
    summary_path.write_text(json.dumps({"repair_targets": [make_aligned_report()]}, ensure_ascii=False), encoding="utf-8")
    registry = RepairTargetRegistry()
    assert registry.load_summary(summary_path) == 1
    assert registry.get_target("crack_0001")[0] is not None
