"""설계문서의 용접봉 기준에 따른 수리 판정 결과를 검증한다."""

import pytest

from pipe_inspect_demo.repair_decision import add_repair_decision, build_mission_repair_plan, calculate_required_rod_length_mm, decide_repair


def test_crack_required_rod_matches_design_example():
    """40×1.5×2mm 크랙의 용접봉 소요량이 설계 예시 약 46mm인지 확인한다."""
    report = {"class": "crack", "measurement": {"length_mm": 40.0, "width_mm": 1.5, "defect_depth_mm": 2.0}}
    required_mm, missing = calculate_required_rod_length_mm(report)
    assert required_mm == pytest.approx(45.837, abs=0.001)
    assert missing == []


def test_repair_is_allowed_only_with_ten_millimeter_reserve():
    """시공 후 10mm 예비량이 남을 때만 용접 수리로 판정하는지 확인한다."""
    report = {"class": "crack", "measurement": {"length_mm": 40.0, "width_mm": 1.5, "defect_depth_mm": 2.0}}
    assert decide_repair(report, 55.837)["repairable"] is True
    decision = decide_repair(report, 55.0)
    assert decision["repairable"] is False
    assert decision["reason"] == "insufficient_welding_rod"


def test_missing_crack_depth_requests_measurement():
    """카메라 거리로 홈 깊이를 대신하지 않고 추가 치수를 요구하는지 확인한다."""
    report = {"class": "crack", "defect": {"camera_depth_m": 0.05}, "measurement": {"length_mm": 30.0, "width_mm": 1.2}}
    decision = decide_repair(report, 1000.0)
    assert decision["action"] == "measurement_required"
    assert decision["missing_measurements"] == ["defect_depth_mm"]


def test_structural_defect_is_always_reported():
    """파이프 단절은 용접봉이 충분해도 관리자 보고 대상으로 판정하는지 확인한다."""
    decision = decide_repair({"class": "파이프 단절"}, 1880.0)
    assert decision["repairable"] is False
    assert decision["action"] == "report_to_manager"
    assert decision["reason"] == "structural_defect_not_robot_repairable"


def test_add_decision_does_not_change_original_report():
    """판정 추가 함수가 검출 노드의 원본 JSON 객체를 변경하지 않는지 확인한다."""
    report = {"defect_id": "crack_0001", "class": "crack", "measurement": {"length_mm": 40.0, "width_mm": 1.5, "defect_depth_mm": 2.0}}
    enriched = add_repair_decision(report, 100.0)
    assert "repair_decision" not in report
    assert enriched["repair_decision"]["action"] == "weld_repair"


def test_mission_plan_adds_fifteen_percent_margin():
    """정찰 결과 총량에 설계 기준 15%를 더해 초기 장착량을 계산하는지 확인한다."""
    reports = [
        {"defect_id": "crack_0001", "class": "crack", "measurement": {"length_mm": 40.0, "width_mm": 1.5, "defect_depth_mm": 2.0}},
        {"defect_id": "pipe_0001", "class": "파이프 단절"},
    ]
    plan = build_mission_repair_plan(reports)
    assert plan["planning_complete"] is True
    assert plan["known_required_rod_total_mm"] == pytest.approx(45.837, abs=0.001)
    assert plan["recommended_initial_load_mm"] == pytest.approx(52.713, abs=0.001)
    assert plan["report_only_defect_ids"] == ["pipe_0001"]


def test_mission_plan_stays_incomplete_when_depth_is_missing():
    """필수 결함 치수가 없으면 확정 장착량 대신 잠정 합계만 제공하는지 확인한다."""
    plan = build_mission_repair_plan([{"defect_id": "crack_0001", "class": "crack", "measurement": {"length_mm": 40.0, "width_mm": 1.5}}])
    assert plan["planning_complete"] is False
    assert plan["recommended_initial_load_mm"] is None
    assert plan["unresolved_defect_ids"] == ["crack_0001"]
