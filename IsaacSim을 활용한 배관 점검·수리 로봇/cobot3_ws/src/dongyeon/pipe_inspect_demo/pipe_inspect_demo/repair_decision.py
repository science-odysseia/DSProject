"""설계 기준에 따라 결함의 용접봉 소요량과 수리 가능 여부를 판정한다."""

import math
from dataclasses import dataclass

ROD_DIAMETER_MM = 2.0
ROD_AREA_MM2 = math.pi * (ROD_DIAMETER_MM / 2.0) ** 2
VOLUME_MARGIN_FACTOR = 1.2
HOLE_VOLUME_FACTOR = 1.3
RESERVE_ROD_LENGTH_MM = 10.0
MISSION_LOAD_MARGIN_FACTOR = 1.15
ROD_COIL_CAPACITY_MM = 1880.0
REPAIRABLE_CLASSES = {"crack", "hole"}
REPORT_ONLY_CLASSES = {"joint_twist", "pipe_disconnection"}
CLASS_ALIASES = {
    "crack": "crack", "크랙": "crack", "균열": "crack",
    "hole": "hole", "구멍": "hole",
    "joint_twist": "joint_twist", "joint twist": "joint_twist", "조인트 비틀림": "joint_twist",
    "pipe_disconnection": "pipe_disconnection", "pipe disconnection": "pipe_disconnection", "파이프 단절": "pipe_disconnection",
}


@dataclass(frozen=True)
class RepairDecisionConfig:
    """용접봉 규격과 손실·예비량 판정 기준을 보관한다."""

    rod_area_mm2: float = ROD_AREA_MM2
    volume_margin_factor: float = VOLUME_MARGIN_FACTOR
    hole_volume_factor: float = HOLE_VOLUME_FACTOR
    reserve_rod_length_mm: float = RESERVE_ROD_LENGTH_MM
    mission_load_margin_factor: float = MISSION_LOAD_MARGIN_FACTOR
    rod_coil_capacity_mm: float = ROD_COIL_CAPACITY_MM


def normalize_defect_class(class_name):
    """한글과 영문 결함 이름을 내부 표준 클래스 이름으로 변환한다."""
    return CLASS_ALIASES.get(str(class_name).strip().lower(), str(class_name).strip().lower())


def calculate_required_rod_length_mm(report, config=RepairDecisionConfig()):
    """결함 치수로 체적을 계산해 필요한 2mm 용접봉 길이를 반환한다."""
    defect_class = normalize_defect_class(report.get("class", ""))
    measurement = report.get("measurement") or {}
    if defect_class == "crack":
        length_mm = _positive_number(measurement.get("length_mm"))
        width_mm = _positive_number(measurement.get("width_mm"))
        depth_mm = _positive_number(measurement.get("defect_depth_mm"))
        if None in (length_mm, width_mm, depth_mm):
            return None, [name for name, value in (("length_mm", length_mm), ("width_mm", width_mm), ("defect_depth_mm", depth_mm)) if value is None]
        volume_mm3 = length_mm * width_mm * depth_mm
    elif defect_class == "hole":
        diameter_mm = _positive_number(measurement.get("diameter_mm"))
        wall_thickness_mm = _positive_number(measurement.get("pipe_wall_thickness_mm"))
        if None in (diameter_mm, wall_thickness_mm):
            return None, [name for name, value in (("diameter_mm", diameter_mm), ("pipe_wall_thickness_mm", wall_thickness_mm)) if value is None]
        volume_mm3 = math.pi * (diameter_mm / 2.0) ** 2 * wall_thickness_mm * config.hole_volume_factor
    else:
        return None, []
    return round(volume_mm3 * config.volume_margin_factor / config.rod_area_mm2, 3), []


def decide_repair(report, remaining_rod_length_mm, config=RepairDecisionConfig()):
    """결함 종류와 용접봉 잔량을 기준으로 수리·보고·추가 측정 행동을 결정한다."""
    defect_class = normalize_defect_class(report.get("class", ""))
    remaining_mm = max(0.0, float(remaining_rod_length_mm))
    base = {
        "defect_class": defect_class,
        "remaining_rod_length_mm": round(remaining_mm, 3),
        "reserve_rod_length_mm": config.reserve_rod_length_mm,
        "required_rod_length_mm": None,
        "remaining_after_repair_mm": None,
    }
    if defect_class in REPORT_ONLY_CLASSES:
        return {**base, "repairable": False, "action": "report_to_manager", "report_required": True, "reason": "structural_defect_not_robot_repairable", "missing_measurements": []}
    if defect_class not in REPAIRABLE_CLASSES:
        return {**base, "repairable": False, "action": "report_to_manager", "report_required": True, "reason": "unsupported_defect_class", "missing_measurements": []}
    required_mm, missing = calculate_required_rod_length_mm(report, config)
    if required_mm is None:
        return {**base, "repairable": False, "action": "measurement_required", "report_required": True, "reason": "insufficient_measurement_for_weld_volume", "missing_measurements": missing}
    remaining_after_mm = remaining_mm - required_mm
    common = {**base, "required_rod_length_mm": required_mm, "remaining_after_repair_mm": round(remaining_after_mm, 3), "missing_measurements": []}
    if remaining_after_mm >= config.reserve_rod_length_mm:
        return {**common, "repairable": True, "action": "weld_repair", "report_required": False, "reason": "sufficient_welding_rod"}
    return {**common, "repairable": False, "action": "report_to_manager", "report_required": True, "reason": "insufficient_welding_rod"}


def add_repair_decision(report, remaining_rod_length_mm, config=RepairDecisionConfig()):
    """원본을 변경하지 않고 repair_decision 필드가 추가된 결함 보고를 반환한다."""
    enriched_report = dict(report)
    enriched_report["repair_decision"] = decide_repair(report, remaining_rod_length_mm, config)
    return enriched_report


def build_mission_repair_plan(reports, config=RepairDecisionConfig()):
    """정찰 결함 목록에서 용접봉 합계와 15% 여유를 포함한 사전 장착 계획을 만든다."""
    items = []
    known_total_mm = 0.0
    unresolved_ids = []
    report_only_ids = []
    for report in reports:
        defect_id = str(report.get("defect_id", ""))
        defect_class = normalize_defect_class(report.get("class", ""))
        required_mm, missing = calculate_required_rod_length_mm(report, config)
        if defect_class in REPORT_ONLY_CLASSES or defect_class not in REPAIRABLE_CLASSES:
            status = "report_only"
            report_only_ids.append(defect_id)
        elif required_mm is None:
            status = "measurement_required"
            unresolved_ids.append(defect_id)
        else:
            status = "planned"
            known_total_mm += required_mm
        items.append({"defect_id": defect_id, "defect_class": defect_class, "status": status, "required_rod_length_mm": required_mm, "missing_measurements": missing})
    recommended_mm = round(known_total_mm * config.mission_load_margin_factor, 3)
    complete = not unresolved_ids
    return {
        "planning_complete": complete,
        "known_required_rod_total_mm": round(known_total_mm, 3),
        "load_margin_percent": round((config.mission_load_margin_factor - 1.0) * 100.0, 1),
        "recommended_initial_load_mm": recommended_mm if complete else None,
        "provisional_initial_load_mm": recommended_mm,
        "coil_capacity_mm": config.rod_coil_capacity_mm,
        "capacity_exceeded": recommended_mm > config.rod_coil_capacity_mm,
        "split_mission_required": recommended_mm > config.rod_coil_capacity_mm,
        "unresolved_defect_ids": unresolved_ids,
        "report_only_defect_ids": report_only_ids,
        "items": items,
    }


def _positive_number(value):
    """값을 양의 실수로 변환하고 유효하지 않으면 None을 반환한다."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) and number > 0.0 else None
