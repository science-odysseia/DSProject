"""정렬 완료 결함의 수리 목표와 실행 상태를 ID별로 관리한다."""

import copy
import json
from pathlib import Path


class RepairTargetRegistry:
    """유효한 수리 목표를 등록하고 요청·결과 상태를 관리한다."""

    def __init__(self):
        """빈 목표와 상태 저장소를 생성한다."""
        self.targets, self.status_by_id = {}, {}

    def add_aligned_report(self, report):
        """정렬 완료 및 좌표 유효성이 확인된 결함 보고를 등록한다."""
        defect_id = str(report.get("defect_id", "")).strip()
        target = report.get("repair_target")
        if not defect_id or report.get("registration") != "aligned" or not isinstance(target, dict):
            return False, "aligned_target_required"
        if not bool(target.get("pose_transform_valid")):
            return False, "pose_transform_invalid"
        position, orientation = target.get("navigation_goal_xyz_m"), target.get("orientation_xyzw")
        if not self._finite_sequence(position, 3) or not self._finite_sequence(orientation, 4):
            return False, "target_pose_invalid"
        self.targets[defect_id] = copy.deepcopy(report)
        self.status_by_id.setdefault(defect_id, "pending")
        return True, "stored"

    def get_target(self, defect_id):
        """요청된 결함의 목표와 현재 상태 또는 사용 불가 사유를 반환한다."""
        defect_id = str(defect_id).strip()
        if defect_id not in self.targets:
            return None, "target_not_found"
        if self.status_by_id.get(defect_id) == "completed":
            return None, "repair_already_completed"
        return copy.deepcopy(self.targets[defect_id]), self.status_by_id.get(defect_id, "pending")

    def update_result(self, defect_id, status):
        """수리로봇 결과 상태를 검증해 등록된 결함에 반영한다."""
        defect_id, status = str(defect_id).strip(), str(status).strip().lower()
        if defect_id not in self.targets:
            return False, "target_not_found"
        if status not in {"in_progress", "completed", "failed"}:
            return False, "invalid_repair_status"
        self.status_by_id[defect_id] = status
        return True, status

    def load_summary(self, summary_path):
        """임무 요약 파일의 정렬 목표를 불러와 등록 개수를 반환한다."""
        summary_path = Path(summary_path).expanduser()
        if not summary_path.is_file():
            return 0
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        reports = summary.get("repair_targets")
        if reports is None:
            reports = [item.get("latest", {}) for item in summary.get("defects", [])]
        return sum(1 for report in reports if self.add_aligned_report(report)[0])

    @staticmethod
    def _finite_sequence(values, size):
        """값 목록이 지정 길이의 유한한 수인지 확인한다."""
        import math
        return isinstance(values, (list, tuple)) and len(values) == size and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in values)
