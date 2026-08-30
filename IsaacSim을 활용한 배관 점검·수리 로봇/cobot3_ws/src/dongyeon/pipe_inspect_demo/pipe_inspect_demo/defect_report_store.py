"""결함 이벤트 원본과 임무별 최신 결함 요약을 파일로 관리한다."""

import json
import os
from datetime import datetime
from pathlib import Path

from pipe_inspect_demo.repair_decision import build_mission_repair_plan


class DefectReportStore:
    """실행별 폴더에 JSONL 이벤트와 결함 ID별 요약 JSON을 저장한다."""

    def __init__(self, output_root, started_at=None):
        """출력 루트 아래에 중복되지 않는 임무 폴더를 생성한다."""
        self.started_at = started_at or datetime.now().astimezone()
        self.mission_dir = self._make_mission_dir(Path(output_root).expanduser())
        self.events_path = self.mission_dir / "defect_events.jsonl"
        self.summary_path = self.mission_dir / "defect_summary.json"
        self.latest_by_id = {}
        self.highest_confidence_by_id = {}
        self.registration_counts = {}
        self.event_count = 0
        self._write_summary("running")

    def _make_mission_dir(self, output_root):
        """밀리초와 순번을 이용해 기존 결과를 덮어쓰지 않는 폴더를 만든다."""
        output_root.mkdir(parents=True, exist_ok=True)
        base = f"mission_{self.started_at.strftime('%Y%m%d_%H%M%S_%f')[:-3]}"
        mission_dir = output_root / base
        index = 1
        while mission_dir.exists():
            mission_dir = output_root / f"{base}_{index:02d}"
            index += 1
        mission_dir.mkdir()
        return mission_dir

    def add_event(self, report):
        """검증된 결함 보고를 JSONL에 추가하고 결함별 최신·최고 신뢰도 요약을 갱신한다."""
        defect_id = report.get("defect_id")
        if not isinstance(defect_id, str) or not defect_id.strip():
            raise ValueError("defect_id가 없는 보고는 저장할 수 없어.")
        with self.events_path.open("a", encoding="utf-8") as event_file:
            event_file.write(json.dumps(report, ensure_ascii=False, separators=(",", ":")) + "\n")
            event_file.flush()
            os.fsync(event_file.fileno())
        self.event_count += 1
        self.latest_by_id[defect_id] = report
        confidence = float(report.get("confidence", 0.0))
        previous = self.highest_confidence_by_id.get(defect_id)
        if previous is None or confidence > float(previous.get("confidence", 0.0)):
            self.highest_confidence_by_id[defect_id] = report
        registration = str(report.get("registration", "unknown"))
        self.registration_counts[registration] = self.registration_counts.get(registration, 0) + 1
        self._write_summary("running")

    def close(self, status="completed"):
        """임무 종료 상태와 종료 시각을 최종 요약에 기록한다."""
        self._write_summary(status, datetime.now().astimezone())

    def _write_summary(self, status, ended_at=None):
        """요약 파일을 임시 파일로 쓴 뒤 원자적으로 교체한다."""
        defects = []
        for defect_id, latest in sorted(self.latest_by_id.items()):
            defects.append({"defect_id": defect_id, "latest": latest, "highest_confidence": self.highest_confidence_by_id[defect_id]})
        summary = {
            "mission": {
                "status": status,
                "started_at": self.started_at.isoformat(),
                "ended_at": ended_at.isoformat() if ended_at else None,
                "event_count": self.event_count,
                "defect_count": len(defects),
                "registration_counts": self.registration_counts,
            },
            "repair_plan": build_mission_repair_plan(self.latest_by_id.values()),
            "repair_targets": [report for report in self.latest_by_id.values() if report.get("registration") == "aligned" and report.get("repair_target", {}).get("pose_transform_valid") is True],
            "defects": defects,
        }
        temporary_path = self.summary_path.with_suffix(".json.tmp")
        with temporary_path.open("w", encoding="utf-8") as summary_file:
            json.dump(summary, summary_file, ensure_ascii=False, indent=2)
            summary_file.write("\n")
            summary_file.flush()
            os.fsync(summary_file.fileno())
        temporary_path.replace(self.summary_path)
