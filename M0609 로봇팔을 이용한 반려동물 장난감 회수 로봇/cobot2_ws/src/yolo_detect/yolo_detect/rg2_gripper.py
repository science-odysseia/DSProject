"""RG2 그리퍼 Modbus 제어 + 재질별 파지 정책."""
import threading
import time

try:
    from pymodbus.client import ModbusTcpClient
    _PYMODBUS_V2 = False
except ImportError:
    from pymodbus.client.sync import ModbusTcpClient
    _PYMODBUS_V2 = True

COMPUTE_BOX_IP = "192.168.1.1"
COMPUTE_BOX_PORT = 502
RG2_DEVICE_ID = 65

COMMAND_REGISTER_START = 0
STATUS_REGISTER_START = 258
STATUS_REGISTER_COUNT = 18
GRIP_STATUS_INDEX = 11
ACTUAL_WIDTH_INDEX = 17

FINGERTIP_OFFSET_INDEX = 0
VENDOR_STATUS_INDEX = 10
VENDOR_GRIP_BIT = 0b10

WIDTH_MIN_MM, WIDTH_MAX_MM = 0.0, 110.0
FORCE_MIN_N, FORCE_MAX_N = 3.0, 40.0

MATERIAL_FORCE_N = {
    "skin": 5.0,
    "rubber": 7.0,
    "plastic": 10.0,
}
DEFAULT_MATERIAL = "skin"

GRASP_WIDTH_RATIO = 0.9

GRASP_EXTRA_CLOSE_MM = 45.0

MIN_TARGET_WIDTH_MM = 2.0
OPEN_CLEARANCE_MM = 50.0

STATUS_POLL_SEC = 0.05
MOVEMENT_THRESHOLD_MM = 0.2
def _signed16(raw):
    """Modbus 레지스터."""
    v = int(raw)
    return v - 65536 if v > 32767 else v

STABLE_TOLERANCE_MM = 0.2
STABLE_SAMPLE_COUNT = 6
MOVEMENT_START_TIMEOUT_SEC = 2.0
NO_MOVEMENT_STABLE_SEC = 0.7
MOTION_TIMEOUT_SEC = 25.0
LOG_INTERVAL_SEC = 0.25

def plan_grasp(material, measured_width_mm):
    """재질과 실측 폭으로 개방 폭, 목표 폭, 파지력을 정함."""
    key = str(material or "").strip().lower()
    applied = key if key in MATERIAL_FORCE_N else DEFAULT_MATERIAL
    force_n = MATERIAL_FORCE_N[applied]
    measured_width_mm = float(measured_width_mm)

    open_width_mm = min(WIDTH_MAX_MM, measured_width_mm + OPEN_CLEARANCE_MM)
    target_width_mm = max(
        MIN_TARGET_WIDTH_MM,
        measured_width_mm * GRASP_WIDTH_RATIO - GRASP_EXTRA_CLOSE_MM)
    target_width_mm = min(target_width_mm, open_width_mm)
    return {"open_width_mm": open_width_mm, "target_width_mm": target_width_mm,
            "force_n": float(force_n), "material": key, "applied_material": applied,
            "measured_width_mm": measured_width_mm,
            "known_material": key in MATERIAL_FORCE_N}

class Rg2Gripper:
    """RG2 Modbus 제어."""

    def __init__(self, ip=COMPUTE_BOX_IP, port=COMPUTE_BOX_PORT, logger=None):
        self._client = ModbusTcpClient(ip, port=port)
        self._ip, self._port = ip, port
        self._lock = threading.Lock()
        self._connected = False
        self._log = logger
        self._offset_logged = False

    def _info(self, msg):
        (self._log.info if self._log else print)(msg)

    def _warn(self, msg):
        (self._log.warn if self._log else print)(msg)

    def connect(self):
        if self._connected:
            return True
        try:
            self._connected = bool(self._client.connect())
        except Exception as e:
            self._connected = False
            self._warn(f"Modbus 연결 예외: {e}")
        if not self._connected:
            self._warn(f"RG2 Compute Box 연결 실패: {self._ip}:{self._port}")
            return False
        if not self._offset_logged:
            self._offset_logged = True
            try:
                r = self._read_registers(STATUS_REGISTER_START, STATUS_REGISTER_COUNT)
                regs = getattr(r, "registers", None)
                if regs and len(regs) > FINGERTIP_OFFSET_INDEX:
                    off = float(regs[FINGERTIP_OFFSET_INDEX]) / 10.0
                    self._info(
                        f"[RG2] 연결됨 {self._ip}:{self._port} | 핑거팁 오프셋 = {off:.1f}mm"
                        + ("  (0이므로 control 1↔폭275 엇갈림은 무해)" if abs(off) < 0.05
                           else "  ⚠️ 0이 아님 — 명령폭(오프셋 미포함)과 측정폭(포함)이 "
                                "이만큼 어긋난다. 상수 주석 ② 참고"))
            except Exception as e:
                self._warn(f"[RG2] 핑거팁 오프셋 확인 실패(무해): {e}")
        return self._connected

    def close(self):
        try:
            self._client.close()
        except Exception:
            pass
        self._connected = False

    def _write_registers(self, address, values):
        if _PYMODBUS_V2:
            return self._client.write_registers(address, values, unit=RG2_DEVICE_ID)
        last = None
        for keyword in ("device_id", "slave", "unit"):
            try:
                return self._client.write_registers(address, values,
                                                    **{keyword: RG2_DEVICE_ID})
            except TypeError as e:
                last = e
        raise last

    def _read_registers(self, address, count):
        if _PYMODBUS_V2:
            return self._client.read_holding_registers(address, count, unit=RG2_DEVICE_ID)
        last = None
        for keyword in ("device_id", "slave", "unit"):
            try:
                return self._client.read_holding_registers(address, count,
                                                           **{keyword: RG2_DEVICE_ID})
            except TypeError as e:
                last = e
        raise last

    def read_status(self):
        """{'actual_width_mm', 'grip'} 또는 None."""
        with self._lock:
            if not self.connect():
                return None
            try:
                result = self._read_registers(STATUS_REGISTER_START, STATUS_REGISTER_COUNT)
            except Exception as e:
                self.close()
                self._warn(f"상태 레지스터 읽기 예외: {e}")
                return None
            if result is None or (hasattr(result, "isError") and result.isError()):
                return None
            registers = getattr(result, "registers", None)
            if not registers or len(registers) < STATUS_REGISTER_COUNT:
                return None
            vendor_word = int(registers[VENDOR_STATUS_INDEX])
            return {"actual_width_mm": _signed16(registers[ACTUAL_WIDTH_INDEX]) / 10.0,
                    "grip": int(registers[GRIP_STATUS_INDEX]) != 0,
                    "grip_raw_269": int(registers[GRIP_STATUS_INDEX]),
                    "vendor_status_268": vendor_word,
                    "vendor_grip_bit": bool(vendor_word & VENDOR_GRIP_BIT),
                    "vendor_busy_bit": bool(vendor_word & 0b1),
                    "registers": list(registers)}

    def read_width(self):
        status = self.read_status()
        return None if status is None else status["actual_width_mm"]

    def command(self, width_mm, force_n):
        """폭·힘을 레지스터에 씀."""
        width_mm, force_n = float(width_mm), float(force_n)
        if not WIDTH_MIN_MM <= width_mm <= WIDTH_MAX_MM:
            raise ValueError(f"width는 {WIDTH_MIN_MM}~{WIDTH_MAX_MM}mm 범위여야 합니다: {width_mm}")
        if not FORCE_MIN_N <= force_n <= FORCE_MAX_N:
            raise ValueError(f"force는 {FORCE_MIN_N}~{FORCE_MAX_N}N 범위여야 합니다: {force_n}")

        with self._lock:
            if not self.connect():
                raise ConnectionError("RG2 Compute Box 연결 실패")
            self._info(f"RG2 명령: width={width_mm:.1f}mm force={force_n:.1f}N")
            try:
                result = self._write_registers(
                    COMMAND_REGISTER_START,
                    [int(round(force_n * 10.0)), int(round(width_mm * 10.0)), 1])
            except Exception:
                self.close()
                raise
            if result is None or (hasattr(result, "isError") and result.isError()):
                self._warn("RG2 명령 응답 이상 — 실제 폭으로 동작 여부를 확인합니다.")

    def wait_until_stopped(self, initial_width_mm, motion_name,
                           timeout_sec=MOTION_TIMEOUT_SEC):
        """실제 폭이 멈출 때까지 대기."""
        start = time.monotonic()
        last_log = 0.0
        previous = initial_width_mm
        last_valid = initial_width_mm
        movement_started = False
        stable_count = 0
        no_move_stable = 0
        no_move_required = max(1, int(NO_MOVEMENT_STABLE_SEC / STATUS_POLL_SEC))

        while True:
            elapsed = time.monotonic() - start
            if elapsed >= timeout_sec:
                raise TimeoutError(f"{motion_name} 시간 초과: last_actual={last_valid:.1f}mm")

            actual = self.read_width()
            if actual is None:
                time.sleep(STATUS_POLL_SEC)
                continue
            last_valid = actual
            change = abs(actual - previous)

            now = time.monotonic()
            if now - last_log >= LOG_INTERVAL_SEC:
                self._info(f"{motion_name}: actual={actual:.1f}mm change={change:.1f} "
                           f"moving={movement_started} stable={stable_count}/{STABLE_SAMPLE_COUNT}")
                last_log = now

            if not movement_started:
                if abs(actual - initial_width_mm) >= MOVEMENT_THRESHOLD_MM:
                    movement_started = True
                    stable_count = 0
                    self._info(f"{motion_name}: 움직임 시작 "
                               f"({initial_width_mm:.1f} → {actual:.1f}mm)")
                else:
                    no_move_stable = no_move_stable + 1 if change <= STABLE_TOLERANCE_MM else 0
                    if elapsed >= MOVEMENT_START_TIMEOUT_SEC and no_move_stable >= no_move_required:
                        self._warn(f"{motion_name}: 움직임 없음 — 이미 해당 위치로 보고 완료")
                        return actual, "already_stable"
            else:
                stable_count = stable_count + 1 if change <= STABLE_TOLERANCE_MM else 0
                if stable_count >= STABLE_SAMPLE_COUNT:
                    self._info(f"{motion_name} 완료: actual={actual:.1f}mm")
                    return actual, "motion_stopped"

            previous = actual
            time.sleep(STATUS_POLL_SEC)

    def move_and_wait(self, width_mm, force_n, motion_name, timeout_sec=MOTION_TIMEOUT_SEC):
        """명령 + 완료 대기를 한 번에."""
        initial = self.read_width()
        if initial is None:
            self._warn(f"{motion_name}: 시작 폭을 못 읽음 — 명령 폭 기준으로 대기")
            initial = width_mm
        self.command(width_mm, force_n)
        return self.wait_until_stopped(initial, motion_name, timeout_sec)

    def open_for(self, plan):
        """접근 전 개방."""
        return self.move_and_wait(plan["open_width_mm"], FORCE_MAX_N, "개방")

    def grasp(self, plan):
        """목표 폭까지 닫아 물체를 잡고 성공 여부를 판정함."""
        final_width, reason = self.move_and_wait(
            plan["target_width_mm"], plan["force_n"], f"{plan['material']} 파지")
        status = self.read_status()
        blocked = final_width > plan["target_width_mm"] + STABLE_TOLERANCE_MM * 2
        if status:
            self._info(
                f"[RG2] 판정근거 — blocked={blocked}(최종 {final_width:.1f} vs 목표 "
                f"{plan['target_width_mm']:.1f}mm) / 269={status['grip_raw_269']} "
                f"-> grip={status['grip']} | 벤더기준 268=0x{status['vendor_status_268']:04x} "
                f"(grip비트={status['vendor_grip_bit']}, busy비트={status['vendor_busy_bit']})")
        else:
            self._warn("[RG2] 파지 후 상태 읽기 실패 — blocked 신호만으로 판정한다")
        return {"final_width_mm": final_width, "stop_reason": reason,
                "gripped": bool(blocked),
                "grip_flag": bool(status and status["grip"]),
                "blocked": bool(blocked),
                "vendor_status_268": (status or {}).get("vendor_status_268"),
                "grip_raw_269": (status or {}).get("grip_raw_269")}

    def release(self):
        return self.move_and_wait(WIDTH_MAX_MM, FORCE_MAX_N, "개방(놓기)")

    def close_fingers(self, force_n=FORCE_MIN_N):
        """손가락을 완전히 닫음."""
        return self.move_and_wait(WIDTH_MIN_MM, force_n, "닫기(종료 자세 정리)")
