"""
EDFViewerMonitor — оболочка вывода данных EDFViewer NEXT.
Чтение бинарных пакетов по UART, отображение в GUI, бинарный лог, статистика.
Формат пакета соответствует SensorDataPacket (AHT20: temp/humidity).
Совместим с firmware: Lifecycle (INIT/CALIBRATING/RUNNING), SensorSystemConfig, calibration_flags.
"""
import argparse
import json
import os
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar, List, Optional, Tuple

import serial
from serial.tools import list_ports

# Путь к папке скрипта или exe (для сохранения геометрии окна)
def _app_dir() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent

CONFIG_FILENAME = "EDFViewerMonitor_geometry.json"
LOCK_FILENAME = "EDFViewerMonitor.lock"
DEFAULT_GEOMETRY = "520x420"

# Версия монитора (fallback, если version.json недоступен)
EDFVIEWER_MONITOR_VERSION = "beta 1.0"


def _project_root() -> Path:
    """Корень проекта edflight (для version.json). При frozen — рядом с exe или на уровень выше."""
    app = _app_dir()
    for candidate in (app, app.parent, app.parent.parent):
        if (candidate / "version.json").exists():
            return candidate
    return app.parent.parent  # edflight при запуске из tools/python_monitor


def get_project_version() -> Tuple[str, str]:
    """Версия и номер сборки из version.json. Возвращает (версия, сборка), например ('beta 1.0', '1.0.0')."""
    path = _project_root() / "version.json"
    if not path.exists():
        return (EDFVIEWER_MONITOR_VERSION, "—")
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        stage = data.get("stage", "beta")
        stage_ver = data.get("stage_version", "1.0")
        build = data.get("build", "—")
        version_str = f"{stage} {stage_ver}".strip()
        return (version_str if version_str else EDFVIEWER_MONITOR_VERSION, str(build))
    except (OSError, json.JSONDecodeError, TypeError):
        return (EDFVIEWER_MONITOR_VERSION, "—")

# --- Transport ---
START_BYTE = 0xAA
END_BYTE = 0x55

# Типы пакетов
PACKET_TYPE_SENSOR_DATA = 0x01
PACKET_TYPE_LOG = 0x02

# Уровни логирования
LOG_LEVEL_ERROR = 0x01
LOG_LEVEL_WARN = 0x02
LOG_LEVEL_INFO = 0x03
LOG_LEVEL_DEBUG = 0x04
LOG_LEVEL_VERBOSE = 0x05

# system_status биты (совпадают с EDF_SYS_STATUS_* в sensor_data_packet.h)
SYS_STATUS_PX4_1_ERR = 0x01
SYS_STATUS_PX4_2_ERR = 0x02       # PX4 ID2 / хаб — нет связи

# SensorDataPacket: ... system_status(B), calibration_flags(B), motor_count(B), crc16(H)
SENSOR_FORMAT = (
    "<II"           # packet_id, timestamp_ms
    + "fffB" * 2    # px4[0], px4[1]: pressure, temp, airspeed, health
    + "hffB"        # aht20: raw_adc, temperature_c, humidity_pct, calibration_id
    + "fffI"        # bmp280: pressure_pa, temperature_c, altitude_m, sensor_id
    + "HfffB"       # rc_throttle_us, mav_pitch_rad, mav_roll_rad, mav_airspeed_mps, mav_armed
    + "fff"         # cpu0_load_pct, cpu1_load_pct, cpu_temp_c
    + "BBBH"        # system_status, calibration_flags, motor_count, crc16
)
SENSOR_SIZE = struct.calcsize(SENSOR_FORMAT)
# Старый размер (без поля motor_count) — для совместимости со старым хабом
SENSOR_FORMAT_OLD = (
    "<II" + "fffB" * 2 + "hffB" + "fffI" + "HfffB" + "fff" + "BBH"
)
SENSOR_SIZE_OLD = struct.calcsize(SENSOR_FORMAT_OLD)

# LogPacket: timestamp_ms(I), level(B), tag(16s), message(128s), crc16(H)
LOG_FORMAT = "<IB16s128sH"
LOG_SIZE = struct.calcsize(LOG_FORMAT)
LOG_TAG_MAX_LEN = 16
LOG_MSG_MAX_LEN = 128


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class SensorPacket:
    """Соответствует SensorDataPacket в C (AHT20)."""
    FORMAT: ClassVar[str] = SENSOR_FORMAT

    packet_id: int
    timestamp_ms: int
    px4_0_pressure: float
    px4_0_temp: float
    px4_0_airspeed: float
    px4_0_health: int
    px4_1_pressure: float
    px4_1_temp: float
    px4_1_airspeed: float
    px4_1_health: int
    aht20_raw: int
    aht20_temp: float
    aht20_humidity: float
    aht20_cal_id: int
    bmp280_pressure: float
    bmp280_temp: float
    bmp280_altitude: float
    bmp280_sensor_id: int
    rc_throttle_us: int
    mav_pitch_rad: float
    mav_roll_rad: float
    mav_airspeed_mps: float
    mav_armed: int
    cpu0_load_pct: float
    cpu1_load_pct: float
    cpu_temp_c: float
    system_status: int
    calibration_flags: int
    motor_count: int      # 1 или 2 — для GlassHub PFD (отправляется с ПК командой MOTOR_COUNT=...)
    crc16: int

    @classmethod
    def from_bytes(cls, data: bytes) -> "SensorPacket":
        vals = struct.unpack(cls.FORMAT, data)
        return cls(*vals)

    def validate_crc(self, data_without_crc: bytes) -> bool:
        return crc16_ccitt(data_without_crc) == self.crc16


@dataclass
class LogPacket:
    """Соответствует LogPacket в C."""
    FORMAT: ClassVar[str] = LOG_FORMAT

    timestamp_ms: int
    level: int
    tag: str
    message: str
    crc16: int

    @classmethod
    def from_bytes(cls, data: bytes) -> "LogPacket":
        vals = struct.unpack(cls.FORMAT, data)
        tag = vals[2].rstrip(b'\0').decode('utf-8', errors='replace')
        message = vals[3].rstrip(b'\0').decode('utf-8', errors='replace')
        return cls(
            timestamp_ms=vals[0],
            level=vals[1],
            tag=tag,
            message=message,
            crc16=vals[4]
        )

    def validate_crc(self, data_without_crc: bytes) -> bool:
        return crc16_ccitt(data_without_crc) == self.crc16


# --- Binary log file (SDL1) ---
FILE_MAGIC = b"SDL1"
FILE_HEADER_SIZE = 128


def make_file_header(packet_size: int, start_ts: int, device_name: str = "EDFViewerMonitor") -> bytes:
    name_b = device_name.encode("ascii", errors="replace")[:32].ljust(32, b"\0")
    part = struct.pack(
        "<4sIIQ32sB",
        FILE_MAGIC,
        FILE_HEADER_SIZE,
        packet_size,
        start_ts,
        name_b,
        4,  # sensor_count
    )
    return part + b"\0" * (FILE_HEADER_SIZE - len(part))


def parse_file_header(data: bytes) -> Optional[dict]:
    if len(data) < 4 + 4 + 4 + 8 + 32 + 1:
        return None
    magic = data[:4]
    if magic != FILE_MAGIC:
        return None
    header_size, packet_size, start_ts = struct.unpack("<IIQ", data[4:20])
    device_name = data[20:52].rstrip(b"\0").decode("ascii", errors="replace")
    sensor_count = data[52]
    return {
        "header_size": header_size,
        "packet_size": packet_size,
        "start_timestamp": start_ts,
        "device_name": device_name,
        "sensor_count": sensor_count,
    }


def load_bin_log(path: Path) -> Tuple[Optional[dict], List[SensorPacket]]:
    """
    Загружает бинарный лог SDL1. Возвращает (инфо заголовка, список пакетов).
    Пакеты с неверным CRC пропускаются.
    """
    header_info = None
    packets: list[SensorPacket] = []
    try:
        with open(path, "rb") as f:
            header_data = f.read(FILE_HEADER_SIZE)
            header_info = parse_file_header(header_data)
            if not header_info or header_info["packet_size"] != SENSOR_SIZE:
                return (None, [])
            while True:
                raw = f.read(SENSOR_SIZE)
                if len(raw) != SENSOR_SIZE:
                    break
                p = SensorPacket.from_bytes(raw)
                if p.validate_crc(raw[:-2]):
                    packets.append(p)
    except (OSError, struct.error):
        return (None, [])
    return (header_info, packets)


# --- Monitor ---
@dataclass
class MonitorStats:
    packets_total: int = 0
    packets_lost: int = 0
    last_packet_id: int = -1
    last_ts: float = 0.0
    rate_hz: float = 0.0

    def update(self, p: SensorPacket) -> None:
        now = time.time()
        if self.last_packet_id >= 0 and p.packet_id > self.last_packet_id + 1:
            self.packets_lost += p.packet_id - self.last_packet_id - 1
        self.last_packet_id = p.packet_id
        self.packets_total += 1
        if self.last_ts > 0:
            self.rate_hz = 1.0 / (now - self.last_ts)
        self.last_ts = now


class SensorMonitor:
    def __init__(self, port: str, baudrate: int = 2000000):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.stats = MonitorStats()
        self._last_packet: Optional[SensorPacket] = None
        self._log_file = None
        self._log_path: Optional[Path] = None
        self._running = False
        self._lock = threading.Lock()
        self._rx_buffer = bytearray()
        self._last_rx_ts = 0.0

    def open(self) -> None:
        # Non-blocking reads; we manage buffering manually
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.0)
        self._running = True

    def close(self) -> None:
        self._running = False
        self.stop_log()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self._rx_buffer.clear()

    def _fill_rx_buffer(self) -> None:
        if not self.ser or not self.ser.is_open:
            return
        try:
            waiting = self.ser.in_waiting
        except Exception:
            waiting = 0
        if waiting <= 0:
            return
        data = self.ser.read(min(1024, waiting))
        if data:
            self._rx_buffer.extend(data)

    def _read_transport_packet(self) -> Optional[Tuple[int, bytes]]:
        """Читает TransportPacket и возвращает (packet_type, payload_bytes)."""
        if not self.ser or not self.ser.is_open:
            return None

        self._fill_rx_buffer()
        buf = self._rx_buffer

        # Ищем стартовый байт
        start_idx = buf.find(bytes([START_BYTE]))
        if start_idx < 0:
            buf.clear()
            return None
        if start_idx > 0:
            del buf[:start_idx]

        # Нужно минимум 3 байта для start + size
        if len(buf) < 3:
            return None

        packet_size = struct.unpack_from("<H", buf, 1)[0]

        # Формат: start(1) + size(2) + type(1) + data + crc(2) + end(1) — принимаем оба размера сенсорного пакета
        if len(buf) >= 4:
            packet_type = buf[3]
            if packet_type in (PACKET_TYPE_SENSOR_DATA, PACKET_TYPE_LOG):
                if packet_type == PACKET_TYPE_SENSOR_DATA:
                    # Хаб может слать старый (без motor_count) или новый размер
                    if packet_size == SENSOR_SIZE:
                        expected_size = SENSOR_SIZE
                    elif packet_size == SENSOR_SIZE_OLD:
                        expected_size = SENSOR_SIZE_OLD
                    else:
                        expected_size = None
                else:
                    expected_size = LOG_SIZE if packet_size == LOG_SIZE else None
                if expected_size is not None:
                    total_len = 1 + 2 + 1 + expected_size + 2 + 1
                    if len(buf) >= total_len and buf[total_len - 1] == END_BYTE:
                        payload = bytes(buf[4:4 + expected_size])
                        del buf[:total_len]
                        return (packet_type, payload)

        # Старый формат без type: start(1) + size(2) + data + crc(2) + end(1)
        for try_size in (SENSOR_SIZE, SENSOR_SIZE_OLD):
            if packet_size == try_size:
                total_len_old = 1 + 2 + try_size + 2 + 1
                if len(buf) >= total_len_old and buf[total_len_old - 1] == END_BYTE:
                    payload = bytes(buf[3:3 + try_size])
                    del buf[:total_len_old]
                    return (PACKET_TYPE_SENSOR_DATA, payload)
                break

        # Если данных мало, ждём следующего чтения
        if len(buf) < 3:
            return None

        # Если пакет битый, сдвигаем буфер на 1 байт
        del buf[0]
        return None

    def read_next_packet(self) -> Optional[Tuple[str, object]]:
        """Читает следующий пакет (sensor/log). Возвращает ('sensor', SensorPacket) или ('log', LogPacket)."""
        result = self._read_transport_packet()
        if not result:
            return None
        packet_type, payload = result

        if packet_type == PACKET_TYPE_SENSOR_DATA:
            if len(payload) == SENSOR_SIZE:
                p = SensorPacket.from_bytes(payload)
            elif len(payload) == SENSOR_SIZE_OLD:
                vals = struct.unpack(SENSOR_FORMAT_OLD, payload)
                # Вставить motor_count=1 перед crc16
                new_vals = vals[:-1] + (1,) + (vals[-1],)
                p = SensorPacket(*new_vals)
            else:
                return None
            if not p.validate_crc(payload[:-2]):
                return None
            with self._lock:
                self.stats.update(p)
                self._last_packet = p
                self._last_rx_ts = time.time()
            return ("sensor", p)

        if packet_type == PACKET_TYPE_LOG:
            if len(payload) != LOG_SIZE:
                return None
            log_pkt = LogPacket.from_bytes(payload)
            if not log_pkt.validate_crc(payload[:-2]):
                return None
            with self._lock:
                self._last_rx_ts = time.time()
            return ("log", log_pkt)

        return None

    def get_last_packet(self) -> Optional[SensorPacket]:
        with self._lock:
            return self._last_packet

    def get_last_rx_ts(self) -> float:
        with self._lock:
            return self._last_rx_ts

    def get_stats(self) -> MonitorStats:
        with self._lock:
            return MonitorStats(
                packets_total=self.stats.packets_total,
                packets_lost=self.stats.packets_lost,
                last_packet_id=self.stats.last_packet_id,
                last_ts=self.stats.last_ts,
                rate_hz=self.stats.rate_hz,
            )

    def start_log(self, path: Optional[Path] = None) -> None:
        self.stop_log()
        path = path or Path(f"EDFViewerMonitor_log_{int(time.time())}.bin")
        self._log_path = path
        self._log_file = open(path, "wb")
        p = self.get_last_packet()
        start_ts = p.timestamp_ms if p else 0
        self._log_file.write(make_file_header(SENSOR_SIZE, start_ts))

    def stop_log(self) -> None:
        if self._log_file:
            self._log_file.close()
            self._log_file = None
        self._log_path = None

    def log_packet(self, p: SensorPacket) -> None:
        if not self._log_file:
            return
        payload = struct.pack(
            SENSOR_FORMAT,
            p.packet_id, p.timestamp_ms,
            p.px4_0_pressure, p.px4_0_temp, p.px4_0_airspeed, p.px4_0_health,
            p.px4_1_pressure, p.px4_1_temp, p.px4_1_airspeed, p.px4_1_health,
            p.aht20_raw, p.aht20_temp, p.aht20_humidity, p.aht20_cal_id,
            p.bmp280_pressure, p.bmp280_temp, p.bmp280_altitude, p.bmp280_sensor_id,
            p.rc_throttle_us, p.mav_pitch_rad, p.mav_roll_rad, p.mav_airspeed_mps, p.mav_armed,
            p.cpu0_load_pct, p.cpu1_load_pct, p.cpu_temp_c,
            p.system_status, p.calibration_flags, p.crc16,
        )
        self._log_file.write(payload)
        self._log_file.flush()


def get_available_ports() -> list:
    """Список доступных COM-портов (имена устройств)."""
    try:
        return sorted(p.device for p in list_ports.comports())
    except Exception:
        return []


# --- Geometry (размер и позиция окна) ---
def _geometry_path() -> Path:
    return _app_dir() / CONFIG_FILENAME


def load_geometry() -> str:
    p = _geometry_path()
    if not p.exists():
        return DEFAULT_GEOMETRY
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        geom = data.get("geometry", DEFAULT_GEOMETRY)
        return geom if isinstance(geom, str) and geom else DEFAULT_GEOMETRY
    except (OSError, json.JSONDecodeError, TypeError):
        return DEFAULT_GEOMETRY


def load_sash_position() -> Optional[int]:
    """Позиция разделителя панелей (пиксели). None — не задана."""
    p = _geometry_path()
    if not p.exists():
        return None
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        v = data.get("sash_position")
        if v is not None and isinstance(v, int) and v > 0:
            return v
    except (OSError, json.JSONDecodeError, TypeError):
        pass
    return None


def save_geometry(geometry: str, sash_position: Optional[int] = None) -> None:
    if not geometry:
        return
    p = _geometry_path()
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        data = {}
    data["geometry"] = geometry
    if sash_position is not None and sash_position > 0:
        data["sash_position"] = sash_position
    try:
        with open(p, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except OSError:
        pass


def _default_geometry_from_screen(root: "tk.Tk") -> str:
    """Размер окна ~1/4 экрана (половина по ширине и высоте), по центру. Используется при первом запуске."""
    sw = root.winfo_screenwidth()
    sh = root.winfo_screenheight()
    w = max(600, sw // 2)
    h = max(500, sh // 2)
    x = max(0, (sw - w) // 2)
    y = max(0, (sh - h) // 2)
    return f"{w}x{h}+{x}+{y}"


MOTOR_MODE_ID1 = "ID1 (1 мотор)"
MOTOR_MODE_ID2 = "ID2 (2 мотора)"


def load_motor_mode() -> str:
    """Загружает сохранённый режим выбора мотора (ID1/ID2)."""
    p = _geometry_path()
    if not p.exists():
        return MOTOR_MODE_ID1
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        mode = data.get("motor_mode", MOTOR_MODE_ID1)
        if mode in (MOTOR_MODE_ID1, MOTOR_MODE_ID2):
            return mode
    except (OSError, json.JSONDecodeError, TypeError, KeyError):
        pass
    return MOTOR_MODE_ID1


def save_motor_mode(mode: str) -> None:
    """Сохраняет режим выбора мотора (ID1/ID2)."""
    if mode not in (MOTOR_MODE_ID1, MOTOR_MODE_ID2):
        return
    p = _geometry_path()
    try:
        try:
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            data = {}
        data["motor_mode"] = mode
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except OSError:
        pass


def load_mavlink_enabled() -> bool:
    """Загружает сохранённое состояние MAVLink (включен/выключен)."""
    p = _geometry_path()
    if not p.exists():
        return True  # По умолчанию включен
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data.get("mavlink_enabled", True)
    except (OSError, json.JSONDecodeError, TypeError, KeyError):
        pass
    return True


def save_mavlink_enabled(enabled: bool) -> None:
    """Сохраняет состояние MAVLink."""
    p = _geometry_path()
    try:
        try:
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            data = {}
        data["mavlink_enabled"] = enabled
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except OSError:
        pass


def load_imu1_enabled() -> bool:
    """Загружает сохранённое состояние IMU1 (BMI160) (включен/выключен)."""
    p = _geometry_path()
    if not p.exists():
        return False  # По умолчанию выключен
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data.get("imu1_enabled", False)
    except (OSError, json.JSONDecodeError, TypeError, KeyError):
        pass
    return False


def save_imu1_enabled(enabled: bool) -> None:
    """Сохраняет состояние IMU1."""
    p = _geometry_path()
    try:
        try:
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            data = {}
        data["imu1_enabled"] = enabled
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except OSError:
        pass


def load_checkbox_state() -> Tuple[bool, bool]:
    """Загружает состояние чекбоксов из конфигурационного файла."""
    p = _geometry_path()
    if not p.exists():
        return (True, True)  # По умолчанию оба включены
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        show_rc = data.get("show_rc_logs", True)
        show_i2c = data.get("show_i2c_logs", True)
        return (show_rc, show_i2c)
    except (OSError, json.JSONDecodeError, TypeError, KeyError):
        return (True, True)


def save_checkbox_state(show_rc: bool, show_i2c: bool) -> None:
    """Сохраняет состояние чекбоксов в конфигурационный файл."""
    p = _geometry_path()
    try:
        try:
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            data = {}
        
        data["show_rc_logs"] = show_rc
        data["show_i2c_logs"] = show_i2c
        
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except OSError:
        pass


def check_single_instance() -> bool:
    """Проверяет, не запущена ли уже другая копия программы. Возвращает True если можно запускаться."""
    lock_path = _app_dir() / LOCK_FILENAME
    try:
        # Пытаемся создать файл блокировки
        if lock_path.exists():
            # Проверяем, не устарел ли файл (если процесс завершился некорректно)
            try:
                import time as time_module
                # Если файл старше 5 минут, считаем что программа завершилась некорректно
                if time_module.time() - lock_path.stat().st_mtime > 300:
                    lock_path.unlink()
                else:
                    return False  # Другая копия работает
            except OSError:
                pass
        
        # Создаем файл блокировки
        lock_path.write_text(str(os.getpid()))
        return True
    except OSError:
        return False


def remove_lock_file() -> None:
    """Удаляет файл блокировки при закрытии программы."""
    lock_path = _app_dir() / LOCK_FILENAME
    try:
        if lock_path.exists():
            lock_path.unlink()
    except OSError:
        pass


# --- GUI ---
def run_gui(port: str, baudrate: int, playback_path: Optional[str] = None) -> None:
    try:
        import tkinter as tk
        from tkinter import ttk, filedialog, messagebox
    except ImportError:
        print("tkinter not available, run headless: python sensor_monitor.py --port COM3 --no-gui")
        return

    monitor = SensorMonitor(port, baudrate)
    root = tk.Tk()
    root.withdraw()

    def _draw_vertical_gradient(canvas: "tk.Canvas", width: int, height: int, colors: List[str]) -> None:
        if height <= 0 or width <= 0:
            return
        steps = max(height, 1)
        stops = [int(i * (steps - 1) / (len(colors) - 1)) for i in range(len(colors))]
        rgb = [tuple(int(colors[i][j:j + 2], 16) for j in (1, 3, 5)) for i in range(len(colors))]
        for y in range(steps):
            for i in range(len(stops) - 1):
                if stops[i] <= y <= stops[i + 1]:
                    t = 0.0 if stops[i + 1] == stops[i] else (y - stops[i]) / (stops[i + 1] - stops[i])
                    r = int(rgb[i][0] + (rgb[i + 1][0] - rgb[i][0]) * t)
                    g = int(rgb[i][1] + (rgb[i + 1][1] - rgb[i][1]) * t)
                    b = int(rgb[i][2] + (rgb[i + 1][2] - rgb[i][2]) * t)
                    color = f"#{r:02x}{g:02x}{b:02x}"
                    canvas.create_line(0, y, width, y, fill=color)
                    break

    def _show_splash(duration_ms: int = 2200) -> None:
        splash = tk.Toplevel(root)
        splash.overrideredirect(True)
        splash.attributes("-topmost", True)
        width, height = 520, 220
        screen_w = splash.winfo_screenwidth()
        screen_h = splash.winfo_screenheight()
        x = int((screen_w - width) / 2)
        y = int((screen_h - height) / 2)
        splash.geometry(f"{width}x{height}+{x}+{y}")

        canvas = tk.Canvas(splash, width=width, height=height, highlightthickness=0)
        canvas.pack(fill="both", expand=True)
        _draw_vertical_gradient(canvas, width, height, ["#1b1b1d", "#2a3c55", "#2c6faa"])

        canvas.create_text(
            width // 2,
            height // 2 - 8,
            text="EDFViewer NEXT",
            fill="#ff9f2d",
            font=("Segoe UI Semibold", 28)
        )
        canvas.create_text(
            width // 2,
            height // 2 + 26,
            text="Monitor",
            fill="#5fb7ff",
            font=("Segoe UI", 20)
        )
        # Версия и сборка внизу заставки
        ver_str, build_str = get_project_version()
        canvas.create_text(
            width // 2,
            height - 22,
            text=f"{ver_str}  ·  Сборка {build_str}",
            fill="#8a9ba8",
            font=("Segoe UI", 11)
        )

        def close_splash() -> None:
            try:
                splash.destroy()
            finally:
                root.deiconify()

        splash.after(duration_ms, close_splash)

    _show_splash()
    root.title(f"EDFViewerMonitor {EDFVIEWER_MONITOR_VERSION}")
    geom = load_geometry()
    if geom == DEFAULT_GEOMETRY:
        geom = _default_geometry_from_screen(root)
    root.geometry(geom)
    root.minsize(520, 420)

    # Bloomberg-style dark theme
    BG_DARK = "#1a1a1a"
    FG_LABEL = "#b0b0b0"
    FG_DATA = "#e6e6e6"
    ACCENT = "#e69833"
    OK_GREEN = "#4ec9b0"
    ERR_RED = "#f48771"
    root.configure(bg=BG_DARK)
    try:
        style = ttk.Style()
        style.theme_use("clam")
        for w in ("TFrame", "TLabelframe", "TLabelframe.Label", "TLabel", "TButton", "TCombobox"):
            style.configure(w, background=BG_DARK, foreground=FG_LABEL, fieldbackground=BG_DARK)
        style.configure("TLabelframe.Label", foreground=ACCENT)
        style.map("TButton", background=[("active", "#2d2d2d")], foreground=[("active", FG_LABEL)])
        style.map("TCombobox", fieldbackground=[("readonly", BG_DARK)], background=[("readonly", "#2d2d2d")])
    except Exception:
        pass

    # Выбор COM-порта и подключение
    frame_conn = ttk.LabelFrame(root, text="Подключение")
    frame_conn.pack(fill="x", padx=5, pady=5)
    row_conn = ttk.Frame(frame_conn)
    row_conn.pack(fill="x")
    ttk.Label(row_conn, text="COM-порт:").pack(side="left", padx=(0, 4))
    ports_var = tk.StringVar(value=port)
    ports_list = get_available_ports()
    if port and port not in ports_list:
        ports_list = [port] + ports_list
    combobox_ports = ttk.Combobox(
        row_conn, textvariable=ports_var, values=ports_list,
        state="readonly", width=14
    )
    combobox_ports.pack(side="left", padx=2)
    if ports_list:
        combobox_ports.set(port if port in ports_list else ports_list[0])
    btn_refresh = ttk.Button(row_conn, text="Обновить")
    btn_connect = ttk.Button(row_conn, text="Подключить")
    btn_disconnect = ttk.Button(row_conn, text="Отключить", state="disabled")
    btn_refresh.pack(side="left", padx=4)
    btn_connect.pack(side="left", padx=2)
    btn_disconnect.pack(side="left", padx=2)
    ttk.Label(row_conn, text="Скорость: %d" % baudrate).pack(side="left", padx=(12, 0))

    reader_thread_started = {"value": False}

    def refresh_ports_list() -> None:
        new_ports = get_available_ports()
        ports_list.clear()
        ports_list.extend(new_ports)
        combobox_ports["values"] = ports_list
        current = combobox_ports.get()
        if ports_list:
            if current not in ports_list:
                combobox_ports.set(ports_list[0])
        else:
            ports_var.set("")

    def set_connected(connected: bool) -> None:
        if connected:
            combobox_ports.config(state="disabled")
            btn_connect.config(state="disabled")
            btn_refresh.config(state="disabled")
            btn_disconnect.config(state="normal")
        else:
            combobox_ports.config(state="readonly")
            btn_connect.config(state="normal")
            btn_refresh.config(state="normal")
            btn_disconnect.config(state="disabled")

    def do_connect() -> None:
        chosen = (combobox_ports.get() or "").strip()
        if not chosen:
            messagebox.showwarning("Порт", "Выберите COM-порт из списка.")
            return
        if monitor.ser and monitor.ser.is_open:
            return
        monitor.port = chosen
        monitor.baudrate = baudrate
        try:
            monitor.open()
        except Exception as e:
            messagebox.showerror("Ошибка", "Не удалось открыть порт %s:\n%s" % (chosen, e))
            return
        set_connected(True)
        reader_thread_started["value"] = True
        rt = threading.Thread(target=reader_thread_body, daemon=True)
        rt.start()
        # Отправить все команды конфигурации на хаб через 0.5 с после открытия порта (чтобы порт не сбрасывался)
        root.after(500, lambda: send_all_config_commands())

    def do_disconnect() -> None:
        monitor.close()
        set_connected(False)
        reader_thread_started["value"] = False

    btn_refresh.config(command=refresh_ports_list)
    btn_connect.config(command=do_connect)
    btn_disconnect.config(command=do_disconnect)

    # Горизонтальная ориентация: слева — Данные, справа — Логи (оба масштабируются при изменении окна)
    main_paned = ttk.PanedWindow(root, orient="horizontal")
    main_paned.pack(fill="both", expand=True, padx=5, pady=5)

    left_frame = ttk.Frame(main_paned)
    main_paned.add(left_frame, weight=1)

    right_frame = ttk.Frame(main_paned)
    main_paned.add(right_frame, weight=1)

    var_connection = tk.StringVar(value="Ожидание пакетов...")
    var_mode = tk.StringVar(value=load_motor_mode())
    var_id2_hub = tk.StringVar(value="ID2 хаб: —")

    # Строка: режим ID1/ID2 + статус + ID2 хаб
    row_toolbar = ttk.Frame(left_frame)
    row_toolbar.pack(fill="x", padx=5, pady=2)
    ttk.Label(row_toolbar, text="Режим:").pack(side="left", padx=(0, 4))
    mode_combo = ttk.Combobox(
        row_toolbar, textvariable=var_mode,
        values=[MOTOR_MODE_ID1, MOTOR_MODE_ID2],
        state="readonly", width=16
    )
    mode_combo.pack(side="left", padx=2)

    def send_motor_count_to_hub() -> None:
        """Отправить текущий режим (ID1/ID2) на Main Hub → GlassHub. Дважды для надёжности."""
        mode = var_mode.get()
        if not (monitor.ser and monitor.ser.is_open):
            return
        cmd = b"MOTOR_COUNT=2\n" if mode == MOTOR_MODE_ID2 else b"MOTOR_COUNT=1\n"
        try:
            for _ in range(2):
                monitor.ser.write(cmd)
            monitor.ser.flush()
            print(f"[DEBUG] Sent command: {cmd.decode('ascii', errors='replace').strip()}")
        except Exception as e:
            print(f"[DEBUG] Failed to send command: {e}")

    def on_motor_mode_change(*_args) -> None:
        mode = var_mode.get()
        save_motor_mode(mode)
        send_motor_count_to_hub()

    var_mode.trace_add("write", on_motor_mode_change)
    # Событие выбора в комбобоксе — гарантированно отправляем команду при смене ID1/ID2
    mode_combo.bind("<<ComboboxSelected>>", lambda _e: send_motor_count_to_hub())
    ttk.Label(row_toolbar, textvariable=var_connection, foreground=ACCENT).pack(side="left", padx=(16, 0))
    lbl_id2 = tk.Label(row_toolbar, textvariable=var_id2_hub, bg=BG_DARK, fg=FG_LABEL, font=("TkDefaultFont", 9))
    lbl_id2.pack(side="left", padx=(16, 0))

    # Строка с чекбоксами MAVLink и IMU1
    row_checkboxes = ttk.Frame(left_frame)
    row_checkboxes.pack(fill="x", padx=5, pady=2)
    
    var_mavlink_enabled = tk.BooleanVar(value=load_mavlink_enabled())
    var_imu1_enabled = tk.BooleanVar(value=load_imu1_enabled())
    
    def send_mavlink_enable(enabled: bool) -> None:
        """Отправить команду включения/отключения MAVLink на Main Hub."""
        if not (monitor.ser and monitor.ser.is_open):
            return
        cmd = b"MAVLINK_ENABLE=1\n" if enabled else b"MAVLINK_ENABLE=0\n"
        try:
            for _ in range(2):
                monitor.ser.write(cmd)
            monitor.ser.flush()
            print(f"[DEBUG] Sent command: {cmd.decode('ascii', errors='replace').strip()}")
        except Exception as e:
            print(f"[DEBUG] Failed to send command: {e}")
    
    def send_imu1_enable(enabled: bool) -> None:
        """Отправить команду включения/отключения IMU1 на Main Hub."""
        if not (monitor.ser and monitor.ser.is_open):
            return
        cmd = b"IMU1_ENABLE=1\n" if enabled else b"IMU1_ENABLE=0\n"
        try:
            for _ in range(2):
                monitor.ser.write(cmd)
            monitor.ser.flush()
            print(f"[DEBUG] Sent command: {cmd.decode('ascii', errors='replace').strip()}")
        except Exception as e:
            print(f"[DEBUG] Failed to send command: {e}")
    
    def on_mavlink_checkbox_change(*_args) -> None:
        enabled = var_mavlink_enabled.get()
        save_mavlink_enabled(enabled)
        send_mavlink_enable(enabled)
        # Если MAVLink включается, отключаем IMU1
        if enabled:
            var_imu1_enabled.set(False)
            save_imu1_enabled(False)
            send_imu1_enable(False)
    
    def on_imu1_checkbox_change(*_args) -> None:
        enabled = var_imu1_enabled.get()
        save_imu1_enabled(enabled)
        send_imu1_enable(enabled)
        # Если IMU1 включается, отключаем MAVLink
        if enabled:
            var_mavlink_enabled.set(False)
            save_mavlink_enabled(False)
            send_mavlink_enable(False)
    
    var_mavlink_enabled.trace_add("write", on_mavlink_checkbox_change)
    var_imu1_enabled.trace_add("write", on_imu1_checkbox_change)
    
    chk_mavlink = ttk.Checkbutton(
        row_checkboxes,
        text="MAVLink",
        variable=var_mavlink_enabled
    )
    chk_mavlink.pack(side="left", padx=(0, 16))
    
    chk_imu1 = ttk.Checkbutton(
        row_checkboxes,
        text="IMU1 (BMI160)",
        variable=var_imu1_enabled
    )
    chk_imu1.pack(side="left", padx=(0, 0))
    
    def send_all_config_commands() -> None:
        """Отправить все команды конфигурации на Main Hub после подключения."""
        send_motor_count_to_hub()
        send_mavlink_enable(var_mavlink_enabled.get())
        send_imu1_enable(var_imu1_enabled.get())

    # Единый блок данных — терминал (перезаписывается только он, масштабируется с окном)
    frame_data = ttk.LabelFrame(left_frame, text="Данные")
    frame_data.pack(fill="both", expand=True, padx=5, pady=2)
    data_text = tk.Text(
        frame_data,
        wrap="none",
        font=("Consolas", 10),
        bg=BG_DARK,
        fg=FG_DATA,
        insertbackground=ACCENT,
        selectbackground="#264f78",
        exportselection=True,
        state="disabled",
        cursor="arrow",
        padx=8,
        pady=6,
    )
    data_text.tag_configure("key", foreground=FG_LABEL)
    data_text.tag_configure("value", foreground=ACCENT)
    data_scroll = ttk.Scrollbar(frame_data, orient="vertical", command=data_text.yview)
    data_text.configure(yscrollcommand=data_scroll.set)
    data_text.pack(side="left", fill="both", expand=True)
    data_scroll.pack(side="right", fill="y")
    data_text.bind("<Key>", lambda e: "break")
    data_text.bind("<Button-1>", lambda e: data_text.focus_set())

    KEY_WIDTH = 22
    BLOCK_WIDTH = 34
    MIN_WIDTH_PER_COL = 280

    def format_data_block(p: Optional[object], show_id2: bool, is_playback: bool = False, num_cols: int = 1) -> str:
        if p is None:
            return "  Ожидание пакетов...\n  Выберите порт и нажмите «Подключить»\n"
        flat = []
        flat.append(("status", "Воспроизведение" if is_playback else "OK"))
        flat.append(("packet_id", str(p.packet_id)))
        flat.append(("timestamp_ms", str(p.timestamp_ms)))
        flat.append(("px4_id1.pressure_pa", f"{p.px4_0_pressure:.2f}"))
        flat.append(("px4_id1.temp_C", f"{p.px4_0_temp:.2f}"))
        flat.append(("px4_id1.airspeed_m_s", f"{p.px4_0_airspeed:.2f}"))
        flat.append(("px4_id1.health", str(p.px4_0_health)))
        if show_id2:
            id2_err = p.system_status & SYS_STATUS_PX4_2_ERR
            hub2_status = "нет связи" if id2_err else "OK"
            flat.append(("hub_id2_packets", hub2_status))
            flat.append(("px4_id2.pressure_pa", f"{p.px4_1_pressure:.2f}"))
            flat.append(("px4_id2.temp_C", f"{p.px4_1_temp:.2f}"))
            flat.append(("px4_id2.airspeed_m_s", f"{p.px4_1_airspeed:.2f}"))
            flat.append(("px4_id2.health", str(p.px4_1_health)))
        flat.append(("aht20.raw_adc", str(p.aht20_raw)))
        flat.append(("aht20.temp_C", f"{p.aht20_temp:.2f}"))
        flat.append(("aht20.humidity_pct", f"{p.aht20_humidity:.2f}"))
        flat.append(("aht20.cal_id", str(p.aht20_cal_id)))
        flat.append(("bmp280.pressure_hPa", f"{p.bmp280_pressure / 100.0:.2f}"))
        flat.append(("bmp280.temp_C", f"{p.bmp280_temp:.2f}"))
        flat.append(("bmp280.altitude_m", f"{p.bmp280_altitude:.2f}"))
        flat.append(("bmp280.sensor_id", str(p.bmp280_sensor_id)))
        rc_pct = (p.rc_throttle_us - 1050) / (1950 - 1050) * 100.0 if p.rc_throttle_us else 0.0
        rc_pct = max(0.0, min(100.0, rc_pct))
        flat.append(("rc_throttle_us", str(p.rc_throttle_us)))
        flat.append(("rc_throttle_pct", f"{rc_pct:.1f}%"))
        flat.append(("mav_pitch_rad", f"{p.mav_pitch_rad:.4f}"))
        flat.append(("mav_roll_rad", f"{p.mav_roll_rad:.4f}"))
        flat.append(("mav_airspeed_mps", f"{p.mav_airspeed_mps:.2f}"))
        flat.append(("mav_armed", "ARM" if p.mav_armed == 1 else "DISARM" if p.mav_armed == 0 else "—"))
        flat.append(("cpu0_load_pct", f"{p.cpu0_load_pct:.1f}%"))
        flat.append(("cpu1_load_pct", f"{p.cpu1_load_pct:.1f}%"))
        flat.append(("cpu_temp_C", f"{p.cpu_temp_c:.1f}"))
        flat.append(("system_status", str(p.system_status)))
        calib = []
        if p.calibration_flags & 0x01:
            calib.append("BMP280")
        if p.calibration_flags & 0x02:
            calib.append("PX4_1")
        if p.calibration_flags & 0x04:
            calib.append("PX4_2")
        if p.calibration_flags & 0x08:
            calib.append("AHT20")
        flat.append(("calibration_flags", " ".join(calib) if calib else "—"))
        if num_cols <= 1:
            return "\n".join(f"{k:<{KEY_WIDTH}}{v}" for k, v in flat) + "\n"
        total = len(flat)
        per_col = (total + num_cols - 1) // num_cols
        columns = []
        for c in range(num_cols):
            start = c * per_col
            end = min(start + per_col, total)
            col_lines = []
            for i in range(start, end):
                k, v = flat[i]
                cell = (k[:KEY_WIDTH].ljust(KEY_WIDTH) + str(v)[:BLOCK_WIDTH - KEY_WIDTH]).ljust(BLOCK_WIDTH)
                col_lines.append(cell)
            columns.append(col_lines)
        max_rows = max(len(col) for col in columns)
        out_lines = []
        for row in range(max_rows):
            parts = []
            for col in columns:
                if row < len(col):
                    parts.append(col[row])
                else:
                    parts.append(" " * BLOCK_WIDTH)
            out_lines.append("".join(parts))
        return "\n".join(out_lines) + "\n"

    def refresh_data_block(content: str, num_cols: int = 1) -> None:
        data_text.configure(state="normal")
        data_text.delete("1.0", "end")
        data_text.insert("1.0", content)
        lines = content.split("\n")
        for i, line in enumerate(lines, 1):
            if not line.strip():
                continue
            if num_cols <= 1:
                if len(line) > KEY_WIDTH:
                    data_text.tag_add("key", f"{i}.0", f"{i}.{KEY_WIDTH}")
                    data_text.tag_add("value", f"{i}.{KEY_WIDTH}", f"{i}.{len(line)}")
            else:
                for c in range(num_cols):
                    start = c * BLOCK_WIDTH
                    end_key = min(start + KEY_WIDTH, len(line))
                    end_block = min(start + BLOCK_WIDTH, len(line))
                    if end_key > start:
                        data_text.tag_add("key", f"{i}.{start}", f"{i}.{end_key}")
                    if end_block > end_key:
                        data_text.tag_add("value", f"{i}.{end_key}", f"{i}.{end_block}")
        data_text.configure(state="disabled")

    refresh_data_block(format_data_block(None, False, False, 1), 1)

    # Статистика (под данными)
    frame_stats = ttk.LabelFrame(left_frame, text="Статистика")
    frame_stats.pack(fill="x", padx=5, pady=2)
    var_rate = tk.StringVar(value="0")
    var_total = tk.StringVar(value="0")
    var_lost = tk.StringVar(value="0")
    row_s = ttk.Frame(frame_stats)
    row_s.pack(fill="x")
    ttk.Label(row_s, text="Пакетов/с:").pack(side="left")
    ttk.Label(row_s, textvariable=var_rate).pack(side="left", padx=8)
    ttk.Label(row_s, text="Всего:").pack(side="left")
    ttk.Label(row_s, textvariable=var_total).pack(side="left", padx=8)
    ttk.Label(row_s, text="Потеряно:").pack(side="left")
    ttk.Label(row_s, textvariable=var_lost).pack(side="left", padx=8)

    # Панель логов — справа от данных (масштабируется с окном)
    frame_log_panel = ttk.LabelFrame(right_frame, text="Логи системы (online)")
    frame_log_panel.pack(fill="both", expand=True, padx=5, pady=5)
    
    # Text widget с прокруткой для логов
    log_text_frame = ttk.Frame(frame_log_panel)
    log_text_frame.pack(fill="both", expand=True, padx=2, pady=2)
    
    log_text = tk.Text(log_text_frame, wrap="word", font=("Consolas", 9), bg="#1e1e1e", fg="#d4d4d4",
                       insertbackground="#ffffff", selectbackground="#264f78", 
                       exportselection=True)  # Разрешаем экспорт выделенного текста в буфер обмена
    log_scrollbar = ttk.Scrollbar(log_text_frame, orient="vertical", command=log_text.yview)
    log_text.config(yscrollcommand=log_scrollbar.set)
    log_text.pack(side="left", fill="both", expand=True)
    log_scrollbar.pack(side="right", fill="y")
    
    # Блокируем только вставку и вырезание, все остальное (включая выделение) разрешено
    def on_key_press(event):
        # Блокируем только вставку и вырезание
        if event.state & 0x4:  # Control key pressed
            if event.keysym.lower() == 'v':  # Ctrl+V - вставка
                return "break"
            elif event.keysym.lower() == 'x':  # Ctrl+X - вырезание
                return "break"
        # Блокируем прямой ввод текста (но разрешаем все остальное включая выделение)
        if len(event.char) > 0 and ord(event.char) >= 32 and not (event.state & 0x4):
            return "break"
        return None
    
    log_text.bind("<Key>", on_key_press)
    
    # Функции для работы с логами (определяем до использования в меню)
    def clear_logs():
        log_text.config(state="normal")
        log_text.delete(1.0, "end")
        log_text.config(state="normal")
    
    def save_logs_to_file():
        try:
            from datetime import datetime
            filename = filedialog.asksaveasfilename(
                defaultextension=".txt",
                filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
                initialfile=f"EDFViewerMonitor_logs_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            )
            if filename:
                log_text.config(state="normal")
                content = log_text.get(1.0, "end-1c")
                log_text.config(state="normal")
                with open(filename, "w", encoding="utf-8") as f:
                    f.write(content)
                messagebox.showinfo("Сохранение", f"Логи сохранены в файл:\n{filename}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Не удалось сохранить логи:\n{e}")
    
    # Контекстное меню для копирования и сохранения
    def show_context_menu(event):
        try:
            context_menu = tk.Menu(root, tearoff=0)
            context_menu.add_command(label="Копировать", command=lambda: log_text.event_generate("<<Copy>>"))
            context_menu.add_command(label="Выделить все", command=lambda: log_text.tag_add("sel", "1.0", "end"))
            context_menu.add_separator()
            context_menu.add_command(label="Сохранить логи в файл...", command=save_logs_to_file)
            context_menu.tk_popup(event.x_root, event.y_root)
        finally:
            context_menu.grab_release()
    
    log_text.bind("<Button-3>", show_context_menu)  # Правая кнопка мыши для контекстного меню
    
    # Убеждаемся что виджет получает фокус при клике для выделения текста
    def on_click(event):
        log_text.focus_set()
        return None
    
    log_text.bind("<Button-1>", on_click)  # Левая кнопка мыши устанавливает фокус

    # Теги для цветовой кодировки
    log_text.tag_config("ERROR", foreground="#f48771", background="#1e1e1e")
    log_text.tag_config("WARN", foreground="#dcdcaa", background="#1e1e1e")
    log_text.tag_config("INFO", foreground="#4ec9b0", background="#1e1e1e")
    log_text.tag_config("DEBUG", foreground="#9cdcfe", background="#1e1e1e")
    log_text.tag_config("VERBOSE", foreground="#808080", background="#1e1e1e")
    
    # Кнопки управления логами
    log_controls = ttk.Frame(frame_log_panel)
    log_controls.pack(fill="x", padx=2, pady=2)
    
    ttk.Button(log_controls, text="Очистить", command=clear_logs).pack(side="left", padx=2)
    ttk.Button(log_controls, text="Сохранить в файл...", command=save_logs_to_file).pack(side="left", padx=2)
    
    # Чекбоксы для фильтрации логов (загружаем сохраненное состояние)
    show_rc_default, show_i2c_default = load_checkbox_state()
    var_show_rc_logs = tk.BooleanVar(value=show_rc_default)
    var_show_i2c_logs = tk.BooleanVar(value=show_i2c_default)
    
    # Обработчики для сохранения состояния при изменении
    def on_rc_checkbox_change():
        save_checkbox_state(var_show_rc_logs.get(), var_show_i2c_logs.get())
    
    def on_i2c_checkbox_change():
        save_checkbox_state(var_show_rc_logs.get(), var_show_i2c_logs.get())
    
    ttk.Checkbutton(log_controls, text="RC", variable=var_show_rc_logs, 
                    command=on_rc_checkbox_change).pack(side="left", padx=4)
    ttk.Checkbutton(log_controls, text="I2C", variable=var_show_i2c_logs,
                    command=on_i2c_checkbox_change).pack(side="left", padx=4)
    
    # Log (бинарный лог в файл)
    frame_log = ttk.Frame(left_frame)
    frame_log.pack(fill="x", padx=5, pady=2)
    var_log_status = tk.StringVar(value="Лог: выкл.")
    log_path_var = tk.StringVar(value="")

    def start_log() -> None:
        path = filedialog.asksaveasfilename(
            defaultextension=".bin",
            filetypes=[("Binary log", "*.bin")],
        )
        if path:
            monitor.start_log(Path(path))
            log_path_var.set(path)
            var_log_status.set("Лог: вкл. " + path)

    def stop_log() -> None:
        monitor.stop_log()
        log_path_var.set("")
        var_log_status.set("Лог: выкл.")

    ttk.Button(frame_log, text="Начать лог", command=start_log).pack(side="left", padx=2)
    ttk.Button(frame_log, text="Остановить лог", command=stop_log).pack(side="left", padx=2)
    ttk.Label(frame_log, textvariable=var_log_status).pack(side="left", padx=8)

    # Воспроизведение .bin лога (все значения по выбранному пакету)
    playback: dict = {
        "packets": [],
        "index": 0,
        "mode": False,
        "path": "",
        "header": None,
    }

    frame_playback = ttk.LabelFrame(left_frame, text="Воспроизведение лога (.bin)")
    frame_playback.pack(fill="x", padx=5, pady=2)
    row_pb = ttk.Frame(frame_playback)
    row_pb.pack(fill="x")
    var_playback_status = tk.StringVar(value="Лог не открыт")
    var_playback_pos = tk.StringVar(value="0 / 0")
    ttk.Label(row_pb, textvariable=var_playback_status, width=45).pack(side="left", padx=4)
    ttk.Label(row_pb, textvariable=var_playback_pos).pack(side="left")
    slider_playback = ttk.Scale(frame_playback, from_=0, to=0, orient="horizontal", length=400)

    def do_load_playback(path: str) -> None:
        path_obj = Path(path)
        if not path_obj.exists():
            messagebox.showerror("Ошибка", "Файл не найден: %s" % path)
            return
        header_info, packets = load_bin_log(path_obj)
        if not header_info:
            messagebox.showerror("Ошибка", "Не удалось прочитать лог: неверный формат или файл повреждён.")
            return
        if not packets:
            messagebox.showwarning("Лог пуст", "В файле нет пакетов с верным CRC.")
            return
        playback["header"] = header_info
        playback["packets"] = packets
        playback["index"] = 0
        playback["mode"] = True
        playback["path"] = str(path_obj)
        var_playback_status.set(path_obj.name)
        n = len(packets)
        var_playback_pos.set("1 / %d" % n)
        slider_playback.config(from_=0, to=max(0, n - 1))
        slider_playback.set(0)

    def open_playback() -> None:
        init_dir = Path.home() / "Documents" / "LOGEDF"
        path = filedialog.askopenfilename(
            title="Открыть бинарный лог SDL1",
            filetypes=[("Binary log", "*.bin"), ("All files", "*.*")],
            initialdir=str(init_dir) if init_dir.exists() else None,
        )
        if path:
            do_load_playback(path)

    def close_playback() -> None:
        playback["mode"] = False
        playback["packets"] = []
        playback["index"] = 0
        playback["path"] = ""
        playback["header"] = None
        var_playback_status.set("Лог не открыт")
        var_playback_pos.set("0 / 0")
        slider_playback.config(from_=0, to=0)

    def on_slider_change(*_args) -> None:
        if not playback["mode"] or not playback["packets"]:
            return
        try:
            val = slider_playback.get()
            i = int(float(val))
            i = max(0, min(i, len(playback["packets"]) - 1))
            playback["index"] = i
            var_playback_pos.set("%d / %d" % (i + 1, len(playback["packets"])))
        except (ValueError, TypeError):
            pass

    ttk.Button(row_pb, text="Открыть лог (.bin)", command=open_playback).pack(side="left", padx=2)
    ttk.Button(row_pb, text="Закрыть лог", command=close_playback).pack(side="left", padx=2)
    slider_playback.pack(fill="x", padx=5, pady=4)
    slider_playback.config(command=on_slider_change)

    def update_ui() -> None:
        if playback["mode"] and playback["packets"]:
            idx = min(playback["index"], len(playback["packets"]) - 1)
            p = playback["packets"][idx]
            var_connection.set("Воспроизведение: %s" % Path(playback["path"]).name)
            var_rate.set("—")
            var_total.set("%d" % len(playback["packets"]))
            var_lost.set("—")
            if var_mode.get() == MOTOR_MODE_ID2:
                id2_err = p.system_status & SYS_STATUS_PX4_2_ERR
                var_id2_hub.set("ID2 хаб: нет связи" if id2_err else "ID2 хаб: OK")
                lbl_id2.configure(fg=OK_GREEN if not id2_err else ERR_RED)
            else:
                var_id2_hub.set("ID2 хаб: —")
                lbl_id2.configure(fg=FG_LABEL)
        else:
            p = monitor.get_last_packet()
            if p is None:
                if not monitor.ser or not monitor.ser.is_open:
                    var_connection.set("Не подключено — выберите порт и нажмите «Подключить»")
                else:
                    last_rx = monitor.get_last_rx_ts()
                    if last_rx > 0 and (time.time() - last_rx) < 1.5:
                        var_connection.set("Связь есть, ждём данные...")
                    else:
                        var_connection.set("Ожидание пакетов...")
                var_rate.set("0")
                var_total.set("0")
                var_lost.set("0")
                var_id2_hub.set("ID2 хаб: —")
                lbl_id2.configure(fg=FG_LABEL)
            else:
                var_connection.set("OK — данные поступают")
                if var_mode.get() == MOTOR_MODE_ID2:
                    id2_err = p.system_status & SYS_STATUS_PX4_2_ERR
                    var_id2_hub.set("ID2 хаб: нет связи" if id2_err else "ID2 хаб: OK")
                    lbl_id2.configure(fg=OK_GREEN if not id2_err else ERR_RED)
                else:
                    var_id2_hub.set("ID2 хаб: —")
                    lbl_id2.configure(fg=FG_LABEL)
            if not playback["mode"]:
                s = monitor.get_stats()
                var_rate.set(f"{s.rate_hz:.1f}")
                var_total.set(str(s.packets_total))
                var_lost.set(str(s.packets_lost))
        show_id2 = var_mode.get() == MOTOR_MODE_ID2
        w = data_text.winfo_width()
        num_cols = max(1, w // MIN_WIDTH_PER_COL) if w > 50 else 1
        content = format_data_block(p, show_id2, playback["mode"], num_cols)
        refresh_data_block(content, num_cols)
        root.after(100, update_ui)

    def append_log(log_pkt: LogPacket) -> None:
        # Фильтрация по чекбоксам RC и I2C
        tag_lower = log_pkt.tag.lower().strip()
        
        # Проверка RC логов (тег содержит "rc" или "pwm")
        is_rc_log = "rc" in tag_lower or "pwm" in tag_lower
        if is_rc_log and not var_show_rc_logs.get():
            return  # Пропускаем RC логи, если чекбокс выключен
        
        # Проверка I2C логов (тег содержит "i2c", "mux", "aht20", "bmp280", "px4")
        is_i2c_log = any(keyword in tag_lower for keyword in ["i2c", "mux", "aht20", "bmp280", "px4"])
        if is_i2c_log and not var_show_i2c_logs.get():
            return  # Пропускаем I2C логи, если чекбокс выключен
        
        # Определяем тег для цветовой кодировки
        level_tag = "INFO"
        if log_pkt.level == LOG_LEVEL_ERROR:
            level_tag = "ERROR"
        elif log_pkt.level == LOG_LEVEL_WARN:
            level_tag = "WARN"
        elif log_pkt.level == LOG_LEVEL_DEBUG:
            level_tag = "DEBUG"
        elif log_pkt.level == LOG_LEVEL_VERBOSE:
            level_tag = "VERBOSE"

        timestamp_str = f"[{log_pkt.timestamp_ms:08d}ms]"
        log_line = f"{timestamp_str} [{log_pkt.tag:16s}] {log_pkt.message}\n"

        log_text.config(state="normal")
        log_text.insert("end", log_line, level_tag)
        log_text.see("end")
        lines = int(log_text.index("end-1c").split(".")[0])
        if lines > 1000:
            log_text.delete("1.0", f"{lines - 1000}.0")
        log_text.config(state="normal")  # Оставляем normal для возможности выделения

    def reader_thread_body() -> None:
        """Поток для чтения пакетов данных и логов."""
        consecutive_none = 0
        while monitor._running and root.winfo_exists():
            try:
                result = monitor.read_next_packet()
                if result:
                    consecutive_none = 0
                    kind, pkt = result
                    if kind == "sensor":
                        monitor.log_packet(pkt)
                    elif kind == "log":
                        try:
                            root.after(0, lambda lp=pkt: append_log(lp))
                        except Exception:
                            pass
                else:
                    consecutive_none += 1
                    # Если долго нет пакетов, увеличиваем задержку
                    if consecutive_none > 100:
                        time.sleep(0.1)
                    else:
                        time.sleep(0.01)  # Небольшая задержка если пакетов нет
            except Exception as e:
                # Не прерываем поток при ошибках чтения
                time.sleep(0.1)
        try:
            root.after(0, lambda: set_connected(False))
        except Exception:
            pass
    

    root.after(100, update_ui)
    # Восстановление позиции разделителя панелей после отрисовки
    saved_sash = load_sash_position()
    if saved_sash is not None:
        def apply_sash() -> None:
            root.update_idletasks()
            try:
                main_paned.sashpos(0, saved_sash)
            except Exception:
                pass
        root.after(100, apply_sash)
    # Автоподключение, если порт передан в командной строке и есть в списке
    if port and port in get_available_ports():
        root.after(300, do_connect)
    # Открыть лог для воспроизведения при запуске (--playback путь)
    if playback_path:
        root.after(500, lambda: do_load_playback(playback_path))

    def on_closing() -> None:
        try:
            sash_pos = None
            try:
                sash_pos = main_paned.sashpos(0)
            except Exception:
                pass
            save_geometry(root.geometry(), sash_position=sash_pos)
            # Сохраняем состояние чекбоксов и режим мотора при закрытии
            save_checkbox_state(var_show_rc_logs.get(), var_show_i2c_logs.get())
            save_motor_mode(var_mode.get())
        except Exception:
            pass
        monitor.close()
        remove_lock_file()  # Удаляем файл блокировки
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


def main() -> None:
    # Проверка на запуск второй копии программы
    if not check_single_instance():
        print("Ошибка: Программа уже запущена!")
        print("Закройте существующее окно или подождите несколько минут и попробуйте снова.")
        try:
            import tkinter as tk
            from tkinter import messagebox
            root = tk.Tk()
            root.withdraw()  # Скрываем главное окно
            messagebox.showerror(
                "Программа уже запущена",
                "EDFViewerMonitor уже работает!\n\n"
                "Закройте существующее окно или подождите несколько минут и попробуйте снова."
            )
            root.destroy()
        except Exception:
            pass
        sys.exit(1)
    
    ap = argparse.ArgumentParser(description="EDFViewerMonitor — оболочка вывода данных EDFViewer NEXT")
    ap.add_argument("--port", default="COM3", help="COM-порт (по умолчанию COM3)")
    ap.add_argument("--baud", type=int, default=2000000, help="Скорость UART")
    ap.add_argument("--no-gui", action="store_true", help="Только консоль + лог в файл")
    ap.add_argument("--log", type=str, default="", help="Путь к бинарному лог-файлу (без GUI)")
    ap.add_argument("--playback", type=str, default="", help="Путь к .bin логу для воспроизведения при запуске (GUI)")
    args = ap.parse_args()

    try:
        if args.no_gui:
            monitor = SensorMonitor(args.port, args.baud)
            if args.log:
                monitor.start_log(Path(args.log))
            monitor.open()
            try:
                while True:
                    p = monitor.read_packet()
                    if p:
                        monitor.log_packet(p)
                        print(p)
            except KeyboardInterrupt:
                pass
            finally:
                monitor.close()
        else:
            run_gui(args.port, args.baud, args.playback or None)
    finally:
        # Удаляем файл блокировки при завершении программы
        remove_lock_file()


if __name__ == "__main__":
    main()
