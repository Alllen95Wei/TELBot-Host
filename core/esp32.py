"""
core/esp32.py - ESP32 UART Sensor Telemetry Receiver for TELBot Host

Receives 25-byte binary sensor telemetry frames over UART (115200 baud, 8N1)
from the ESP32 coprocessor:
  - Header (6B): 0xAA, 0x55, LENGTH=18, SEQ, CMD=4 (TELEMETRY), PART=4 (SENSORS)
  - Payload (18B, little-endian <hhhIII):
      * upper_rpm     (int16):  Upper flywheel speed in RPM
      * lower_rpm     (int16):  Lower flywheel speed in RPM
      * heading_deg   (int16):  Earth-relative magnetic heading in degrees
      * dist_front_mm (uint32): Front LiDAR measured distance in mm
      * dist_right_mm (uint32): Right LiDAR measured distance in mm
      * dist_left_mm  (uint32): Left LiDAR measured distance in mm
  - Footer (1B): CRC-8 (poly=0x07, init=0x00) over [LENGTH..PAYLOAD]
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import serial

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from config import ESP32 as ESP32Config
from core.comms import CmdType, Packet, PacketParser, PartType

logger = logging.getLogger("TELBot.ESP32")


@dataclass
class ESP32SensorSnapshot:
    """Thread-safe snapshot of the most recent sensor frame received from ESP32."""

    upper_rpm: int
    lower_rpm: int
    heading_deg: int
    dist_front_mm: int
    dist_right_mm: int
    dist_left_mm: int
    dist_front_cm: Optional[float]
    dist_right_cm: Optional[float]
    dist_left_cm: Optional[float]
    seq: int
    timestamp: float  # time.monotonic() when received
    wall_time: float  # time.time() when received

    def is_fresh(self, stale_timeout_s: float = ESP32Config.STALE_TIMEOUT_S) -> bool:
        return (time.monotonic() - self.timestamp) <= stale_timeout_s

    def to_dict(self, stale_timeout_s: float = ESP32Config.STALE_TIMEOUT_S) -> dict:
        age_s = max(0.0, time.monotonic() - self.timestamp)
        return {
            "source": "esp32",
            "upper_rpm": self.upper_rpm,
            "lower_rpm": self.lower_rpm,
            "heading_deg": self.heading_deg,
            "dist_front_mm": self.dist_front_mm,
            "dist_right_mm": self.dist_right_mm,
            "dist_left_mm": self.dist_left_mm,
            "dist_front_cm": self.dist_front_cm,
            "dist_right_cm": self.dist_right_cm,
            "dist_left_cm": self.dist_left_cm,
            "seq": self.seq,
            "age_ms": round(age_s * 1000.0, 1),
            "stale": age_s > stale_timeout_s,
            "timestamp": round(self.wall_time, 3),
        }


class ESP32SensorReceiver:
    """
    Dedicated UART receiver for streaming sensor telemetry from the ESP32 into RPi 5 Host.
    Automatically recovers from serial disconnects or missing device nodes without
    blocking the main Host process.
    """

    def __init__(
        self,
        port: str = ESP32Config.PORT,
        baudrate: int = ESP32Config.BAUD_RATE,
        timeout: float = ESP32Config.TIMEOUT,
        stale_timeout_s: float = ESP32Config.STALE_TIMEOUT_S,
        dry_run: bool = False,
    ) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.stale_timeout_s = float(stale_timeout_s)
        self.dry_run = bool(dry_run)

        self._serial: Optional[serial.Serial] = None
        self._parser = PacketParser()
        self._running = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None

        self._lock = threading.Lock()
        self._latest_snapshot: Optional[ESP32SensorSnapshot] = None
        self._callbacks: list[Callable[[ESP32SensorSnapshot], None]] = []

        self._rx_packets = 0
        self._rx_errors = 0
        self._last_rx_time = 0.0
        self._serial_connected = False

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    @property
    def latest_snapshot(self) -> Optional[ESP32SensorSnapshot]:
        with self._lock:
            return self._latest_snapshot

    def is_online(self) -> bool:
        """Returns True if a valid ESP32 sensor frame was received within stale_timeout_s."""
        snap = self.latest_snapshot
        if snap is None:
            return False
        return snap.is_fresh(self.stale_timeout_s)

    def register_callback(self, callback: Callable[[ESP32SensorSnapshot], None]) -> None:
        """Registers a callback invoked immediately on each decoded ESP32 sensor frame."""
        with self._lock:
            self._callbacks.append(callback)

    def start(self) -> None:
        """Starts the background ESP32 UART receiver thread."""
        if self._running.is_set():
            return

        self._running.set()
        if self.dry_run:
            logger.info(
                f"ESP32SensorReceiver started in DRY-RUN mode (port={self.port}, baud={self.baudrate})."
            )
            return

        self._rx_thread = threading.Thread(
            target=self._rx_worker,
            name="ESP32-UART-RX-Worker",
            daemon=True,
        )
        self._rx_thread.start()
        logger.info(
            f"ESP32SensorReceiver listening on {self.port} @ {self.baudrate} baud."
        )

    def stop(self) -> None:
        """Stops the receiver thread and closes the serial port."""
        self._running.clear()
        if (
            self._rx_thread is not None
            and self._rx_thread.is_alive()
            and threading.current_thread() is not self._rx_thread
        ):
            self._rx_thread.join(timeout=1.0)
        self._close_serial()

    def _close_serial(self) -> None:
        self._serial_connected = False
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass
            finally:
                self._serial = None

    def _rx_worker(self) -> None:
        warned_missing_port = False
        while self._running.is_set():
            if self._serial is None or not self._serial.is_open:
                try:
                    self._serial = serial.Serial(
                        port=self.port,
                        baudrate=self.baudrate,
                        timeout=self.timeout,
                    )
                    self._parser.clear()
                    self._serial_connected = True
                    warned_missing_port = False
                    logger.info(f"Connected to ESP32 UART on {self.port} ({self.baudrate} baud).")
                except Exception as e:
                    self._serial_connected = False
                    if not warned_missing_port:
                        logger.warning(
                            f"ESP32 UART port '{self.port}' unavailable ({e}); "
                            "will retry automatically in background."
                        )
                        warned_missing_port = True
                    time.sleep(1.0)
                    continue

            try:
                waiting = self._serial.in_waiting
                chunk = self._serial.read(waiting if waiting > 0 else 1)
                if chunk:
                    self.feed_bytes(chunk)
            except serial.SerialException as e:
                self._rx_errors += 1
                logger.warning(f"ESP32 serial read error on {self.port}: {e}. Reconnecting...")
                self._close_serial()
                time.sleep(0.5)
            except Exception as e:
                self._rx_errors += 1
                logger.debug(f"Unexpected error in ESP32 RX worker: {e}")
                time.sleep(0.02)

    def feed_bytes(self, chunk: bytes) -> list[ESP32SensorSnapshot]:
        """
        Feeds raw incoming serial bytes into PacketParser and processes any complete
        ESP32 sensor telemetry packets.
        """
        packets = self._parser.feed(chunk)
        snapshots: list[ESP32SensorSnapshot] = []
        for pkt in packets:
            if pkt.cmd == CmdType.TELEMETRY and pkt.part == PartType.SENSORS:
                try:
                    parsed = pkt.parse_telemetry()
                    if parsed.get("source") == "esp32":
                        snap = self._record_parsed_telemetry(parsed, seq=pkt.seq)
                        snapshots.append(snap)
                except Exception as e:
                    self._rx_errors += 1
                    logger.debug(f"Failed to parse ESP32 telemetry payload: {e}")
        return snapshots

    def inject_telemetry(
        self,
        upper_rpm: int,
        lower_rpm: int,
        heading_deg: int,
        dist_front_mm: int,
        dist_right_mm: int,
        dist_left_mm: int,
        seq: int = 0,
    ) -> ESP32SensorSnapshot:
        """
        Encodes and feeds a synthetic ESP32 telemetry frame through the full binary
        packet parser pipeline (used for testing and simulator injection).
        """
        pkt = Packet.create_esp32_telemetry(
            upper_rpm=upper_rpm,
            lower_rpm=lower_rpm,
            heading_deg=heading_deg,
            dist_front_mm=dist_front_mm,
            dist_right_mm=dist_right_mm,
            dist_left_mm=dist_left_mm,
            seq=seq,
        )
        snaps = self.feed_bytes(pkt.encode())
        if not snaps:
            raise RuntimeError("Failed to decode injected ESP32 telemetry packet")
        return snaps[-1]

    def _record_parsed_telemetry(self, parsed: dict, seq: int = 0) -> ESP32SensorSnapshot:
        now_mono = time.monotonic()
        now_wall = time.time()
        snapshot = ESP32SensorSnapshot(
            upper_rpm=int(parsed["upper_rpm"]),
            lower_rpm=int(parsed["lower_rpm"]),
            heading_deg=int(parsed["heading_deg"]),
            dist_front_mm=int(parsed["dist_front_mm"]),
            dist_right_mm=int(parsed["dist_right_mm"]),
            dist_left_mm=int(parsed["dist_left_mm"]),
            dist_front_cm=parsed.get("dist_front_cm"),
            dist_right_cm=parsed.get("dist_right_cm"),
            dist_left_cm=parsed.get("dist_left_cm"),
            seq=int(seq) & 0xFF,
            timestamp=now_mono,
            wall_time=now_wall,
        )

        with self._lock:
            self._latest_snapshot = snapshot
            self._rx_packets += 1
            self._last_rx_time = now_mono
            callbacks = list(self._callbacks)

        for cb in callbacks:
            try:
                cb(snapshot)
            except Exception as e:
                logger.error(f"Error in ESP32 sensor callback: {e}")

        return snapshot

    def get_stats(self) -> dict:
        with self._lock:
            snap = self._latest_snapshot
            rx_packets = self._rx_packets
            rx_errors = self._rx_errors
            last_rx_time = self._last_rx_time
            connected = self._serial_connected

        online = snap is not None and snap.is_fresh(self.stale_timeout_s)
        return {
            "port": self.port,
            "baudrate": self.baudrate,
            "is_running": self.is_running,
            "dry_run": self.dry_run,
            "serial_connected": connected,
            "online": online,
            "rx_packets": rx_packets,
            "rx_errors": rx_errors,
            "last_rx_time": last_rx_time,
        }
