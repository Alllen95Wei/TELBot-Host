import itertools
import logging
import queue
import struct
import threading
import time
from enum import IntEnum
from typing import Callable, Optional, Tuple

import serial

logger = logging.getLogger("TELBot.comms")

# ==============================================================================
# Protocol Constants
# ==============================================================================
MAGIC_HEADER = b"\xAA\x55"
HEADER_SIZE = 6  # MAGIC (2B) + LEN (1B) + SEQ (1B) + CMD (1B) + PART (1B)
MIN_PACKET_SIZE = HEADER_SIZE + 1  # Header + CRC8 (1B)
MAX_PAYLOAD_SIZE = 240  # Well within 1-byte length limit (0-255)


# ==============================================================================
# Enums
# ==============================================================================
class CmdType(IntEnum):
    STOP = 0
    PING = 1
    RUN = 2
    ACK = 3
    TELEMETRY = 4
    CONFIG = 5


class PartType(IntEnum):
    ALL = 0
    CHASSIS = 1
    CONVEYOR = 2
    SHOOTER = 3
    SENSORS = 4


# ==============================================================================
# CRC-8 (Polynomial 0x07: x^8 + x^2 + x + 1, init 0x00)
# Precomputed lookup table for high performance real-time operations
# ==============================================================================
def _generate_crc8_table(poly: int = 0x07) -> list[int]:
    table = []
    for byte in range(256):
        curr = byte
        for _ in range(8):
            if curr & 0x80:
                curr = ((curr << 1) ^ poly) & 0xFF
            else:
                curr = (curr << 1) & 0xFF
        table.append(curr)
    return table


_CRC8_TABLE = _generate_crc8_table(0x07)


def calculate_crc8(data: bytes | bytearray) -> int:
    """Calculate CRC-8 checksum over the given bytes."""
    crc = 0x00
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


# ==============================================================================
# Packet Model
# ==============================================================================
class Packet:
    """
    Standard binary packet format for Host (RPi 5) <-> Slave (RPi Pico W):

    Offset  Field       Size    Description
    ---------------------------------------------------------------
    0..1    MAGIC       2B      0xAA, 0x55 frame sync header
    2       LENGTH      1B      Payload length (N bytes, 0..240)
    3       SEQ         1B      Sequence counter (0..255)
    4       CMD         1B      CmdType enum
    5       PART        1B      PartType enum
    6..N+5  PAYLOAD     NB      Binary payload (little-endian)
    N+6     CRC8        1B      CRC-8 over [LENGTH, SEQ, CMD, PART, PAYLOAD]
    """

    def __init__(
        self,
        cmd: CmdType | int,
        part: PartType | int,
        payload: bytes | str | int = b"",
        seq: int = 0,
    ) -> None:
        self.cmd = CmdType(cmd)
        self.part = PartType(part)
        self.seq = seq & 0xFF

        if isinstance(payload, bytes):
            self.payload = payload
        elif isinstance(payload, str):
            self.payload = payload.encode("utf-8")
        elif isinstance(payload, int):
            self.payload = struct.pack("<i", payload)
        else:
            raise TypeError(f"Unsupported payload type: {type(payload)}")

        if len(self.payload) > MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload exceeds max size of {MAX_PAYLOAD_SIZE} bytes")

    def encode(self) -> bytes:
        """Serializes the packet into framed binary wire format."""
        payload_len = len(self.payload)
        # Header without magic bytes: [LENGTH (1B), SEQ (1B), CMD (1B), PART (1B)]
        body_header = struct.pack(
            "<BBBB",
            payload_len,
            self.seq,
            int(self.cmd),
            int(self.part),
        )
        body = body_header + self.payload
        crc = calculate_crc8(body)
        return MAGIC_HEADER + body + bytes([crc])

    @classmethod
    def decode(cls, raw: bytes | bytearray) -> Optional["Packet"]:
        """
        Attempts to decode a single complete packet from bytes.
        Returns None if packet is malformed or CRC check fails.
        """
        if len(raw) < MIN_PACKET_SIZE:
            return None

        # Verify magic header
        if raw[:2] != MAGIC_HEADER:
            return None

        payload_len = raw[2]
        expected_total_len = HEADER_SIZE + payload_len + 1
        if len(raw) != expected_total_len:
            return None

        # Verify CRC8 over [LENGTH, SEQ, CMD, PART, PAYLOAD]
        body = raw[2:-1]
        received_crc = raw[-1]
        if calculate_crc8(body) != received_crc:
            return None

        _, seq, cmd_val, part_val = struct.unpack("<BBBB", raw[2:6])
        payload = bytes(raw[6:-1])

        try:
            return cls(
                cmd=CmdType(cmd_val),
                part=PartType(part_val),
                payload=payload,
                seq=seq,
            )
        except ValueError:
            # Unknown CmdType or PartType
            return None

    def __repr__(self) -> str:
        return (
            f"<Packet seq={self.seq} cmd={self.cmd.name} "
            f"part={self.part.name} payload_len={len(self.payload)} "
            f"payload={self.payload.hex()}>"
        )

    # --------------------------------------------------------------------------
    # Specialized helper factories for common commands
    # --------------------------------------------------------------------------
    @classmethod
    def create_stop(cls, part: PartType = PartType.ALL, seq: int = 0) -> "Packet":
        """Emergency or normal stop packet."""
        return cls(cmd=CmdType.STOP, part=part, payload=b"", seq=seq)

    @classmethod
    def create_ping(cls, seq: int = 0) -> "Packet":
        """Heartbeat ping packet."""
        return cls(cmd=CmdType.PING, part=PartType.ALL, payload=b"", seq=seq)

    # --- Chassis Subsystem (PartType.CHASSIS) ---
    @classmethod
    def create_chassis_drive(
        cls, vx: int, vy: int, omega: int, seq: int = 0
    ) -> "Packet":
        """
        Drive chassis motors with 3 signed 16-bit integers (-1000 to +1000 scaled):
        vx: forward/backward velocity (corresponds to y_speed on Pico)
        vy: strafe left/right velocity (corresponds to x_speed on Pico)
        omega: rotational angular velocity (corresponds to z_rotation on Pico)
        """
        payload = struct.pack("<hhh", vx, vy, omega)
        return cls(cmd=CmdType.RUN, part=PartType.CHASSIS, payload=payload, seq=seq)

    def parse_chassis_drive(self) -> Tuple[int, int, int]:
        """Unpacks chassis drive payload into (vx, vy, omega)."""
        if len(self.payload) != 6:
            raise ValueError(f"Chassis payload expected 6 bytes, got {len(self.payload)}")
        return struct.unpack("<hhh", self.payload)

    # --- Shooter Subsystem (PartType.SHOOTER) ---
    @classmethod
    def create_shooter_command(
        cls, flywheel: int, angle: int = 0, spinner: int = 0, seq: int = 0
    ) -> "Packet":
        """
        Full control of Shooter subsystem:
        flywheel: flywheel velocity (-1000 to +1000)
        angle: motorized pitch angle speed (-1000 to +1000)
        spinner: feeder spinner speed (-1000 to +1000)
        """
        payload = struct.pack("<hhh", flywheel, angle, spinner)
        return cls(cmd=CmdType.RUN, part=PartType.SHOOTER, payload=payload, seq=seq)

    @classmethod
    def create_shooter_flywheel(cls, speed: int, seq: int = 0) -> "Packet":
        """Sets shooter flywheel speed only (-1000 to +1000)."""
        payload = struct.pack("<h", speed)
        return cls(cmd=CmdType.RUN, part=PartType.SHOOTER, payload=payload, seq=seq)

    def parse_shooter_command(self) -> Tuple[int, int, int]:
        """
        Unpacks shooter command.
        If payload has 6 bytes: returns (flywheel, angle, spinner).
        If payload has 2 bytes: returns (flywheel, 0, 0).
        """
        if len(self.payload) == 6:
            return struct.unpack("<hhh", self.payload)
        elif len(self.payload) == 2:
            (flywheel,) = struct.unpack("<h", self.payload)
            return (flywheel, 0, 0)
        raise ValueError(f"Shooter payload expected 2 or 6 bytes, got {len(self.payload)}")

    # --- Conveyor Subsystem (PartType.CONVEYOR) ---
    @classmethod
    def create_conveyor_drive(cls, speed: int, seq: int = 0) -> "Packet":
        """Sets conveyor/feeder motor speed (-1000 to +1000)."""
        payload = struct.pack("<h", speed)
        return cls(cmd=CmdType.RUN, part=PartType.CONVEYOR, payload=payload, seq=seq)

    def parse_conveyor_drive(self) -> int:
        """Unpacks conveyor speed (-1000 to +1000)."""
        if len(self.payload) != 2:
            raise ValueError(f"Conveyor payload expected 2 bytes, got {len(self.payload)}")
        return struct.unpack("<h", self.payload)[0]

    # --- Sensor Telemetry (PartType.SENSORS) ---
    @classmethod
    def create_telemetry(
        cls, battery_mv: int, enc_lf: int = 0, enc_rf: int = 0, status_flags: int = 0, seq: int = 0
    ) -> "Packet":
        """
        Packs sensor telemetry from Pico to Host:
        battery_mv: uint16 in millivolts (e.g. 12400 for 12.4V)
        enc_lf: int16 left front/side encoder
        enc_rf: int16 right front/side encoder
        status_flags: uint8 bitmask (e.g. bit 0 = chassis enabled, bit 1 = shooter running)
        """
        payload = struct.pack("<HhhB", battery_mv, enc_lf, enc_rf, status_flags)
        return cls(cmd=CmdType.TELEMETRY, part=PartType.SENSORS, payload=payload, seq=seq)

    def parse_telemetry(self) -> dict:
        """Unpacks telemetry payload into structured dictionary."""
        if len(self.payload) != 7:
            raise ValueError(f"Telemetry payload expected 7 bytes, got {len(self.payload)}")
        battery_mv, enc_lf, enc_rf, status = struct.unpack("<HhhB", self.payload)
        return {
            "battery_mv": battery_mv,
            "battery_v": round(battery_mv / 1000.0, 2),
            "enc_lf": enc_lf,
            "enc_rf": enc_rf,
            "status_flags": status,
        }


# ==============================================================================
# Streaming Packet Parser (Handles UART fragmentation & sliding-window resync)
# ==============================================================================
class PacketParser:
    """
    Buffers incoming bytes from a serial stream, recovers frame synchronization,
    and yields complete, CRC-validated Packet instances.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> list[Packet]:
        """
        Feed an incoming byte chunk from serial into the parser.
        Returns a list of all fully-received valid packets.
        """
        self._buffer.extend(chunk)
        packets: list[Packet] = []

        while len(self._buffer) >= MIN_PACKET_SIZE:
            # 1. Slide window until magic header matches
            if self._buffer[0] != MAGIC_HEADER[0] or self._buffer[1] != MAGIC_HEADER[1]:
                # Advance 1 byte to find next potential header
                del self._buffer[0]
                continue

            # 2. Check if we have enough bytes for the reported payload length
            payload_len = self._buffer[2]
            total_len = HEADER_SIZE + payload_len + 1  # +1 for CRC8

            # If reported payload length is suspiciously large, slide past false header
            if payload_len > MAX_PAYLOAD_SIZE:
                del self._buffer[0]
                continue

            # Wait for remainder of packet to arrive
            if len(self._buffer) < total_len:
                break

            # 3. Extract candidate frame and verify CRC
            candidate_raw = bytes(self._buffer[:total_len])
            packet = Packet.decode(candidate_raw)

            if packet is not None:
                # Valid frame: consume the frame from buffer
                del self._buffer[:total_len]
                packets.append(packet)
            else:
                # Corrupted frame or false positive header:
                # Discard the first header byte and resume search
                del self._buffer[0]

        return packets

    def clear(self) -> None:
        """Clears the parser buffer."""
        self._buffer.clear()


# ==============================================================================
# Dual-Threaded UART Communicator
# ==============================================================================
class UARTComm:
    """
    Dual-threaded, non-blocking serial communication manager.

    - Dedicated TX Worker Thread: pulls outgoing packets from a PriorityQueue and writes to UART.
    - Dedicated RX Worker Thread: continuously reads incoming bytes, recovers framed packets,
      and dispatches them to registered callbacks and telemetry storage.
    - Fully thread-safe: API servers (FastAPI/WebSockets/REST) can enqueue commands at any time.
    """

    def __init__(
        self,
        port: str = "/dev/ttyAMA0",
        baudrate: int = 115200,
        timeout: float = 0.1,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self._ser: Optional[serial.Serial] = None
        self._running = threading.Event()

        # Threads
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None

        # Thread-safe PriorityQueue: items are (priority: int, seq_tiebreaker: int, packet: Optional[Packet])
        # Priority 0 = Emergency STOP / Urgent
        # Priority 1 = High (CONFIG, ACK, PING)
        # Priority 2 = Normal (RUN, Teleoperation drive commands)
        self._tx_queue: queue.PriorityQueue[Tuple[int, int, Optional[Packet]]] = queue.PriorityQueue()
        self._queue_counter = itertools.count()
        self._seq_counter = 0

        # Parser & incoming data handling
        self._parser = PacketParser()
        self._rx_callbacks: dict[CmdType, list[Callable[[Packet], None]]] = {}
        self._global_callbacks: list[Callable[[Packet], None]] = []

        # Latest telemetry snapshot & statistics
        self._latest_telemetry: Optional[Packet] = None
        self._stats_lock = threading.Lock()
        self._stats = {
            "tx_packets": 0,
            "rx_packets": 0,
            "tx_errors": 0,
            "rx_errors": 0,
            "last_tx_time": 0.0,
            "last_rx_time": 0.0,
        }

    # --------------------------------------------------------------------------
    # Lifecycle Management
    # --------------------------------------------------------------------------
    def start(self) -> None:
        """Starts serial port connection and spawns TX and RX worker threads."""
        if self._running.is_set():
            logger.warning("UARTComm is already running.")
            return

        logger.info(f"Connecting to UART on {self.port} at {self.baudrate} baud...")
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=0.2,
        )
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        self._running.set()

        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="UART-RX-Worker", daemon=True
        )
        self._tx_thread = threading.Thread(
            target=self._tx_loop, name="UART-TX-Worker", daemon=True
        )

        self._rx_thread.start()
        self._tx_thread.start()
        logger.info("UART dual-threaded workers successfully started.")

    def stop(self) -> None:
        """Stops worker threads and closes the serial port cleanly."""
        if not self._running.is_set():
            return

        logger.info("Stopping UARTComm workers...")
        self._running.clear()

        # Send unblocking sentinel item to TX queue
        self._tx_queue.put_nowait((0, next(self._queue_counter), None))

        if self._tx_thread and self._tx_thread.is_alive():
            self._tx_thread.join(timeout=1.0)
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)

        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception as e:
                logger.error(f"Error closing serial port: {e}")

        self._parser.clear()
        logger.info("UARTComm stopped.")

    def __enter__(self) -> "UARTComm":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    # --------------------------------------------------------------------------
    # Queue Interface for API Server & External Callers
    # --------------------------------------------------------------------------
    def send_packet(self, packet: Packet, priority: int = 2) -> None:
        """
        Thread-safe method to append a packet into the TX queue.
        Can be called directly from FastAPI endpoints, WebSockets, or background loops.
        """
        # Assign auto-incrementing sequence number if not set
        if packet.seq == 0:
            self._seq_counter = (self._seq_counter + 1) & 0xFF
            packet.seq = self._seq_counter

        self._tx_queue.put_nowait((priority, next(self._queue_counter), packet))

    def send_stop(self, part: PartType = PartType.ALL, clear_pending_drive: bool = True) -> None:
        """
        Sends an emergency or normal STOP command with top priority (0).
        Optionally clears any pending drive commands to prevent buffered movement.
        """
        if clear_pending_drive:
            self.clear_drive_queue()

        pkt = Packet.create_stop(part=part)
        self.send_packet(pkt, priority=0)

    def send_drive(self, vx: int, vy: int, omega: int, priority: int = 2) -> None:
        """Enqueues a chassis drive command."""
        pkt = Packet.create_chassis_drive(vx=vx, vy=vy, omega=omega)
        self.send_packet(pkt, priority=priority)

    def send_shooter_command(
        self, flywheel: int, angle: int = 0, spinner: int = 0, priority: int = 2
    ) -> None:
        """Enqueues a full shooter command (flywheel, angle, spinner)."""
        pkt = Packet.create_shooter_command(flywheel=flywheel, angle=angle, spinner=spinner)
        self.send_packet(pkt, priority=priority)

    def send_shooter_flywheel(self, speed: int, priority: int = 2) -> None:
        """Enqueues a shooter flywheel command."""
        pkt = Packet.create_shooter_flywheel(speed=speed)
        self.send_packet(pkt, priority=priority)

    def send_conveyor(self, speed: int, priority: int = 2) -> None:
        """Enqueues a conveyor/feeder drive command."""
        pkt = Packet.create_conveyor_drive(speed=speed)
        self.send_packet(pkt, priority=priority)

    def send_ping(self, priority: int = 1) -> None:
        """Enqueues a keep-alive ping."""
        pkt = Packet.create_ping()
        self.send_packet(pkt, priority=priority)

    def clear_drive_queue(self) -> int:
        """
        Drains all pending drive (RUN) commands from TX queue to prevent stale actuation.
        Returns the number of discarded drive commands.
        """
        removed = 0
        preserved: list[Tuple[int, int, Optional[Packet]]] = []
        try:
            while not self._tx_queue.empty():
                item = self._tx_queue.get_nowait()
                _, _, pkt = item
                if pkt is not None and pkt.cmd == CmdType.RUN and pkt.part == PartType.CHASSIS:
                    removed += 1
                    self._tx_queue.task_done()
                else:
                    preserved.append(item)
        except queue.Empty:
            pass

        for item in preserved:
            self._tx_queue.put_nowait(item)

        return removed

    # --------------------------------------------------------------------------
    # Callbacks & Telemetry Subscription
    # --------------------------------------------------------------------------
    def register_callback(
        self,
        callback: Callable[[Packet], None],
        cmd_type: Optional[CmdType] = None,
    ) -> None:
        """
        Registers a callback invoked whenever a valid packet arrives.
        If cmd_type is specified, only packets matching that command trigger the callback.
        """
        if cmd_type is None:
            self._global_callbacks.append(callback)
        else:
            self._rx_callbacks.setdefault(cmd_type, []).append(callback)

    @property
    def latest_telemetry(self) -> Optional[Packet]:
        """Returns the most recently received telemetry packet."""
        return self._latest_telemetry

    def get_stats(self) -> dict:
        """Returns a snapshot of communication statistics."""
        with self._stats_lock:
            stats = dict(self._stats)
            stats["queue_depth"] = self._tx_queue.qsize()
            stats["is_running"] = self._running.is_set()
            return stats

    # --------------------------------------------------------------------------
    # Worker Threads (TX & RX)
    # --------------------------------------------------------------------------
    def _tx_loop(self) -> None:
        """Dedicated TX worker thread."""
        logger.debug("TX worker loop started.")
        while self._running.is_set():
            try:
                # Wait for outgoing packet from queue (timeout allows checking _running)
                item = self._tx_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            _, _, packet = item
            if packet is None:
                # Sentinel wake-up on shutdown
                self._tx_queue.task_done()
                break

            if self._ser and self._ser.is_open:
                try:
                    wire_data = packet.encode()
                    self._ser.write(wire_data)
                    self._ser.flush()

                    with self._stats_lock:
                        self._stats["tx_packets"] += 1
                        self._stats["last_tx_time"] = time.time()
                except serial.SerialException as e:
                    logger.error(f"UART TX write failed: {e}")
                    with self._stats_lock:
                        self._stats["tx_errors"] += 1
                except Exception as e:
                    logger.error(f"Unexpected TX error: {e}")
                    with self._stats_lock:
                        self._stats["tx_errors"] += 1

            self._tx_queue.task_done()

        logger.debug("TX worker loop terminated.")

    def _rx_loop(self) -> None:
        """Dedicated RX worker thread."""
        logger.debug("RX worker loop started.")
        while self._running.is_set():
            if not self._ser or not self._ser.is_open:
                time.sleep(0.05)
                continue

            try:
                # Read available bytes or block up to self.timeout
                waiting = self._ser.in_waiting
                chunk = self._ser.read(waiting if waiting > 0 else 64)
                if not chunk:
                    continue

                packets = self._parser.feed(chunk)
                now = time.time()

                for packet in packets:
                    with self._stats_lock:
                        self._stats["rx_packets"] += 1
                        self._stats["last_rx_time"] = now

                    if packet.cmd == CmdType.TELEMETRY:
                        self._latest_telemetry = packet

                    # Dispatch to registered callbacks
                    for cb in self._global_callbacks:
                        try:
                            cb(packet)
                        except Exception as e:
                            logger.error(f"Error in global RX callback: {e}")

                    for cb in self._rx_callbacks.get(packet.cmd, []):
                        try:
                            cb(packet)
                        except Exception as e:
                            logger.error(f"Error in {packet.cmd.name} callback: {e}")

            except serial.SerialException as e:
                logger.error(f"UART RX read failed: {e}")
                with self._stats_lock:
                    self._stats["rx_errors"] += 1
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Unexpected RX error: {e}")
                with self._stats_lock:
                    self._stats["rx_errors"] += 1
                time.sleep(0.05)

        logger.debug("RX worker loop terminated.")


# ==============================================================================
# Unit Verification / Demonstration
# ==============================================================================
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    print("Testing Subsystem Packet Formats & UARTComm...")

    # 1. Test Chassis Drive Packet
    pkt_chassis = Packet.create_chassis_drive(vx=500, vy=-200, omega=100, seq=10)
    raw_chassis = pkt_chassis.encode()
    dec_chassis = Packet.decode(raw_chassis)
    assert dec_chassis is not None
    assert dec_chassis.parse_chassis_drive() == (500, -200, 100)
    print(f"Chassis Packet OK: {dec_chassis}")

    # 2. Test Shooter Full Command Packet
    pkt_shooter = Packet.create_shooter_command(flywheel=850, angle=150, spinner=1000, seq=11)
    raw_shooter = pkt_shooter.encode()
    dec_shooter = Packet.decode(raw_shooter)
    assert dec_shooter is not None
    assert dec_shooter.parse_shooter_command() == (850, 150, 1000)
    print(f"Shooter Packet OK: {dec_shooter}")

    # 3. Test Conveyor Command Packet
    pkt_conveyor = Packet.create_conveyor_drive(speed=750, seq=12)
    raw_conveyor = pkt_conveyor.encode()
    dec_conveyor = Packet.decode(raw_conveyor)
    assert dec_conveyor is not None
    assert dec_conveyor.parse_conveyor_drive() == 750
    print(f"Conveyor Packet OK: {dec_conveyor}")

    # 4. Test Telemetry Packet
    pkt_telemetry = Packet.create_telemetry(battery_mv=12450, enc_lf=1024, enc_rf=-1020, status_flags=0b0101, seq=13)
    raw_telemetry = pkt_telemetry.encode()
    dec_telemetry = Packet.decode(raw_telemetry)
    assert dec_telemetry is not None
    telem_data = dec_telemetry.parse_telemetry()
    assert telem_data["battery_mv"] == 12450
    assert telem_data["battery_v"] == 12.45
    assert telem_data["enc_lf"] == 1024
    assert telem_data["enc_rf"] == -1020
    assert telem_data["status_flags"] == 5
    print(f"Telemetry Packet OK: {telem_data}")

    # 5. Test Priority Queue & Purging
    comm = UARTComm(port="/dev/ttyAMA0")
    comm.send_drive(vx=100, vy=0, omega=0)
    comm.send_shooter_command(flywheel=500)
    comm.send_stop(clear_pending_drive=True)

    items = []
    while not comm._tx_queue.empty():
        items.append(comm._tx_queue.get_nowait())
    priorities = [it[0] for it in items]
    cmds = [it[2].cmd for it in items if it[2]]
    assert priorities[0] == 0  # STOP is priority 0
    assert CmdType.STOP in cmds
    assert CmdType.RUN in cmds  # Shooter is RUN but not CHASSIS, so preserved
    print("Priority and queue-clearing assertions verified successfully!")
    print("All subsystem packet verification checks passed!")
