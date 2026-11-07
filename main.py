import argparse
import asyncio
import logging
import os
import sys
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path
from typing import Callable, Optional

import gpiod
import uvicorn
from gpiod.line import Bias, Direction, Edge, Value

from comms import CmdType, Packet, PartType, UARTComm
from config import EStop as EStopConfig
from config import Safety as SafetyConfig
from config import Server as ServerConfig
from config import UART as UARTConfig
from pose import PoseEstimator
from web_panel.server import app, set_pose_estimator, set_uart_comm

logger = logging.getLogger("TELBot.Main")


# ==============================================================================
# Physical E-Stop Hardware Interrupt Monitor (libgpiod v2)
# ==============================================================================
class EStopMonitor:
    """
    Monitors a physical E-Stop switch via Linux kernel GPIO edge interrupts.

    Default wiring (Normally Closed to GND with internal pull-up):
      - Normal / Closed to GND  -> Value.INACTIVE (LOW)
      - Pressed / Wire Open     -> Value.ACTIVE   (HIGH, RISING_EDGE) -> triggers E-Stop
      - Released back to Closed -> Value.INACTIVE (LOW, FALLING_EDGE) -> triggers restart
    """

    def __init__(
        self,
        chip_path: str,
        pin: int,
        normally_closed: bool,
        debounce_ms: int,
        on_press: Callable[[], None],
        on_release: Callable[[], None],
        dry_run: bool = False,
    ) -> None:
        self.chip_path = chip_path
        self.pin = pin
        self.normally_closed = normally_closed
        self.debounce_ms = debounce_ms
        self.on_press = on_press
        self.on_release = on_release
        self.dry_run = dry_run

        self._request: Optional[gpiod.LineRequest] = None
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._latched = False

    def _is_pressed_value(self, val: Value) -> bool:
        return (val == Value.ACTIVE) if self.normally_closed else (val == Value.INACTIVE)

    def _is_pressed_edge(self, event_type: gpiod.EdgeEvent.Type) -> bool:
        if self.normally_closed:
            return event_type == gpiod.EdgeEvent.Type.RISING_EDGE
        return event_type == gpiod.EdgeEvent.Type.FALLING_EDGE

    def start(self) -> None:
        """Requests the GPIO line with kernel edge detection and spawns the listener thread."""
        if self._running.is_set():
            return

        if self.dry_run:
            logger.info(
                f"E-Stop monitor in DRY-RUN mode (GPIO {self.pin} on {self.chip_path} bypassed)."
            )
            self._running.set()
            return

        mode_label = "Normally-Closed" if self.normally_closed else "Normally-Open"
        logger.info(
            f"Configuring physical E-Stop on {self.chip_path} BCM GPIO {self.pin} ({mode_label}, debounce={self.debounce_ms}ms)..."
        )

        self._request = gpiod.request_lines(
            self.chip_path,
            consumer="TELBot-EStop",
            config={
                self.pin: gpiod.LineSettings(
                    direction=Direction.INPUT,
                    bias=Bias.PULL_UP,
                    edge_detection=Edge.BOTH,
                    debounce_period=timedelta(milliseconds=self.debounce_ms),
                )
            },
        )

        # Boot-while-pressed safeguard: lock motion immediately without entering a restart loop
        initial_val = self._request.get_value(self.pin)
        if self._is_pressed_value(initial_val):
            self._latched = True
            logger.critical(
                f"Physical E-Stop is ACTIVE at startup on GPIO {self.pin} "
                "(button pressed or NC circuit open)! Motion commands are locked until released."
            )
            self.on_press()

        self._running.set()
        self._thread = threading.Thread(
            target=self._event_loop, name="EStop-GPIO-Worker", daemon=True
        )
        self._thread.start()

    def _event_loop(self) -> None:
        """Dedicated kernel edge-interrupt listener thread."""
        while self._running.is_set() and self._request is not None:
            try:
                if not self._request.wait_edge_events(timeout=timedelta(milliseconds=100)):
                    continue

                events = self._request.read_edge_events()
                for event in events:
                    if self._is_pressed_edge(event.event_type):
                        if not self._latched:
                            self._latched = True
                            logger.critical(
                                f"PHYSICAL E-STOP TRIGGERED on GPIO {self.pin}! Sending immediate STOP_ALL."
                            )
                        self.on_press()
                    else:
                        # Confirm settled line state is unpressed before triggering full restart
                        current_val = self._request.get_value(self.pin)
                        if self._latched and not self._is_pressed_value(current_val):
                            self._latched = False
                            logger.warning(
                                f"Physical E-Stop RELEASED on GPIO {self.pin}. Initiating host script restart..."
                            )
                            self.on_release()
                            return
            except Exception as e:
                if self._running.is_set():
                    logger.error(f"E-Stop GPIO monitor error: {e}")
                time.sleep(0.05)

    def stop(self) -> None:
        """Stops the interrupt listener thread and releases the GPIO line descriptor."""
        self._running.clear()
        if self._thread and self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)
        if self._request is not None:
            try:
                self._request.release()
            except Exception as e:
                logger.error(f"Failed to release GPIO line {self.pin}: {e}")
            finally:
                self._request = None


# ==============================================================================
# Unified Host Orchestrator
# ==============================================================================
class RobotHost:
    """
    Coordinates UART communication, physical E-Stop monitoring, Pico link
    heartbeat (200ms PING), drive timeout watchdog, telemetry processing,
    and the FastAPI/Uvicorn web server.
    """

    def __init__(
        self,
        uart_port: str = UARTConfig.PORT,
        baud_rate: int = UARTConfig.BAUD_RATE,
        host: str = ServerConfig.HOST,
        port: int = ServerConfig.PORT,
        estop_pin: int = EStopConfig.PIN,
        dry_run: bool = False,
        disable_estop: bool = False,
        enable_ssl: bool = True,
        ssl_certfile: str = ServerConfig.SSL_CERTFILE,
        ssl_keyfile: str = ServerConfig.SSL_KEYFILE,
        log_level: str = "INFO",
    ) -> None:
        self.host = host
        self.port = port
        self.dry_run = dry_run
        self.disable_estop = disable_estop
        self.enable_ssl = enable_ssl
        self.log_level = log_level.lower()

        root_dir = Path(__file__).resolve().parent
        cert_path = Path(ssl_certfile)
        key_path = Path(ssl_keyfile)
        self.ssl_certfile = str(cert_path if cert_path.is_absolute() else (root_dir / cert_path))
        self.ssl_keyfile = str(key_path if key_path.is_absolute() else (root_dir / key_path))

        self.ping_interval = SafetyConfig.PING_INTERVAL
        self.drive_timeout = SafetyConfig.DRIVE_TIMEOUT

        self.uart = UARTComm(
            port=uart_port,
            baudrate=baud_rate,
            timeout=UARTConfig.TIMEOUT,
            dry_run=dry_run,
        )
        self.uart.register_callback(self._on_telemetry, cmd_type=CmdType.TELEMETRY)
        self.uart.register_callback(self._on_ack, cmd_type=CmdType.ACK)

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._uvicorn_server: Optional[uvicorn.Server] = None
        self._restart_requested = False

        self.latest_sensor_state: dict = {}
        self.last_ack_time: float = 0.0
        self.pose_estimator = PoseEstimator()

        self.estop = EStopMonitor(
            chip_path=EStopConfig.CHIP,
            pin=estop_pin,
            normally_closed=EStopConfig.NORMALLY_CLOSED,
            debounce_ms=EStopConfig.DEBOUNCE_MS,
            on_press=self._handle_estop_press,
            on_release=self._handle_estop_release,
            dry_run=(dry_run or disable_estop),
        )

    @property
    def restart_requested(self) -> bool:
        return self._restart_requested

    # --------------------------------------------------------------------------
    # Hardware Callbacks (Thread-safe)
    # --------------------------------------------------------------------------
    def _handle_estop_press(self) -> None:
        """Called directly from the GPIO interrupt thread when E-Stop engages."""
        self.uart.trigger_estop()

    def _handle_estop_release(self) -> None:
        """Called from the GPIO interrupt thread when E-Stop is released."""
        if self._loop is not None and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(self.request_restart)
        else:
            self.request_restart()

    def request_restart(self) -> None:
        """Signals Uvicorn and background loops to tear down cleanly for os.execv."""
        if self._restart_requested:
            return
        self._restart_requested = True
        if self._uvicorn_server is not None:
            self._uvicorn_server.should_exit = True

    def _on_telemetry(self, packet: Packet) -> None:
        """Processes incoming TELEMETRY frames from the Pico slave."""
        try:
            data = packet.parse_telemetry()
            self.latest_sensor_state = data
            self.process_sensor_feedback(data)
        except Exception as e:
            logger.debug(f"Failed to parse telemetry packet: {e}")

    def _on_ack(self, packet: Packet) -> None:
        """Tracks heartbeat ACK responses from the Pico slave."""
        self.last_ack_time = time.monotonic()
        logger.debug(f"Received ACK from Pico (seq={packet.seq}).")

    def process_sensor_feedback(self, sensor_data: dict) -> None:
        """
        Updates field pose estimation when 3-directional LiDAR + heading readings
        are present, and provides a hook for closed-loop control.
        """
        if all(
            k in sensor_data
            for k in ("dist_front_cm", "dist_left_cm", "dist_right_cm", "heading_deg")
        ):
            self.pose_estimator.update(
                dist_front_cm=float(sensor_data["dist_front_cm"]),
                dist_left_cm=float(sensor_data["dist_left_cm"]),
                dist_right_cm=float(sensor_data["dist_right_cm"]),
                raw_heading_deg=float(sensor_data["heading_deg"]),
            )

    # --------------------------------------------------------------------------
    # Async Safety & Heartbeat Task
    # --------------------------------------------------------------------------
    async def _safety_and_heartbeat_loop(self) -> None:
        """
        1. Transmits a PING packet every 200 ms to satisfy the Pico's 400 ms watchdog.
        2. Auto-zeros chassis velocity if no drive update arrives within DRIVE_TIMEOUT.
        """
        last_ping = 0.0
        tick_interval = 0.05  # 50 ms resolution

        while not self._restart_requested:
            if self._uvicorn_server is not None and self._uvicorn_server.should_exit:
                break

            now = time.monotonic()

            # 1. Periodic 200ms PING heartbeat to Pico
            if (now - last_ping) >= self.ping_interval:
                self.uart.send_ping()
                last_ping = now

            # 2. Chassis operator timeout safeguard
            if (
                not self.uart.estop_active
                and self.uart.is_driving
                and (now - self.uart.last_drive_time) > self.drive_timeout
            ):
                logger.warning(
                    f"No chassis drive update for >{self.drive_timeout:.2f}s; zeroing chassis velocity."
                )
                self.uart.send_drive(0, 0, 0)

            await asyncio.sleep(tick_interval)

    # --------------------------------------------------------------------------
    # Main Runtime Lifecycle
    # --------------------------------------------------------------------------
    async def run(self) -> None:
        """Starts UART, E-Stop monitor, safety heartbeat loop, and FastAPI server."""
        self._loop = asyncio.get_running_loop()
        safety_task: Optional[asyncio.Task] = None

        # Configure Uvicorn server instance upfront so E-Stop callbacks can signal it safely
        use_ssl = (
            self.enable_ssl
            and os.path.isfile(self.ssl_certfile)
            and os.path.isfile(self.ssl_keyfile)
        )
        if self.enable_ssl and not use_ssl:
            logger.warning(
                f"SSL certificates not found at '{self.ssl_certfile}' / '{self.ssl_keyfile}'. "
                "Falling back to plain HTTP (run ./scripts/generate_certs.sh to enable HTTPS for WebHID)."
            )

        uv_config = uvicorn.Config(
            app=app,
            host=self.host,
            port=self.port,
            log_level=self.log_level,
            log_config=None,
            ssl_certfile=self.ssl_certfile if use_ssl else None,
            ssl_keyfile=self.ssl_keyfile if use_ssl else None,
        )
        self._uvicorn_server = uvicorn.Server(uv_config)

        try:
            # 1. Start UART workers and share with FastAPI
            self.uart.start()
            set_uart_comm(self.uart)
            set_pose_estimator(self.pose_estimator)

            # 2. Start physical E-Stop interrupt listener
            self.estop.start()

            # 3. Run safety/PING loop alongside Uvicorn
            safety_task = asyncio.create_task(
                self._safety_and_heartbeat_loop(), name="Host-Safety-Loop"
            )

            scheme = "https" if use_ssl else "http"
            logger.info(
                f"TELBot Host running on {scheme}://{self.host}:{self.port} "
                f"(UART={self.uart.port}, dry_run={self.dry_run})"
            )

            if not self._restart_requested:
                await self._uvicorn_server.serve()
        finally:
            logger.info("Initiating safe host shutdown sequence...")
            if safety_task is not None:
                safety_task.cancel()
                await asyncio.gather(safety_task, return_exceptions=True)

            # Release GPIO line before stopping UART or restarting process
            self.estop.stop()

            # Ensure final STOP_ALL is transmitted onto the wire before closing UART
            if self.uart.is_running:
                self.uart.send_stop(PartType.ALL, clear_pending_drive=True)
                self.uart.flush_tx(timeout=0.2)
                self.uart.stop()

            logger.info("TELBot Host shutdown complete.")


# ==============================================================================
# CLI Entrypoint
# ==============================================================================
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="TELBot Raspberry Pi 5 Host Controller")
    parser.add_argument(
        "--uart-port",
        default=UARTConfig.PORT,
        help=f"UART device path (default: {UARTConfig.PORT})",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=UARTConfig.BAUD_RATE,
        help=f"UART baud rate (default: {UARTConfig.BAUD_RATE})",
    )
    parser.add_argument(
        "--host",
        default=ServerConfig.HOST,
        help=f"API server bind host (default: {ServerConfig.HOST})",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=ServerConfig.PORT,
        help=f"API server bind port (default: {ServerConfig.PORT})",
    )
    parser.add_argument(
        "--estop-pin",
        type=int,
        default=EStopConfig.PIN,
        help=f"BCM GPIO pin for physical E-Stop button (default: {EStopConfig.PIN})",
    )
    parser.add_argument(
        "--no-estop",
        action="store_true",
        help="Disable physical GPIO E-Stop monitoring (useful when switch is not wired yet)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run server with simulated UART and GPIO for software/UI testing",
    )
    parser.add_argument(
        "--no-ssl",
        action="store_true",
        help="Disable HTTPS and serve over plain HTTP",
    )
    parser.add_argument(
        "--ssl-certfile",
        default=ServerConfig.SSL_CERTFILE,
        help=f"Path to TLS certificate file (default: {ServerConfig.SSL_CERTFILE})",
    )
    parser.add_argument(
        "--ssl-keyfile",
        default=ServerConfig.SSL_KEYFILE,
        help=f"Path to TLS private key file (default: {ServerConfig.SSL_KEYFILE})",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity (default: INFO)",
    )
    return parser.parse_args()


def setup_logging(log_level: str) -> Path:
    log_dir = Path(__file__).resolve().parent / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"

    logging.basicConfig(
        level=getattr(logging, log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(log_file, encoding="utf-8"),
        ],
    )
    return log_file


def main() -> None:
    args = parse_args()
    log_file = setup_logging(args.log_level)
    logger.info(f"Logging to file: {log_file}")

    host = RobotHost(
        uart_port=args.uart_port,
        baud_rate=args.baud,
        host=args.host,
        port=args.port,
        estop_pin=args.estop_pin,
        dry_run=args.dry_run,
        disable_estop=args.no_estop,
        enable_ssl=not args.no_ssl,
        ssl_certfile=args.ssl_certfile,
        ssl_keyfile=args.ssl_keyfile,
        log_level=args.log_level,
    )

    try:
        asyncio.run(host.run())
    except KeyboardInterrupt:
        pass

    if host.restart_requested:
        logger.warning("Restarting TELBot Host process via os.execv...")
        logging.shutdown()
        sys.stdout.flush()
        sys.stderr.flush()
        os.execv(sys.executable, [sys.executable, *sys.argv])


if __name__ == "__main__":
    main()
