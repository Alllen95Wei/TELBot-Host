import asyncio
import json
import logging
import os
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

# Add root directory to path so comms and config can be imported
PANEL_DIR = Path(__file__).resolve().parent
STATIC_DIR = PANEL_DIR / "static"
ROOT_DIR = PANEL_DIR.parent
sys.path.append(str(ROOT_DIR))

from config import Camera as CameraConfig
from config import ESP32 as ESP32Config
from config import Field as FieldConfig
from config import Server as ServerConfig
from config import SpecialGrid as SpecialGridConfig
from config import UART as UARTConfig
from core import (
    ESP32SensorReceiver,
    ESP32SensorSnapshot,
    MatchManager,
    MatchMode,
    PartType,
    PoseEstimator,
    UARTComm,
    VisionManager,
    VisionTarget,
)

logger = logging.getLogger("TELBot.Server")

# ==============================================================================
# Shared Hardware Communicator & Pose Estimator (Injectable from main.py)
# ==============================================================================
uart = UARTComm(
    port=UARTConfig.PORT,
    baudrate=UARTConfig.BAUD_RATE,
    timeout=UARTConfig.TIMEOUT,
)
pose_estimator = PoseEstimator()
match_manager = MatchManager(on_stop_required=lambda: uart.send_stop())
vision_manager = VisionManager()
esp32_receiver = ESP32SensorReceiver(
    port=ESP32Config.PORT,
    baudrate=ESP32Config.BAUD_RATE,
    timeout=ESP32Config.TIMEOUT,
    stale_timeout_s=ESP32Config.STALE_TIMEOUT_S,
)


def _on_standalone_esp32_frame(snap: ESP32SensorSnapshot) -> None:
    """Updates PoseEstimator when ESP32SensorReceiver runs inside server.py."""
    get_pose_estimator().update_from_mm(
        dist_front_mm=snap.dist_front_mm,
        dist_right_mm=snap.dist_right_mm,
        dist_left_mm=snap.dist_left_mm,
        raw_heading_deg=float(snap.heading_deg),
    )


esp32_receiver.register_callback(_on_standalone_esp32_frame)


def set_uart_comm(comm: UARTComm) -> None:
    """Injects an externally managed UARTComm instance (used by main.py)."""
    global uart
    uart = comm
    app.state.uart = comm
    match_manager.on_stop_required = lambda: comm.send_stop()


def get_uart_comm() -> UARTComm:
    """Returns the active UARTComm instance."""
    return getattr(app.state, "uart", uart)


def set_pose_estimator(estimator: PoseEstimator) -> None:
    """Injects an externally managed PoseEstimator instance (used by main.py)."""
    global pose_estimator
    pose_estimator = estimator
    app.state.pose_estimator = estimator


def get_pose_estimator() -> PoseEstimator:
    """Returns the active PoseEstimator instance."""
    return getattr(app.state, "pose_estimator", pose_estimator)


def set_match_manager(manager: MatchManager) -> None:
    """Injects an externally managed MatchManager instance (used by main.py)."""
    global match_manager
    match_manager = manager
    app.state.match_manager = manager


def get_match_manager() -> MatchManager:
    """Returns the active MatchManager instance."""
    return getattr(app.state, "match_manager", match_manager)


def set_vision_manager(manager: VisionManager) -> None:
    """Injects an externally managed VisionManager instance (used by main.py)."""
    global vision_manager
    vision_manager = manager
    app.state.vision_manager = manager


def get_vision_manager() -> VisionManager:
    """Returns the active VisionManager instance."""
    return getattr(app.state, "vision_manager", vision_manager)


def set_esp32_receiver(receiver: ESP32SensorReceiver) -> None:
    """Injects an externally managed ESP32SensorReceiver instance (used by main.py)."""
    global esp32_receiver
    esp32_receiver = receiver
    app.state.esp32_receiver = receiver


def get_esp32_receiver() -> ESP32SensorReceiver:
    """Returns the active ESP32SensorReceiver instance."""
    return getattr(app.state, "esp32_receiver", esp32_receiver)


def _build_combined_telemetry() -> Optional[dict]:
    """
    Merges latest telemetry from Pico UART and ESP32 Sensor UART into a single
    dictionary for the Web UI and REST clients.
    """
    comm = get_uart_comm()
    esp32 = get_esp32_receiver()
    combined: dict = {}

    telem_pkt = comm.latest_telemetry
    if telem_pkt is not None:
        try:
            combined.update(telem_pkt.parse_telemetry())
        except Exception:
            combined["raw_hex"] = telem_pkt.payload.hex()

    esp32_snap = esp32.latest_snapshot
    if esp32_snap is not None:
        snap_dict = esp32_snap.to_dict(stale_timeout_s=esp32.stale_timeout_s)
        combined["esp32"] = snap_dict
        if not snap_dict["stale"]:
            combined["upper_rpm"] = snap_dict["upper_rpm"]
            combined["lower_rpm"] = snap_dict["lower_rpm"]
            combined["heading_deg"] = snap_dict["heading_deg"]
            combined["dist_front_mm"] = snap_dict["dist_front_mm"]
            combined["dist_right_mm"] = snap_dict["dist_right_mm"]
            combined["dist_left_mm"] = snap_dict["dist_left_mm"]
            combined["dist_front_cm"] = snap_dict["dist_front_cm"]
            combined["dist_right_cm"] = snap_dict["dist_right_cm"]
            combined["dist_left_cm"] = snap_dict["dist_left_cm"]

    return combined if combined else None


# Active video streaming proxy writers to cleanly tear down on shutdown
_active_stream_writers: set[asyncio.StreamWriter] = set()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Manages the startup and shutdown lifecycle of the UART worker threads
    when running standalone (skips if already started by main.py).
    """
    comm = get_uart_comm()
    esp32 = get_esp32_receiver()
    app.state.uart = comm
    app.state.esp32_receiver = esp32
    managed_internally = False
    esp32_managed_internally = False

    if not comm.is_running:
        logger.info("Initializing UART dual-threaded communication workers (standalone mode)...")
        try:
            comm.start()
            managed_internally = True
        except Exception as e:
            logger.error(f"Failed to start UART worker threads: {e}")

    if not esp32.is_running:
        try:
            esp32.start()
            esp32_managed_internally = True
        except Exception as e:
            logger.error(f"Failed to start ESP32 UART receiver: {e}")

    yield

    # Terminate any active uStreamer proxy connections on server shutdown
    for w in list(_active_stream_writers):
        try:
            w.close()
        except Exception:
            pass
    _active_stream_writers.clear()

    if esp32_managed_internally:
        esp32.stop()

    if managed_internally:
        logger.info("Shutting down UART communication workers (standalone mode)...")
        comm.stop()


# ==============================================================================
# FastAPI Application
# ==============================================================================
app = FastAPI(
    title="TELBot Host API",
    description="Real-time control and telemetry server for TELBot robot",
    lifespan=lifespan,
)
app.state.uart = uart

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

if STATIC_DIR.is_dir():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


@app.get("/", include_in_schema=False)
def serve_web_panel():
    """Serves the offline Material You (BeerCSS) operator panel."""
    index_file = PANEL_DIR / "index.html"
    if not index_file.is_file():
        raise HTTPException(status_code=404, detail="web_panel/index.html not found")
    return FileResponse(path=str(index_file), media_type="text/html")


def _require_ready_uart() -> UARTComm:
    """Validates that UARTComm is running and E-Stop is not active."""
    comm = get_uart_comm()
    if not comm.is_running:
        raise HTTPException(status_code=503, detail="UART communicator is not running")
    if comm.estop_active:
        raise HTTPException(
            status_code=423,
            detail="Emergency Stop (E-Stop) is active; motion commands are locked",
        )
    return comm


def _require_teleop_allowed() -> UARTComm:
    """
    Validates that UART is ready, E-Stop is not active, and current match mode
    permits human driver / tele-op commands.
    """
    comm = _require_ready_uart()
    allowed, reason = get_match_manager().check_teleop_allowed()
    if not allowed:
        raise HTTPException(
            status_code=423,
            detail=f"Motion command rejected: {reason}",
        )
    return comm


# ==============================================================================
# Request Models
# ==============================================================================
class DriveRequest(BaseModel):
    vx: int = Field(default=0, ge=-1000, le=1000, description="Forward/backward velocity (-1000 to +1000)")
    vy: int = Field(default=0, ge=-1000, le=1000, description="Strafe left/right velocity (-1000 to +1000)")
    omega: int = Field(default=0, ge=-1000, le=1000, description="Rotational angular velocity (-1000 to +1000)")


class ShooterRequest(BaseModel):
    flywheel: int = Field(default=0, ge=-1000, le=1000, description="Flywheel speed (-1000 to +1000)")
    angle_run: bool = Field(default=False, description="Whether pitch angle stepper is running")
    angle_reversed: bool = Field(default=False, description="Whether pitch angle direction is reversed")


class ShooterFlywheelRequest(BaseModel):
    speed: int = Field(default=0, ge=-1000, le=1000, description="Flywheel speed only (-1000 to +1000)")


class ShooterAngleRequest(BaseModel):
    run: bool = Field(default=True, description="Whether pitch angle stepper is running")
    reversed: bool = Field(default=False, description="Whether pitch angle direction is reversed")


class ConveyorRequest(BaseModel):
    trigger: int = Field(default=0, ge=-1000, le=1000, description="Trigger motor speed (-1000 to +1000)")
    spinner_run: bool = Field(default=False, description="Whether feeder spinner is running")
    spinner_reversed: bool = Field(default=False, description="Whether spinner direction is reversed")


class ConveyorTriggerRequest(BaseModel):
    speed: int = Field(default=0, ge=-1000, le=1000, description="Trigger motor speed (-1000 to +1000)")


class ConveyorSpinnerRequest(BaseModel):
    run: bool = Field(default=True, description="Whether feeder spinner is running")
    reversed: bool = Field(default=False, description="Whether spinner direction is reversed")


class StopRequest(BaseModel):
    part: int = Field(
        default=PartType.ALL.value,
        description="Subsystem to stop (0=ALL, 1=CHASSIS, 2=CONVEYOR, 3=SHOOTER, 4=SENSORS)"
    )
    clear_pending_drive: bool = Field(default=True, description="Purge pending drive commands from queue")


class PoseSensorUpdateRequest(BaseModel):
    dist_front_cm: float = Field(..., description="Front LiDAR distance in cm")
    dist_right_cm: float = Field(..., description="Right LiDAR distance in cm")
    raw_heading_deg: float = Field(..., description="Geomagnetic heading in degrees")
    dist_left_cm: Optional[float] = Field(
        default=None, description="Optional Left LiDAR distance in cm"
    )
    upper_rpm: Optional[int] = Field(
        default=None, description="Optional simulated upper flywheel speed in RPM"
    )
    lower_rpm: Optional[int] = Field(
        default=None, description="Optional simulated lower flywheel speed in RPM"
    )


class ESP32TelemetrySimulateRequest(BaseModel):
    upper_rpm: int = Field(default=0, ge=-10000, le=10000, description="Upper flywheel speed (RPM)")
    lower_rpm: int = Field(default=0, ge=-10000, le=10000, description="Lower flywheel speed (RPM)")
    heading_deg: int = Field(default=0, ge=-360, le=360, description="Earth-relative heading (deg)")
    dist_front_mm: int = Field(
        default=3500, ge=0, le=0xFFFFFFFF, description="Front LiDAR distance (mm)"
    )
    dist_right_mm: int = Field(
        default=2000, ge=0, le=0xFFFFFFFF, description="Right LiDAR distance (mm)"
    )
    dist_left_mm: int = Field(
        default=2000, ge=0, le=0xFFFFFFFF, description="Left LiDAR distance (mm)"
    )


class PoseCalibrateRequest(BaseModel):
    raw_heading_deg: Optional[float] = Field(
        default=None,
        description="Raw heading reading to calibrate as target_field_deg (uses latest reading if omitted)",
    )
    target_field_deg: float = Field(
        default=0.0,
        description="Target field angle in degrees (default 0.0 = facing Top 3x3 grid wall)",
    )
    reset_history: bool = Field(
        default=False,
        description="Whether to clear cached valid LiDAR distances and pose history",
    )


class VisionTargetRequest(BaseModel):
    timestamp: float = Field(default=0.0, description="Detector timestamp")
    active_target: str = Field(..., description="Active column: 'left', 'center', 'right', or 'none'")
    target_detected: bool = Field(default=False, description="Whether target is detected")
    force: bool = Field(default=False, description="Force override an already latched target")


# ==============================================================================
# REST API Endpoints
# ==============================================================================
@app.get("/api/status")
def get_status():
    """Retrieve communication statistics, E-Stop state, queue depth, match state, vision target, pose, and latest telemetry."""
    comm = get_uart_comm()
    esp32 = get_esp32_receiver()
    estimator = get_pose_estimator()
    match_mgr = get_match_manager()
    vision_mgr = get_vision_manager()
    stats = comm.get_stats()
    parsed_telem = _build_combined_telemetry()
    latest_pose = estimator.latest_pose

    return {
        "status": "estop" if comm.estop_active else "ok",
        "estop_active": comm.estop_active,
        "stats": stats,
        "esp32": esp32.get_stats(),
        "match": match_mgr.get_status(),
        "vision": vision_mgr.get_status(),
        "latest_telemetry": parsed_telem,
        "pose": latest_pose.to_dict() if latest_pose is not None else None,
    }


# ==============================================================================
# Match Control Endpoints (Autonomous 1:00 -> AUTO_WAIT -> Tele-Op 2:00)
# ==============================================================================
@app.get("/api/match")
def get_match_status():
    """Returns the current match state, mode, timer, and teleop allowance."""
    return get_match_manager().get_status()


@app.post("/api/match/auto/start")
def start_auto_phase():
    """Starts the 1:00 Autonomous phase (locks controller input) and unlatches vision target."""
    get_vision_manager().reset()
    return get_match_manager().start_auto()


@app.post("/api/match/teleop/enable")
def enable_teleop_phase():
    """
    Operator trigger: enables the 2:00 Tele-Op phase (unlocks controller input).
    Called manually after Auto completes or to begin driving.
    """
    return get_match_manager().enable_teleop()


@app.post("/api/match/practice/start")
def start_practice_phase():
    """Starts unlimited Tele-Op practice mode without a match countdown timer."""
    return get_match_manager().start_practice()


@app.post("/api/match/disarm")
def disarm_robot():
    """Disarms robot, resets match state machine, and halts all motors."""
    return get_match_manager().disarm()


@app.post("/api/match/reset")
def reset_match_phase():
    """Resets match timers, unlatches vision target, and returns robot to DISARMED state."""
    get_vision_manager().reset()
    return get_match_manager().reset_match()


# ==============================================================================
# Vision Target Endpoints (1x3 Special Row Detection from Laptop YOLO)
# ==============================================================================
@app.get("/api/vision/target")
def get_vision_target():
    """Returns the active 1x3 target column ('left', 'center', 'right', 'none') and field coordinates."""
    return get_vision_manager().get_status()


@app.post("/api/vision/target")
def update_vision_target(req: VisionTargetRequest):
    """
    Receives YOLO detection from operator's laptop.
    Once detected with target_detected=True, latches the target until Auto ends or match reset.
    """
    target = get_vision_manager().update(
        active_target=req.active_target,
        target_detected=req.target_detected,
        timestamp=req.timestamp,
        force=req.force,
    )
    return {"status": "ok", "vision": target.to_dict()}


@app.post("/api/vision/reset")
def reset_vision_target():
    """Manually resets the latched vision target back to NONE."""
    target = get_vision_manager().reset()
    return {"status": "reset", "vision": target.to_dict()}


@app.get("/api/pose")
def get_pose():
    """Returns the latest field pose estimate and field dimensions."""
    estimator = get_pose_estimator()
    latest_pose = estimator.latest_pose
    return {
        "field": {
            "width_cm": FieldConfig.WIDTH_CM,
            "length_cm": FieldConfig.LENGTH_CM,
        },
        "heading_offset_deg": estimator.heading_offset_deg,
        "pose": latest_pose.to_dict() if latest_pose is not None else None,
    }


@app.post("/api/pose/update")
def update_pose(req: PoseSensorUpdateRequest):
    """Updates the pose estimator with Front, Right & Left LiDAR distances (cm), geomagnetic heading (deg), and optional simulated RPMs."""
    if req.upper_rpm is not None or req.lower_rpm is not None:
        left_mm = (
            max(0, int(round(req.dist_left_cm * 10.0)))
            if req.dist_left_cm is not None
            else 0xFFFFFFFF
        )
        snap = get_esp32_receiver().inject_telemetry(
            upper_rpm=int(req.upper_rpm or 0),
            lower_rpm=int(req.lower_rpm or 0),
            heading_deg=int(round(req.raw_heading_deg)),
            dist_front_mm=max(0, int(round(req.dist_front_cm * 10.0))),
            dist_right_mm=max(0, int(round(req.dist_right_cm * 10.0))),
            dist_left_mm=left_mm,
        )
        latest_pose = get_pose_estimator().latest_pose
        return {
            "status": "ok",
            "pose": latest_pose.to_dict() if latest_pose is not None else None,
            "esp32": snap.to_dict(),
        }

    estimator = get_pose_estimator()
    estimate = estimator.update(
        dist_front_cm=req.dist_front_cm,
        dist_right_cm=req.dist_right_cm,
        raw_heading_deg=req.raw_heading_deg,
        dist_left_cm=req.dist_left_cm,
    )
    return {"status": "ok", "pose": estimate.to_dict()}


@app.post("/api/esp32/simulate")
def simulate_esp32_telemetry(req: ESP32TelemetrySimulateRequest):
    """
    Encodes a 25-byte binary ESP32 sensor frame (<hhhIII + CRC8) and feeds it
    through the ESP32SensorReceiver parser and PoseEstimator pipeline.
    """
    snap = get_esp32_receiver().inject_telemetry(
        upper_rpm=req.upper_rpm,
        lower_rpm=req.lower_rpm,
        heading_deg=req.heading_deg,
        dist_front_mm=req.dist_front_mm,
        dist_right_mm=req.dist_right_mm,
        dist_left_mm=req.dist_left_mm,
    )
    latest_pose = get_pose_estimator().latest_pose
    return {
        "status": "ok",
        "esp32": snap.to_dict(),
        "pose": latest_pose.to_dict() if latest_pose is not None else None,
    }


@app.post("/api/pose/calibrate")
def calibrate_pose(req: PoseCalibrateRequest):
    """Calibrates field heading zero-offset and/or resets cached valid LiDAR distances."""
    estimator = get_pose_estimator()
    if req.reset_history:
        estimator.reset_history()

    raw_heading = req.raw_heading_deg
    if raw_heading is None and estimator.latest_pose is not None:
        raw_heading = estimator.latest_pose.raw_heading_deg

    if raw_heading is not None:
        estimator.calibrate_heading(raw_heading, target_field_deg=req.target_field_deg)

    return {
        "status": "calibrated",
        "heading_offset_deg": estimator.heading_offset_deg,
        "reset_history": req.reset_history,
    }


@app.post("/api/drive")
def drive_robot(req: DriveRequest):
    """
    Appends a new chassis drive command into the TX PriorityQueue (Priority 2).
    Thread-safe and returns immediately.
    """
    comm = _require_teleop_allowed()
    comm.send_drive(vx=req.vx, vy=req.vy, omega=req.omega)
    return {"status": "enqueued", "subsystem": "chassis", "vx": req.vx, "vy": req.vy, "omega": req.omega}


@app.post("/api/shooter")
def control_shooter(req: ShooterRequest):
    """
    Controls shooter flywheels and pitch angle stepper state (run, reversed).
    """
    comm = _require_teleop_allowed()
    comm.send_shooter_command(
        flywheel=req.flywheel,
        angle_run=req.angle_run,
        angle_reversed=req.angle_reversed,
    )
    return {
        "status": "enqueued",
        "subsystem": "shooter",
        "flywheel": req.flywheel,
        "angle_run": req.angle_run,
        "angle_reversed": req.angle_reversed,
    }


@app.post("/api/shooter/flywheel")
def control_shooter_flywheel(req: ShooterFlywheelRequest):
    """
    Sets only the shooter flywheel speed.
    """
    comm = _require_teleop_allowed()
    comm.send_shooter_flywheel(speed=req.speed)
    return {"status": "enqueued", "subsystem": "shooter", "flywheel": req.speed}


@app.post("/api/shooter/angle")
def control_shooter_angle(req: ShooterAngleRequest):
    """
    Sets only the shooter pitch angle stepper state (run: bool, reversed: bool).
    """
    comm = _require_teleop_allowed()
    comm.send_shooter_angle(run=req.run, reversed=req.reversed)
    return {
        "status": "enqueued",
        "subsystem": "shooter",
        "angle_run": req.run,
        "angle_reversed": req.reversed,
    }


@app.post("/api/shooter/angle/disable")
def disable_shooter_angle():
    """
    Disables the shooter pitch angle stepper driver (de-energizes TB6560 EN pin).
    """
    comm = _require_ready_uart()
    comm.send_shooter_disable_angle()
    return {
        "status": "enqueued",
        "subsystem": "shooter",
        "angle_enabled": False,
    }


@app.post("/api/conveyor")
def control_conveyor(req: ConveyorRequest):
    """
    Controls all conveyor components: trigger motor speed and spinner state (run, reversed).
    """
    comm = _require_teleop_allowed()
    comm.send_conveyor_command(
        trigger=req.trigger,
        spinner_run=req.spinner_run,
        spinner_reversed=req.spinner_reversed,
    )
    return {
        "status": "enqueued",
        "subsystem": "conveyor",
        "trigger": req.trigger,
        "spinner_run": req.spinner_run,
        "spinner_reversed": req.spinner_reversed,
    }


@app.post("/api/conveyor/trigger")
def control_conveyor_trigger(req: ConveyorTriggerRequest):
    """
    Sets only the conveyor trigger motor speed.
    """
    comm = _require_teleop_allowed()
    comm.send_conveyor_trigger(speed=req.speed)
    return {"status": "enqueued", "subsystem": "conveyor", "trigger": req.speed}


@app.post("/api/conveyor/spinner")
def control_conveyor_spinner(req: ConveyorSpinnerRequest):
    """
    Sets only the conveyor spinner state (run: bool, reversed: bool).
    """
    comm = _require_teleop_allowed()
    comm.send_conveyor_spinner(run=req.run, reversed=req.reversed)
    return {
        "status": "enqueued",
        "subsystem": "conveyor",
        "spinner_run": req.run,
        "spinner_reversed": req.reversed,
    }


@app.post("/api/stop")
def stop_robot(req: StopRequest):
    """
    Appends an emergency or normal STOP command into the TX PriorityQueue (Priority 0).
    Purges any stale drive commands to halt motors instantly.
    """
    comm = get_uart_comm()
    comm.send_stop(part=PartType(req.part), clear_pending_drive=req.clear_pending_drive)
    return {"status": "stopped", "part": req.part, "estop_active": comm.estop_active}


@app.post("/api/ping")
def ping_slave():
    """Sends a keep-alive ping to the slave Pico."""
    comm = get_uart_comm()
    comm.send_ping()
    return {"status": "ping_sent", "estop_active": comm.estop_active}


# ==============================================================================
# HTTPS Root CA Download & uStreamer Video Pass-Through (Option A)
# ==============================================================================
@app.get("/ca.crt")
def download_ca_certificate():
    """Downloads the TELBot Local Root CA certificate for zero-warning HTTPS in browsers."""
    ca_path = ROOT_DIR / ServerConfig.CA_CERTFILE
    if not ca_path.is_file():
        raise HTTPException(
            status_code=404,
            detail="Root CA certificate not found. Run ./scripts/generate_certs.sh first.",
        )
    return FileResponse(
        path=str(ca_path),
        media_type="application/x-x509-ca-cert",
        filename="telbot-ca.crt",
    )


async def _open_ustreamer_http(path: str, port: int) -> tuple[int, dict[str, str], asyncio.StreamReader, asyncio.StreamWriter]:
    """Opens a local HTTP/1.0 connection to uStreamer and parses the response headers."""
    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection("127.0.0.1", port), timeout=2.0
        )
    except Exception as e:
        raise HTTPException(
            status_code=503,
            detail=f"Video server (uStreamer) is not reachable on 127.0.0.1:{port} ({e})",
        )

    try:
        req_bytes = (
            f"GET {path} HTTP/1.0\r\n"
            f"Host: 127.0.0.1:{port}\r\n"
            "Connection: close\r\n\r\n"
        ).encode("ascii")
        writer.write(req_bytes)
        await writer.drain()

        status_line = await asyncio.wait_for(reader.readline(), timeout=2.0)
        if not status_line:
            raise RuntimeError("Empty response from uStreamer")
        parts = status_line.decode("latin-1", errors="replace").strip().split(" ", 2)
        status_code = int(parts[1]) if len(parts) >= 2 and parts[1].isdigit() else 502

        headers: dict[str, str] = {}
        while True:
            line = await asyncio.wait_for(reader.readline(), timeout=2.0)
            if line in (b"\r\n", b"\n", b""):
                break
            decoded = line.decode("latin-1", errors="replace").strip()
            if ":" in decoded:
                k, v = decoded.split(":", 1)
                headers[k.strip().lower()] = v.strip()

        return status_code, headers, reader, writer
    except Exception:
        writer.close()
        raise


@app.get("/api/video/stream")
async def proxy_video_stream(
    port: int = Query(default=CameraConfig.PORT, ge=1, le=65535)
):
    """
    Streams the live MJPEG feed from the local uStreamer process over the same
    HTTPS origin so browsers never encounter Mixed Content blocks.
    """
    status_code, headers, reader, writer = await _open_ustreamer_http("/stream", port)
    if status_code != 200:
        writer.close()
        raise HTTPException(
            status_code=status_code,
            detail=f"uStreamer /stream returned HTTP {status_code}",
        )

    _active_stream_writers.add(writer)

    content_type = headers.get(
        "content-type", "multipart/x-mixed-replace;boundary=boundarydonotcross"
    )

    async def stream_generator():
        try:
            while True:
                chunk = await reader.read(16384)
                if not chunk:
                    break
                yield chunk
        except (asyncio.CancelledError, ConnectionResetError):
            pass
        finally:
            _active_stream_writers.discard(writer)
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:
                pass

    return StreamingResponse(
        stream_generator(),
        media_type=content_type,
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0",
            "Pragma": "no-cache",
            "X-Accel-Buffering": "no",
        },
    )


@app.get("/api/video/snapshot")
async def proxy_video_snapshot(
    port: int = Query(default=CameraConfig.PORT, ge=1, le=65535)
):
    """Proxies a single JPEG snapshot from the local uStreamer process."""
    status_code, headers, reader, writer = await _open_ustreamer_http("/snapshot", port)
    try:
        body = await asyncio.wait_for(reader.read(), timeout=3.0)
    finally:
        writer.close()

    if status_code != 200:
        raise HTTPException(
            status_code=status_code,
            detail=f"uStreamer /snapshot returned HTTP {status_code}",
        )
    return Response(
        content=body,
        media_type=headers.get("content-type", "image/jpeg"),
        headers={"Cache-Control": "no-store, no-cache, max-age=0"},
    )


@app.get("/api/video/state")
async def proxy_video_state(
    port: int = Query(default=CameraConfig.PORT, ge=1, le=65535)
):
    """Proxies the live JSON state from the local uStreamer process."""
    try:
        status_code, _, reader, writer = await _open_ustreamer_http("/state", port)
        try:
            body = await asyncio.wait_for(reader.read(), timeout=2.0)
        finally:
            writer.close()
        return JSONResponse(status_code=status_code, content=json.loads(body.decode("utf-8")))
    except HTTPException:
        return JSONResponse(
            status_code=200,
            content={
                "ok": False,
                "online": False,
                "error": f"uStreamer is not running on 127.0.0.1:{port}",
            },
        )
    except Exception as e:
        return JSONResponse(
            status_code=200,
            content={"ok": False, "online": False, "error": str(e)},
        )


# ==============================================================================
# WebSocket Endpoint for Low-Latency Joystick & Control Streaming
# ==============================================================================
@app.websocket("/ws/control")
async def websocket_control(websocket: WebSocket):
    """
    WebSocket endpoint for real-time WebHID / Gamepad input stream.
    Receives JSON messages such as:
      {"type": "drive", "vx": 300, "vy": 0, "omega": -100}
      {"type": "shooter", "flywheel": 800, "angle_run": true, "angle_reversed": false}
      {"type": "conveyor", "trigger": 500, "spinner_run": true, "spinner_reversed": false}
      {"type": "stop", "part": 0}
    """
    await websocket.accept()
    logger.info("WebHID/Gamepad WebSocket client connected.")
    comm = get_uart_comm()

    try:
        while True:
            data = await websocket.receive_json()
            msg_type = data.get("type")
            accepted = True
            match_mgr = get_match_manager()
            teleop_allowed, reason = match_mgr.check_teleop_allowed()

            if msg_type in ("drive", "shooter", "conveyor") and not teleop_allowed:
                accepted = False
            elif msg_type == "drive":
                vx = int(data.get("vx", 0))
                vy = int(data.get("vy", 0))
                omega = int(data.get("omega", 0))
                accepted = comm.send_drive(vx=vx, vy=vy, omega=omega)

            elif msg_type == "shooter":
                flywheel = int(data.get("flywheel", 0))
                angle_run = bool(data.get("angle_run", False))
                angle_reversed = bool(data.get("angle_reversed", False))
                accepted = comm.send_shooter_command(
                    flywheel=flywheel,
                    angle_run=angle_run,
                    angle_reversed=angle_reversed,
                )

            elif msg_type == "shooter_disable_angle":
                accepted = comm.send_shooter_disable_angle()

            elif msg_type == "conveyor":
                trigger = int(data.get("trigger", 0))
                spinner_run = bool(data.get("spinner_run", False))
                spinner_reversed = bool(data.get("spinner_reversed", False))
                accepted = comm.send_conveyor_command(
                    trigger=trigger,
                    spinner_run=spinner_run,
                    spinner_reversed=spinner_reversed,
                )

            elif msg_type == "stop":
                part_val = int(data.get("part", PartType.ALL.value))
                comm.send_stop(part=PartType(part_val))

            elif msg_type == "vision":
                active_target = str(data.get("active_target", "none"))
                target_detected = bool(data.get("target_detected", False))
                ts = float(data.get("timestamp", 0.0))
                force = bool(data.get("force", False))
                get_vision_manager().update(
                    active_target=active_target,
                    target_detected=target_detected,
                    timestamp=ts,
                    force=force,
                )

            # Read latest stats, pose & telemetry snapshot to send back to client
            stats = comm.get_stats()
            parsed_telem = _build_combined_telemetry()
            latest_pose = get_pose_estimator().latest_pose

            await websocket.send_json({
                "estop_active": comm.estop_active,
                "command_accepted": accepted,
                "queue_depth": stats["queue_depth"],
                "tx_packets": stats["tx_packets"],
                "rx_packets": stats["rx_packets"],
                "esp32": get_esp32_receiver().get_stats(),
                "match": match_mgr.get_status(),
                "vision": get_vision_manager().get_status(),
                "telemetry": parsed_telem,
                "pose": latest_pose.to_dict() if latest_pose is not None else None,
            })

    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected. Issuing safety stop...")
        comm.send_stop()
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        comm.send_stop()
