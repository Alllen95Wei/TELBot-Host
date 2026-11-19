import asyncio
import json
import logging
import os
import sys
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import Body, FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
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
from config import ShooterPID as ShooterPIDConfig
from config import SpecialGrid as SpecialGridConfig
from config import UART as UARTConfig
from core import (
    AutoRoutineRunner,
    ChassisPoseController,
    ConveyorController,
    ESP32SensorReceiver,
    ESP32SensorSnapshot,
    MatchManager,
    MatchMode,
    PartType,
    PoseEstimator,
    RoutineStore,
    ShooterSpeedController,
    UARTComm,
    VisionManager,
    VisionTarget,
    step_auto_routine,
    step_chassis_control,
    step_conveyor_control,
    step_shooter_control,
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
shooter_controller = ShooterSpeedController()
conveyor_controller = ConveyorController()
chassis_controller = ChassisPoseController()
routine_store = RoutineStore()
auto_runner = AutoRoutineRunner(routine_store)


def _on_match_stop_with_comm(comm_ref: UARTComm) -> None:
    get_auto_runner().abort("Match stopped")
    get_shooter_controller().stop()
    get_conveyor_controller().stop()
    get_chassis_controller().stop()
    comm_ref.send_stop()


match_manager = MatchManager(on_stop_required=lambda: _on_match_stop_with_comm(get_uart_comm()))
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
    match_manager.on_stop_required = lambda: _on_match_stop_with_comm(comm)


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


def set_shooter_controller(controller: ShooterSpeedController) -> None:
    """Injects an externally managed ShooterSpeedController instance (used by main.py)."""
    global shooter_controller
    shooter_controller = controller
    app.state.shooter_controller = controller


def get_shooter_controller() -> ShooterSpeedController:
    """Returns the active ShooterSpeedController instance."""
    return getattr(app.state, "shooter_controller", shooter_controller)


def set_conveyor_controller(controller: ConveyorController) -> None:
    """Injects an externally managed ConveyorController instance (used by main.py)."""
    global conveyor_controller
    conveyor_controller = controller
    app.state.conveyor_controller = controller


def get_conveyor_controller() -> ConveyorController:
    """Returns the active ConveyorController instance."""
    return getattr(app.state, "conveyor_controller", conveyor_controller)


def set_chassis_controller(controller: ChassisPoseController) -> None:
    """Injects an externally managed ChassisPoseController instance (used by main.py)."""
    global chassis_controller
    chassis_controller = controller
    app.state.chassis_controller = controller


def get_chassis_controller() -> ChassisPoseController:
    """Returns the active ChassisPoseController instance."""
    return getattr(app.state, "chassis_controller", chassis_controller)


def set_routine_store(store: RoutineStore) -> None:
    """Injects an externally managed RoutineStore instance (used by main.py)."""
    global routine_store
    routine_store = store
    app.state.routine_store = store


def get_routine_store() -> RoutineStore:
    """Returns the active RoutineStore instance."""
    return getattr(app.state, "routine_store", routine_store)


def set_auto_runner(runner: AutoRoutineRunner) -> None:
    """Injects an externally managed AutoRoutineRunner instance (used by main.py)."""
    global auto_runner
    auto_runner = runner
    app.state.auto_runner = runner


def get_auto_runner() -> AutoRoutineRunner:
    """Returns the active AutoRoutineRunner instance."""
    return getattr(app.state, "auto_runner", auto_runner)


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
            combined["ball_detected"] = snap_dict["ball_detected"]

    if combined:
        combined["shooter_pid"] = get_shooter_controller().get_status()
        combined["conveyor"] = get_conveyor_controller().get_status()
        combined["chassis_pid"] = get_chassis_controller().get_status()
        combined["auto_routine"] = get_auto_runner().get_status()
        return combined
    return None


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

    pid_task: Optional[asyncio.Task] = None
    if not getattr(app.state, "control_loop_managed", False):
        async def _standalone_shooter_pid_loop() -> None:
            tick_interval = 1.0 / max(1.0, float(ShooterPIDConfig.CONTROL_LOOP_HZ))
            last_sent_flywheel = (0, 0)
            last_sent_spinner = (False, False)
            last_sent_drive = (0, 0, 0)
            while True:
                loop_start = time.monotonic()
                comm_ref = get_uart_comm()
                match_ref = get_match_manager()
                esp32_ref = get_esp32_receiver()
                pose_ref = get_pose_estimator()
                shooter_ref = get_shooter_controller()
                conveyor_ref = get_conveyor_controller()
                chassis_ref = get_chassis_controller()
                vision_ref = get_vision_manager()
                step_auto_routine(
                    runner=get_auto_runner(),
                    uart=comm_ref,
                    match_manager=match_ref,
                    pose_estimator=pose_ref,
                    vision_manager=vision_ref,
                    esp32_receiver=esp32_ref,
                    chassis_controller=chassis_ref,
                    shooter_controller=shooter_ref,
                    conveyor_controller=conveyor_ref,
                    now=loop_start,
                )
                last_sent_flywheel = step_shooter_control(
                    controller=shooter_ref,
                    uart=comm_ref,
                    match_manager=match_ref,
                    pose_estimator=pose_ref,
                    vision_manager=vision_ref,
                    esp32_receiver=esp32_ref,
                    last_sent_flywheel=last_sent_flywheel,
                    now=loop_start,
                )
                last_sent_spinner = step_conveyor_control(
                    conveyor_controller=conveyor_ref,
                    shooter_controller=shooter_ref,
                    uart=comm_ref,
                    match_manager=match_ref,
                    esp32_receiver=esp32_ref,
                    last_sent_spinner=last_sent_spinner,
                    now=loop_start,
                )
                last_sent_drive = step_chassis_control(
                    controller=chassis_ref,
                    uart=comm_ref,
                    match_manager=match_ref,
                    pose_estimator=pose_ref,
                    last_sent_drive=last_sent_drive,
                    now=loop_start,
                )
                elapsed = time.monotonic() - loop_start
                await asyncio.sleep(max(0.001, tick_interval - elapsed))

        pid_task = asyncio.create_task(
            _standalone_shooter_pid_loop(), name="Standalone-Shooter-PID-Loop"
        )

    yield

    if pid_task is not None:
        pid_task.cancel()
        await asyncio.gather(pid_task, return_exceptions=True)
        get_auto_runner().abort("Standalone server shutdown")
        get_shooter_controller().stop()
        get_conveyor_controller().stop()
        get_chassis_controller().stop()

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


@app.get("/auto", include_in_schema=False)
def serve_auto_editor():
    """Serves the dedicated Autonomous Routine Editor page."""
    editor_file = PANEL_DIR / "auto_editor.html"
    if not editor_file.is_file():
        raise HTTPException(status_code=404, detail="web_panel/auto_editor.html not found")
    return FileResponse(path=str(editor_file), media_type="text/html")


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


def _require_chassis_p2p_practice_allowed() -> UARTComm:
    """
    Validates that UART is ready, E-Stop is not active, and current match mode
    is PRACTICE (remote P2P waypoint triggers are locked out in AUTO and TELEOP).
    """
    comm = _require_ready_uart()
    mode = get_match_manager().mode
    if mode != MatchMode.PRACTICE:
        raise HTTPException(
            status_code=423,
            detail=(
                "Chassis P2P waypoint command rejected: only permitted in PRACTICE mode "
                f"(current mode: {mode.value})"
            ),
        )
    return comm


# ==============================================================================
# Request Models
# ==============================================================================
class DriveRequest(BaseModel):
    vx: int = Field(default=0, ge=-1000, le=1000, description="Forward/backward velocity (-1000 to +1000)")
    vy: int = Field(default=0, ge=-1000, le=1000, description="Strafe left/right velocity (-1000 to +1000)")
    omega: int = Field(default=0, ge=-1000, le=1000, description="Rotational angular velocity (-1000 to +1000)")


class ChassisPIDUpdateRequest(BaseModel):
    axis: str = Field(default="all", description="Target axis for gain updates: 'x', 'y', 'xy', 'heading', or 'all'")
    kp: Optional[float] = Field(default=None, ge=0.0)
    ki: Optional[float] = Field(default=None, ge=0.0)
    kd: Optional[float] = Field(default=None, ge=0.0)
    max_linear_speed: Optional[float] = Field(default=None, gt=0.0, le=1000.0)
    max_angular_speed: Optional[float] = Field(default=None, gt=0.0, le=1000.0)
    min_linear_cmd: Optional[float] = Field(default=None, ge=0.0, le=1000.0)
    min_angular_cmd: Optional[float] = Field(default=None, ge=0.0, le=1000.0)
    pos_tolerance_cm: Optional[float] = Field(default=None, ge=0.0)
    heading_tolerance_deg: Optional[float] = Field(default=None, ge=0.0, le=180.0)
    settle_time_s: Optional[float] = Field(default=None, ge=0.0)
    pose_stale_timeout_s: Optional[float] = Field(default=None, gt=0.0)
    max_blind_hold_s: Optional[float] = Field(default=None, ge=0.0)
    wall_margin_cm: Optional[float] = Field(default=None, ge=0.0)
    enabled: Optional[bool] = Field(default=None, description="Enable or disable chassis P2P PID")
    reset: bool = Field(default=False, description="Reset PID integrators and settle timers")


class ChassisGotoRequest(BaseModel):
    target_x_cm: float = Field(..., description="Target field X coordinate in cm")
    target_y_cm: float = Field(..., description="Target field Y coordinate in cm")
    target_heading_deg: Optional[float] = Field(
        default=None, description="Target field heading in degrees (0 deg = Top wall)"
    )
    aim_at_goal: bool = Field(
        default=False,
        description="Compute target heading automatically to aim at active/center goal",
    )
    goal_x_cm: Optional[float] = Field(default=None, description="Optional explicit goal X coordinate (cm)")
    goal_y_cm: Optional[float] = Field(default=None, description="Optional explicit goal Y coordinate (cm)")
    stop_on_arrival: bool = Field(
        default=True,
        description="Automatically disable P2P and stop chassis once settled at target",
    )


class ShooterRequest(BaseModel):
    flywheel: int = Field(default=0, ge=-1000, le=1000, description="Flywheel speed (-1000 to +1000)")
    upper_flywheel: Optional[int] = Field(
        default=None, ge=-1000, le=1000, description="Independent upper flywheel speed (-1000 to +1000)"
    )
    lower_flywheel: Optional[int] = Field(
        default=None, ge=-1000, le=1000, description="Independent lower flywheel speed (-1000 to +1000)"
    )
    target_rpm: Optional[float] = Field(
        default=None, ge=0.0, le=ShooterPIDConfig.MAX_RPM, description="Shared target RPM (0 to MAX_RPM)"
    )
    mode: Optional[str] = Field(default=None, description="Speed control mode: 'auto' or 'manual'")
    angle_run: bool = Field(default=False, description="Whether pitch angle stepper is running")
    angle_reversed: bool = Field(default=False, description="Whether pitch angle direction is reversed")


class ShooterFlywheelRequest(BaseModel):
    speed: int = Field(default=0, ge=-1000, le=1000, description="Flywheel speed only (-1000 to +1000)")
    lower_speed: Optional[int] = Field(
        default=None, ge=-1000, le=1000, description="Optional independent lower flywheel speed (-1000 to +1000)"
    )
    target_rpm: Optional[float] = Field(
        default=None, ge=0.0, le=ShooterPIDConfig.MAX_RPM, description="Shared target RPM (0 to MAX_RPM)"
    )
    mode: Optional[str] = Field(default=None, description="Speed control mode: 'auto' or 'manual'")


class ShooterPIDUpdateRequest(BaseModel):
    mode: Optional[str] = Field(default=None, description="Speed mode: 'auto' or 'manual'")
    target_rpm: Optional[float] = Field(
        default=None, ge=0.0, le=ShooterPIDConfig.MAX_RPM, description="Manual / fallback target RPM"
    )
    enabled: Optional[bool] = Field(default=None, description="Enable or disable closed-loop shooter PID")
    wheel: str = Field(default="both", description="Target PID wheel for gain updates: 'upper', 'lower', or 'both'")
    kp: Optional[float] = Field(default=None, ge=0.0)
    ki: Optional[float] = Field(default=None, ge=0.0)
    kd: Optional[float] = Field(default=None, ge=0.0)
    kv: Optional[float] = Field(default=None, ge=0.0)
    ks: Optional[float] = Field(default=None, ge=0.0)
    reset: bool = Field(default=False, description="Reset PID integral and derivative history")


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


class ConveyorAutoReverseUpdateRequest(BaseModel):
    enabled: Optional[bool] = Field(default=None, description="Enable or disable automatic spinner anti-jam reverse")
    no_ball_delay_s: Optional[float] = Field(default=None, ge=0.0, description="Continuous no-ball time while shooting before reversing (seconds)")
    reverse_duration_s: Optional[float] = Field(default=None, ge=0.0, description="Duration of each automatic reverse pulse (seconds)")
    cooldown_s: Optional[float] = Field(default=None, ge=0.0, description="Normal dwell time between consecutive reverse pulses (seconds)")
    reset: bool = Field(default=False, description="Reset auto-reverse state machine back to idle")


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
    ball_detected: Optional[bool] = Field(
        default=None, description="Optional simulated IR ball sensor state"
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
    ball_detected: bool = Field(
        default=False, description="IR ball sensor state (False=no ball, True=ball detected)"
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


class AutoActiveRequest(BaseModel):
    routine_id: str = Field(..., description="Routine ID to activate for Autonomous phase")


class AutoRunRequest(BaseModel):
    routine_id: Optional[str] = Field(
        default=None, description="Optional routine ID to execute (defaults to active routine)"
    )


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
        "shooter_pid": get_shooter_controller().get_status(),
        "conveyor": get_conveyor_controller().get_status(),
        "chassis_pid": get_chassis_controller().get_status(),
        "auto_routine": get_auto_runner().get_status(),
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
    """Starts the 1:00 Autonomous phase (locks controller input), resets vision, and starts active auto routine."""
    get_vision_manager().reset()
    get_chassis_controller().stop()
    get_shooter_controller().stop()
    get_conveyor_controller().stop()
    match_status = get_match_manager().start_auto()
    auto_state = get_auto_runner().start()
    return {**match_status, "auto_routine": auto_state}


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
    """Updates the pose estimator with Front, Right & Left LiDAR distances (cm), geomagnetic heading (deg), and optional simulated RPMs / IR ball state."""
    if req.upper_rpm is not None or req.lower_rpm is not None or req.ball_detected is not None:
        left_mm = (
            max(0, int(round(req.dist_left_cm * 10.0)))
            if req.dist_left_cm is not None
            else 0xFFFFFFFF
        )
        esp32_rx = get_esp32_receiver()
        latest_snap = esp32_rx.latest_snapshot
        ball_detected = (
            bool(req.ball_detected)
            if req.ball_detected is not None
            else (bool(latest_snap.ball_detected) if latest_snap is not None else False)
        )
        snap = esp32_rx.inject_telemetry(
            upper_rpm=int(req.upper_rpm or 0),
            lower_rpm=int(req.lower_rpm or 0),
            heading_deg=int(round(req.raw_heading_deg)),
            dist_front_mm=max(0, int(round(req.dist_front_cm * 10.0))),
            dist_right_mm=max(0, int(round(req.dist_right_cm * 10.0))),
            dist_left_mm=left_mm,
            ball_detected=ball_detected,
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
    Encodes a 26-byte binary ESP32 sensor frame (<hhhIIIB + CRC8) and feeds it
    through the ESP32SensorReceiver parser and PoseEstimator pipeline.
    """
    snap = get_esp32_receiver().inject_telemetry(
        upper_rpm=req.upper_rpm,
        lower_rpm=req.lower_rpm,
        heading_deg=req.heading_deg,
        dist_front_mm=req.dist_front_mm,
        dist_right_mm=req.dist_right_mm,
        dist_left_mm=req.dist_left_mm,
        ball_detected=req.ball_detected,
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


@app.get("/api/auto/routines")
def list_auto_routines():
    """Lists all saved autonomous routines, the active routine ID, and live runner status."""
    store = get_routine_store()
    return {
        "status": "ok",
        "active_routine_id": store.active_routine_id,
        "routines": store.list_routines(),
        "runner": get_auto_runner().get_status(),
    }


@app.get("/api/auto/routines/{routine_id}")
def get_auto_routine(routine_id: str):
    """Fetches a single autonomous routine JSON definition by ID."""
    store = get_routine_store()
    try:
        routine = store.get_routine(routine_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e))
    return {
        "status": "ok",
        "routine": routine.to_dict(),
        "is_active": (routine.id == store.active_routine_id),
    }


@app.post("/api/auto/routines")
def save_auto_routine(payload: dict = Body(...)):
    """Creates or updates a structured autonomous routine JSON definition."""
    store = get_routine_store()
    try:
        saved = store.save_routine(payload)
        if bool(payload.get("set_active")):
            store.set_active_routine(saved.id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    return {
        "status": "saved",
        "routine": saved.to_dict(),
        "active_routine_id": store.active_routine_id,
        "routines": store.list_routines(),
    }


@app.delete("/api/auto/routines/{routine_id}")
def delete_auto_routine(routine_id: str):
    """Deletes a saved autonomous routine unless it is the last one or currently running."""
    runner = get_auto_runner()
    if runner.is_running and runner.state.routine_id == routine_id:
        raise HTTPException(
            status_code=409,
            detail=f"Cannot delete routine '{routine_id}' while it is running",
        )
    store = get_routine_store()
    try:
        store.delete_routine(routine_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e))
    return {
        "status": "deleted",
        "active_routine_id": store.active_routine_id,
        "routines": store.list_routines(),
    }


@app.post("/api/auto/active")
def set_active_auto_routine(req: AutoActiveRequest):
    """Selects which routine runs during the 1:00 Autonomous phase."""
    store = get_routine_store()
    try:
        active = store.set_active_routine(req.routine_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e))
    return {
        "status": "ok",
        "active_routine_id": store.active_routine_id,
        "routine": active.to_dict(),
    }


@app.post("/api/auto/run")
def run_auto_routine(req: Optional[AutoRunRequest] = Body(default=None)):
    """Starts running an autonomous routine (permitted in PRACTICE or AUTO mode)."""
    _require_ready_uart()
    mode = get_match_manager().mode
    if mode not in (MatchMode.PRACTICE, MatchMode.AUTO):
        raise HTTPException(
            status_code=423,
            detail=(
                "Autonomous routine run rejected: only permitted in PRACTICE or AUTO mode "
                f"(current mode: {mode.value})"
            ),
        )

    store = get_routine_store()
    routine = None
    if req is not None and req.routine_id:
        try:
            routine = store.get_routine(req.routine_id)
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
        except KeyError as e:
            raise HTTPException(status_code=404, detail=str(e))

    get_chassis_controller().stop()
    get_shooter_controller().stop()
    get_conveyor_controller().stop()
    state = get_auto_runner().start(routine=routine)
    return {"status": "started", "auto_routine": state}


@app.post("/api/auto/abort")
def abort_auto_routine():
    """Aborts any running autonomous routine and stops all controlled subsystems."""
    comm = get_uart_comm()
    state = get_auto_runner().abort(
        reason="Aborted by operator",
        uart=comm,
        chassis_controller=get_chassis_controller(),
        shooter_controller=get_shooter_controller(),
        conveyor_controller=get_conveyor_controller(),
    )
    return {"status": "aborted", "auto_routine": state}


@app.post("/api/drive")
def drive_robot(req: DriveRequest):
    """
    Appends a new chassis drive command into the TX PriorityQueue (Priority 2).
    Any non-zero manual drive command immediately cancels active auto routine and Chassis P2P PID.
    """
    comm = _require_teleop_allowed()
    chassis_ctrl = get_chassis_controller()
    runner = get_auto_runner()
    if req.vx != 0 or req.vy != 0 or req.omega != 0:
        if runner.is_running:
            runner.abort(
                reason="Preempted by manual driver input",
                uart=comm,
                chassis_controller=chassis_ctrl,
                shooter_controller=get_shooter_controller(),
                conveyor_controller=get_conveyor_controller(),
            )
        chassis_ctrl.stop()
        comm.send_drive(vx=req.vx, vy=req.vy, omega=req.omega)
        return {"status": "enqueued", "subsystem": "chassis", "vx": req.vx, "vy": req.vy, "omega": req.omega}

    if runner.is_running or chassis_ctrl.enabled:
        return {
            "status": "ignored_p2p_active",
            "subsystem": "chassis",
            "vx": 0,
            "vy": 0,
            "omega": 0,
            "chassis_pid": chassis_ctrl.get_status(),
            "auto_routine": runner.get_status(),
        }

    comm.send_drive(vx=0, vy=0, omega=0)
    return {"status": "enqueued", "subsystem": "chassis", "vx": 0, "vy": 0, "omega": 0}


@app.get("/api/chassis/pid")
def get_chassis_pid():
    """Returns current 3-DOF chassis P2P PID status, gains, and configuration."""
    ctrl = get_chassis_controller()
    return {
        "status": "ok",
        "state": ctrl.get_status(),
        "gains": ctrl.get_gains(),
        "config": ctrl.get_config(),
    }


@app.post("/api/chassis/pid")
def update_chassis_pid(req: ChassisPIDUpdateRequest):
    """Updates chassis P2P PID gains, speed caps, tolerances, enable state, or reset."""
    ctrl = get_chassis_controller()
    try:
        if any(g is not None for g in (req.kp, req.ki, req.kd)):
            ctrl.set_gains(
                axis=req.axis,
                kp=req.kp,
                ki=req.ki,
                kd=req.kd,
            )
        if any(
            v is not None
            for v in (
                req.max_linear_speed,
                req.max_angular_speed,
                req.min_linear_cmd,
                req.min_angular_cmd,
                req.pos_tolerance_cm,
                req.heading_tolerance_deg,
                req.settle_time_s,
                req.pose_stale_timeout_s,
                req.max_blind_hold_s,
                req.wall_margin_cm,
            )
        ):
            ctrl.set_config(
                max_linear_speed=req.max_linear_speed,
                max_angular_speed=req.max_angular_speed,
                min_linear_cmd=req.min_linear_cmd,
                min_angular_cmd=req.min_angular_cmd,
                pos_tolerance_cm=req.pos_tolerance_cm,
                heading_tolerance_deg=req.heading_tolerance_deg,
                settle_time_s=req.settle_time_s,
                pose_stale_timeout_s=req.pose_stale_timeout_s,
                max_blind_hold_s=req.max_blind_hold_s,
                wall_margin_cm=req.wall_margin_cm,
            )
        if req.enabled is not None:
            ctrl.set_enabled(req.enabled)
        if req.reset:
            ctrl.reset()
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

    return {
        "status": "ok",
        "state": ctrl.get_status(),
        "gains": ctrl.get_gains(),
        "config": ctrl.get_config(),
    }


@app.post("/api/chassis/goto")
def goto_chassis_waypoint(req: ChassisGotoRequest):
    """Starts a closed-loop P2P move to a field waypoint (Practice mode only)."""
    _require_chassis_p2p_practice_allowed()
    ctrl = get_chassis_controller()
    try:
        if req.aim_at_goal or req.target_heading_deg is None:
            if req.goal_x_cm is not None and req.goal_y_cm is not None:
                gx = req.goal_x_cm
                gy = req.goal_y_cm
            else:
                vt = get_vision_manager().latest_target
                if (
                    vt is not None
                    and vt.latched
                    and vt.target_x_cm is not None
                    and vt.target_y_cm is not None
                ):
                    gx = vt.target_x_cm
                    gy = vt.target_y_cm
                else:
                    gx = SpecialGridConfig.CENTER_X_CM
                    gy = SpecialGridConfig.TOP_Y_CM
            ctrl.set_target_aim_at_goal(
                target_x_cm=req.target_x_cm,
                target_y_cm=req.target_y_cm,
                goal_x_cm=gx,
                goal_y_cm=gy,
                enable=True,
                stop_on_arrival=req.stop_on_arrival,
            )
        else:
            ctrl.set_target(
                target_x_cm=req.target_x_cm,
                target_y_cm=req.target_y_cm,
                target_heading_deg=req.target_heading_deg,
                enable=True,
                stop_on_arrival=req.stop_on_arrival,
            )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

    return {
        "status": "ok",
        "chassis_pid": ctrl.get_status(),
    }


@app.post("/api/chassis/cancel")
def cancel_chassis_waypoint():
    """Cancels active chassis P2P PID motion and zeroes chassis velocity."""
    comm = _require_ready_uart()
    ctrl = get_chassis_controller()
    ctrl.stop()
    comm.send_drive(0, 0, 0)
    return {
        "status": "cancelled",
        "chassis_pid": ctrl.get_status(),
    }


@app.post("/api/shooter")
def control_shooter(req: ShooterRequest):
    """
    Controls shooter flywheels (via closed-loop PID or explicit split commands)
    and pitch angle stepper state (run, reversed).
    """
    comm = _require_teleop_allowed()
    ctrl = get_shooter_controller()
    if req.upper_flywheel is not None or req.lower_flywheel is not None:
        upper = req.flywheel if req.upper_flywheel is None else req.upper_flywheel
        lower = upper if req.lower_flywheel is None else req.lower_flywheel
        ctrl.stop()
        comm.send_shooter_command(
            upper_flywheel=upper,
            lower_flywheel=lower,
            angle_run=req.angle_run,
            angle_reversed=req.angle_reversed,
        )
        return {
            "status": "enqueued",
            "subsystem": "shooter",
            "flywheel": upper,
            "upper_flywheel": upper,
            "lower_flywheel": lower,
            "angle_run": req.angle_run,
            "angle_reversed": req.angle_reversed,
        }

    ctrl.set_operator_command(
        flywheel_cmd=req.flywheel,
        mode=req.mode,
        target_rpm=req.target_rpm,
    )
    comm.send_shooter_angle(run=req.angle_run, reversed=req.angle_reversed)
    return {
        "status": "enqueued",
        "subsystem": "shooter",
        "flywheel": req.flywheel,
        "angle_run": req.angle_run,
        "angle_reversed": req.angle_reversed,
        "shooter_pid": ctrl.get_status(),
    }


@app.post("/api/shooter/flywheel")
def control_shooter_flywheel(req: ShooterFlywheelRequest):
    """
    Sets shooter flywheel speed via closed-loop PID (or direct split lower_speed override).
    """
    comm = _require_teleop_allowed()
    ctrl = get_shooter_controller()
    if req.lower_speed is not None:
        ctrl.stop()
        comm.send_shooter_flywheel(speed=req.speed, lower_speed=req.lower_speed)
        return {
            "status": "enqueued",
            "subsystem": "shooter",
            "flywheel": req.speed,
            "lower_flywheel": req.lower_speed,
        }

    ctrl.set_operator_command(
        flywheel_cmd=req.speed,
        mode=req.mode,
        target_rpm=req.target_rpm,
    )
    return {
        "status": "enqueued",
        "subsystem": "shooter",
        "flywheel": req.speed,
        "shooter_pid": ctrl.get_status(),
    }


@app.get("/api/shooter/pid")
def get_shooter_pid():
    """Returns current dual flywheel PID controller status and configured gains."""
    ctrl = get_shooter_controller()
    return {
        "status": "ok",
        "state": ctrl.get_status(),
        "gains": ctrl.get_gains(),
    }


@app.post("/api/shooter/pid")
def update_shooter_pid(req: ShooterPIDUpdateRequest):
    """Updates flywheel PID mode, target RPM, enable state, or runtime gains."""
    ctrl = get_shooter_controller()
    try:
        if req.mode is not None:
            ctrl.set_mode(req.mode)
        if req.target_rpm is not None:
            ctrl.set_manual_rpm(
                req.target_rpm,
                switch_to_manual=(req.mode != "auto" if req.mode is not None else True),
            )
        if any(g is not None for g in (req.kp, req.ki, req.kd, req.kv, req.ks)):
            ctrl.set_gains(
                wheel=req.wheel,
                kp=req.kp,
                ki=req.ki,
                kd=req.kd,
                kv=req.kv,
                ks=req.ks,
            )
        if req.enabled is not None:
            ctrl.set_enabled(req.enabled)
        if req.reset:
            ctrl.reset()
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

    return {
        "status": "ok",
        "state": ctrl.get_status(),
        "gains": ctrl.get_gains(),
    }


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


@app.post("/api/shooter/angle/reset_steps")
def reset_shooter_angle_steps():
    """
    Resets the cumulative shooter pitch stepper step counter on the Pico to 0.
    """
    comm = _require_ready_uart()
    comm.send_shooter_reset_angle_step()
    return {
        "status": "enqueued",
        "subsystem": "shooter",
        "angle_steps_reset": True,
    }


@app.post("/api/conveyor")
def control_conveyor(req: ConveyorRequest):
    """
    Controls all conveyor components: trigger motor speed and spinner state (run, reversed).
    """
    comm = _require_teleop_allowed()
    ctrl = get_conveyor_controller()
    eff_trig, eff_run, eff_rev = ctrl.set_operator_command(
        trigger=req.trigger,
        spinner_run=req.spinner_run,
        spinner_reversed=req.spinner_reversed,
    )
    comm.send_conveyor_command(
        trigger=eff_trig,
        spinner_run=eff_run,
        spinner_reversed=eff_rev,
    )
    return {
        "status": "enqueued",
        "subsystem": "conveyor",
        "trigger": eff_trig,
        "spinner_run": eff_run,
        "spinner_reversed": eff_rev,
        "conveyor": ctrl.get_status(),
    }


@app.post("/api/conveyor/trigger")
def control_conveyor_trigger(req: ConveyorTriggerRequest):
    """
    Sets only the conveyor trigger motor speed.
    """
    comm = _require_teleop_allowed()
    eff_trig = get_conveyor_controller().set_operator_trigger(req.speed)
    comm.send_conveyor_trigger(speed=eff_trig)
    return {"status": "enqueued", "subsystem": "conveyor", "trigger": eff_trig}


@app.post("/api/conveyor/spinner")
def control_conveyor_spinner(req: ConveyorSpinnerRequest):
    """
    Sets only the conveyor spinner state (run: bool, reversed: bool).
    """
    comm = _require_teleop_allowed()
    ctrl = get_conveyor_controller()
    eff_run, eff_rev = ctrl.set_operator_spinner(run=req.run, reversed=req.reversed)
    comm.send_conveyor_spinner(run=eff_run, reversed=eff_rev)
    return {
        "status": "enqueued",
        "subsystem": "conveyor",
        "spinner_run": eff_run,
        "spinner_reversed": eff_rev,
        "conveyor": ctrl.get_status(),
    }


@app.get("/api/conveyor/auto_reverse")
def get_conveyor_auto_reverse():
    """Returns automatic spinner anti-jam reverse state and timing configuration."""
    ctrl = get_conveyor_controller()
    return {
        "status": "ok",
        "state": ctrl.get_status(),
        "config": ctrl.get_config(),
    }


@app.post("/api/conveyor/auto_reverse")
def update_conveyor_auto_reverse(req: ConveyorAutoReverseUpdateRequest):
    """Updates automatic spinner anti-jam reverse parameters at runtime."""
    ctrl = get_conveyor_controller()
    try:
        if any(
            v is not None
            for v in (req.enabled, req.no_ball_delay_s, req.reverse_duration_s, req.cooldown_s)
        ):
            ctrl.set_config(
                enabled=req.enabled,
                no_ball_delay_s=req.no_ball_delay_s,
                reverse_duration_s=req.reverse_duration_s,
                cooldown_s=req.cooldown_s,
            )
        if req.reset:
            ctrl.reset_auto_reverse()
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

    return {
        "status": "ok",
        "state": ctrl.get_status(),
        "config": ctrl.get_config(),
    }


@app.post("/api/stop")
def stop_robot(req: StopRequest):
    """
    Appends an emergency or normal STOP command into the TX PriorityQueue (Priority 0).
    Purges any stale drive commands to halt motors instantly.
    """
    part = PartType(req.part)
    if part == PartType.ALL:
        get_auto_runner().abort("Emergency / Stop All command")
    if part in (PartType.ALL, PartType.CHASSIS):
        get_chassis_controller().stop()
    if part in (PartType.ALL, PartType.SHOOTER):
        get_shooter_controller().stop()
    if part in (PartType.ALL, PartType.CONVEYOR):
        get_conveyor_controller().stop()
    comm = get_uart_comm()
    comm.send_stop(part=part, clear_pending_drive=req.clear_pending_drive)
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
    last_angle_state = (False, False)

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
                chassis_ctrl = get_chassis_controller()
                runner = get_auto_runner()
                if vx != 0 or vy != 0 or omega != 0:
                    if runner.is_running:
                        runner.abort(
                            reason="Preempted by manual driver input",
                            uart=comm,
                            chassis_controller=chassis_ctrl,
                            shooter_controller=get_shooter_controller(),
                            conveyor_controller=get_conveyor_controller(),
                        )
                    chassis_ctrl.stop()
                    accepted = comm.send_drive(vx=vx, vy=vy, omega=omega)
                elif runner.is_running or chassis_ctrl.enabled:
                    # Ignore idle centered-stick keep-alives while auto routine or P2P is driving
                    accepted = True
                else:
                    accepted = comm.send_drive(vx=0, vy=0, omega=0)

            elif msg_type == "shooter":
                if comm.estop_active:
                    accepted = False
                    get_shooter_controller().stop()
                elif (
                    get_auto_runner().is_running
                    and int(data.get("flywheel", 0)) == 0
                    and not bool(data.get("angle_run", False))
                ):
                    accepted = True
                else:
                    angle_run = bool(data.get("angle_run", False))
                    angle_reversed = bool(data.get("angle_reversed", False))
                    get_shooter_controller().set_operator_command(
                        flywheel_cmd=int(data.get("flywheel", 0)),
                        mode=data.get("mode"),
                        target_rpm=data.get("target_rpm"),
                    )
                    angle_state = (angle_run, angle_reversed)
                    if angle_state != last_angle_state or angle_run:
                        comm.send_shooter_angle(run=angle_run, reversed=angle_reversed)
                        last_angle_state = angle_state
                    accepted = True

            elif msg_type == "shooter_disable_angle":
                last_angle_state = (False, False)
                accepted = comm.send_shooter_disable_angle()

            elif msg_type == "shooter_reset_angle_steps":
                accepted = comm.send_shooter_reset_angle_step()

            elif msg_type == "conveyor":
                if (
                    get_auto_runner().is_running
                    and int(data.get("trigger", 0)) == 0
                    and not bool(data.get("spinner_run", False))
                ):
                    accepted = True
                else:
                    trigger = int(data.get("trigger", 0))
                    spinner_run = bool(data.get("spinner_run", False))
                    spinner_reversed = bool(data.get("spinner_reversed", False))
                    eff_trig, eff_run, eff_rev = get_conveyor_controller().set_operator_command(
                        trigger=trigger,
                        spinner_run=spinner_run,
                        spinner_reversed=spinner_reversed,
                    )
                    accepted = comm.send_conveyor_command(
                        trigger=eff_trig,
                        spinner_run=eff_run,
                        spinner_reversed=eff_rev,
                    )

            elif msg_type == "stop":
                part_val = int(data.get("part", PartType.ALL.value))
                part = PartType(part_val)
                if part == PartType.ALL:
                    get_auto_runner().abort("WebSocket STOP ALL")
                if part in (PartType.ALL, PartType.CHASSIS):
                    get_chassis_controller().stop()
                if part in (PartType.ALL, PartType.SHOOTER):
                    get_shooter_controller().stop()
                if part in (PartType.ALL, PartType.CONVEYOR):
                    get_conveyor_controller().stop()
                comm.send_stop(part=part)

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
                "shooter_pid": get_shooter_controller().get_status(),
                "conveyor": get_conveyor_controller().get_status(),
                "chassis_pid": get_chassis_controller().get_status(),
                "auto_routine": get_auto_runner().get_status(),
            })

    except WebSocketDisconnect:
        if get_match_manager().mode == MatchMode.AUTO:
            logger.info("WebSocket client disconnected during AUTO; leaving autonomous routine active.")
        else:
            logger.info("WebSocket client disconnected. Issuing safety stop...")
            get_auto_runner().abort("WebSocket disconnected")
            get_chassis_controller().stop()
            get_shooter_controller().stop()
            get_conveyor_controller().stop()
            comm.send_stop()
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        if get_match_manager().mode != MatchMode.AUTO:
            get_auto_runner().abort(f"WebSocket error: {e}")
            get_chassis_controller().stop()
            get_shooter_controller().stop()
            get_conveyor_controller().stop()
            comm.send_stop()
