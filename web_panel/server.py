import asyncio
import logging
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

import sys
import os

# Add root directory to path so comms and config can be imported
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from comms import CmdType, Packet, PartType, UARTComm
from config import UART as UARTConfig

logger = logging.getLogger("TELBot.Server")

# ==============================================================================
# Shared Hardware Communicator Instance
# ==============================================================================
uart = UARTComm(
    port=UARTConfig.PORT,
    baudrate=UARTConfig.BAUD_RATE,
    timeout=UARTConfig.TIMEOUT,
)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manages the startup and shutdown lifecycle of the UART worker threads."""
    logger.info("Initializing UART dual-threaded communication workers...")
    try:
        uart.start()
    except Exception as e:
        logger.error(f"Failed to start UART worker threads: {e}")
    yield
    logger.info("Shutting down UART communication workers...")
    uart.stop()


# ==============================================================================
# FastAPI Application
# ==============================================================================
app = FastAPI(
    title="TELBot Host API",
    description="Real-time control and telemetry server for TELBot robot",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ==============================================================================
# Request Models
# ==============================================================================
class DriveRequest(BaseModel):
    vx: int = Field(default=0, ge=-1000, le=1000, description="Forward/backward velocity (-1000 to +1000)")
    vy: int = Field(default=0, ge=-1000, le=1000, description="Strafe left/right velocity (-1000 to +1000)")
    omega: int = Field(default=0, ge=-1000, le=1000, description="Rotational angular velocity (-1000 to +1000)")


class ShooterRequest(BaseModel):
    flywheel: int = Field(default=0, ge=-1000, le=1000, description="Flywheel speed (-1000 to +1000)")
    angle: int = Field(default=0, ge=-1000, le=1000, description="Pitch angle adjustment speed (-1000 to +1000)")
    spinner: int = Field(default=0, ge=-1000, le=1000, description="Feeder spinner speed (-1000 to +1000)")


class ShooterFlywheelRequest(BaseModel):
    speed: int = Field(default=0, ge=-1000, le=1000, description="Flywheel speed only (-1000 to +1000)")


class ConveyorRequest(BaseModel):
    speed: int = Field(default=0, ge=-1000, le=1000, description="Conveyor / feeder belt speed (-1000 to +1000)")


class StopRequest(BaseModel):
    part: int = Field(
        default=PartType.ALL.value,
        description="Subsystem to stop (0=ALL, 1=CHASSIS, 2=CONVEYOR, 3=SHOOTER, 4=SENSORS)"
    )
    clear_pending_drive: bool = Field(default=True, description="Purge pending drive commands from queue")


# ==============================================================================
# REST API Endpoints
# ==============================================================================
@app.get("/api/status")
def get_status():
    """Retrieve communication statistics, queue depth, and latest telemetry."""
    stats = uart.get_stats()
    telemetry = uart.latest_telemetry
    parsed_telem = None

    if telemetry is not None:
        try:
            parsed_telem = telemetry.parse_telemetry()
        except Exception:
            parsed_telem = {"raw_hex": telemetry.payload.hex()}

    return {
        "status": "ok",
        "stats": stats,
        "latest_telemetry": parsed_telem,
    }


@app.post("/api/drive")
def drive_robot(req: DriveRequest):
    """
    Appends a new chassis drive command into the TX PriorityQueue (Priority 2).
    Thread-safe and returns immediately.
    """
    if not uart.is_running:
        raise HTTPException(status_code=503, detail="UART communicator is not running")

    uart.send_drive(vx=req.vx, vy=req.vy, omega=req.omega)
    return {"status": "enqueued", "subsystem": "chassis", "vx": req.vx, "vy": req.vy, "omega": req.omega}


@app.post("/api/shooter")
def control_shooter(req: ShooterRequest):
    """
    Controls all shooter components: flywheels, pitch angle adjustment, and feeder spinner.
    """
    if not uart.is_running:
        raise HTTPException(status_code=503, detail="UART communicator is not running")

    uart.send_shooter_command(flywheel=req.flywheel, angle=req.angle, spinner=req.spinner)
    return {
        "status": "enqueued",
        "subsystem": "shooter",
        "flywheel": req.flywheel,
        "angle": req.angle,
        "spinner": req.spinner,
    }


@app.post("/api/shooter/flywheel")
def control_shooter_flywheel(req: ShooterFlywheelRequest):
    """
    Sets only the shooter flywheel speed.
    """
    if not uart.is_running:
        raise HTTPException(status_code=503, detail="UART communicator is not running")

    uart.send_shooter_flywheel(speed=req.speed)
    return {"status": "enqueued", "subsystem": "shooter", "flywheel": req.speed}


@app.post("/api/conveyor")
def control_conveyor(req: ConveyorRequest):
    """
    Controls the conveyor / ball feeder motor.
    """
    if not uart.is_running:
        raise HTTPException(status_code=503, detail="UART communicator is not running")

    uart.send_conveyor(speed=req.speed)
    return {"status": "enqueued", "subsystem": "conveyor", "speed": req.speed}


@app.post("/api/stop")
def stop_robot(req: StopRequest):
    """
    Appends an emergency or normal STOP command into the TX PriorityQueue (Priority 0).
    Purges any stale drive commands to halt motors instantly.
    """
    uart.send_stop(part=PartType(req.part), clear_pending_drive=req.clear_pending_drive)
    return {"status": "stopped", "part": req.part}


@app.post("/api/ping")
def ping_slave():
    """Sends a keep-alive ping to the slave Pico."""
    uart.send_ping()
    return {"status": "ping_sent"}


# ==============================================================================
# WebSocket Endpoint for Low-Latency Joystick & Control Streaming
# ==============================================================================
@app.websocket("/ws/control")
async def websocket_control(websocket: WebSocket):
    """
    WebSocket endpoint for real-time WebHID / Gamepad input stream.
    Receives JSON messages such as:
      {"type": "drive", "vx": 300, "vy": 0, "omega": -100}
      {"type": "shooter", "flywheel": 800, "angle": 0, "spinner": 1000}
      {"type": "conveyor", "speed": 500}
      {"type": "stop", "part": 0}
    """
    await websocket.accept()
    logger.info("WebHID/Gamepad WebSocket client connected.")
    try:
        while True:
            data = await websocket.receive_json()
            msg_type = data.get("type")

            if msg_type == "drive":
                vx = int(data.get("vx", 0))
                vy = int(data.get("vy", 0))
                omega = int(data.get("omega", 0))
                uart.send_drive(vx=vx, vy=vy, omega=omega)

            elif msg_type == "shooter":
                flywheel = int(data.get("flywheel", 0))
                angle = int(data.get("angle", 0))
                spinner = int(data.get("spinner", 0))
                uart.send_shooter_command(flywheel=flywheel, angle=angle, spinner=spinner)

            elif msg_type == "conveyor":
                speed = int(data.get("speed", 0))
                uart.send_conveyor(speed=speed)

            elif msg_type == "stop":
                part_val = int(data.get("part", PartType.ALL.value))
                uart.send_stop(part=PartType(part_val))

            # Read latest stats & telemetry snapshot to send back to client
            stats = uart.get_stats()
            telem = uart.latest_telemetry
            parsed_telem = None
            if telem:
                try:
                    parsed_telem = telem.parse_telemetry()
                except Exception:
                    pass

            await websocket.send_json({
                "queue_depth": stats["queue_depth"],
                "tx_packets": stats["tx_packets"],
                "rx_packets": stats["rx_packets"],
                "telemetry": parsed_telem,
            })

    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected. Issuing safety stop...")
        uart.send_stop()
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        uart.send_stop()
