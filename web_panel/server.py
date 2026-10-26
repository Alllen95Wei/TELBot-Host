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


class StopRequest(BaseModel):
    part: int = Field(default=PartType.ALL.value, description="Part to stop (0=ALL, 1=CHASSIS, etc.)")
    clear_pending_drive: bool = Field(default=True, description="Purge pending drive commands from queue")


# ==============================================================================
# REST API Endpoints
# ==============================================================================
@app.get("/api/status")
def get_status():
    """Retrieve communication statistics, queue depth, and worker health."""
    stats = uart.get_stats()
    telemetry = uart.latest_telemetry
    return {
        "status": "ok",
        "stats": stats,
        "latest_telemetry": telemetry.payload.hex() if telemetry else None,
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
    return {"status": "enqueued", "vx": req.vx, "vy": req.vy, "omega": req.omega}


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
# WebSocket Endpoint for Low-Latency Joystick Streaming
# ==============================================================================
@app.websocket("/ws/control")
async def websocket_control(websocket: WebSocket):
    """
    WebSocket endpoint for real-time WebHID / Gamepad input stream.
    Receives JSON messages like: {"type": "drive", "vx": 300, "vy": 0, "omega": -100}
    or {"type": "stop"}
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

            elif msg_type == "stop":
                uart.send_stop()

            # Respond with immediate status snapshot
            stats = uart.get_stats()
            await websocket.send_json({
                "queue_depth": stats["queue_depth"],
                "tx_packets": stats["tx_packets"],
            })

    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected. Issuing safety stop...")
        uart.send_stop()
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        uart.send_stop()
