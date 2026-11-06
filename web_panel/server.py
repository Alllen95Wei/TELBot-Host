import asyncio
import json
import logging
import os
import sys
from contextlib import asynccontextmanager
from pathlib import Path

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

from comms import PartType, UARTComm
from config import Camera as CameraConfig
from config import Server as ServerConfig
from config import UART as UARTConfig

logger = logging.getLogger("TELBot.Server")

# ==============================================================================
# Shared Hardware Communicator Instance (Injectable from main.py)
# ==============================================================================
uart = UARTComm(
    port=UARTConfig.PORT,
    baudrate=UARTConfig.BAUD_RATE,
    timeout=UARTConfig.TIMEOUT,
)


def set_uart_comm(comm: UARTComm) -> None:
    """Injects an externally managed UARTComm instance (used by main.py)."""
    global uart
    uart = comm
    app.state.uart = comm


def get_uart_comm() -> UARTComm:
    """Returns the active UARTComm instance."""
    return getattr(app.state, "uart", uart)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Manages the startup and shutdown lifecycle of the UART worker threads
    when running standalone (skips if already started by main.py).
    """
    comm = get_uart_comm()
    app.state.uart = comm
    managed_internally = False

    if not comm.is_running:
        logger.info("Initializing UART dual-threaded communication workers (standalone mode)...")
        try:
            comm.start()
            managed_internally = True
        except Exception as e:
            logger.error(f"Failed to start UART worker threads: {e}")

    yield

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


class ShooterFlywheelRequest(BaseModel):
    speed: int = Field(default=0, ge=-1000, le=1000, description="Flywheel speed only (-1000 to +1000)")


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


# ==============================================================================
# REST API Endpoints
# ==============================================================================
@app.get("/api/status")
def get_status():
    """Retrieve communication statistics, E-Stop state, queue depth, and latest telemetry."""
    comm = get_uart_comm()
    stats = comm.get_stats()
    telemetry = comm.latest_telemetry
    parsed_telem = None

    if telemetry is not None:
        try:
            parsed_telem = telemetry.parse_telemetry()
        except Exception:
            parsed_telem = {"raw_hex": telemetry.payload.hex()}

    return {
        "status": "estop" if comm.estop_active else "ok",
        "estop_active": comm.estop_active,
        "stats": stats,
        "latest_telemetry": parsed_telem,
    }


@app.post("/api/drive")
def drive_robot(req: DriveRequest):
    """
    Appends a new chassis drive command into the TX PriorityQueue (Priority 2).
    Thread-safe and returns immediately.
    """
    comm = _require_ready_uart()
    comm.send_drive(vx=req.vx, vy=req.vy, omega=req.omega)
    return {"status": "enqueued", "subsystem": "chassis", "vx": req.vx, "vy": req.vy, "omega": req.omega}


@app.post("/api/shooter")
def control_shooter(req: ShooterRequest):
    """
    Controls shooter flywheels and pitch angle adjustment.
    """
    comm = _require_ready_uart()
    comm.send_shooter_command(flywheel=req.flywheel, angle=req.angle)
    return {
        "status": "enqueued",
        "subsystem": "shooter",
        "flywheel": req.flywheel,
        "angle": req.angle,
    }


@app.post("/api/shooter/flywheel")
def control_shooter_flywheel(req: ShooterFlywheelRequest):
    """
    Sets only the shooter flywheel speed.
    """
    comm = _require_ready_uart()
    comm.send_shooter_flywheel(speed=req.speed)
    return {"status": "enqueued", "subsystem": "shooter", "flywheel": req.speed}


@app.post("/api/conveyor")
def control_conveyor(req: ConveyorRequest):
    """
    Controls all conveyor components: trigger motor speed and spinner state (run, reversed).
    """
    comm = _require_ready_uart()
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
    comm = _require_ready_uart()
    comm.send_conveyor_trigger(speed=req.speed)
    return {"status": "enqueued", "subsystem": "conveyor", "trigger": req.speed}


@app.post("/api/conveyor/spinner")
def control_conveyor_spinner(req: ConveyorSpinnerRequest):
    """
    Sets only the conveyor spinner state (run: bool, reversed: bool).
    """
    comm = _require_ready_uart()
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
        except asyncio.CancelledError:
            pass
        finally:
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
      {"type": "shooter", "flywheel": 800, "angle": 0}
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

            if msg_type == "drive":
                vx = int(data.get("vx", 0))
                vy = int(data.get("vy", 0))
                omega = int(data.get("omega", 0))
                accepted = comm.send_drive(vx=vx, vy=vy, omega=omega)

            elif msg_type == "shooter":
                flywheel = int(data.get("flywheel", 0))
                angle = int(data.get("angle", 0))
                accepted = comm.send_shooter_command(flywheel=flywheel, angle=angle)

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

            # Read latest stats & telemetry snapshot to send back to client
            stats = comm.get_stats()
            telem = comm.latest_telemetry
            parsed_telem = None
            if telem:
                try:
                    parsed_telem = telem.parse_telemetry()
                except Exception:
                    pass

            await websocket.send_json({
                "estop_active": comm.estop_active,
                "command_accepted": accepted,
                "queue_depth": stats["queue_depth"],
                "tx_packets": stats["tx_packets"],
                "rx_packets": stats["rx_packets"],
                "telemetry": parsed_telem,
            })

    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected. Issuing safety stop...")
        comm.send_stop()
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        comm.send_stop()
