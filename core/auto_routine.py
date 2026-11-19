"""
core/auto_routine.py - Autonomous Routine Storage & 50 Hz Sequence Runner for TELBot

Implements:
  - AutoStepType: Supported autonomous step primitives.
  - AutoStep & AutoRoutineDefinition: Validated JSON routine models.
  - RoutineStore: Thread-safe JSON persistence in routines/*.json + _active.json.
  - AutoRoutineRunnerState & AutoRoutineRunner: Non-blocking 50 Hz sequence FSM
    coordinating ChassisPoseController, ShooterSpeedController, ConveyorController,
    VisionManager, and UARTComm.
  - step_auto_routine: 50 Hz host loop step with E-Stop, match phase, and 60s cutoff gating.
"""

from __future__ import annotations

import json
import logging
import math
import re
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from config import AutoRoutine as AutoRoutineConfig
from config import ChassisPID as ChassisPIDConfig
from config import ESP32 as ESP32Config
from config import Field as FieldConfig
from config import ShooterPID as ShooterPIDConfig
from config import SpecialGrid as SpecialGridConfig
from core.comms import UARTComm
from core.control import (
    ChassisPoseController,
    ConveyorController,
    ShooterSpeedController,
)
from core.esp32 import ESP32SensorReceiver
from core.match import MatchManager, MatchMode
from core.pose import PoseEstimator
from core.vision import VisionManager

logger = logging.getLogger("TELBot.AutoRoutine")

_ROUTINE_ID_RE = re.compile(r"^[a-zA-Z0-9_-]{1,64}$")
_VALID_GOAL_COLUMNS = ("left", "center", "right")
_VALID_AIM_MODES = ("explicit", "active_1x3", "column")
_VALID_SHOOTER_MODES = ("auto", "manual")
_VALID_PITCH_ACTIONS = ("up", "down", "disable")


class AutoStepType(str, Enum):
    WAIT_VISION = "wait_vision"
    DRIVE_TO = "drive_to"
    AIM_AT = "aim_at"
    SHOOTER_SET = "shooter_set"
    PITCH_STEP = "pitch_step"
    FEED_BALLS = "feed_balls"
    WAIT = "wait"
    STOP_ALL = "stop_all"


def validate_routine_id(routine_id: str) -> str:
    """Validates a routine identifier slug and rejects path traversal or reserved names."""
    rid = str(routine_id).strip()
    if rid == "_active" or not _ROUTINE_ID_RE.match(rid):
        raise ValueError(
            f"Invalid routine ID {routine_id!r}. Use 1-64 characters of [a-zA-Z0-9_-] (not '_active')."
        )
    return rid


def _clamp_field_xy(x_cm: float, y_cm: float) -> tuple[float, float]:
    margin = float(ChassisPIDConfig.WALL_MARGIN_CM)
    min_x = margin
    max_x = max(min_x, float(FieldConfig.WIDTH_CM) - margin)
    min_y = margin
    max_y = max(min_y, float(FieldConfig.LENGTH_CM) - margin)
    return (
        round(max(min_x, min(max_x, float(x_cm))), 2),
        round(max(min_y, min(max_y, float(y_cm))), 2),
    )


def _finite_float(val: object, name: str) -> float:
    if isinstance(val, bool) or not isinstance(val, (int, float)):
        raise ValueError(f"{name} must be a number, got {val!r}")
    f = float(val)
    if not math.isfinite(f):
        raise ValueError(f"{name} must be finite, got {val!r}")
    return f


@dataclass
class AutoStep:
    """Single validated autonomous step inside an AutoRoutineDefinition."""

    id: str
    type: str
    label: str = ""
    x_cm: float = 200.0
    y_cm: float = 260.0
    aim_mode: str = "explicit"
    heading_deg: float = 0.0
    goal_column: str = "center"
    stop_on_arrival: bool = True
    pre_spin_shooter: bool = False
    shooter_mode: str = "auto"
    shooter_rpm: float = 3900.0
    enabled: bool = True
    mode: str = "auto"
    target_rpm: float = 3900.0
    wait_at_speed: bool = True
    action: str = "up"
    trigger_speed: int = 700
    spinner_run: bool = True
    require_at_speed: bool = True
    at_speed_timeout_s: float = 1.5
    ball_count: int = 2
    duration_s: float = 2.0
    timeout_s: float = 5.0
    fallback_target: str = "center"

    @classmethod
    def from_dict(cls, data: dict, index: int = 1) -> "AutoStep":
        if not isinstance(data, dict):
            raise ValueError(f"Step #{index} must be an object")

        raw_type = str(data.get("type", "")).strip().lower()
        try:
            step_type = AutoStepType(raw_type).value
        except ValueError as exc:
            raise ValueError(f"Step #{index} has unsupported type {raw_type!r}") from exc

        raw_id = str(data.get("id") or f"step_{index}").strip()
        step_id = raw_id[:64] if raw_id else f"step_{index}"
        label = str(data.get("label") or "").strip()[:120]

        x_cm, y_cm = _clamp_field_xy(
            _finite_float(data.get("x_cm", 200.0), "x_cm"),
            _finite_float(data.get("y_cm", 260.0), "y_cm"),
        )
        heading_deg = round(
            _finite_float(data.get("heading_deg", 0.0), "heading_deg") % 360.0, 2
        )

        aim_mode = str(data.get("aim_mode", "explicit")).strip().lower()
        if aim_mode not in _VALID_AIM_MODES:
            raise ValueError(f"Invalid aim_mode {aim_mode!r}")

        goal_column = str(data.get("goal_column", "center")).strip().lower()
        if goal_column not in _VALID_GOAL_COLUMNS:
            raise ValueError(f"Invalid goal_column {goal_column!r}")

        fallback_target = str(data.get("fallback_target", "center")).strip().lower()
        if fallback_target not in _VALID_GOAL_COLUMNS:
            raise ValueError(f"Invalid fallback_target {fallback_target!r}")

        shooter_mode = str(data.get("shooter_mode", "auto")).strip().lower()
        if shooter_mode not in _VALID_SHOOTER_MODES:
            raise ValueError(f"Invalid shooter_mode {shooter_mode!r}")

        mode = str(data.get("mode", "auto")).strip().lower()
        if mode not in _VALID_SHOOTER_MODES:
            raise ValueError(f"Invalid mode {mode!r}")

        action = str(data.get("action", "up")).strip().lower()
        if action not in _VALID_PITCH_ACTIONS:
            raise ValueError(f"Invalid pitch action {action!r}")

        max_rpm = float(ShooterPIDConfig.MAX_RPM)
        shooter_rpm = round(
            max(0.0, min(max_rpm, _finite_float(data.get("shooter_rpm", 3900.0), "shooter_rpm"))),
            1,
        )
        target_rpm = round(
            max(0.0, min(max_rpm, _finite_float(data.get("target_rpm", 3900.0), "target_rpm"))),
            1,
        )

        trigger_speed = max(
            -1000,
            min(1000, int(round(_finite_float(data.get("trigger_speed", 700), "trigger_speed")))),
        )
        ball_count = max(
            0, min(10, int(round(_finite_float(data.get("ball_count", 2), "ball_count"))))
        )
        duration_s = round(
            max(0.05, min(30.0, _finite_float(data.get("duration_s", 2.0), "duration_s"))),
            2,
        )
        timeout_s = round(
            max(0.05, min(60.0, _finite_float(data.get("timeout_s", 5.0), "timeout_s"))),
            2,
        )
        at_speed_timeout_s = round(
            max(
                0.0,
                min(10.0, _finite_float(data.get("at_speed_timeout_s", 1.5), "at_speed_timeout_s")),
            ),
            2,
        )

        return cls(
            id=step_id,
            type=step_type,
            label=label,
            x_cm=x_cm,
            y_cm=y_cm,
            aim_mode=aim_mode,
            heading_deg=heading_deg,
            goal_column=goal_column,
            stop_on_arrival=bool(data.get("stop_on_arrival", True)),
            pre_spin_shooter=bool(data.get("pre_spin_shooter", False)),
            shooter_mode=shooter_mode,
            shooter_rpm=shooter_rpm,
            enabled=bool(data.get("enabled", True)),
            mode=mode,
            target_rpm=target_rpm,
            wait_at_speed=bool(data.get("wait_at_speed", True)),
            action=action,
            trigger_speed=trigger_speed,
            spinner_run=bool(data.get("spinner_run", True)),
            require_at_speed=bool(data.get("require_at_speed", True)),
            at_speed_timeout_s=at_speed_timeout_s,
            ball_count=ball_count,
            duration_s=duration_s,
            timeout_s=timeout_s,
            fallback_target=fallback_target,
        )

    def estimated_duration_s(self) -> float:
        if self.type in (AutoStepType.WAIT_VISION.value, AutoStepType.DRIVE_TO.value, AutoStepType.AIM_AT.value):
            return float(self.timeout_s)
        if self.type == AutoStepType.SHOOTER_SET.value:
            return float(self.timeout_s) if (self.enabled and self.wait_at_speed) else 0.0
        if self.type == AutoStepType.PITCH_STEP.value:
            return 0.0 if self.action == "disable" else float(self.duration_s)
        if self.type == AutoStepType.FEED_BALLS.value:
            return float(self.duration_s) + (float(self.at_speed_timeout_s) if self.require_at_speed else 0.0)
        if self.type == AutoStepType.WAIT.value:
            return float(self.duration_s)
        return 0.0

    def to_dict(self) -> dict:
        base = {
            "id": self.id,
            "type": self.type,
            "label": self.label,
        }
        if self.type == AutoStepType.WAIT_VISION.value:
            base.update({
                "timeout_s": self.timeout_s,
                "fallback_target": self.fallback_target,
            })
        elif self.type == AutoStepType.DRIVE_TO.value:
            base.update({
                "x_cm": self.x_cm,
                "y_cm": self.y_cm,
                "aim_mode": self.aim_mode,
                "heading_deg": self.heading_deg,
                "goal_column": self.goal_column,
                "stop_on_arrival": self.stop_on_arrival,
                "pre_spin_shooter": self.pre_spin_shooter,
                "shooter_mode": self.shooter_mode,
                "shooter_rpm": self.shooter_rpm,
                "timeout_s": self.timeout_s,
            })
        elif self.type == AutoStepType.AIM_AT.value:
            base.update({
                "aim_mode": self.aim_mode,
                "heading_deg": self.heading_deg,
                "goal_column": self.goal_column,
                "stop_on_arrival": self.stop_on_arrival,
                "timeout_s": self.timeout_s,
            })
        elif self.type == AutoStepType.SHOOTER_SET.value:
            base.update({
                "enabled": self.enabled,
                "mode": self.mode,
                "target_rpm": self.target_rpm,
                "wait_at_speed": self.wait_at_speed,
                "timeout_s": self.timeout_s,
            })
        elif self.type == AutoStepType.PITCH_STEP.value:
            base.update({
                "action": self.action,
                "duration_s": self.duration_s,
            })
        elif self.type == AutoStepType.FEED_BALLS.value:
            base.update({
                "trigger_speed": self.trigger_speed,
                "spinner_run": self.spinner_run,
                "require_at_speed": self.require_at_speed,
                "at_speed_timeout_s": self.at_speed_timeout_s,
                "ball_count": self.ball_count,
                "duration_s": self.duration_s,
            })
        elif self.type == AutoStepType.WAIT.value:
            base.update({
                "duration_s": self.duration_s,
            })
        return base


@dataclass
class AutoRoutineDefinition:
    """Full autonomous routine specification persisted as JSON."""

    id: str
    name: str
    description: str = ""
    start_pose: dict = field(
        default_factory=lambda: {"x_cm": 200.0, "y_cm": 80.0, "heading_deg": 0.0}
    )
    updated_at: float = field(default_factory=time.time)
    steps: list[AutoStep] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict) -> "AutoRoutineDefinition":
        if not isinstance(data, dict):
            raise ValueError("Routine payload must be a JSON object")

        routine_id = validate_routine_id(str(data.get("id", "")))
        name = str(data.get("name") or routine_id).strip()[:100]
        if not name:
            name = routine_id
        description = str(data.get("description") or "").strip()[:400]

        raw_start = data.get("start_pose") or {}
        if not isinstance(raw_start, dict):
            raise ValueError("start_pose must be an object")
        sx, sy = _clamp_field_xy(
            _finite_float(raw_start.get("x_cm", 200.0), "start_pose.x_cm"),
            _finite_float(raw_start.get("y_cm", 80.0), "start_pose.y_cm"),
        )
        sh = round(
            _finite_float(raw_start.get("heading_deg", 0.0), "start_pose.heading_deg") % 360.0,
            2,
        )

        raw_steps = data.get("steps", [])
        if not isinstance(raw_steps, list):
            raise ValueError("steps must be a list")
        if len(raw_steps) > int(AutoRoutineConfig.MAX_STEPS):
            raise ValueError(
                f"Routine cannot exceed {AutoRoutineConfig.MAX_STEPS} steps (got {len(raw_steps)})"
            )
        steps = [AutoStep.from_dict(s, index=i + 1) for i, s in enumerate(raw_steps)]

        updated_at = float(data.get("updated_at") or time.time())
        return cls(
            id=routine_id,
            name=name,
            description=description,
            start_pose={"x_cm": sx, "y_cm": sy, "heading_deg": sh},
            updated_at=updated_at,
            steps=steps,
        )

    def estimated_duration_s(self) -> float:
        return round(sum(s.estimated_duration_s() for s in self.steps), 2)

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "start_pose": dict(self.start_pose),
            "updated_at": self.updated_at,
            "steps": [s.to_dict() for s in self.steps],
        }

    def summary_dict(self, is_active: bool = False) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "step_count": len(self.steps),
            "estimated_duration_s": self.estimated_duration_s(),
            "updated_at": self.updated_at,
            "is_active": bool(is_active),
        }


def _default_routines_payloads() -> list[dict]:
    now_ts = time.time()
    return [
        {
            "id": AutoRoutineConfig.DEFAULT_ROUTINE_ID,
            "name": "1x3 Active Goal (2 Balls) + Center Follow-up",
            "description": (
                "Waits up to 2.0s for laptop YOLO 1x3 latch, drives to midfield while pre-spinning "
                "flywheels in Auto-Distance mode, feeds 2 balls, and safe-stops."
            ),
            "start_pose": {"x_cm": 200.0, "y_cm": 80.0, "heading_deg": 0.0},
            "updated_at": now_ts,
            "steps": [
                {
                    "id": "step_1",
                    "type": "wait_vision",
                    "label": "Wait for YOLO 1x3 Latch",
                    "timeout_s": 2.0,
                    "fallback_target": "center",
                },
                {
                    "id": "step_2",
                    "type": "drive_to",
                    "label": "Drive to Midfield Shooting Spot",
                    "x_cm": 200.0,
                    "y_cm": 260.0,
                    "aim_mode": "active_1x3",
                    "heading_deg": 0.0,
                    "goal_column": "center",
                    "stop_on_arrival": True,
                    "pre_spin_shooter": True,
                    "shooter_mode": "auto",
                    "shooter_rpm": 3900.0,
                    "timeout_s": 5.0,
                },
                {
                    "id": "step_3",
                    "type": "feed_balls",
                    "label": "Shoot 2 Balls into Active 1x3",
                    "trigger_speed": 700,
                    "spinner_run": True,
                    "require_at_speed": True,
                    "at_speed_timeout_s": 1.5,
                    "ball_count": 2,
                    "duration_s": 2.5,
                },
                {
                    "id": "step_4",
                    "type": "stop_all",
                    "label": "Safe Stop",
                },
            ],
        },
        {
            "id": "left_flank_1x3_auto",
            "name": "Left Flank -> Active 1x3 (2 Balls)",
            "description": (
                "Starts on left side, waits for 1x3 vision latch, advances to (140, 290) aimed at active "
                "1x3 goal, shoots 2 balls, then stops."
            ),
            "start_pose": {"x_cm": 110.0, "y_cm": 80.0, "heading_deg": 0.0},
            "updated_at": now_ts,
            "steps": [
                {
                    "id": "step_1",
                    "type": "wait_vision",
                    "label": "Wait for YOLO 1x3 Latch",
                    "timeout_s": 2.0,
                    "fallback_target": "left",
                },
                {
                    "id": "step_2",
                    "type": "drive_to",
                    "label": "Advance Left Shooting Lane",
                    "x_cm": 140.0,
                    "y_cm": 290.0,
                    "aim_mode": "active_1x3",
                    "heading_deg": 0.0,
                    "goal_column": "left",
                    "stop_on_arrival": True,
                    "pre_spin_shooter": True,
                    "shooter_mode": "auto",
                    "shooter_rpm": 3600.0,
                    "timeout_s": 5.0,
                },
                {
                    "id": "step_3",
                    "type": "feed_balls",
                    "label": "Feed 2 Balls",
                    "trigger_speed": 700,
                    "spinner_run": True,
                    "require_at_speed": True,
                    "at_speed_timeout_s": 1.5,
                    "ball_count": 2,
                    "duration_s": 2.5,
                },
                {
                    "id": "step_4",
                    "type": "stop_all",
                    "label": "Safe Stop",
                },
            ],
        },
    ]


class RoutineStore:
    """Thread-safe JSON storage for autonomous routines in routines/*.json + _active.json."""

    def __init__(self, routines_dir: Optional[Path] = None) -> None:
        self._lock = threading.RLock()
        self.routines_dir = (
            Path(routines_dir)
            if routines_dir is not None
            else (ROOT_DIR / AutoRoutineConfig.ROUTINES_DIR)
        )
        self._active_file = self.routines_dir / AutoRoutineConfig.ACTIVE_STATE_FILE
        self._active_routine_id: str = AutoRoutineConfig.DEFAULT_ROUTINE_ID
        self._ensure_initialized_locked()

    def _routine_path(self, routine_id: str) -> Path:
        rid = validate_routine_id(routine_id)
        return self.routines_dir / f"{rid}.json"

    def _list_routine_files_locked(self) -> list[Path]:
        if not self.routines_dir.is_dir():
            return []
        files = []
        for p in sorted(self.routines_dir.glob("*.json")):
            if p.name == AutoRoutineConfig.ACTIVE_STATE_FILE:
                continue
            files.append(p)
        return files

    def _write_json_locked(self, path: Path, payload: dict) -> None:
        tmp_path = path.with_suffix(".json.tmp")
        tmp_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        tmp_path.replace(path)

    def _save_active_id_locked(self, routine_id: str) -> None:
        self._active_routine_id = routine_id
        self._write_json_locked(self._active_file, {"active_routine_id": routine_id})

    def _ensure_initialized_locked(self) -> None:
        self.routines_dir.mkdir(parents=True, exist_ok=True)
        if not self._list_routine_files_locked():
            for raw in _default_routines_payloads():
                routine = AutoRoutineDefinition.from_dict(raw)
                self._write_json_locked(self._routine_path(routine.id), routine.to_dict())
            self._save_active_id_locked(AutoRoutineConfig.DEFAULT_ROUTINE_ID)
            return

        # Load persisted active pointer if valid
        loaded_active: Optional[str] = None
        if self._active_file.is_file():
            try:
                data = json.loads(self._active_file.read_text(encoding="utf-8"))
                cand = validate_routine_id(str(data.get("active_routine_id", "")))
                if self._routine_path(cand).is_file():
                    loaded_active = cand
            except Exception:
                loaded_active = None

        if loaded_active is None:
            files = self._list_routine_files_locked()
            default_path = self._routine_path(AutoRoutineConfig.DEFAULT_ROUTINE_ID)
            if default_path.is_file():
                loaded_active = AutoRoutineConfig.DEFAULT_ROUTINE_ID
            elif files:
                loaded_active = files[0].stem
        if loaded_active is not None:
            self._save_active_id_locked(loaded_active)

    @property
    def active_routine_id(self) -> str:
        with self._lock:
            return self._active_routine_id

    def list_routines(self) -> list[dict]:
        with self._lock:
            summaries: list[dict] = []
            for p in self._list_routine_files_locked():
                try:
                    raw = json.loads(p.read_text(encoding="utf-8"))
                    routine = AutoRoutineDefinition.from_dict(raw)
                    summaries.append(
                        routine.summary_dict(is_active=(routine.id == self._active_routine_id))
                    )
                except Exception as e:
                    logger.warning(f"Skipping unreadable routine file {p.name}: {e}")
            return summaries

    def get_routine(self, routine_id: str) -> AutoRoutineDefinition:
        with self._lock:
            path = self._routine_path(routine_id)
            if not path.is_file():
                raise KeyError(f"Routine {routine_id!r} not found")
            raw = json.loads(path.read_text(encoding="utf-8"))
            return AutoRoutineDefinition.from_dict(raw)

    def save_routine(self, data: dict) -> AutoRoutineDefinition:
        with self._lock:
            payload = dict(data)
            payload["updated_at"] = time.time()
            routine = AutoRoutineDefinition.from_dict(payload)
            self._write_json_locked(self._routine_path(routine.id), routine.to_dict())
            if not self._routine_path(self._active_routine_id).is_file():
                self._save_active_id_locked(routine.id)
            return routine

    def delete_routine(self, routine_id: str) -> None:
        with self._lock:
            rid = validate_routine_id(routine_id)
            path = self._routine_path(rid)
            if not path.is_file():
                raise KeyError(f"Routine {rid!r} not found")
            files = self._list_routine_files_locked()
            if len(files) <= 1:
                raise ValueError("Cannot delete the last remaining autonomous routine")
            path.unlink()
            if self._active_routine_id == rid:
                remaining = self._list_routine_files_locked()
                if remaining:
                    self._save_active_id_locked(remaining[0].stem)

    def set_active_routine(self, routine_id: str) -> AutoRoutineDefinition:
        with self._lock:
            routine = self.get_routine(routine_id)
            self._save_active_id_locked(routine.id)
            return routine

    def get_active_routine(self) -> AutoRoutineDefinition:
        with self._lock:
            try:
                return self.get_routine(self._active_routine_id)
            except KeyError:
                files = self._list_routine_files_locked()
                if not files:
                    self._ensure_initialized_locked()
                    files = self._list_routine_files_locked()
                first_id = files[0].stem
                self._save_active_id_locked(first_id)
                return self.get_routine(first_id)


@dataclass
class AutoRoutineRunnerState:
    """Thread-safe telemetry snapshot of the 50 Hz autonomous routine sequence runner."""

    running: bool = False
    status: str = "idle"
    status_message: str = "Autonomous runner idle"
    routine_id: Optional[str] = None
    routine_name: Optional[str] = None
    current_step_index: int = -1
    total_steps: int = 0
    current_step_id: Optional[str] = None
    current_step_type: Optional[str] = None
    current_step_label: Optional[str] = None
    step_elapsed_s: float = 0.0
    step_timeout_s: float = 0.0
    routine_elapsed_s: float = 0.0
    balls_fed_in_step: int = 0
    resolved_goal_column: Optional[str] = None
    steps_summary: list[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


class AutoRoutineRunner:
    """
    Non-blocking 50 Hz sequence runner executing AutoRoutineDefinition steps by
    driving ChassisPoseController, ShooterSpeedController, ConveyorController,
    VisionManager, and UARTComm.
    """

    def __init__(self, store: RoutineStore) -> None:
        self._lock = threading.RLock()
        self._store = store
        self._routine: Optional[AutoRoutineDefinition] = None
        self._running: bool = False
        self._status: str = "idle"
        self._status_message: str = "Autonomous runner idle"
        self._step_index: int = -1
        self._step_entered: bool = False
        self._routine_start_time: float = 0.0
        self._step_start_time: float = 0.0
        self._aim_anchor_xy: Optional[tuple[float, float]] = None
        self._feed_phase: str = "wait_speed"
        self._feed_start_time: float = 0.0
        self._prev_ball_detected: Optional[bool] = None
        self._balls_fed_in_step: int = 0
        self._resolved_goal_column: Optional[str] = None
        self._state = AutoRoutineRunnerState()

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    @property
    def state(self) -> AutoRoutineRunnerState:
        with self._lock:
            return AutoRoutineRunnerState(**self._state.to_dict())

    def get_status(self) -> dict:
        with self._lock:
            return self._state.to_dict()

    def _build_steps_summary_locked(self, routine: AutoRoutineDefinition) -> list[dict]:
        return [
            {
                "id": s.id,
                "type": s.type,
                "label": s.label or s.type,
            }
            for s in routine.steps
        ]

    def _stop_actuators(
        self,
        uart: Optional[UARTComm] = None,
        chassis_controller: Optional[ChassisPoseController] = None,
        shooter_controller: Optional[ShooterSpeedController] = None,
        conveyor_controller: Optional[ConveyorController] = None,
    ) -> None:
        if chassis_controller is not None:
            chassis_controller.stop()
        if shooter_controller is not None:
            shooter_controller.stop()
        if conveyor_controller is not None:
            conveyor_controller.stop()
        if uart is not None and not uart.estop_active:
            uart.send_drive(0, 0, 0)
            uart.send_shooter_flywheel(0, 0)
            uart.send_shooter_angle(run=False, reversed=False)
            uart.send_conveyor_command(0, False, False)

    def _refresh_state_locked(self, t: float) -> None:
        routine = self._routine
        if routine is None:
            self._state = AutoRoutineRunnerState(
                running=self._running,
                status=self._status,
                status_message=self._status_message,
            )
            return

        total = len(routine.steps)
        step: Optional[AutoStep] = (
            routine.steps[self._step_index]
            if (0 <= self._step_index < total)
            else None
        )
        step_elapsed = (
            max(0.0, t - self._step_start_time) if (self._running and step is not None) else 0.0
        )
        routine_elapsed = (
            max(0.0, t - self._routine_start_time)
            if (self._routine_start_time > 0.0 and (self._running or self._status in ("completed", "aborted")))
            else 0.0
        )
        step_timeout = step.estimated_duration_s() if step is not None else 0.0

        self._state = AutoRoutineRunnerState(
            running=self._running,
            status=self._status,
            status_message=self._status_message,
            routine_id=routine.id,
            routine_name=routine.name,
            current_step_index=self._step_index,
            total_steps=total,
            current_step_id=step.id if step is not None else None,
            current_step_type=step.type if step is not None else None,
            current_step_label=(step.label or step.type) if step is not None else None,
            step_elapsed_s=round(step_elapsed, 2),
            step_timeout_s=round(step_timeout, 2),
            routine_elapsed_s=round(routine_elapsed, 2),
            balls_fed_in_step=self._balls_fed_in_step,
            resolved_goal_column=self._resolved_goal_column,
            steps_summary=self._build_steps_summary_locked(routine),
        )

    def start(
        self,
        routine: Optional[AutoRoutineDefinition] = None,
        now: Optional[float] = None,
    ) -> dict:
        """Starts executing the given (or currently active) routine from step 0."""
        t = time.monotonic() if now is None else float(now)
        with self._lock:
            target_routine = routine if routine is not None else self._store.get_active_routine()
            self._routine = target_routine
            self._routine_start_time = t
            self._step_start_time = t
            self._aim_anchor_xy = None
            self._feed_phase = "wait_speed"
            self._feed_start_time = t
            self._prev_ball_detected = None
            self._balls_fed_in_step = 0
            self._resolved_goal_column = None

            if not target_routine.steps:
                self._running = False
                self._step_index = -1
                self._step_entered = False
                self._status = "completed"
                self._status_message = f"Routine '{target_routine.name}' completed (0 steps)"
                self._refresh_state_locked(t)
                return self._state.to_dict()

            self._running = True
            self._step_index = 0
            self._step_entered = False
            self._status = "running"
            first = target_routine.steps[0]
            self._status_message = f"Step 1/{len(target_routine.steps)}: {first.label or first.type}"
            self._refresh_state_locked(t)
            logger.info(f"Started autonomous routine '{target_routine.id}' ({len(target_routine.steps)} steps).")
            return self._state.to_dict()

    def abort(
        self,
        reason: str = "Aborted by operator",
        *,
        uart: Optional[UARTComm] = None,
        chassis_controller: Optional[ChassisPoseController] = None,
        shooter_controller: Optional[ShooterSpeedController] = None,
        conveyor_controller: Optional[ConveyorController] = None,
        now: Optional[float] = None,
    ) -> dict:
        """Aborts any running autonomous routine and stops all controlled subsystems."""
        t = time.monotonic() if now is None else float(now)
        with self._lock:
            was_running = self._running
            self._running = False
            self._step_entered = False
            self._stop_actuators(
                uart=uart,
                chassis_controller=chassis_controller,
                shooter_controller=shooter_controller,
                conveyor_controller=conveyor_controller,
            )
            if was_running:
                self._status = "aborted"
                self._status_message = str(reason)
                logger.warning(f"Autonomous routine aborted: {reason}")
            elif self._status not in ("completed", "aborted"):
                self._status = "idle"
                self._status_message = "Autonomous runner idle"
            self._refresh_state_locked(t)
            return self._state.to_dict()

    def _resolve_goal_coords(
        self,
        aim_mode: str,
        goal_column: str,
        vision_manager: VisionManager,
    ) -> tuple[tuple[float, float], str]:
        if aim_mode == "active_1x3":
            vt = vision_manager.latest_target
            if (
                vt is not None
                and vt.latched
                and vt.target_x_cm is not None
                and vt.target_y_cm is not None
                and vt.active_target in _VALID_GOAL_COLUMNS
            ):
                return (float(vt.target_x_cm), float(vt.target_y_cm)), vt.active_target
            return (float(SpecialGridConfig.CENTER_X_CM), float(SpecialGridConfig.TOP_Y_CM)), "center"

        col = goal_column if goal_column in _VALID_GOAL_COLUMNS else "center"
        coords = SpecialGridConfig.get_coords(col) or (
            float(SpecialGridConfig.CENTER_X_CM),
            float(SpecialGridConfig.TOP_Y_CM),
        )
        return (float(coords[0]), float(coords[1])), col

    def _apply_shooter_config(
        self,
        shooter_controller: ShooterSpeedController,
        enabled: bool,
        mode: str,
        rpm: float,
    ) -> None:
        shooter_controller.set_mode(mode)
        if not enabled:
            shooter_controller.set_enabled(False)
            return
        shooter_controller.set_manual_rpm(rpm, switch_to_manual=(mode == "manual"))
        shooter_controller.set_enabled(True)

    def _is_shooter_ready_for_step(
        self,
        shooter_controller: ShooterSpeedController,
        elapsed_s: float,
        open_loop_wait_s: float,
    ) -> bool:
        st = shooter_controller.state
        if not st.enabled or st.target_rpm <= 0.0:
            return False
        if st.closed_loop:
            return bool(st.at_speed)
        return elapsed_s >= open_loop_wait_s

    def _enter_step_locked(
        self,
        step: AutoStep,
        t: float,
        uart: UARTComm,
        pose_estimator: PoseEstimator,
        vision_manager: VisionManager,
        esp32_receiver: ESP32SensorReceiver,
        chassis_controller: ChassisPoseController,
        shooter_controller: ShooterSpeedController,
        conveyor_controller: ConveyorController,
    ) -> bool:
        """
        Executes one-shot step entry actions.
        Returns True if the step completed instantaneously and should advance immediately.
        """
        self._step_start_time = t
        self._step_entered = True
        self._aim_anchor_xy = None
        self._balls_fed_in_step = 0

        stype = step.type

        if stype == AutoStepType.WAIT_VISION.value:
            vt = vision_manager.latest_target
            if vt is not None and vt.latched and vt.active_target in _VALID_GOAL_COLUMNS:
                self._resolved_goal_column = vt.active_target
                return True
            return False

        if stype == AutoStepType.DRIVE_TO.value:
            if step.pre_spin_shooter:
                self._apply_shooter_config(
                    shooter_controller, True, step.shooter_mode, step.shooter_rpm
                )
            if step.aim_mode == "explicit":
                chassis_controller.set_target(
                    step.x_cm,
                    step.y_cm,
                    step.heading_deg,
                    enable=True,
                    stop_on_arrival=step.stop_on_arrival,
                )
            else:
                (gx, gy), col = self._resolve_goal_coords(
                    step.aim_mode, step.goal_column, vision_manager
                )
                self._resolved_goal_column = col
                chassis_controller.set_target_aim_at_goal(
                    step.x_cm,
                    step.y_cm,
                    gx,
                    gy,
                    enable=True,
                    stop_on_arrival=step.stop_on_arrival,
                )
            return False

        if stype == AutoStepType.AIM_AT.value:
            pose = pose_estimator.latest_pose
            if pose is not None and pose.valid and pose.x_cm is not None and pose.y_cm is not None:
                self._aim_anchor_xy = (float(pose.x_cm), float(pose.y_cm))
                if step.aim_mode == "explicit":
                    chassis_controller.set_target(
                        self._aim_anchor_xy[0],
                        self._aim_anchor_xy[1],
                        step.heading_deg,
                        enable=True,
                        stop_on_arrival=step.stop_on_arrival,
                    )
                else:
                    (gx, gy), col = self._resolve_goal_coords(
                        step.aim_mode, step.goal_column, vision_manager
                    )
                    self._resolved_goal_column = col
                    chassis_controller.set_target_aim_at_goal(
                        self._aim_anchor_xy[0],
                        self._aim_anchor_xy[1],
                        gx,
                        gy,
                        enable=True,
                        stop_on_arrival=step.stop_on_arrival,
                    )
            return False

        if stype == AutoStepType.SHOOTER_SET.value:
            self._apply_shooter_config(
                shooter_controller, step.enabled, step.mode, step.target_rpm
            )
            if not step.enabled or not step.wait_at_speed:
                return True
            return False

        if stype == AutoStepType.PITCH_STEP.value:
            if step.action == "disable":
                if not uart.estop_active:
                    uart.send_shooter_disable_angle()
                return True
            if not uart.estop_active:
                uart.send_shooter_angle(run=True, reversed=(step.action == "down"))
            return False

        if stype == AutoStepType.FEED_BALLS.value:
            self._feed_phase = "wait_speed"
            self._feed_start_time = t
            self._prev_ball_detected = None
            conveyor_controller.set_operator_command(0, False, False)
            if not shooter_controller.enabled:
                self._apply_shooter_config(
                    shooter_controller,
                    True,
                    "auto",
                    float(ShooterPIDConfig.DEFAULT_MANUAL_RPM),
                )
            if not step.require_at_speed or self._is_shooter_ready_for_step(
                shooter_controller,
                0.0,
                min(step.at_speed_timeout_s, AutoRoutineConfig.OPEN_LOOP_SPINUP_WAIT_S),
            ):
                self._start_feeding_phase_locked(
                    step, t, uart, esp32_receiver, conveyor_controller
                )
            return False

        if stype == AutoStepType.WAIT.value:
            return False

        if stype == AutoStepType.STOP_ALL.value:
            self._stop_actuators(
                uart=uart,
                chassis_controller=chassis_controller,
                shooter_controller=shooter_controller,
                conveyor_controller=conveyor_controller,
            )
            return True

        return True

    def _start_feeding_phase_locked(
        self,
        step: AutoStep,
        t: float,
        uart: UARTComm,
        esp32_receiver: ESP32SensorReceiver,
        conveyor_controller: ConveyorController,
    ) -> None:
        self._feed_phase = "feeding"
        self._feed_start_time = t
        snap = esp32_receiver.latest_snapshot
        if snap is not None and snap.is_fresh(esp32_receiver.stale_timeout_s):
            self._prev_ball_detected = bool(snap.ball_detected)
        else:
            self._prev_ball_detected = None

        eff_trig, eff_run, eff_rev = conveyor_controller.set_operator_command(
            step.trigger_speed, step.spinner_run, False
        )
        if not uart.estop_active:
            uart.send_conveyor_command(
                trigger=eff_trig,
                spinner_run=eff_run,
                spinner_reversed=eff_rev,
            )

    def _tick_step_locked(
        self,
        step: AutoStep,
        t: float,
        uart: UARTComm,
        pose_estimator: PoseEstimator,
        vision_manager: VisionManager,
        esp32_receiver: ESP32SensorReceiver,
        chassis_controller: ChassisPoseController,
        shooter_controller: ShooterSpeedController,
        conveyor_controller: ConveyorController,
    ) -> bool:
        """
        Evaluates active step completion on a 50 Hz tick.
        Returns True when the step has finished and should advance.
        """
        elapsed = max(0.0, t - self._step_start_time)
        stype = step.type

        if stype == AutoStepType.WAIT_VISION.value:
            vt = vision_manager.latest_target
            if vt is not None and vt.latched and vt.active_target in _VALID_GOAL_COLUMNS:
                self._resolved_goal_column = vt.active_target
                return True
            if elapsed >= step.timeout_s:
                vision_manager.update(
                    active_target=step.fallback_target,
                    target_detected=True,
                    timestamp=time.time(),
                    force=True,
                )
                self._resolved_goal_column = step.fallback_target
                logger.warning(
                    f"wait_vision timed out after {step.timeout_s:.2f}s; latched fallback '{step.fallback_target}'."
                )
                return True
            return False

        if stype == AutoStepType.DRIVE_TO.value:
            if chassis_controller.is_at_target():
                return True
            if elapsed >= step.timeout_s:
                if step.stop_on_arrival:
                    chassis_controller.stop()
                logger.warning(
                    f"drive_to step '{step.id}' timed out after {step.timeout_s:.2f}s."
                )
                return True
            return False

        if stype == AutoStepType.AIM_AT.value:
            if self._aim_anchor_xy is None:
                pose = pose_estimator.latest_pose
                if pose is not None and pose.valid and pose.x_cm is not None and pose.y_cm is not None:
                    self._aim_anchor_xy = (float(pose.x_cm), float(pose.y_cm))
                    if step.aim_mode == "explicit":
                        chassis_controller.set_target(
                            self._aim_anchor_xy[0],
                            self._aim_anchor_xy[1],
                            step.heading_deg,
                            enable=True,
                            stop_on_arrival=step.stop_on_arrival,
                        )
                    else:
                        (gx, gy), col = self._resolve_goal_coords(
                            step.aim_mode, step.goal_column, vision_manager
                        )
                        self._resolved_goal_column = col
                        chassis_controller.set_target_aim_at_goal(
                            self._aim_anchor_xy[0],
                            self._aim_anchor_xy[1],
                            gx,
                            gy,
                            enable=True,
                            stop_on_arrival=step.stop_on_arrival,
                        )
            elif chassis_controller.is_at_target():
                return True

            if elapsed >= step.timeout_s:
                if step.stop_on_arrival:
                    chassis_controller.stop()
                return True
            return False

        if stype == AutoStepType.SHOOTER_SET.value:
            if self._is_shooter_ready_for_step(
                shooter_controller, elapsed, AutoRoutineConfig.OPEN_LOOP_SPINUP_WAIT_S
            ):
                return True
            return elapsed >= step.timeout_s

        if stype == AutoStepType.PITCH_STEP.value:
            if elapsed >= step.duration_s:
                if not uart.estop_active:
                    uart.send_shooter_angle(run=False, reversed=False)
                return True
            return False

        if stype == AutoStepType.FEED_BALLS.value:
            if self._feed_phase == "wait_speed":
                open_wait = min(
                    step.at_speed_timeout_s, AutoRoutineConfig.OPEN_LOOP_SPINUP_WAIT_S
                )
                if (
                    self._is_shooter_ready_for_step(shooter_controller, elapsed, open_wait)
                    or elapsed >= step.at_speed_timeout_s
                ):
                    self._start_feeding_phase_locked(
                        step, t, uart, esp32_receiver, conveyor_controller
                    )
                return False

            snap = esp32_receiver.latest_snapshot
            esp32_fresh = snap is not None and snap.is_fresh(esp32_receiver.stale_timeout_s)
            if esp32_fresh and snap is not None:
                curr_ball = bool(snap.ball_detected)
                if self._prev_ball_detected is True and curr_ball is False:
                    self._balls_fed_in_step += 1
                self._prev_ball_detected = curr_ball

            feed_elapsed = max(0.0, t - self._feed_start_time)
            ball_goal_met = (
                step.ball_count > 0
                and esp32_fresh
                and self._balls_fed_in_step >= step.ball_count
            )
            if ball_goal_met or feed_elapsed >= step.duration_s:
                conveyor_controller.set_operator_command(0, False, False)
                if not uart.estop_active:
                    uart.send_conveyor_command(0, False, False)
                return True
            return False

        if stype == AutoStepType.WAIT.value:
            return elapsed >= step.duration_s

        if stype == AutoStepType.STOP_ALL.value:
            return True

        return True

    def update(
        self,
        uart: UARTComm,
        pose_estimator: PoseEstimator,
        vision_manager: VisionManager,
        esp32_receiver: ESP32SensorReceiver,
        chassis_controller: ChassisPoseController,
        shooter_controller: ShooterSpeedController,
        conveyor_controller: ConveyorController,
        now: Optional[float] = None,
    ) -> dict:
        """Evaluates one 50 Hz tick of the running autonomous routine sequence."""
        t = time.monotonic() if now is None else float(now)
        with self._lock:
            if not self._running or self._routine is None:
                self._refresh_state_locked(t)
                return self._state.to_dict()

            steps = self._routine.steps
            total = len(steps)

            # Bound instant transitions to at most remaining steps + 1
            for _ in range(total + 1):
                if not self._running or self._step_index < 0 or self._step_index >= total:
                    break

                step = steps[self._step_index]
                if not self._step_entered:
                    done_now = self._enter_step_locked(
                        step=step,
                        t=t,
                        uart=uart,
                        pose_estimator=pose_estimator,
                        vision_manager=vision_manager,
                        esp32_receiver=esp32_receiver,
                        chassis_controller=chassis_controller,
                        shooter_controller=shooter_controller,
                        conveyor_controller=conveyor_controller,
                    )
                else:
                    done_now = self._tick_step_locked(
                        step=step,
                        t=t,
                        uart=uart,
                        pose_estimator=pose_estimator,
                        vision_manager=vision_manager,
                        esp32_receiver=esp32_receiver,
                        chassis_controller=chassis_controller,
                        shooter_controller=shooter_controller,
                        conveyor_controller=conveyor_controller,
                    )

                if not done_now:
                    self._status = "running"
                    self._status_message = (
                        f"Step {self._step_index + 1}/{total}: {step.label or step.type}"
                    )
                    break

                # Advance to next step or complete routine
                self._step_index += 1
                self._step_entered = False
                if self._step_index >= total:
                    self._running = False
                    self._step_index = total - 1
                    self._status = "completed"
                    self._status_message = (
                        f"Routine '{self._routine.name}' completed ({total} steps)"
                    )
                    self._stop_actuators(
                        uart=uart,
                        chassis_controller=chassis_controller,
                        shooter_controller=shooter_controller,
                        conveyor_controller=conveyor_controller,
                    )
                    logger.info(self._status_message)
                    break

            self._refresh_state_locked(t)
            return self._state.to_dict()


def step_auto_routine(
    runner: AutoRoutineRunner,
    uart: UARTComm,
    match_manager: MatchManager,
    pose_estimator: PoseEstimator,
    vision_manager: VisionManager,
    esp32_receiver: ESP32SensorReceiver,
    chassis_controller: ChassisPoseController,
    shooter_controller: ShooterSpeedController,
    conveyor_controller: ConveyorController,
    now: Optional[float] = None,
) -> dict:
    """
    Executes one 50 Hz autonomous routine sequence tick with E-Stop, match mode,
    and 60s Autonomous phase expiration gating. Call before subsystem step functions.
    """
    if uart.estop_active or match_manager.mode not in (
        MatchMode.AUTO,
        MatchMode.PRACTICE,
    ):
        if runner.is_running:
            reason = (
                "E-Stop active"
                if uart.estop_active
                else f"Match mode exited to {match_manager.mode.value}"
            )
            return runner.abort(
                reason=reason,
                uart=uart,
                chassis_controller=chassis_controller,
                shooter_controller=shooter_controller,
                conveyor_controller=conveyor_controller,
                now=now,
            )
        return runner.get_status()

    if match_manager.mode == MatchMode.AUTO and runner.is_running:
        match_status = match_manager.get_status()
        phase_dur = float(match_status.get("phase_duration", 0.0))
        phase_rem = float(match_status.get("phase_time_remaining", 0.0))
        if (phase_dur > 0.0 and phase_rem <= 0.0) or (
            runner.state.routine_elapsed_s >= float(AutoRoutineConfig.MAX_ROUTINE_DURATION_S)
        ):
            return runner.abort(
                reason="Autonomous 60s phase expired",
                uart=uart,
                chassis_controller=chassis_controller,
                shooter_controller=shooter_controller,
                conveyor_controller=conveyor_controller,
                now=now,
            )

    if runner.is_running:
        return runner.update(
            uart=uart,
            pose_estimator=pose_estimator,
            vision_manager=vision_manager,
            esp32_receiver=esp32_receiver,
            chassis_controller=chassis_controller,
            shooter_controller=shooter_controller,
            conveyor_controller=conveyor_controller,
            now=now,
        )

    return runner.get_status()


# ==============================================================================
# Unit & 50 Hz Closed-Loop Multi-Step Simulation Tests
# ==============================================================================
if __name__ == "__main__":
    import tempfile
    from core.control import (
        step_chassis_control,
        step_conveyor_control,
        step_shooter_control,
    )

    print("Testing RoutineStore & AutoRoutineDefinition validation...")
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)
        store = RoutineStore(routines_dir=tmp_path)
        listed = store.list_routines()
        assert len(listed) == 2
        assert store.active_routine_id == AutoRoutineConfig.DEFAULT_ROUTINE_ID

        # Path traversal & reserved ID rejection
        for bad_id in ("../evil", "_active", "bad/name", ""):
            try:
                validate_routine_id(bad_id)
                assert False, f"Expected ValueError for {bad_id!r}"
            except ValueError:
                pass

        # Wall-margin clamping & CRUD persistence
        custom = store.save_routine({
            "id": "test_custom",
            "name": "Custom Test",
            "start_pose": {"x_cm": 5.0, "y_cm": 10.0, "heading_deg": 725.0},
            "steps": [
                {
                    "id": "s1",
                    "type": "drive_to",
                    "x_cm": 400.0,
                    "y_cm": 500.0,
                    "aim_mode": "explicit",
                    "heading_deg": 370.0,
                },
                {"id": "s2", "type": "stop_all"},
            ],
        })
        assert custom.start_pose["x_cm"] == ChassisPIDConfig.WALL_MARGIN_CM
        assert custom.start_pose["y_cm"] == ChassisPIDConfig.WALL_MARGIN_CM
        assert custom.start_pose["heading_deg"] == 5.0
        assert custom.steps[0].x_cm == FieldConfig.WIDTH_CM - ChassisPIDConfig.WALL_MARGIN_CM
        assert custom.steps[0].y_cm == FieldConfig.LENGTH_CM - ChassisPIDConfig.WALL_MARGIN_CM
        assert custom.steps[0].heading_deg == 10.0

        store.set_active_routine("test_custom")
        store_reloaded = RoutineStore(routines_dir=tmp_path)
        assert store_reloaded.active_routine_id == "test_custom"
        assert store_reloaded.get_active_routine().id == "test_custom"

        store_reloaded.delete_routine("test_custom")
        assert store_reloaded.active_routine_id != "test_custom"
    print("RoutineStore & validation tests PASSED!")

    print("Testing 50 Hz Closed-Loop Multi-Step AutoRoutineRunner Simulation...")
    with tempfile.TemporaryDirectory() as tmpdir, UARTComm(dry_run=True) as dry_uart:
        store = RoutineStore(routines_dir=Path(tmpdir))
        runner = AutoRoutineRunner(store)
        match_mgr = MatchManager()
        pose_est = PoseEstimator(
            front_offset_cm=(0.0, 0.0),
            left_offset_cm=(0.0, 0.0),
            right_offset_cm=(0.0, 0.0),
        )
        vision_mgr = VisionManager()
        esp_rx = ESP32SensorReceiver(dry_run=True)
        ch_ctrl = ChassisPoseController()
        sh_ctrl = ShooterSpeedController()
        cv_ctrl = ConveyorController()

        sim_routine = AutoRoutineDefinition.from_dict({
            "id": "sim_routine",
            "name": "50Hz Sim Routine",
            "start_pose": {"x_cm": 200.0, "y_cm": 120.0, "heading_deg": 0.0},
            "steps": [
                {
                    "id": "s1",
                    "type": "wait_vision",
                    "label": "Wait Vision",
                    "timeout_s": 1.0,
                    "fallback_target": "center",
                },
                {
                    "id": "s2",
                    "type": "drive_to",
                    "label": "Drive to Spot",
                    "x_cm": 200.0,
                    "y_cm": 240.0,
                    "aim_mode": "active_1x3",
                    "stop_on_arrival": True,
                    "pre_spin_shooter": True,
                    "shooter_mode": "auto",
                    "timeout_s": 6.0,
                },
                {
                    "id": "s3",
                    "type": "feed_balls",
                    "label": "Feed 2 Balls",
                    "trigger_speed": 700,
                    "spinner_run": True,
                    "require_at_speed": True,
                    "at_speed_timeout_s": 2.0,
                    "ball_count": 2,
                    "duration_s": 4.0,
                },
                {"id": "s4", "type": "stop_all", "label": "Stop All"},
            ],
        })

        match_mgr.start_auto()
        runner.start(sim_routine, now=0.0)

        px, py, ph = 200.0, 120.0, 0.0
        dt = 0.02
        t = 0.0
        last_fw = (0, 0)
        last_sp = (False, False)
        last_drv = (0, 0, 0)

        for tick in range(400):
            # Latch vision "right" at t=0.10s
            if tick == 5:
                vision_mgr.update("right", True, timestamp=t)

            # Simulate ball falling edges once in feed_balls feeding phase
            st_now = runner.state
            ball_state = True
            if st_now.current_step_id == "s3":
                # Emit True -> False transitions at feeding ticks
                if tick % 12 < 6:
                    ball_state = True
                else:
                    ball_state = False

            # Update ESP32 snapshot with target RPM so shooter PID reports at_speed
            target_rpm_int = int(round(sh_ctrl.state.target_rpm)) if sh_ctrl.enabled else 0
            esp_rx.inject_telemetry(
                upper_rpm=target_rpm_int,
                lower_rpm=target_rpm_int,
                heading_deg=int(round(ph)),
                dist_front_mm=max(50, int(round((FieldConfig.LENGTH_CM - py) * 10.0))),
                dist_right_mm=max(50, int(round((FieldConfig.WIDTH_CM - px) * 10.0))),
                dist_left_mm=max(50, int(round(px * 10.0))),
                ball_detected=ball_state,
            )
            cos_h = max(0.5, abs(math.cos(math.radians(ph))))
            # Align simulated LiDAR distances with rotated ray foreshortening at heading ph
            pose_est.update(
                dist_front_cm=max(5.0, (FieldConfig.LENGTH_CM - py) / cos_h),
                dist_right_cm=max(5.0, (FieldConfig.WIDTH_CM - px) / cos_h),
                raw_heading_deg=ph,
                dist_left_cm=max(5.0, px / cos_h),
            )

            step_auto_routine(
                runner=runner,
                uart=dry_uart,
                match_manager=match_mgr,
                pose_estimator=pose_est,
                vision_manager=vision_mgr,
                esp32_receiver=esp_rx,
                chassis_controller=ch_ctrl,
                shooter_controller=sh_ctrl,
                conveyor_controller=cv_ctrl,
                now=t,
            )
            last_fw = step_shooter_control(
                sh_ctrl, dry_uart, match_mgr, pose_est, vision_mgr, esp_rx, last_fw, now=t
            )
            last_sp = step_conveyor_control(
                cv_ctrl, sh_ctrl, dry_uart, match_mgr, esp_rx, last_sp, now=t
            )
            last_drv = step_chassis_control(
                ch_ctrl, dry_uart, match_mgr, pose_est, last_drv, now=t
            )

            # Advance simple 2D holonomic plant
            vx_cmd, vy_cmd, w_cmd = last_drv
            th_rad = math.radians(ph)
            v_fwd = vx_cmd * 0.15
            v_right = vy_cmd * 0.15
            px += (v_fwd * math.sin(th_rad) + v_right * math.cos(th_rad)) * dt
            py += (v_fwd * math.cos(th_rad) - v_right * math.sin(th_rad)) * dt
            ph = (ph + (w_cmd * 0.18) * dt) % 360.0
            t += dt

            if not runner.is_running:
                break

        final_st = runner.state
        assert final_st.status == "completed", final_st
        assert final_st.resolved_goal_column == "right"
        assert math.hypot(200.0 - px, 240.0 - py) <= ChassisPIDConfig.POS_TOLERANCE_CM + 1.0
        assert not ch_ctrl.enabled and not sh_ctrl.enabled and not cv_ctrl.is_active

        # Test wait_vision timeout fallback & E-Stop / TELEOP abort gating
        vision_mgr.reset()
        wait_only = AutoRoutineDefinition.from_dict({
            "id": "wait_fallback",
            "name": "Wait Fallback",
            "steps": [{"id": "w1", "type": "wait_vision", "timeout_s": 0.2, "fallback_target": "left"}],
        })
        runner.start(wait_only, now=200.0)
        step_auto_routine(runner, dry_uart, match_mgr, pose_est, vision_mgr, esp_rx, ch_ctrl, sh_ctrl, cv_ctrl, now=200.05)
        assert runner.is_running is True
        step_auto_routine(runner, dry_uart, match_mgr, pose_est, vision_mgr, esp_rx, ch_ctrl, sh_ctrl, cv_ctrl, now=200.26)
        assert runner.is_running is False and runner.state.status == "completed"
        assert vision_mgr.latest_target.active_target == "left" and vision_mgr.latest_target.latched

        # Verify TELEOP mode immediately aborts running routine
        match_mgr.start_practice()
        runner.start(sim_routine, now=300.0)
        assert runner.is_running is True
        match_mgr.enable_teleop()
        step_auto_routine(runner, dry_uart, match_mgr, pose_est, vision_mgr, esp_rx, ch_ctrl, sh_ctrl, cv_ctrl, now=300.02)
        assert runner.is_running is False and runner.state.status == "aborted"

        # Verify E-Stop immediately aborts running routine
        match_mgr.start_auto()
        runner.start(sim_routine, now=400.0)
        dry_uart.trigger_estop()
        step_auto_routine(runner, dry_uart, match_mgr, pose_est, vision_mgr, esp_rx, ch_ctrl, sh_ctrl, cv_ctrl, now=400.02)
        assert runner.is_running is False and runner.state.status == "aborted"

    print("All AutoRoutineRunner tests PASSED!")
