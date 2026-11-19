"""
core/control.py - Subsystem Closed-Loop PID & Conveyor Controllers for TELBot

Implements:
  - ShooterControlState: Thread-safe telemetry snapshot of the dual-flywheel PID controller.
  - ShooterSpeedController: Dual independent closed-loop flywheel RPM PID controller
    with distance-based auto-speed calculation, manual override, and open-loop feedforward fallback.
  - ConveyorControlState: Thread-safe state snapshot of conveyor trigger & spinner arbitration.
  - ConveyorController: Automatic spinner anti-jam reverse when shooter is running with no ball detected.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from config import ConveyorAutoReverse as ConveyorAutoReverseConfig
from config import ESP32 as ESP32Config
from config import ShooterPID as ShooterPIDConfig
from config import SpecialGrid as SpecialGridConfig
from core.comms import UARTComm
from core.esp32 import ESP32SensorReceiver, ESP32SensorSnapshot
from core.match import MatchManager, MatchMode
from core.pid import PIDConfig, PIDController
from core.pose import PoseEstimate, PoseEstimator
from core.vision import VisionManager, VisionTarget


def _build_flywheel_pid_config(
    kp: float,
    ki: float,
    kd: float,
    kv: float,
    ks: float,
    max_rpm: float,
) -> PIDConfig:
    """Builds a unidirectional flywheel RPM PIDConfig with shared safeguards."""
    return PIDConfig(
        kp=float(kp),
        ki=float(ki),
        kd=float(kd),
        kv=float(kv),
        ks=float(ks),
        out_min=float(ShooterPIDConfig.OUT_MIN),
        out_max=float(ShooterPIDConfig.OUT_MAX),
        i_zone=float(ShooterPIDConfig.I_ZONE_RPM),
        max_integral=float(ShooterPIDConfig.MAX_INTEGRAL),
        angle_wrap=False,
        zero_on_zero_setpoint=True,
        zero_setpoint_tolerance=1e-6,
        d_filter_alpha=float(ShooterPIDConfig.D_FILTER_ALPHA),
        error_tolerance=float(ShooterPIDConfig.RPM_TOLERANCE),
        max_dt=0.5,
    )


@dataclass
class ShooterControlState:
    """Snapshot of dual flywheel target, measurements, PID diagnostics, and output commands."""

    enabled: bool = False
    mode: str = "auto"
    closed_loop: bool = False
    target_rpm: float = 0.0
    configured_rpm: float = float(ShooterPIDConfig.DEFAULT_MANUAL_RPM)
    distance_cm: Optional[float] = None
    upper_measured_rpm: Optional[float] = None
    lower_measured_rpm: Optional[float] = None
    upper_cmd: int = 0
    lower_cmd: int = 0
    upper_at_speed: bool = False
    lower_at_speed: bool = False
    at_speed: bool = False
    upper_pid: dict = field(default_factory=dict)
    lower_pid: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)


class ShooterSpeedController:
    """
    Closed-loop dual flywheel RPM controller with shared target RPM,
    distance-based auto-speed calculation, manual override, and open-loop fallback.
    """

    def __init__(
        self,
        upper_config: Optional[PIDConfig] = None,
        lower_config: Optional[PIDConfig] = None,
        max_rpm: float = ShooterPIDConfig.MAX_RPM,
        distance_rpm_table: Optional[tuple[tuple[float, float], ...]] = None,
    ) -> None:
        self._lock = threading.RLock()
        self.max_rpm = float(max_rpm)

        raw_table = (
            ShooterPIDConfig.DISTANCE_TO_RPM_TABLE
            if distance_rpm_table is None
            else distance_rpm_table
        )
        self._distance_rpm_table: tuple[tuple[float, float], ...] = tuple(
            sorted((float(d), float(rpm)) for d, rpm in raw_table)
        )

        u_cfg = upper_config or _build_flywheel_pid_config(
            kp=ShooterPIDConfig.UPPER_KP,
            ki=ShooterPIDConfig.UPPER_KI,
            kd=ShooterPIDConfig.UPPER_KD,
            kv=ShooterPIDConfig.UPPER_KV,
            ks=ShooterPIDConfig.UPPER_KS,
            max_rpm=self.max_rpm,
        )
        l_cfg = lower_config or _build_flywheel_pid_config(
            kp=ShooterPIDConfig.LOWER_KP,
            ki=ShooterPIDConfig.LOWER_KI,
            kd=ShooterPIDConfig.LOWER_KD,
            kv=ShooterPIDConfig.LOWER_KV,
            ks=ShooterPIDConfig.LOWER_KS,
            max_rpm=self.max_rpm,
        )

        self.upper_pid = PIDController(config=u_cfg)
        self.lower_pid = PIDController(config=l_cfg)

        self._enabled: bool = False
        self._mode: str = "auto"
        self._manual_target_rpm: float = float(ShooterPIDConfig.DEFAULT_MANUAL_RPM)
        self._state = ShooterControlState(
            configured_rpm=self._manual_target_rpm,
            upper_pid=self.upper_pid.state.to_dict(),
            lower_pid=self.lower_pid.state.to_dict(),
        )

    @property
    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    @property
    def mode(self) -> str:
        with self._lock:
            return self._mode

    @property
    def manual_target_rpm(self) -> float:
        with self._lock:
            return self._manual_target_rpm

    @property
    def state(self) -> ShooterControlState:
        with self._lock:
            return ShooterControlState(**self._state.to_dict())

    def interpolate_distance_to_rpm(self, dist_cm: float) -> Optional[float]:
        """Evaluates piecewise-linear distance (cm) -> target RPM lookup table."""
        dist = float(dist_cm)
        if not math.isfinite(dist) or dist <= 0.0 or not self._distance_rpm_table:
            return None

        tbl = self._distance_rpm_table
        if dist <= tbl[0][0]:
            return max(0.0, min(self.max_rpm, tbl[0][1]))
        if dist >= tbl[-1][0]:
            return max(0.0, min(self.max_rpm, tbl[-1][1]))

        for (d0, r0), (d1, r1) in zip(tbl, tbl[1:]):
            if d0 <= dist <= d1:
                if abs(d1 - d0) <= 1e-9:
                    rpm = r0
                else:
                    t = (dist - d0) / (d1 - d0)
                    rpm = r0 + t * (r1 - r0)
                return max(0.0, min(self.max_rpm, rpm))

        return max(0.0, min(self.max_rpm, tbl[0][1]))

    def compute_auto_target_rpm(
        self,
        pose: Optional[PoseEstimate],
        vision_target: Optional[VisionTarget] = None,
    ) -> tuple[Optional[float], Optional[float]]:
        """
        Computes (target_rpm, dist_cm) to the active 1x3 target (or top-wall center
        goal (200, 500) by default) from the current valid field pose.
        """
        if (
            vision_target is not None
            and vision_target.latched
            and vision_target.target_x_cm is not None
            and vision_target.target_y_cm is not None
        ):
            target_x = float(vision_target.target_x_cm)
            target_y = float(vision_target.target_y_cm)
        else:
            target_x = float(SpecialGridConfig.CENTER_X_CM)
            target_y = float(SpecialGridConfig.TOP_Y_CM)

        if (
            pose is None
            or not pose.valid
            or pose.x_cm is None
            or pose.y_cm is None
        ):
            return None, None

        dist_cm = math.hypot(target_x - float(pose.x_cm), target_y - float(pose.y_cm))
        rpm = self.interpolate_distance_to_rpm(dist_cm)
        if rpm is None:
            return None, round(dist_cm, 2)
        return round(rpm, 1), round(dist_cm, 2)

    def set_mode(self, mode: str) -> None:
        normalized = str(mode).lower().strip()
        if normalized not in ("auto", "manual"):
            raise ValueError(f"Shooter mode must be 'auto' or 'manual', got {mode!r}")
        with self._lock:
            self._mode = normalized
            self._state.mode = normalized

    def set_manual_rpm(self, rpm: float, switch_to_manual: bool = True) -> None:
        clamped = max(0.0, min(self.max_rpm, float(rpm)))
        with self._lock:
            self._manual_target_rpm = clamped
            if switch_to_manual:
                self._mode = "manual"
                self._state.mode = "manual"
            self._state.configured_rpm = clamped

    def set_manual_duty(self, duty: int, switch_to_manual: bool = True) -> None:
        clamped_duty = max(0, min(1000, int(duty)))
        self.set_manual_rpm(
            (clamped_duty / 1000.0) * self.max_rpm,
            switch_to_manual=switch_to_manual,
        )

    def set_enabled(self, enabled: bool) -> None:
        flag = bool(enabled)
        with self._lock:
            self._enabled = flag
            self._state.enabled = flag
            if not flag:
                self.upper_pid.reset()
                self.lower_pid.reset()
                self._state.closed_loop = False
                self._state.target_rpm = 0.0
                self._state.upper_cmd = 0
                self._state.lower_cmd = 0
                self._state.upper_at_speed = False
                self._state.lower_at_speed = False
                self._state.at_speed = False
                self._state.upper_pid = self.upper_pid.state.to_dict()
                self._state.lower_pid = self.lower_pid.state.to_dict()

    def set_operator_command(
        self,
        flywheel_cmd: int,
        mode: Optional[str] = None,
        target_rpm: Optional[float] = None,
    ) -> None:
        """Applies an operator shoot request while keeping a single shared setpoint."""
        with self._lock:
            if mode is not None:
                norm_mode = str(mode).lower().strip()
                if norm_mode in ("auto", "manual"):
                    self._mode = norm_mode
                    self._state.mode = norm_mode

            if flywheel_cmd <= 0 and (target_rpm is None or target_rpm <= 0):
                self.set_enabled(False)
                return

            if target_rpm is not None:
                rpm = max(0.0, min(self.max_rpm, float(target_rpm)))
            else:
                clamped_cmd = max(0, min(1000, int(flywheel_cmd)))
                rpm = (clamped_cmd / 1000.0) * self.max_rpm

            self._manual_target_rpm = rpm
            self._enabled = True
            self._state.enabled = True
            self._state.configured_rpm = rpm

    def reset(self) -> None:
        """Resets both PID instances and clears live command/readiness state."""
        with self._lock:
            self.upper_pid.reset()
            self.lower_pid.reset()
            self._state.closed_loop = False
            self._state.target_rpm = 0.0
            self._state.upper_cmd = 0
            self._state.lower_cmd = 0
            self._state.upper_at_speed = False
            self._state.lower_at_speed = False
            self._state.at_speed = False
            self._state.upper_pid = self.upper_pid.state.to_dict()
            self._state.lower_pid = self.lower_pid.state.to_dict()

    def stop(self) -> tuple[int, int]:
        """Disables the shooter controller, resets PIDs, and returns (0, 0)."""
        with self._lock:
            self.set_enabled(False)
            self.reset()
            return (0, 0)

    def set_gains(
        self,
        wheel: str = "both",
        kp: Optional[float] = None,
        ki: Optional[float] = None,
        kd: Optional[float] = None,
        kv: Optional[float] = None,
        ks: Optional[float] = None,
    ) -> None:
        """Updates PID/feedforward gains on upper, lower, or both flywheels."""
        target = str(wheel).lower().strip()
        if target not in ("upper", "lower", "both"):
            raise ValueError(f"wheel must be 'upper', 'lower', or 'both', got {wheel!r}")

        with self._lock:
            if target in ("upper", "both"):
                self.upper_pid.set_gains(kp=kp, ki=ki, kd=kd, kv=kv, ks=ks)
            if target in ("lower", "both"):
                self.lower_pid.set_gains(kp=kp, ki=ki, kd=kd, kv=kv, ks=ks)

    def get_gains(self) -> dict:
        with self._lock:
            return {
                "upper": self.upper_pid.config.to_dict(),
                "lower": self.lower_pid.config.to_dict(),
            }

    def is_at_speed(self) -> bool:
        with self._lock:
            return self._state.at_speed

    def update(
        self,
        pose: Optional[PoseEstimate] = None,
        vision_target: Optional[VisionTarget] = None,
        esp32_snapshot: Optional[ESP32SensorSnapshot] = None,
        stale_timeout_s: float = ESP32Config.STALE_TIMEOUT_S,
        dt: Optional[float] = None,
        now: Optional[float] = None,
    ) -> tuple[int, int]:
        """
        Computes independent (upper_cmd, lower_cmd) in [0, 1000] against the shared
        target RPM, falling back to open-loop feedforward when ESP32 feedback is stale/absent.
        """
        with self._lock:
            auto_rpm, dist_cm = self.compute_auto_target_rpm(pose, vision_target)
            configured_rpm = (
                auto_rpm
                if self._mode == "auto" and auto_rpm is not None
                else self._manual_target_rpm
            )
            target_rpm = configured_rpm if self._enabled else 0.0

            rpm_fresh = (
                esp32_snapshot is not None
                and esp32_snapshot.is_fresh(stale_timeout_s)
            )
            upper_measured = float(esp32_snapshot.upper_rpm) if rpm_fresh else None
            lower_measured = float(esp32_snapshot.lower_rpm) if rpm_fresh else None

            if not self._enabled or target_rpm <= 0.0:
                self.upper_pid.reset()
                self.lower_pid.reset()
                upper_cmd = 0
                lower_cmd = 0
                closed_loop = False
            elif rpm_fresh and upper_measured is not None and lower_measured is not None:
                u_out = self.upper_pid.update(target_rpm, upper_measured, dt=dt, now=now)
                l_out = self.lower_pid.update(target_rpm, lower_measured, dt=dt, now=now)
                upper_cmd = max(0, min(1000, int(round(u_out))))
                lower_cmd = max(0, min(1000, int(round(l_out))))
                closed_loop = True
            else:
                # Prevent integral windup when RPM feedback is missing or stale
                self.upper_pid.reset()
                self.lower_pid.reset()
                ff_cmd = max(
                    0, min(1000, int(round(1000.0 * target_rpm / self.max_rpm)))
                )
                upper_cmd = ff_cmd
                lower_cmd = ff_cmd
                closed_loop = False

            upper_at_speed = bool(
                self._enabled and closed_loop and self.upper_pid.at_setpoint()
            )
            lower_at_speed = bool(
                self._enabled and closed_loop and self.lower_pid.at_setpoint()
            )
            at_speed = upper_at_speed and lower_at_speed

            self._state = ShooterControlState(
                enabled=self._enabled,
                mode=self._mode,
                closed_loop=closed_loop,
                target_rpm=round(target_rpm, 1),
                configured_rpm=round(configured_rpm, 1),
                distance_cm=dist_cm,
                upper_measured_rpm=(
                    round(upper_measured, 1) if upper_measured is not None else None
                ),
                lower_measured_rpm=(
                    round(lower_measured, 1) if lower_measured is not None else None
                ),
                upper_cmd=upper_cmd,
                lower_cmd=lower_cmd,
                upper_at_speed=upper_at_speed,
                lower_at_speed=lower_at_speed,
                at_speed=at_speed,
                upper_pid=self.upper_pid.state.to_dict(),
                lower_pid=self.lower_pid.state.to_dict(),
            )
            return (upper_cmd, lower_cmd)

    def get_status(self) -> dict:
        with self._lock:
            return self._state.to_dict()


def step_shooter_control(
    controller: ShooterSpeedController,
    uart: UARTComm,
    match_manager: MatchManager,
    pose_estimator: PoseEstimator,
    vision_manager: VisionManager,
    esp32_receiver: ESP32SensorReceiver,
    last_sent_flywheel: tuple[int, int],
    now: Optional[float] = None,
) -> tuple[int, int]:
    """
    Executes one 50 Hz flywheel PID control tick with E-Stop and match phase gating.
    Returns the updated (upper_cmd, lower_cmd) last-sent pair.
    """
    if uart.estop_active or match_manager.mode not in (
        MatchMode.AUTO,
        MatchMode.TELEOP,
        MatchMode.PRACTICE,
    ):
        if controller.enabled or last_sent_flywheel != (0, 0):
            controller.stop()
            if not uart.estop_active:
                uart.send_shooter_flywheel(0, 0)
        return (0, 0)

    upper_cmd, lower_cmd = controller.update(
        pose=pose_estimator.latest_pose,
        vision_target=vision_manager.latest_target,
        esp32_snapshot=esp32_receiver.latest_snapshot,
        stale_timeout_s=esp32_receiver.stale_timeout_s,
        now=now,
    )
    if (upper_cmd > 0 or lower_cmd > 0) or last_sent_flywheel != (0, 0):
        uart.send_shooter_flywheel(upper_cmd, lower_cmd)
        return (upper_cmd, lower_cmd)
    return last_sent_flywheel


@dataclass
class ConveyorControlState:
    """Snapshot of conveyor operator intent, effective commands, and spinner auto-reverse FSM."""

    auto_reverse_enabled: bool = bool(ConveyorAutoReverseConfig.ENABLED)
    auto_reversing: bool = False
    state: str = "idle"
    ball_detected: Optional[bool] = None
    shooter_running: bool = False
    operator_trigger: int = 0
    operator_spinner_run: bool = False
    operator_spinner_reversed: bool = False
    effective_trigger: int = 0
    effective_spinner_run: bool = False
    effective_spinner_reversed: bool = False
    no_ball_elapsed_s: float = 0.0
    reverse_remaining_s: float = 0.0
    cooldown_remaining_s: float = 0.0
    reverse_count: int = 0

    def to_dict(self) -> dict:
        return asdict(self)


class ConveyorController:
    """
    Arbitrates operator conveyor commands with automatic anti-jam spinner reversal.
    When the shooter is running and ESP32 IR telemetry reports ball_detected=False
    for NO_BALL_DELAY_S, reverses the spinner for REVERSE_DURATION_S (with COOLDOWN_S
    dwell between pulses) without losing or clobbering operator intent.
    """

    def __init__(
        self,
        enabled: bool = ConveyorAutoReverseConfig.ENABLED,
        no_ball_delay_s: float = ConveyorAutoReverseConfig.NO_BALL_DELAY_S,
        reverse_duration_s: float = ConveyorAutoReverseConfig.REVERSE_DURATION_S,
        cooldown_s: float = ConveyorAutoReverseConfig.COOLDOWN_S,
    ) -> None:
        self._lock = threading.RLock()
        self._auto_reverse_enabled = bool(enabled)
        self._no_ball_delay_s = max(0.0, float(no_ball_delay_s))
        self._reverse_duration_s = max(0.0, float(reverse_duration_s))
        self._cooldown_s = max(0.0, float(cooldown_s))

        self._operator_trigger: int = 0
        self._operator_spinner_run: bool = False
        self._operator_spinner_reversed: bool = False

        self._phase: str = "idle"
        self._phase_start_time: Optional[float] = None
        self._reverse_count: int = 0
        self._state = ConveyorControlState(
            auto_reverse_enabled=self._auto_reverse_enabled
        )

    @property
    def auto_reversing(self) -> bool:
        with self._lock:
            return self._phase == "reversing"

    @property
    def is_active(self) -> bool:
        with self._lock:
            return (
                self._operator_trigger != 0
                or self._operator_spinner_run
                or self._phase == "reversing"
            )

    @property
    def state(self) -> ConveyorControlState:
        with self._lock:
            return ConveyorControlState(**self._state.to_dict())

    def _effective_command_locked(self) -> tuple[int, bool, bool]:
        if self._phase == "reversing":
            return (self._operator_trigger, True, True)
        return (
            self._operator_trigger,
            self._operator_spinner_run,
            self._operator_spinner_reversed,
        )

    def _refresh_effective_state_locked(self) -> None:
        eff_trig, eff_run, eff_rev = self._effective_command_locked()
        self._state.auto_reverse_enabled = self._auto_reverse_enabled
        self._state.auto_reversing = self._phase == "reversing"
        self._state.state = self._phase
        self._state.operator_trigger = self._operator_trigger
        self._state.operator_spinner_run = self._operator_spinner_run
        self._state.operator_spinner_reversed = self._operator_spinner_reversed
        self._state.effective_trigger = eff_trig
        self._state.effective_spinner_run = eff_run
        self._state.effective_spinner_reversed = eff_rev
        self._state.reverse_count = self._reverse_count

    def set_operator_command(
        self,
        trigger: int,
        spinner_run: bool,
        spinner_reversed: bool,
    ) -> tuple[int, bool, bool]:
        """Stores operator conveyor intent and returns the arbitrated effective command."""
        with self._lock:
            self._operator_trigger = max(-1000, min(1000, int(trigger)))
            self._operator_spinner_run = bool(spinner_run)
            self._operator_spinner_reversed = bool(spinner_reversed)
            self._refresh_effective_state_locked()
            return self._effective_command_locked()

    def set_operator_trigger(self, speed: int) -> int:
        """Stores operator trigger speed and returns the effective trigger speed."""
        with self._lock:
            self._operator_trigger = max(-1000, min(1000, int(speed)))
            self._refresh_effective_state_locked()
            return self._operator_trigger

    def set_operator_spinner(
        self, run: bool, reversed: bool
    ) -> tuple[bool, bool]:
        """Stores operator spinner intent and returns the effective spinner command."""
        with self._lock:
            self._operator_spinner_run = bool(run)
            self._operator_spinner_reversed = bool(reversed)
            self._refresh_effective_state_locked()
            _, eff_run, eff_rev = self._effective_command_locked()
            return (eff_run, eff_rev)

    def set_config(
        self,
        enabled: Optional[bool] = None,
        no_ball_delay_s: Optional[float] = None,
        reverse_duration_s: Optional[float] = None,
        cooldown_s: Optional[float] = None,
    ) -> None:
        """Updates automatic spinner reverse parameters at runtime."""
        for name, val in (
            ("no_ball_delay_s", no_ball_delay_s),
            ("reverse_duration_s", reverse_duration_s),
            ("cooldown_s", cooldown_s),
        ):
            if val is not None and float(val) < 0.0:
                raise ValueError(f"{name} must be >= 0.0, got {val!r}")

        with self._lock:
            if enabled is not None:
                self._auto_reverse_enabled = bool(enabled)
                if not self._auto_reverse_enabled:
                    self._phase = "idle"
                    self._phase_start_time = None
                    self._state.no_ball_elapsed_s = 0.0
                    self._state.reverse_remaining_s = 0.0
                    self._state.cooldown_remaining_s = 0.0
            if no_ball_delay_s is not None:
                self._no_ball_delay_s = float(no_ball_delay_s)
            if reverse_duration_s is not None:
                self._reverse_duration_s = float(reverse_duration_s)
                if self._reverse_duration_s <= 0.0:
                    self._phase = "idle"
                    self._phase_start_time = None
                    self._state.reverse_remaining_s = 0.0
            if cooldown_s is not None:
                self._cooldown_s = float(cooldown_s)
            self._refresh_effective_state_locked()

    def get_config(self) -> dict:
        with self._lock:
            return {
                "enabled": self._auto_reverse_enabled,
                "no_ball_delay_s": self._no_ball_delay_s,
                "reverse_duration_s": self._reverse_duration_s,
                "cooldown_s": self._cooldown_s,
            }

    def get_status(self) -> dict:
        with self._lock:
            return self._state.to_dict()

    def reset_auto_reverse(self) -> None:
        """Resets the auto-reverse state machine back to idle while preserving operator intent."""
        with self._lock:
            self._phase = "idle"
            self._phase_start_time = None
            self._state.no_ball_elapsed_s = 0.0
            self._state.reverse_remaining_s = 0.0
            self._state.cooldown_remaining_s = 0.0
            self._refresh_effective_state_locked()

    def stop(self) -> tuple[int, bool, bool]:
        """Clears operator intent, resets auto-reverse FSM, and returns (0, False, False)."""
        with self._lock:
            self._operator_trigger = 0
            self._operator_spinner_run = False
            self._operator_spinner_reversed = False
            self._phase = "idle"
            self._phase_start_time = None
            self._state.shooter_running = False
            self._state.no_ball_elapsed_s = 0.0
            self._state.reverse_remaining_s = 0.0
            self._state.cooldown_remaining_s = 0.0
            self._refresh_effective_state_locked()
            return (0, False, False)

    def update(
        self,
        shooter_running: bool,
        esp32_snapshot: Optional[ESP32SensorSnapshot] = None,
        stale_timeout_s: float = ESP32Config.STALE_TIMEOUT_S,
        now: Optional[float] = None,
    ) -> tuple[int, bool, bool]:
        """
        Evaluates automatic spinner anti-jam reverse and returns the arbitrated
        conveyor command tuple (trigger, spinner_run, spinner_reversed).
        """
        t = time.monotonic() if now is None else float(now)
        esp32_fresh = (
            esp32_snapshot is not None
            and esp32_snapshot.is_fresh(stale_timeout_s)
        )
        ball_detected: Optional[bool] = (
            bool(esp32_snapshot.ball_detected) if esp32_fresh else None
        )

        with self._lock:
            shooter_active = bool(shooter_running)
            if (
                not self._auto_reverse_enabled
                or self._reverse_duration_s <= 0.0
                or not shooter_active
                or ball_detected is None
                or ball_detected is True
            ):
                self._phase = "idle"
                self._phase_start_time = None
                no_ball_elapsed_s = 0.0
                reverse_remaining_s = 0.0
                cooldown_remaining_s = 0.0
            else:
                if self._phase == "idle":
                    if self._no_ball_delay_s <= 0.0:
                        self._phase = "reversing"
                        self._phase_start_time = t
                        self._reverse_count += 1
                    else:
                        self._phase = "waiting"
                        self._phase_start_time = t
                elif self._phase == "waiting":
                    elapsed = max(0.0, t - (self._phase_start_time if self._phase_start_time is not None else t))
                    if elapsed >= self._no_ball_delay_s:
                        self._phase = "reversing"
                        self._phase_start_time = t
                        self._reverse_count += 1
                elif self._phase == "reversing":
                    elapsed = max(0.0, t - (self._phase_start_time if self._phase_start_time is not None else t))
                    if elapsed >= self._reverse_duration_s:
                        if self._cooldown_s > 0.0:
                            self._phase = "cooldown"
                        else:
                            self._phase = "waiting"
                        self._phase_start_time = t
                elif self._phase == "cooldown":
                    elapsed = max(0.0, t - (self._phase_start_time if self._phase_start_time is not None else t))
                    if elapsed >= self._cooldown_s:
                        self._phase = "reversing"
                        self._phase_start_time = t
                        self._reverse_count += 1

                phase_elapsed = max(
                    0.0,
                    t - (self._phase_start_time if self._phase_start_time is not None else t),
                )
                no_ball_elapsed_s = round(phase_elapsed, 3) if self._phase == "waiting" else 0.0
                reverse_remaining_s = (
                    round(max(0.0, self._reverse_duration_s - phase_elapsed), 3)
                    if self._phase == "reversing"
                    else 0.0
                )
                cooldown_remaining_s = (
                    round(max(0.0, self._cooldown_s - phase_elapsed), 3)
                    if self._phase == "cooldown"
                    else 0.0
                )

            eff_trig, eff_run, eff_rev = self._effective_command_locked()
            self._state = ConveyorControlState(
                auto_reverse_enabled=self._auto_reverse_enabled,
                auto_reversing=(self._phase == "reversing"),
                state=self._phase,
                ball_detected=ball_detected,
                shooter_running=shooter_active,
                operator_trigger=self._operator_trigger,
                operator_spinner_run=self._operator_spinner_run,
                operator_spinner_reversed=self._operator_spinner_reversed,
                effective_trigger=eff_trig,
                effective_spinner_run=eff_run,
                effective_spinner_reversed=eff_rev,
                no_ball_elapsed_s=no_ball_elapsed_s,
                reverse_remaining_s=reverse_remaining_s,
                cooldown_remaining_s=cooldown_remaining_s,
                reverse_count=self._reverse_count,
            )
            return (eff_trig, eff_run, eff_rev)


def step_conveyor_control(
    conveyor_controller: ConveyorController,
    shooter_controller: ShooterSpeedController,
    uart: UARTComm,
    match_manager: MatchManager,
    esp32_receiver: ESP32SensorReceiver,
    last_sent_spinner: tuple[bool, bool],
    now: Optional[float] = None,
) -> tuple[bool, bool]:
    """
    Executes one 50 Hz conveyor spinner anti-jam tick with E-Stop and match phase gating.
    Returns the updated (spinner_run, spinner_reversed) last-sent pair.
    """
    if uart.estop_active or match_manager.mode not in (
        MatchMode.AUTO,
        MatchMode.TELEOP,
        MatchMode.PRACTICE,
    ):
        if conveyor_controller.is_active or last_sent_spinner != (False, False):
            conveyor_controller.stop()
            if not uart.estop_active:
                uart.send_conveyor_command(0, False, False)
        return (False, False)

    shooter_running = bool(
        shooter_controller.enabled and shooter_controller.state.target_rpm > 0.0
    )
    _, eff_run, eff_rev = conveyor_controller.update(
        shooter_running=shooter_running,
        esp32_snapshot=esp32_receiver.latest_snapshot,
        stale_timeout_s=esp32_receiver.stale_timeout_s,
        now=now,
    )
    current_spinner = (eff_run, eff_rev)
    if current_spinner != last_sent_spinner or conveyor_controller.auto_reversing:
        uart.send_conveyor_spinner(run=eff_run, reversed=eff_rev)
        return current_spinner
    return last_sent_spinner


# ==============================================================================
# Unit Verification
# ==============================================================================
if __name__ == "__main__":
    import time

    print("Testing ShooterSpeedController...")
    ctrl = ShooterSpeedController()

    # 1. Zero target / disabled immediately returns (0, 0) and resets integrators
    assert ctrl.update() == (0, 0)
    assert ctrl.upper_pid.integral == 0.0
    assert ctrl.lower_pid.integral == 0.0

    # 2. Open-loop fallback when ESP32 is missing or stale
    ctrl.set_manual_rpm(4400.0, switch_to_manual=True)
    ctrl.set_enabled(True)
    expected_ff = round(1000.0 * 4400.0 / 5500.0)
    u_cmd, l_cmd = ctrl.update(esp32_snapshot=None)
    assert (u_cmd, l_cmd) == (expected_ff, expected_ff)
    assert ctrl.state.closed_loop is False
    assert ctrl.upper_pid.integral == 0.0 and ctrl.lower_pid.integral == 0.0

    stale_snap = ESP32SensorSnapshot(
        upper_rpm=1000,
        lower_rpm=1000,
        heading_deg=0,
        dist_front_mm=2000,
        dist_right_mm=2000,
        dist_left_mm=2000,
        dist_front_cm=200.0,
        dist_right_cm=200.0,
        dist_left_cm=200.0,
        seq=1,
        timestamp=time.monotonic() - 2.0,
        wall_time=time.time() - 2.0,
    )
    assert ctrl.update(esp32_snapshot=stale_snap, stale_timeout_s=0.5) == (expected_ff, expected_ff)
    assert ctrl.state.closed_loop is False

    # 3. Closed-loop dual PID with asymmetric RPMs, non-negative clamp, and at_speed check
    fresh_asym = ESP32SensorSnapshot(
        upper_rpm=4100,
        lower_rpm=4500,
        heading_deg=0,
        dist_front_mm=2000,
        dist_right_mm=2000,
        dist_left_mm=2000,
        dist_front_cm=200.0,
        dist_right_cm=200.0,
        dist_left_cm=200.0,
        seq=2,
        timestamp=time.monotonic(),
        wall_time=time.time(),
    )
    u_cmd, l_cmd = ctrl.update(esp32_snapshot=fresh_asym, dt=0.02)
    assert ctrl.state.closed_loop is True
    assert u_cmd > l_cmd >= 0
    assert ctrl.is_at_speed() is False

    # Overshoot check -> clamped >= 0 (never negative braking)
    fresh_over = ESP32SensorSnapshot(
        upper_rpm=6000,
        lower_rpm=6000,
        heading_deg=0,
        dist_front_mm=2000,
        dist_right_mm=2000,
        dist_left_mm=2000,
        dist_front_cm=200.0,
        dist_right_cm=200.0,
        dist_left_cm=200.0,
        seq=3,
        timestamp=time.monotonic(),
        wall_time=time.time(),
    )
    u_over, l_over = ctrl.update(esp32_snapshot=fresh_over, dt=0.02)
    assert u_over >= 0 and l_over >= 0

    # Converged within tolerance -> at_speed=True
    fresh_ready = ESP32SensorSnapshot(
        upper_rpm=4380,
        lower_rpm=4420,
        heading_deg=0,
        dist_front_mm=2000,
        dist_right_mm=2000,
        dist_left_mm=2000,
        dist_front_cm=200.0,
        dist_right_cm=200.0,
        dist_left_cm=200.0,
        seq=4,
        timestamp=time.monotonic(),
        wall_time=time.time(),
    )
    ctrl.update(esp32_snapshot=fresh_ready, dt=0.02)
    assert ctrl.is_at_speed() is True

    # Disabling resets integrators immediately
    assert ctrl.stop() == (0, 0)
    assert ctrl.upper_pid.integral == 0.0 and ctrl.lower_pid.integral == 0.0

    # 4. Auto distance-to-RPM interpolation (center goal, latched 1x3 targets, invalid pose fallback)
    ctrl.set_mode("auto")
    ctrl.set_enabled(True)
    pose_center = PoseEstimate(
        x_cm=200.0,
        y_cm=300.0,  # 200 cm away from (200, 500) -> 3450 RPM
        heading_deg=0.0,
        raw_heading_deg=0.0,
        dist_front_cm=200.0,
        dist_left_cm=200.0,
        dist_right_cm=200.0,
        last_valid_left_cm=200.0,
        last_valid_right_cm=200.0,
        last_valid_front_cm=200.0,
        lr_status="both_valid",
        front_status="valid",
        valid=True,
        timestamp=time.time(),
    )
    ctrl.update(pose=pose_center)
    assert abs(ctrl.state.distance_cm - 200.0) < 0.1
    assert abs(ctrl.state.target_rpm - 3450.0) < 1e-6

    vision_left = VisionTarget(
        active_target="left",
        target_detected=True,
        latched=True,
        target_x_cm=140.0,
        target_y_cm=500.0,
    )
    pose_at_140 = PoseEstimate(
        x_cm=140.0,
        y_cm=360.0,  # 140 cm straight below (140, 500) -> 2950 RPM
        heading_deg=0.0,
        raw_heading_deg=0.0,
        dist_front_cm=140.0,
        dist_left_cm=140.0,
        dist_right_cm=260.0,
        last_valid_left_cm=140.0,
        last_valid_right_cm=260.0,
        last_valid_front_cm=140.0,
        lr_status="both_valid",
        front_status="valid",
        valid=True,
        timestamp=time.time(),
    )
    ctrl.update(pose=pose_at_140, vision_target=vision_left)
    assert abs(ctrl.state.distance_cm - 140.0) < 0.1
    assert abs(ctrl.state.target_rpm - 2950.0) < 1e-6

    # Invalid pose -> falls back to configured manual RPM
    invalid_pose = PoseEstimate(
        x_cm=None,
        y_cm=None,
        heading_deg=0.0,
        raw_heading_deg=0.0,
        dist_front_cm=0.0,
        dist_left_cm=None,
        dist_right_cm=0.0,
        last_valid_left_cm=None,
        last_valid_right_cm=None,
        last_valid_front_cm=None,
        lr_status="no_valid_beam",
        front_status="no_valid_beam",
        valid=False,
        timestamp=time.time(),
    )
    ctrl.update(pose=invalid_pose, vision_target=vision_left)
    assert abs(ctrl.state.target_rpm - ctrl.manual_target_rpm) < 1e-6
    print("All ShooterSpeedController tests PASSED!")

    print("Testing ConveyorController...")
    conv = ConveyorController(
        enabled=True,
        no_ball_delay_s=0.5,
        reverse_duration_s=0.4,
        cooldown_s=1.0,
    )

    def _make_snap(ball_detected: bool, age_s: float = 0.0) -> ESP32SensorSnapshot:
        return ESP32SensorSnapshot(
            upper_rpm=4400,
            lower_rpm=4400,
            heading_deg=0,
            dist_front_mm=2000,
            dist_right_mm=2000,
            dist_left_mm=2000,
            dist_front_cm=200.0,
            dist_right_cm=200.0,
            dist_left_cm=200.0,
            seq=10,
            timestamp=time.monotonic() - age_s,
            wall_time=time.time() - age_s,
            ball_detected=ball_detected,
        )

    # 1. Idle when shooter is off, ESP32 is missing/stale, or ball is detected
    snap_no_ball = _make_snap(ball_detected=False)
    snap_with_ball = _make_snap(ball_detected=True)
    snap_stale = _make_snap(ball_detected=False, age_s=2.0)

    assert conv.update(shooter_running=False, esp32_snapshot=snap_no_ball, now=0.0) == (0, False, False)
    assert conv.state.state == "idle"
    assert conv.update(shooter_running=True, esp32_snapshot=None, now=1.0) == (0, False, False)
    assert conv.state.state == "idle"
    assert conv.update(shooter_running=True, esp32_snapshot=snap_stale, stale_timeout_s=0.5, now=2.0) == (0, False, False)
    assert conv.state.state == "idle"
    assert conv.update(shooter_running=True, esp32_snapshot=snap_with_ball, now=3.0) == (0, False, False)
    assert conv.state.state == "idle"

    # 2. Delay -> auto-reverse (True, True) -> cooldown -> repeat pulse when operator spinner was OFF
    assert conv.update(shooter_running=True, esp32_snapshot=snap_no_ball, now=10.0) == (0, False, False)
    assert conv.state.state == "waiting"
    assert conv.update(shooter_running=True, esp32_snapshot=snap_no_ball, now=10.49) == (0, False, False)
    assert conv.state.state == "waiting"
    # Delay elapses at 10.50 -> enters reversing
    assert conv.update(shooter_running=True, esp32_snapshot=snap_no_ball, now=10.50) == (0, True, True)
    assert conv.auto_reversing is True
    assert conv.state.state == "reversing"
    assert conv.state.reverse_count == 1
    # Still reversing before 10.90; operator command must NOT clobber auto-reverse
    assert conv.set_operator_command(700, True, False) == (700, True, True)
    assert conv.update(shooter_running=True, esp32_snapshot=snap_no_ball, now=10.89) == (700, True, True)
    # Reverse finishes at 10.90 -> cooldown restores operator intent (700, True, False)
    assert conv.update(shooter_running=True, esp32_snapshot=snap_no_ball, now=10.90) == (700, True, False)
    assert conv.auto_reversing is False
    assert conv.state.state == "cooldown"
    # Cooldown finishes at 11.90 -> second reverse pulse
    assert conv.update(shooter_running=True, esp32_snapshot=snap_no_ball, now=11.90) == (700, True, True)
    assert conv.state.reverse_count == 2

    # 3. Immediate cancellation mid-reverse when ball_detected becomes True
    assert conv.update(shooter_running=True, esp32_snapshot=snap_with_ball, now=12.00) == (700, True, False)
    assert conv.auto_reversing is False
    assert conv.state.state == "idle"

    # 4. Immediate cancellation mid-reverse when shooter stops (with operator spinner OFF -> returns to OFF)
    conv.set_operator_command(0, False, False)
    conv.update(shooter_running=True, esp32_snapshot=snap_no_ball, now=20.0)
    assert conv.update(shooter_running=True, esp32_snapshot=snap_no_ball, now=20.5) == (0, True, True)
    assert conv.update(shooter_running=False, esp32_snapshot=snap_no_ball, now=20.6) == (0, False, False)
    assert conv.state.state == "idle"

    # 5. step_conveyor_control integration with dry-run UARTComm
    with UARTComm(dry_run=True) as dry_uart:
        match_mgr = MatchManager()
        match_mgr.start_practice()
        esp_rx = ESP32SensorReceiver(dry_run=True)
        shooter_ctrl = ShooterSpeedController()
        shooter_ctrl.set_manual_rpm(4400.0, switch_to_manual=True)
        shooter_ctrl.set_enabled(True)
        shooter_ctrl.update()
        conv_ctrl = ConveyorController(
            enabled=True, no_ball_delay_s=0.5, reverse_duration_s=0.4, cooldown_s=1.0
        )
        esp_rx.inject_telemetry(4400, 4400, 0, 2000, 2000, 2000, ball_detected=False)

        last_sp = (False, False)
        last_sp = step_conveyor_control(
            conv_ctrl, shooter_ctrl, dry_uart, match_mgr, esp_rx, last_sp, now=100.0
        )
        assert last_sp == (False, False)
        # Advance past 0.5s delay -> triggers spinner reverse packet
        tx_before = dry_uart.get_stats()["tx_packets"]
        last_sp = step_conveyor_control(
            conv_ctrl, shooter_ctrl, dry_uart, match_mgr, esp_rx, last_sp, now=100.55
        )
        assert last_sp == (True, True)
        dry_uart.flush_tx(timeout=0.2)
        assert dry_uart.get_stats()["tx_packets"] > tx_before
        # Advance past 0.4s reverse -> restores (False, False)
        last_sp = step_conveyor_control(
            conv_ctrl, shooter_ctrl, dry_uart, match_mgr, esp_rx, last_sp, now=100.96
        )
        assert last_sp == (False, False)
        # Disarming halts active spinner immediately
        conv_ctrl.set_operator_command(500, True, False)
        last_sp = step_conveyor_control(
            conv_ctrl, shooter_ctrl, dry_uart, match_mgr, esp_rx, last_sp, now=101.0
        )
        assert last_sp == (True, False)
        match_mgr.disarm()
        last_sp = step_conveyor_control(
            conv_ctrl, shooter_ctrl, dry_uart, match_mgr, esp_rx, last_sp, now=101.1
        )
        assert last_sp == (False, False)
        assert conv_ctrl.is_active is False

    print("All ConveyorController tests PASSED!")
