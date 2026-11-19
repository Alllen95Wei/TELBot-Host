"""
core/control.py - Subsystem Closed-Loop PID Controllers for TELBot

Implements:
  - ShooterControlState: Thread-safe telemetry snapshot of the dual-flywheel PID controller.
  - ShooterSpeedController: Dual independent closed-loop flywheel RPM PID controller
    with distance-based auto-speed calculation, manual override, and open-loop feedforward fallback.
"""

from __future__ import annotations

import math
import sys
import threading
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

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
