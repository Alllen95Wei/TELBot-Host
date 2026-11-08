"""
core/pid.py - Generic Reusable PID + Feedforward Controller for TELBot

Control Law:
    u(t) = K_v * r(t) + K_s * sgn(r(t)) + K_p * e(t) + K_i * \\int e(\\tau) d\\tau - K_d * dy(t)/dt

Safeguards:
  - Derivative-on-measurement (-K_d * dy/dt) to prevent derivative kick on setpoint changes,
    with angle-wrap awareness and optional exponential low-pass filtering.
  - Output clamping [out_min, out_max] (e.g., out_min=0 for flywheels to prevent reverse braking).
  - Three-layer anti-windup:
      1. i_zone: only accumulates integral when |e(t)| <= i_zone (resets outside zone).
      2. Integrator freeze on saturation: stops integrating when output is saturated in the
         same direction as the error.
      3. Integral clamping: bounds the integral accumulator within [-max_integral, +max_integral].
  - Continuous angle wrapping across the 0 deg / 360 deg boundary into [-180, +180) deg.
  - Immediate reset() when target is 0 (when zero_on_zero_setpoint=True) or subsystem stops.
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from typing import Optional


def wrap_angle_deg(angle_deg: float) -> float:
    """
    Wraps any angle in degrees into the continuous signed interval [-180.0, +180.0).
    Examples:
        wrap_angle_deg(0.0)    ->   0.0
        wrap_angle_deg(185.0)  -> -175.0
        wrap_angle_deg(-190.0) -> +170.0
        wrap_angle_deg(360.0)  ->   0.0
    """
    return ((float(angle_deg) + 180.0) % 360.0) - 180.0


def _sign(val: float, eps: float = 1e-9) -> float:
    """Returns +1.0 if val > eps, -1.0 if val < -eps, else 0.0."""
    if val > eps:
        return 1.0
    if val < -eps:
        return -1.0
    return 0.0


@dataclass
class PIDConfig:
    """Configuration parameters and gains for PIDController."""

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    kv: float = 0.0
    ks: float = 0.0
    out_min: float = -1000.0
    out_max: float = 1000.0
    i_zone: Optional[float] = None
    max_integral: Optional[float] = None
    angle_wrap: bool = False
    zero_on_zero_setpoint: bool = False
    zero_setpoint_tolerance: float = 1e-6
    d_filter_alpha: float = 1.0  # 1.0 = no filtering; (0, 1) = EMA low-pass filter on dy/dt
    error_tolerance: float = 0.0
    derivative_tolerance: Optional[float] = None
    max_dt: float = 0.5  # Max allowed dt (s) before treating step as a fresh start

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class PIDState:
    """Snapshot of the latest PID computation for telemetry and diagnostics."""

    setpoint: float = 0.0
    measurement: float = 0.0
    error: float = 0.0
    derivative: float = 0.0
    integral: float = 0.0
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0
    ff_term: float = 0.0
    unclamped_output: float = 0.0
    output: float = 0.0
    saturated: bool = False
    dt: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


class PIDController:
    """
    Reusable closed-loop PID controller with velocity/static feedforward,
    derivative-on-measurement, anti-windup, output clamping, and angle wrapping.
    """

    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        kv: float = 0.0,
        ks: float = 0.0,
        *,
        out_min: float = -1000.0,
        out_max: float = 1000.0,
        i_zone: Optional[float] = None,
        max_integral: Optional[float] = None,
        angle_wrap: bool = False,
        zero_on_zero_setpoint: bool = False,
        zero_setpoint_tolerance: float = 1e-6,
        d_filter_alpha: float = 1.0,
        error_tolerance: float = 0.0,
        derivative_tolerance: Optional[float] = None,
        max_dt: float = 0.5,
        config: Optional[PIDConfig] = None,
    ) -> None:
        if config is not None:
            self.config = PIDConfig(**asdict(config))
        else:
            self.config = PIDConfig(
                kp=float(kp),
                ki=float(ki),
                kd=float(kd),
                kv=float(kv),
                ks=float(ks),
                out_min=float(out_min),
                out_max=float(out_max),
                i_zone=float(i_zone) if i_zone is not None else None,
                max_integral=float(max_integral) if max_integral is not None else None,
                angle_wrap=bool(angle_wrap),
                zero_on_zero_setpoint=bool(zero_on_zero_setpoint),
                zero_setpoint_tolerance=float(zero_setpoint_tolerance),
                d_filter_alpha=float(d_filter_alpha),
                error_tolerance=float(error_tolerance),
                derivative_tolerance=(
                    float(derivative_tolerance) if derivative_tolerance is not None else None
                ),
                max_dt=float(max_dt),
            )

        self._validate_config()

        self._integral: float = 0.0
        self._prev_measurement: Optional[float] = None
        self._filtered_derivative: float = 0.0
        self._last_time: Optional[float] = None
        self._has_sample: bool = False
        self._state: PIDState = PIDState()

    def _validate_config(self) -> None:
        if self.config.out_min > self.config.out_max:
            raise ValueError(
                f"out_min ({self.config.out_min}) cannot exceed out_max ({self.config.out_max})"
            )
        if not (0.0 < self.config.d_filter_alpha <= 1.0):
            raise ValueError(
                f"d_filter_alpha must be in (0.0, 1.0], got {self.config.d_filter_alpha}"
            )
        if self.config.i_zone is not None and self.config.i_zone < 0.0:
            raise ValueError(f"i_zone must be non-negative, got {self.config.i_zone}")
        if self.config.max_integral is not None and self.config.max_integral < 0.0:
            raise ValueError(
                f"max_integral must be non-negative, got {self.config.max_integral}"
            )

    @property
    def state(self) -> PIDState:
        """Returns the latest PIDState snapshot."""
        return self._state

    @property
    def integral(self) -> float:
        """Returns the current accumulated integral value."""
        return self._integral

    def set_gains(
        self,
        kp: Optional[float] = None,
        ki: Optional[float] = None,
        kd: Optional[float] = None,
        kv: Optional[float] = None,
        ks: Optional[float] = None,
    ) -> None:
        """Updates PID and feedforward gains at runtime."""
        if kp is not None:
            self.config.kp = float(kp)
        if ki is not None:
            self.config.ki = float(ki)
        if kd is not None:
            self.config.kd = float(kd)
        if kv is not None:
            self.config.kv = float(kv)
        if ks is not None:
            self.config.ks = float(ks)

    def set_output_limits(self, out_min: float, out_max: float) -> None:
        """Updates output clamping limits [out_min, out_max]."""
        if out_min > out_max:
            raise ValueError(f"out_min ({out_min}) cannot exceed out_max ({out_max})")
        self.config.out_min = float(out_min)
        self.config.out_max = float(out_max)

    def reset(self) -> None:
        """
        Immediately clears integral windup, derivative history, and timing state.
        Called when target is 0 (if configured) or when a subsystem is stopped/disabled.
        """
        self._integral = 0.0
        self._prev_measurement = None
        self._filtered_derivative = 0.0
        self._last_time = None
        self._has_sample = False
        self._state = PIDState()

    def _compute_error(self, setpoint: float, measurement: float) -> float:
        raw_error = setpoint - measurement
        if self.config.angle_wrap:
            return wrap_angle_deg(raw_error)
        return raw_error

    def _compute_measurement_delta(self, measurement: float, prev_measurement: float) -> float:
        raw_delta = measurement - prev_measurement
        if self.config.angle_wrap:
            return wrap_angle_deg(raw_delta)
        return raw_delta

    def update(
        self,
        setpoint: float,
        measurement: float,
        dt: Optional[float] = None,
        now: Optional[float] = None,
    ) -> float:
        """
        Computes the next control output u(t) given current setpoint r(t) and measurement y(t).

        Args:
            setpoint: Target reference r(t).
            measurement: Current process variable y(t).
            dt: Optional explicit time step in seconds. If omitted, computed from monotonic clock.
            now: Optional monotonic timestamp in seconds (used when dt is omitted).

        Returns:
            Clamped control output u(t) in [out_min, out_max].
        """
        setpoint = float(setpoint)
        measurement = float(measurement)

        if not (math.isfinite(setpoint) and math.isfinite(measurement)):
            self.reset()
            return 0.0

        # Immediate reset and zero output when target is 0 (e.g., flywheel stop)
        if (
            self.config.zero_on_zero_setpoint
            and abs(setpoint) <= self.config.zero_setpoint_tolerance
        ):
            self.reset()
            self._state = PIDState(
                setpoint=setpoint,
                measurement=measurement,
                output=0.0,
            )
            return 0.0

        # Resolve time step dt
        curr_time = float(now) if now is not None else time.monotonic()
        if dt is None:
            if self._last_time is None:
                step_dt = 0.0
            else:
                step_dt = curr_time - self._last_time
                if step_dt < 0.0 or step_dt > self.config.max_dt:
                    # Guard against clock jumps or long pauses between control phases
                    step_dt = 0.0
                    self._prev_measurement = None
                    self._filtered_derivative = 0.0
        else:
            step_dt = float(dt)
            if step_dt < 0.0:
                raise ValueError(f"dt must be non-negative, got {step_dt}")
        self._last_time = curr_time

        # 1. Error calculation (with optional 0/360 degree wrapping)
        error = self._compute_error(setpoint, measurement)

        # 2. Derivative-on-measurement: -K_d * dy(t)/dt
        if self._prev_measurement is not None and step_dt > 1e-9:
            dy = self._compute_measurement_delta(measurement, self._prev_measurement)
            raw_derivative = dy / step_dt
            alpha = self.config.d_filter_alpha
            self._filtered_derivative = (
                alpha * raw_derivative + (1.0 - alpha) * self._filtered_derivative
            )
        else:
            self._filtered_derivative = 0.0
        self._prev_measurement = measurement
        derivative = self._filtered_derivative

        # 3. Feedforward, Proportional, and Derivative terms
        cfg = self.config
        ff_term = cfg.kv * setpoint + cfg.ks * _sign(setpoint, cfg.zero_setpoint_tolerance)
        p_term = cfg.kp * error
        d_term = -cfg.kd * derivative

        # 4. Integral term with three-layer anti-windup
        if cfg.ki == 0.0:
            self._integral = 0.0
        elif cfg.i_zone is not None and abs(error) > cfg.i_zone:
            # Outside active i_zone: clear integral to prevent windup during large slews
            self._integral = 0.0
        elif step_dt > 0.0:
            candidate_integral = self._integral + error * step_dt
            if cfg.max_integral is not None:
                candidate_integral = max(
                    -cfg.max_integral, min(cfg.max_integral, candidate_integral)
                )

            candidate_i_term = cfg.ki * candidate_integral
            candidate_unclamped = ff_term + p_term + candidate_i_term + d_term

            # Integrator freeze on saturation: only integrate if not driving deeper into saturation
            saturating_high = candidate_unclamped > cfg.out_max and (cfg.ki * error) > 0.0
            saturating_low = candidate_unclamped < cfg.out_min and (cfg.ki * error) < 0.0

            if not (saturating_high or saturating_low):
                self._integral = candidate_integral

        i_term = cfg.ki * self._integral

        # 5. Total output and clamping
        unclamped_output = ff_term + p_term + i_term + d_term
        output = max(cfg.out_min, min(cfg.out_max, unclamped_output))
        saturated = output != unclamped_output

        self._has_sample = True
        self._state = PIDState(
            setpoint=setpoint,
            measurement=measurement,
            error=error,
            derivative=derivative,
            integral=self._integral,
            p_term=p_term,
            i_term=i_term,
            d_term=d_term,
            ff_term=ff_term,
            unclamped_output=unclamped_output,
            output=output,
            saturated=saturated,
            dt=step_dt,
        )
        return output

    def at_setpoint(
        self,
        error_tolerance: Optional[float] = None,
        derivative_tolerance: Optional[float] = None,
    ) -> bool:
        """
        Returns True if a sample has been processed and the current error (and optionally
        measurement derivative) is within the specified tolerance bounds.
        """
        if not self._has_sample:
            return False

        err_tol = (
            float(error_tolerance)
            if error_tolerance is not None
            else self.config.error_tolerance
        )
        if abs(self._state.error) > err_tol:
            return False

        deriv_tol = (
            float(derivative_tolerance)
            if derivative_tolerance is not None
            else self.config.derivative_tolerance
        )
        if deriv_tol is not None and abs(self._state.derivative) > deriv_tol:
            return False

        return True

