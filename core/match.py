"""
core/match.py - Match Phase & Mode Management for TELBot

Enforces game rules:
  1. Autonomous (1:00): Remote controller commands are locked out.
  2. Auto Timeout: Motors immediately stop and system enters AUTO_WAIT.
     Robot will NOT start Tele-Op automatically.
  3. Tele-Op (2:00): Operator must explicitly enable Tele-Op to take control.
  4. Match Over: Tele-Op expires at 2:00; all motors stop.
"""

import logging
import threading
import time
from enum import Enum
from typing import Callable, Optional

from config import Match as MatchConfig

logger = logging.getLogger("TELBot.Match")


class MatchMode(str, Enum):
    DISARMED = "disarmed"       # Safe idle state (motion locked)
    AUTO = "auto"               # Autonomous phase (1:00, remote control locked)
    AUTO_WAIT = "auto_wait"     # Auto complete, waiting for operator to enable Tele-Op
    TELEOP = "teleop"           # Tele-Op phase (2:00, remote control active)
    MATCH_OVER = "match_over"   # Full match ended (motion locked)
    PRACTICE = "practice"       # Free practice mode (no 2:00 timer lockout)


class MatchManager:
    """
    State machine managing game phases, countdown timers, controller gating,
    and operator transitions.
    """

    def __init__(
        self,
        auto_duration_s: float = MatchConfig.AUTO_DURATION_S,
        teleop_duration_s: float = MatchConfig.TELEOP_DURATION_S,
        lock_controller_in_auto: bool = MatchConfig.LOCK_CONTROLLER_IN_AUTO,
        on_stop_required: Optional[Callable[[], None]] = None,
    ) -> None:
        self.auto_duration_s = float(auto_duration_s)
        self.teleop_duration_s = float(teleop_duration_s)
        self.lock_controller_in_auto = lock_controller_in_auto
        self.on_stop_required = on_stop_required

        self._lock = threading.Lock()
        self._mode = MatchMode.DISARMED
        self._phase_start_time: float = 0.0
        self._phase_duration: float = 0.0

    @property
    def mode(self) -> MatchMode:
        with self._lock:
            return self._mode

    # --------------------------------------------------------------------------
    # Phase Transitions
    # --------------------------------------------------------------------------
    def start_auto(self) -> dict:
        """Starts the 1:00 Autonomous phase and locks out remote controllers."""
        with self._lock:
            self._mode = MatchMode.AUTO
            self._phase_start_time = time.monotonic()
            self._phase_duration = self.auto_duration_s
            logger.info(f"Match mode switched to AUTONOMOUS ({self.auto_duration_s:.0f}s). Remote controls locked.")
            return self._get_status_unlocked()

    def enable_teleop(self) -> dict:
        """
        Operator manual trigger to start the 2:00 Tele-Op phase.
        Can be activated after Auto completes (AUTO_WAIT), directly from DISARMED,
        or after MATCH_OVER.
        """
        with self._lock:
            self._mode = MatchMode.TELEOP
            self._phase_start_time = time.monotonic()
            self._phase_duration = self.teleop_duration_s
            logger.info(f"Match mode switched to TELE-OP ({self.teleop_duration_s:.0f}s). Driver control enabled.")
            return self._get_status_unlocked()

    def start_practice(self) -> dict:
        """Enables tele-op in practice mode without a match countdown timer."""
        with self._lock:
            self._mode = MatchMode.PRACTICE
            self._phase_start_time = time.monotonic()
            self._phase_duration = 0.0
            logger.info("Match mode switched to PRACTICE (Unlimited Tele-Op).")
            return self._get_status_unlocked()

    def disarm(self) -> dict:
        """Disarms robot and locks all motion."""
        with self._lock:
            self._mode = MatchMode.DISARMED
            self._phase_start_time = 0.0
            self._phase_duration = 0.0
            logger.info("Robot DISARMED.")
            if self.on_stop_required:
                self.on_stop_required()
            return self._get_status_unlocked()

    def reset_match(self) -> dict:
        """Resets all match timers and returns robot to DISARMED state."""
        return self.disarm()

    # --------------------------------------------------------------------------
    # Timer & Ticking
    # --------------------------------------------------------------------------
    def tick(self) -> MatchMode:
        """
        Evaluates active countdown timers.
        Transitions:
          - AUTO (expired)   -> AUTO_WAIT (stops motors, requires manual Tele-Op enable)
          - TELEOP (expired) -> MATCH_OVER (stops motors, match concluded)
        """
        stop_needed = False

        with self._lock:
            if self._mode == MatchMode.AUTO and self._phase_duration > 0:
                elapsed = time.monotonic() - self._phase_start_time
                if elapsed >= self._phase_duration:
                    self._mode = MatchMode.AUTO_WAIT
                    self._phase_start_time = time.monotonic()
                    self._phase_duration = 0.0
                    stop_needed = True
                    logger.warning(
                        "Autonomous phase expired (1:00). All motors stopped. "
                        "Robot will NOT start Tele-Op automatically; operator must manually enable Tele-Op."
                    )

            elif self._mode == MatchMode.TELEOP and self._phase_duration > 0:
                elapsed = time.monotonic() - self._phase_start_time
                if elapsed >= self._phase_duration:
                    self._mode = MatchMode.MATCH_OVER
                    self._phase_start_time = time.monotonic()
                    self._phase_duration = 0.0
                    stop_needed = True
                    logger.warning("Tele-Op phase expired (2:00). MATCH OVER! All motors stopped.")

        if stop_needed and self.on_stop_required:
            self.on_stop_required()

        with self._lock:
            return self._mode

    # --------------------------------------------------------------------------
    # Controller Access Gating
    # --------------------------------------------------------------------------
    def check_teleop_allowed(self) -> tuple[bool, str]:
        """
        Returns (is_allowed, reason).
        Checks if human operator commands (WebHID / WebSocket / REST drive) are accepted.
        """
        self.tick()
        with self._lock:
            if self._mode == MatchMode.TELEOP:
                return True, "Tele-Op active"
            if self._mode == MatchMode.PRACTICE:
                return True, "Practice mode active"
            if self._mode == MatchMode.AUTO:
                return False, "Autonomous phase active (remote controller locked)"
            if self._mode == MatchMode.AUTO_WAIT:
                return False, "Autonomous phase finished. Waiting for operator to enable Tele-Op"
            if self._mode == MatchMode.MATCH_OVER:
                return False, "Match is over. Reset match to drive"
            return False, "Robot is disarmed. Enable Tele-Op to drive"

    # --------------------------------------------------------------------------
    # Telemetry & Status
    # --------------------------------------------------------------------------
    def _get_status_unlocked(self) -> dict:
        now = time.monotonic()
        logger.debug("Phase dur.:%f / Phase start time:%f", self._phase_duration, self._phase_start_time)
        if self._phase_duration > 0 and self._phase_start_time > 0:
            elapsed = max(0.0, now - self._phase_start_time)
            remaining = max(0.0, self._phase_duration - elapsed)
        else:
            elapsed = now - self._phase_start_time
            remaining = 0.0

        teleop_allowed, reason = False, ""
        if self._mode == MatchMode.TELEOP:
            teleop_allowed, reason = True, "Tele-Op active"
        elif self._mode == MatchMode.PRACTICE:
            teleop_allowed, reason = True, "Practice mode active"
        elif self._mode == MatchMode.AUTO:
            teleop_allowed, reason = False, "Autonomous phase active (remote controller locked)"
        elif self._mode == MatchMode.AUTO_WAIT:
            teleop_allowed, reason = False, "Autonomous phase finished. Waiting for operator to enable Tele-Op"
        elif self._mode == MatchMode.MATCH_OVER:
            teleop_allowed, reason = False, "Match is over. Reset match to drive"
        else:
            teleop_allowed, reason = False, "Robot is disarmed. Enable Tele-Op to drive"

        return {
            "mode": self._mode.value,
            "phase_time_remaining": round(remaining, 1),
            "phase_time_elapsed": round(elapsed, 1),
            "phase_duration": round(self._phase_duration, 1),
            "auto_duration": self.auto_duration_s,
            "teleop_duration": self.teleop_duration_s,
            "teleop_allowed": teleop_allowed,
            "status_message": reason,
        }

    def get_status(self) -> dict:
        self.tick()
        with self._lock:
            return self._get_status_unlocked()

