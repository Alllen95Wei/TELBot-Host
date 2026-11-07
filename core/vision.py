"""
core/vision.py - Vision Target Tracking & Latching for TELBot

Tracks the active target of the special 1x3 grid row during Autonomous mode.
Receives YOLO detections from the operator's laptop:
{
  "timestamp": 0,
  "active_target": "left/center/right/none",
  "target_detected": false
}

Behavior:
  - Once set with target_detected=True, the active target latches and persists
    until reset (at the end of Auto or match reset), since only 1 field activates
    and stays active for the entire autonomous phase.
"""

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from config import SpecialGrid

logger = logging.getLogger("TELBot.Vision")


class TargetColumn(str, Enum):
    LEFT = "left"
    CENTER = "center"
    RIGHT = "right"
    NONE = "none"


@dataclass
class VisionTarget:
    active_target: str = TargetColumn.NONE.value
    target_detected: bool = False
    timestamp: float = 0.0
    latched: bool = False
    latched_at: float = 0.0
    target_x_cm: Optional[float] = None
    target_y_cm: Optional[float] = None

    def to_dict(self) -> dict:
        coords = None
        if self.target_x_cm is not None and self.target_y_cm is not None:
            coords = {"x_cm": round(self.target_x_cm, 1), "y_cm": round(self.target_y_cm, 1)}

        return {
            "active_target": self.active_target,
            "target_detected": self.target_detected,
            "latched": self.latched,
            "timestamp": self.timestamp,
            "latched_at": round(self.latched_at, 2) if self.latched_at > 0 else None,
            "target_coords": coords,
        }


class VisionManager:
    """
    Thread-safe container managing detection of the active 1x3 goal target.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._target = VisionTarget()

    @property
    def latest_target(self) -> VisionTarget:
        with self._lock:
            return self._target

    def update(
        self,
        active_target: str,
        target_detected: bool,
        timestamp: float = 0.0,
        force: bool = False,
    ) -> VisionTarget:
        """
        Updates target detection state.
        Once a valid target ('left', 'center', 'right') is detected, it latches
        and remains locked until explicitly reset.
        """
        target_str = str(active_target).lower().strip()
        if target_str not in ("left", "center", "right"):
            target_str = "none"

        with self._lock:
            # If already latched and not forcing an update, maintain the latched target
            if self._target.latched and not force:
                return self._target

            if target_detected and target_str != "none":
                coords = SpecialGrid.get_coords(target_str)
                self._target = VisionTarget(
                    active_target=target_str,
                    target_detected=True,
                    timestamp=timestamp,
                    latched=True,
                    latched_at=time.monotonic(),
                    target_x_cm=coords[0] if coords else None,
                    target_y_cm=coords[1] if coords else None,
                )
                logger.info(
                    f"Active target LATCHED: '{target_str.upper()}' at "
                    f"X={self._target.target_x_cm}cm, Y={self._target.target_y_cm}cm"
                )
            else:
                coords = SpecialGrid.get_coords(target_str) if target_str != "none" else None
                self._target = VisionTarget(
                    active_target=target_str,
                    target_detected=target_detected,
                    timestamp=timestamp,
                    latched=False,
                    latched_at=0.0,
                    target_x_cm=coords[0] if coords else None,
                    target_y_cm=coords[1] if coords else None,
                )

            return self._target

    def reset(self) -> VisionTarget:
        """Clears latched target back to 'none'."""
        with self._lock:
            self._target = VisionTarget()
            logger.info("Active vision target reset to NONE.")
            return self._target

    def get_status(self) -> dict:
        """Returns JSON-serializable status dictionary."""
        with self._lock:
            return self._target.to_dict()

