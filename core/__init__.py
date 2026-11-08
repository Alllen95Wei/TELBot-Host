"""
TELBot Core Package
Contains hardware communication, packet protocols, and field pose estimation.
"""

from core.comms import CmdType, Packet, PacketParser, PartType, UARTComm
from core.match import MatchManager, MatchMode
from core.pid import PIDConfig, PIDController, PIDState, wrap_angle_deg
from core.pose import PoseEstimate, PoseEstimator
from core.vision import TargetColumn, VisionManager, VisionTarget

__all__ = [
    "CmdType",
    "Packet",
    "PacketParser",
    "PartType",
    "UARTComm",
    "PIDConfig",
    "PIDController",
    "PIDState",
    "wrap_angle_deg",
    "PoseEstimate",
    "PoseEstimator",
    "MatchManager",
    "MatchMode",
    "TargetColumn",
    "VisionManager",
    "VisionTarget",
]

