"""
TELBot Core Package
Contains hardware communication, packet protocols, and field pose estimation.
"""

from core.auto_routine import (
    AutoRoutineDefinition,
    AutoRoutineRunner,
    AutoRoutineRunnerState,
    AutoStep,
    AutoStepType,
    RoutineStore,
    step_auto_routine,
)
from core.comms import CmdType, Packet, PacketParser, PartType, UARTComm
from core.control import (
    ChassisControlState,
    ChassisPoseController,
    ConveyorControlState,
    ConveyorController,
    ShooterControlState,
    ShooterSpeedController,
    step_chassis_control,
    step_conveyor_control,
    step_shooter_control,
)
from core.esp32 import ESP32SensorReceiver, ESP32SensorSnapshot
from core.match import MatchManager, MatchMode
from core.pid import PIDConfig, PIDController, PIDState, wrap_angle_deg
from core.pose import PoseEstimate, PoseEstimator
from core.vision import TargetColumn, VisionManager, VisionTarget

__all__ = [
    "AutoRoutineDefinition",
    "AutoRoutineRunner",
    "AutoRoutineRunnerState",
    "AutoStep",
    "AutoStepType",
    "RoutineStore",
    "step_auto_routine",
    "CmdType",
    "Packet",
    "PacketParser",
    "PartType",
    "UARTComm",
    "ChassisControlState",
    "ChassisPoseController",
    "ConveyorControlState",
    "ConveyorController",
    "ShooterControlState",
    "ShooterSpeedController",
    "step_chassis_control",
    "step_conveyor_control",
    "step_shooter_control",
    "ESP32SensorReceiver",
    "ESP32SensorSnapshot",
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
