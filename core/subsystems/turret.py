from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d
from lib import logger, utils
from lib.components.relative_position_control_module import RelativePositionControlModule
import core.constants as constants

class Turret(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Turret

    self._hasInitialZeroReset: bool = False

    self._turret = RelativePositionControlModule(self._constants.TURRET_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def setSpeed(self, getInput: Callable[[], units.percent]) -> Command:
    return self.runEnd(
      lambda: self._turret.setSpeed(getInput() * self._constants.INPUT_LIMIT),
      lambda: self.reset()
    ).withName("Turret:SetSpeed")
  
  def setPosition(self, position: units.degrees) -> Command:
    return self.run(
      lambda: self._turret.setPosition(position)
    ).withName("Turret:SetPosition")
  
  def getPosition(self) -> units.degrees:
    return self._turret.getPosition()

  def isAtTargetPosition(self) -> bool:
    return self._turret.isAtTargetPosition()

  def alignToTargetHeading(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d]) -> Command:
    return self.startRun(
      lambda: self._initTargetHeadingAlignment(getTargetPose()),
      lambda: self._runTargetHeadingAlignment(getRobotPose())
    ).finallyDo(
      lambda end: self._endTargetHeadingAlignment()
    )
  
  def _initTargetHeadingAlignment(self, targetPose: Pose3d) -> None:
    self._targetPose = targetPose

  def _runTargetHeadingAlignment(self, robotPose: Pose2d) -> None:
    self._turret.setPosition(utils.wrapAngle(utils.getTargetHeading(robotPose, self._targetPose)))

  def _endTargetHeadingAlignment(self) -> None:
    self._targetPose = None

  def isAlignedToTargetHeading(self) -> bool:
    return self.isAtTargetPosition()
  
  def isAtSoftLimit(self) -> bool:
    return self._turret.isAtSoftLimit()

  def resetToHome(self) -> Command:
    return self._turret.resetToHome(self).withName("Turret:ResetToHome")

  def isHomed(self) -> bool:
    return self._turret.isHomed()

  def reset(self) -> None:
    self._turret.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Turret/IsAtTargetPosition", self.isAtTargetPosition())