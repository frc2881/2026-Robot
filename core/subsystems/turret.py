import math
from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
import wpimath
from wpimath import units
from wpimath.geometry import Pose2d, Transform2d, Rotation2d, Pose3d, Rotation3d
from lib import logger, utils
from lib.components.relative_position_control_module import RelativePositionControlModule
import core.constants as constants

class Turret(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Turret

    self._turretTransform = Transform2d(
      constants.Subsystems.Launcher.LAUNCHER_TRANSFORM.translation().toTranslation2d(), 
      constants.Subsystems.Launcher.LAUNCHER_TRANSFORM.rotation().toRotation2d()
    )

    self._turret = RelativePositionControlModule(self._constants.TURRET_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def setHeading(self, heading: units.degrees) -> Command:
    return self.runEnd(
      lambda: self._turret.setPosition(heading),
      lambda: self.reset()
    )
  
  def getHeading(self) -> units.degrees:
    return self._turret.getPosition()

  def alignToTargetHeading(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose2d]) -> Command:
    return self.runEnd(
      lambda: self._turret.setPosition(self._getTurretTargetHeading(getRobotPose().transformBy(self._turretTransform), getTargetPose())),
      lambda: self.reset()
    )
  
  def _getTurretTargetHeading(turretPose: Pose2d, targetPose: Pose2d) -> units.degrees:
    return wpimath.inputModulus(units.radiansToDegrees(math.atan2(targetPose.Y() - turretPose.Y(), targetPose.X() - turretPose.X()) - (turretPose.rotation().radians())), -10, 350)

  def isAlignedToTargetHeading(self) -> bool:
    return self._turret.isAtTargetPosition()
  
  def isAtSoftLimit(self) -> bool:
    return self._turret.isAtSoftLimit()

  def resetToHome(self) -> Command:
    return self._turret.resetToHome(self).withName("Turret:ResetToHome")

  def isHomed(self) -> bool:
    return self._turret.isHomed()

  def reset(self) -> None:
    self._turret.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Turret/IsAlignedToTargetHeading", self.isAlignedToTargetHeading())
    SmartDashboard.putNumber("Robot/Turret/Heading", self.getHeading())