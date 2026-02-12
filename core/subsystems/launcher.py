from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d
from lib import logger, utils
from lib.classes import MotorIdleMode
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.follower_module import FollowerModule
import core.constants as constants

class Launcher(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

    self._targetDistances = tuple(t.distance for t in self._constants.TARGET_SPEEDS)
    self._targetSpeeds = tuple(t.speed for t in self._constants.TARGET_SPEEDS)

    self._launcher = VelocityControlModule(self._constants.LAUNCHER_CONFIG)
    self._launcherFollower = FollowerModule(self._constants.LAUNCHER_FOLLOWER_CONFIG)
    self._accelerator = VelocityControlModule(self._constants.ACCELERATOR_CONFIG)

    self._launcher.setIdleMode(MotorIdleMode.Coast)
    self._launcherFollower.setIdleMode(MotorIdleMode.Coast)
    self._accelerator.setIdleMode(MotorIdleMode.Coast)

  def periodic(self) -> None:
    self._updateTelemetry()

  def run_(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d]) -> Command:
    return self.runEnd(
      lambda: [
        targetDistance := utils.getTargetDistance(Pose3d(getRobotPose()).transformBy(self._constants.LAUNCHER_TRANSFORM), getTargetPose()),
        launcherSpeed := utils.getInterpolatedValue(targetDistance, self._targetDistances, self._targetSpeeds),
        self._launcher.setSpeed(launcherSpeed),
        self._accelerator.setSpeed(launcherSpeed * self._constants.ACCELERATOR_SPEED_RATIO)
      ],
      lambda: self.reset()
    ).withName("Launcher:Run")
  
  def isAtTargetSpeed(self) -> bool:
    return self._launcher.isAtTargetSpeed() and self._accelerator.isAtTargetSpeed()

  def isRunning(self) -> bool:
    return self._launcher.getSpeed() != 0 and self._accelerator.getSpeed != 0

  def reset(self) -> None:
    self._launcher.reset()
    self._accelerator.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Launcher/IsRunning", self.isRunning())
    SmartDashboard.putBoolean("Robot/Launcher/IsAtTargetSpeed", self.isAtTargetSpeed())
    SmartDashboard.putNumber("Robot/Launcher/Speed", self._launcher.getSpeed())