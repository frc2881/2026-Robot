from typing import TYPE_CHECKING, Callable, Optional
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d, Transform2d
if TYPE_CHECKING: from wpimath.kinematics import ChassisSpeeds
from lib import logger, utils
from lib.classes import Alliance
from core.classes import Target
import core.constants as constants

class Targeting():
  def __init__(
      self,
      getRobotPose: Callable[[], Pose2d],
      getChassisSpeeds: Callable[[], ChassisSpeeds]
    ) -> None:
    self._getRobotPose = getRobotPose
    self._getChassisSpeeds = getChassisSpeeds

    self._alliance: Optional[Alliance] = None
    self._targets: dict[Target, Pose3d] = {}
    self._targetLaunchDistances = tuple(t.distance for t in constants.Services.Targeting.TARGET_LAUNCH_METRICS)
    self._targetLaunchSpeeds = tuple(t.speed for t in constants.Services.Targeting.TARGET_LAUNCH_METRICS)
    self._targetLaunchTimes = tuple(t.time for t in constants.Services.Targeting.TARGET_LAUNCH_METRICS)
    
    SmartDashboard.putNumber("Robot/Targeting/Launch/SpeedOverride", 0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTargets()
    self._updateTelemetry()

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.Targets.TARGETS[self._alliance]

  def getTargetPose(self, target: Target) -> Pose3d:
    return self._targets.get(target, Pose3d(self._getRobotPose()))
  
  def getNearestTargetPose(self, targets: list[Target]) -> Pose3d:
    return Pose3d(self._getRobotPose()).nearest([self._targets[target] for target in self._targets if target in targets])
  
  def getLaunchPose(self) -> Pose3d:
    return Pose3d(self._getRobotPose()).transformBy(constants.Subsystems.Launcher.LAUNCHER_TRANSFORM)

  def getLaunchHeading(self, target: Target) -> units.degrees:
    heading = utils.wrapAngle(utils.getTargetHeading(self.getLaunchPose(), self.getTargetPose(target), isRobotRelative = True), constants.Subsystems.Turret.WRAP_ANGLE_INPUT_RANGE)
    SmartDashboard.putNumber("Robot/Targeting/Launch/Heading", heading)
    return heading

  def getLaunchSpeed(self, target: Target) -> units.percent:
    distance = utils.getTargetDistance(self.getLaunchPose(), self.getTargetPose(target))
    speed = utils.getInterpolatedValue(distance, self._targetLaunchDistances, self._targetLaunchSpeeds)
    SmartDashboard.putNumber("Robot/Targeting/Launch/Distance", distance)
    SmartDashboard.putNumber("Robot/Targeting/Launch/Speed", speed)
    speedOverride = SmartDashboard.getNumber("Robot/Targeting/Launch/SpeedOverride", 0)
    if speedOverride != 0: speed = speedOverride
    return speed
  
  def _updateTelemetry(self) -> None:
    pass
