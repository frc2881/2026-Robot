from typing import TYPE_CHECKING, Callable, Optional
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d
if TYPE_CHECKING: from wpimath.kinematics import ChassisSpeeds
from lib import logger, utils
from lib.classes import Alliance
from core.classes import Target, TargetLaunchInfo
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

    self._targetLaunchInfos: dict[Target, TargetLaunchInfo] = {
      Target.Hub: TargetLaunchInfo(),
      Target.ShuttleLeft: TargetLaunchInfo(),
      Target.ShuttleRight: TargetLaunchInfo()
    }
    
    SmartDashboard.putNumber("Robot/Targeting/SpeedOverride", 0)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateTargets()
    self._updateTargetLaunchInfos()
    self._updateTelemetry()

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.Targets.TARGETS[self._alliance]

  def _updateTargetLaunchInfos(self) -> None:
    for target in self._targetLaunchInfos:
      launchPose = self._getLaunchPose()
      targetPose = self.getTargetPose(target)
      distance = utils.getTargetDistance(launchPose, targetPose)
      speed = utils.getInterpolatedValue(distance, self._targetLaunchDistances, self._targetLaunchSpeeds)
      heading = utils.wrapAngle(utils.getTargetHeading(launchPose, targetPose, isRobotRelative = True), constants.Subsystems.Turret.WRAP_ANGLE_INPUT_RANGE)
      self._targetLaunchInfos[target].distance = distance
      self._targetLaunchInfos[target].speed = speed
      self._targetLaunchInfos[target].heading = heading
      
  def getTargetPose(self, target: Target) -> Pose3d:
    return self._targets.get(target, Pose3d(self._getRobotPose()))
  
  def getNearestTargetPose(self, targets: list[Target]) -> Pose3d:
    return Pose3d(self._getRobotPose()).nearest([self._targets[target] for target in self._targets if target in targets])
  
  def _getLaunchPose(self) -> Pose3d:
    return Pose3d(self._getRobotPose()).transformBy(constants.Subsystems.Launcher.LAUNCHER_TRANSFORM)

  def getLaunchHeading(self, target: Target) -> units.degrees:
    return self._targetLaunchInfos[target].heading

  def getLaunchSpeed(self, target: Target) -> units.percent:
    speedOverride = SmartDashboard.getNumber("Robot/Targeting/SpeedOverride", 0)
    return speedOverride if speedOverride != 0 else self._targetLaunchInfos[target].speed
  
  def _updateTelemetry(self) -> None:
    for target in self._targetLaunchInfos:
      params = self._targetLaunchInfos[target]
      SmartDashboard.putNumber(f'Robot/Targeting/{ target.name }/Distance', params.distance)
      SmartDashboard.putNumber(f'Robot/Targeting/{ target.name }/Speed', params.speed)
      SmartDashboard.putNumber(f'Robot/Targeting/{ target.name }/Heading', params.heading)
