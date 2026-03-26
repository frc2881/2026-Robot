from typing import TYPE_CHECKING, Callable, Optional
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d, Translation3d
from wpimath.kinematics import ChassisSpeeds
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
    self._targetLaunchVelocities = tuple(t.distance / t.time for t in constants.Services.Targeting.TARGET_LAUNCH_METRICS)

    self._targetLaunchInfos: dict[Target, TargetLaunchInfo] = {
      Target.Hub: TargetLaunchInfo(),
      Target.ShuttleLeft: TargetLaunchInfo(),
      Target.ShuttleRight: TargetLaunchInfo()
    }

    SmartDashboard.putNumber("Robot/Targeting/DistanceMin", self._targetLaunchDistances[0])
    SmartDashboard.putNumber("Robot/Targeting/DistanceMax", self._targetLaunchDistances[-1])
    SmartDashboard.putNumber("Robot/Targeting/HeadingMin", constants.Subsystems.Turret.TURRET_CONFIG.constants.motorSoftLimitReverse)
    SmartDashboard.putNumber("Robot/Targeting/HeadingMax", constants.Subsystems.Turret.TURRET_CONFIG.constants.motorSoftLimitForward)

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
    launcherPose = self._getLauncherPose()
    launcherRotation = launcherPose.rotation().toRotation2d()
    launcherVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(self._getChassisSpeeds(), launcherRotation)
    launcherVector = Translation3d(launcherVelocity.vx, launcherVelocity.vy, 0)
    launcherTranslation = launcherPose.translation() + (launcherVector * constants.Services.Targeting.LOCALIZATION_LATENCY_COMPENSATION)
    for target in self._targetLaunchInfos:
      targetTranslation = self.getTargetPose(target).translation() - launcherTranslation
      targetDistance = targetTranslation.norm()
      targetVector = (targetTranslation / targetDistance) * (targetDistance / utils.getInterpolatedValue(targetDistance, self._targetLaunchDistances, self._targetLaunchTimes)) - launcherVector
      targetEffectiveDistance = utils.getInterpolatedValue(targetVector.norm(), self._targetLaunchVelocities, self._targetLaunchDistances)
      self._targetLaunchInfos[target].distance = targetEffectiveDistance
      self._targetLaunchInfos[target].speed = utils.getInterpolatedValue(targetEffectiveDistance, self._targetLaunchDistances, self._targetLaunchSpeeds)
      self._targetLaunchInfos[target].heading = utils.wrapAngle(targetVector.toTranslation2d().angle().degrees() - launcherRotation.degrees(), constants.Subsystems.Turret.WRAP_ANGLE_INPUT_RANGE)

  def getTargetPose(self, target: Target) -> Pose3d:
    return self._targets.get(target, Pose3d(self._getRobotPose()))
  
  def getNearestTargetPose(self, targets: list[Target]) -> Pose3d:
    return Pose3d(self._getRobotPose()).nearest([self._targets[target] for target in self._targets if target in targets])
  
  def _getLauncherPose(self) -> Pose3d:
    return Pose3d(self._getRobotPose()).transformBy(constants.Subsystems.Launcher.LAUNCHER_TRANSFORM)

  def getLaunchHeading(self, target: Target) -> units.degrees:
    return self._targetLaunchInfos[target].heading

  def getLaunchSpeed(self, target: Target) -> units.percent:
    speedOverride = SmartDashboard.getNumber("Robot/Targeting/SpeedOverride", 0)
    return speedOverride if speedOverride != 0 else self._targetLaunchInfos[target].speed
  
  def _updateTelemetry(self) -> None:
    for target in self._targetLaunchInfos:
      info = self._targetLaunchInfos[target]
      SmartDashboard.putNumber(f'Robot/Targeting/{ target.name }/Distance', info.distance)
      SmartDashboard.putNumber(f'Robot/Targeting/{ target.name }/Speed', info.speed)
      SmartDashboard.putNumber(f'Robot/Targeting/{ target.name }/Heading', info.heading)
