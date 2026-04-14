from typing import TYPE_CHECKING, Callable, Optional
import math
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d, Twist2d, Pose3d
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

    self._targetLaunchInfos: dict[Target, TargetLaunchInfo] = {
      Target.Hub: TargetLaunchInfo(),
      Target.ShuttleLeft: TargetLaunchInfo(),
      Target.ShuttleRight: TargetLaunchInfo()
    }

    self._prvx: units.meters_per_second = 0
    self._prvy: units.meters_per_second = 0
    self._prvo: units.radians_per_second = 0

    self._distanceMin = self._targetLaunchDistances[0]
    self._distanceMax = self._targetLaunchDistances[-1]
    self._headingMin = constants.Subsystems.Turret.ROTATION_RANGE.min
    self._headingMax = constants.Subsystems.Turret.ROTATION_RANGE.max

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
    robotPose = self._getRobotPose()
    rrv = self._getChassisSpeeds()
    frv = ChassisSpeeds.fromRobotRelativeSpeeds(rrv, robotPose.rotation())
    llc = constants.Services.Targeting.LOCALIZATION_LATENCY_COMPENSATION
    prp = robotPose.exp(
      Twist2d(
        rrv.vx * llc + 0.5 * ((rrv.vx - self._prvx) / 0.02) * llc * llc, 
        rrv.vy * llc + 0.5 * ((rrv.vy - self._prvy) / 0.02) * llc * llc, 
        rrv.omega * llc + 0.5 * ((rrv.omega - self._prvo) / 0.02) * llc * llc
      )
    )
    self._prvx = rrv.vx
    self._prvy = rrv.vy
    self._prvo = rrv.omega
    h = prp.rotation().radians()
    cos = math.cos(h)
    sin = math.sin(h)
    ltx = constants.Subsystems.Launcher.LAUNCHER_TRANSFORM.X()
    lty = constants.Subsystems.Launcher.LAUNCHER_TRANSFORM.Y()
    lx = prp.X() + ltx * cos - lty * sin
    ly = prp.Y() + ltx * sin + lty * cos
    vx = frv.vx + (-(ltx * sin + lty * cos)) * frv.omega
    vy = frv.vy + (ltx * cos - lty * sin) * frv.omega
    for target in self._targetLaunchInfos:
      tt = self.getTargetPose(target).translation().toTranslation2d()
      rx = tt.X() - lx
      ry = tt.Y() - ly
      targetLaunchDistance: units.meters = math.hypot(rx, ry)
      targetLaunchSpeed: units.percent = 0
      targetLaunchRotation = Rotation2d()
      if math.hypot(vx, vy) < constants.Services.Targeting.VELOCITY_COMPENSATION_THRESHOLD:
        targetLaunchSpeed = utils.getInterpolatedValue(targetLaunchDistance, self._targetLaunchDistances, self._targetLaunchSpeeds)
        targetLaunchRotation = Rotation2d(tt.X() - lx, tt.Y() - ly)
      else:
        drag = constants.Services.Targeting.FUEL_LAUNCH_DRAG_COEFFICIENT
        tof = utils.getInterpolatedValue(targetLaunchDistance, self._targetLaunchDistances, self._targetLaunchTimes)
        dtof = (1.0 - math.exp(-drag * tof)) / drag
        targetLaunchDistance = math.hypot((rx - vx * dtof), (ry - vy * dtof))
        targetLaunchSpeed = utils.getInterpolatedValue(targetLaunchDistance, self._targetLaunchDistances, self._targetLaunchSpeeds)
        targetLaunchRotation = Rotation2d((tt.X() - vx * dtof) - lx, (tt.Y() - vy * dtof) - ly)
      targetLaunchHeading = utils.wrapAngle(targetLaunchRotation.degrees() - robotPose.rotation().degrees(), constants.Subsystems.Turret.WRAP_ANGLE_INPUT_RANGE)
      self._targetLaunchInfos[target].distance = targetLaunchDistance
      self._targetLaunchInfos[target].speed = targetLaunchSpeed
      self._targetLaunchInfos[target].heading = targetLaunchHeading
      self._targetLaunchInfos[target].isDistanceValid = utils.isValueWithinRange(targetLaunchDistance, self._distanceMin, self._distanceMax)
      self._targetLaunchInfos[target].isHeadingValid = utils.isValueWithinRange(targetLaunchHeading, self._headingMin, self._headingMax)

  def getTargetPose(self, target: Target) -> Pose3d:
    return self._targets.get(target, Pose3d(self._getRobotPose()))
  
  def getNearestTargetPose(self, targets: list[Target]) -> Pose3d:
    return Pose3d(self._getRobotPose()).nearest([self._targets[target] for target in self._targets if target in targets])
  
  def getLaunchHeading(self, target: Target) -> units.degrees:
    return self._targetLaunchInfos[target].heading

  def getLaunchSpeed(self, target: Target) -> units.percent:
    return self._targetLaunchInfos[target].speed
  
  def _updateTelemetry(self) -> None:
    for target in self._targetLaunchInfos:
      info = self._targetLaunchInfos[target]
      SmartDashboard.putNumber(f'Robot/Targeting/{ target.name }/Distance', info.distance)
      SmartDashboard.putNumber(f'Robot/Targeting/{ target.name }/Speed', info.speed)
      SmartDashboard.putNumber(f'Robot/Targeting/{ target.name }/Heading', info.heading)
      SmartDashboard.putBoolean(f'Robot/Targeting/{ target.name }/IsDistanceValid', info.isDistanceValid)
      SmartDashboard.putBoolean(f'Robot/Targeting/{ target.name }/IsHeadingValid', info.isHeadingValid)
