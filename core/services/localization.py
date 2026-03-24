from typing import TYPE_CHECKING, Callable
from wpilib import SmartDashboard, Timer
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d
if TYPE_CHECKING: from wpimath.kinematics import SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from ntcore import NetworkTableInstance
from lib import logger, utils
if TYPE_CHECKING: from lib.sensors.pose import PoseSensor
import core.constants as constants

class Localization():
  def __init__(
      self,
      getGyroHeading: Callable[[], units.degrees],
      getDriveModulePositions: Callable[[], tuple[SwerveModulePosition, ...]],
      poseSensors: tuple[PoseSensor, ...]
    ) -> None:
    self._getGyroHeading = getGyroHeading
    self._getDriveModulePositions = getDriveModulePositions
    self._poseSensors = poseSensors

    self._poseEstimator = SwerveDrive4PoseEstimator(
      constants.Subsystems.Drive.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(self._getGyroHeading()),
      self._getDriveModulePositions(),
      Pose2d()
    )
    
    self._robotPose = Pose2d()
    self._hasValidVisionTarget: bool = False
    self._validVisionTargetBufferTimer = Timer()
    
    self._robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/Robot/Localization/Pose", Pose2d).publish()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateRobotPose()
    self._updateTelemetry()

  def _updateRobotPose(self) -> None:
    self._poseEstimator.update(Rotation2d.fromDegrees(self._getGyroHeading()), self._getDriveModulePositions())
    hasValidVisionTarget = False
    for poseSensor in self._poseSensors:
      poseSensorResult = poseSensor.getLatestResult()
      if (
        poseSensorResult is not None and
        utils.isPoseInBounds(poseSensorResult.estimatedPose.toPose2d(), constants.Game.Field.BOUNDS) and
        poseSensorResult.bestTargetDistance <= constants.Services.Localization.VISION_MAX_TARGET_DISTANCE and
        poseSensorResult.bestTargetAmbiguity <= constants.Services.Localization.VISION_MAX_TARGET_AMBIGUITY and
        poseSensorResult.bestTargetReprojectionError <= constants.Services.Localization.VISION_MAX_TARGET_REPROJECTION_ERROR
      ):
        stdDevXY = constants.Services.Localization.VISION_STDDEV_XY_COEFF * poseSensorResult.bestTargetDistance
        stdDevZ = constants.Services.Localization.VISION_STDDEV_Z_COEFF * poseSensorResult.bestTargetDistance
        if poseSensorResult.bestTargetReprojectionError >= 0:
          stdDevXY *= constants.Services.Localization.VISION_STDDEV_TARGET_REPROJECTION_ERROR_SCALE_FACTOR * poseSensorResult.bestTargetReprojectionError
          stdDevZ *= constants.Services.Localization.VISION_STDDEV_TARGET_REPROJECTION_ERROR_SCALE_FACTOR * poseSensorResult.bestTargetReprojectionError
        else: 
          stdDevXY *= constants.Services.Localization.VISION_STDDEV_TARGET_AMBIGUITY_SCALE_FACTOR * poseSensorResult.bestTargetAmbiguity
          stdDevZ = float("inf")
        self._poseEstimator.addVisionMeasurement(
          poseSensorResult.estimatedPose.toPose2d(), 
          poseSensorResult.timestamp,
          (stdDevXY, stdDevXY, stdDevZ)
        )
        hasValidVisionTarget = True
    self._robotPose = self._poseEstimator.getEstimatedPosition()
    if hasValidVisionTarget:
      self._hasValidVisionTarget = True
      self._validVisionTargetBufferTimer.restart()
    else:
      if self._hasValidVisionTarget and self._validVisionTargetBufferTimer.hasElapsed(0.1):
        self._hasValidVisionTarget = False

  def getRobotPose(self) -> Pose2d:
    return self._robotPose

  def resetRobotPose(self, pose: Pose2d) -> None:
    self._poseEstimator.resetPose(pose)
  
  def hasValidVisionTarget(self) -> bool:
    return self._hasValidVisionTarget

  def _updateTelemetry(self) -> None:
    self._robotPosePublisher.set(self.getRobotPose())
    SmartDashboard.putBoolean("Robot/Localization/HasValidVisionTarget", self.hasValidVisionTarget())
