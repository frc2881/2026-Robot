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
      estimatedRobotPose = poseSensor.getEstimatedRobotPose()
      if estimatedRobotPose is not None:
        estimatedPose = estimatedRobotPose.estimatedPose.toPose2d()
        if utils.isPoseInBounds(estimatedPose, constants.Game.Field.BOUNDS):
          for target in estimatedRobotPose.targetsUsed:
            if self._isValidVisionTarget(target.getPoseAmbiguity(), target.getBestCameraToTarget().translation().norm()):
              hasValidVisionTarget = True
          if hasValidVisionTarget:
            self._poseEstimator.addVisionMeasurement(
              estimatedPose, 
              estimatedRobotPose.timestampSeconds,
              constants.Services.Localization.VISION_ESTIMATE_MULTI_TAG_STANDARD_DEVIATIONS
              if len(estimatedRobotPose.targetsUsed) > 1 else
              constants.Services.Localization.VISION_ESTIMATE_SINGLE_TAG_STANDARD_DEVIATIONS
            )     
    self._robotPose = self._poseEstimator.getEstimatedPosition()
    if hasValidVisionTarget:
      self._hasValidVisionTarget = True
      self._validVisionTargetBufferTimer.restart()
    else:
      if self._hasValidVisionTarget and self._validVisionTargetBufferTimer.hasElapsed(0.2):
        self._hasValidVisionTarget = False

  def _isValidVisionTarget(self, ambiguity: units.percent, distance: units.meters) -> bool:
    return (
      utils.isValueInRange(ambiguity, -1, constants.Services.Localization.VISION_MAX_POSE_AMBIGUITY) and
      distance <= constants.Services.Localization.VISION_MAX_TARGET_DISTANCE  
    )

  def hasValidVisionTarget(self) -> bool:
    return self._hasValidVisionTarget

  def getRobotPose(self) -> Pose2d:
    return self._robotPose

  def resetRobotPose(self, pose: Pose2d) -> None:
    self._poseEstimator.resetPose(pose)
  
  def _updateTelemetry(self) -> None:
    self._robotPosePublisher.set(self.getRobotPose())
    SmartDashboard.putBoolean("Robot/Localization/HasValidVisionTarget", self.hasValidVisionTarget())
