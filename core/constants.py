import wpilib
from wpimath import units
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from rev import SparkLowLevel, AbsoluteEncoderConfig
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from lib import logger, utils
from lib.classes import (
  RobotType,
  Alliance, 
  PID, 
  MotorModel,
  SwerveModuleGearKit,
  SwerveModuleConstants, 
  SwerveModuleConfig, 
  SwerveModuleLocation, 
  TargetAlignmentConstants,
  RotationAlignmentConstants,
  PoseSensorConfig
)
from core.classes import Target
import lib.constants

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2026-rebuilt-andymark.json')

class Subsystems:
  class Drive:
    BUMPER_LENGTH: units.meters = units.inchesToMeters(19.5) #TODO: configure real value
    BUMPER_WIDTH: units.meters = units.inchesToMeters(19.5) #TODO: configure real value
    WHEEL_BASE: units.meters = units.inchesToMeters(9.125) #TODO: configure real value
    TRACK_WIDTH: units.meters = units.inchesToMeters(9.125) #TODO: configure real value
    
    TRANSLATION_MAX_VELOCITY: units.meters_per_second = 4.46 #TODO: configure real value
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 720.0 #TODO: configure real value

    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkMax, #TODO: configure real value
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorFreeSpeed = lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEO], #TODO: configure real value
      drivingMotorReduction = lib.constants.Drive.SWERVE_MODULE_GEAR_RATIOS[SwerveModuleGearKit.Medium], #TODO: configure real value
      drivingMotorCurrentLimit = 80,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1.0, 0, 0),
      turningMotorAbsoluteEncoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder #TODO: configure real value
    )

    SWERVE_MODULE_CONFIGS: tuple[SwerveModuleConfig, ...] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants)
    )

    DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.translation for c in SWERVE_MODULE_CONFIGS))

    PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    TARGET_ALIGNMENT_CONSTANTS = TargetAlignmentConstants(
      translationPID = PID(5.0, 0, 0),
      translationMaxVelocity = 1.5, #TODO: configure real value
      translationMaxAcceleration = 0.75, #TODO: configure real value
      translationPositionTolerance = 0.025, #TODO: configure real value
      translationVelocityTolerance = 0.1,
      rotationPID = PID(5.0, 0, 0),
      rotationMaxVelocity = 720.0, #TODO: configure real value
      rotationMaxAcceleration = 360.0, #TODO: configure real value
      rotationPositionTolerance = 0.5, #TODO: configure real value
      rotationVelocityTolerance = 1.0
    )

    TARGET_LOCK_CONSTANTS = RotationAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5 #TODO: configure real value
    )

    DRIFT_CORRECTION_CONSTANTS = RotationAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5 #TODO: configure real value
    )

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

class Services:
  class Localization:
    VISION_MAX_POSE_AMBIGUITY: units.percent = 0.2
    VISION_MAX_ESTIMATED_POSE_DELTA: units.meters = 1.0
    VISION_ESTIMATE_MULTI_TAG_STANDARD_DEVIATIONS: tuple[units.meters, units.meters, units.radians] = (0.05, 0.05, units.degreesToRadians(2.5))
    VISION_ESTIMATE_SINGLE_TAG_STANDARD_DEVIATIONS: tuple[units.meters, units.meters, units.radians] = (0.3, 0.3, units.degreesToRadians(15.0))

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kUSB1 #TODO: configure real value
  
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        name = "Front", #TODO: configure real value
        transform = Transform3d(Translation3d(0.107311, -0.050843, 0.264506), Rotation3d(0.001834, -0.569486, -0.027619)), #TODO: configure real value
        stream = "http://10.28.81.6:1182/?action=stream", #TODO: configure real value
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1182/?action=stream" #TODO: configure real value

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1

class Game:
  class Robot:
    TYPE = RobotType.Competition
    NAME: str = "2026-Robot" #TODO: configure real value

  class Commands:
    AUTO_TARGET_ALIGNMENT_TIMEOUT: units.seconds = 1.5 #TODO: configure real value

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    BOUNDS = (Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
        Alliance.Red: {
          Target.Hub: Pose3d(11.918, 4.032, 1.263, Rotation3d(Rotation2d.fromDegrees(180))), #TODO: configure real value
          Target.CornerLeft: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(90))), #TODO: configure real value
          Target.CornerRight: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(-90))), #TODO: configure real value
          Target.TowerRight: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(180))), #TODO: configure real value
          Target.TowerRight: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(180))), #TODO: configure real value
          Target.Outpost: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(180))), #TODO: configure real value
          Target.Depot: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(-90))) #TODO: configure real value
        },
        Alliance.Blue: {
          Target.Hub: Pose3d(4.623, 4.032, 1.263, Rotation3d(Rotation2d.fromDegrees(0))), #TODO: configure real value
          Target.CornerLeft: Pose3d(0.280, 7.790, 0, Rotation3d(Rotation2d.fromDegrees(-90))), #TODO: configure real value
          Target.CornerRight: Pose3d(0.280, 0.280, 0, Rotation3d(Rotation2d.fromDegrees(90))), #TODO: configure real value
          Target.TowerRight: Pose3d(1.385, 3.150, 0, Rotation3d(Rotation2d.fromDegrees(0))), #TODO: configure real value
          Target.TowerRight: Pose3d(1.385, 3.150, 0, Rotation3d(Rotation2d.fromDegrees(0))), #TODO: configure real value
          Target.Outpost: Pose3d(0.280, 0.650, 0, Rotation3d(Rotation2d.fromDegrees(0))), #TODO: configure real value
          Target.Depot: Pose3d(1.385, 5.125, 0, Rotation3d(Rotation2d.fromDegrees(90))) #TODO: configure real value
        }
      }
