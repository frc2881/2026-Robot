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
  Range,
  Value, 
  MotorModel,
  FeedForwardGains,
  SwerveModuleGearKit,
  SwerveModuleConstants, 
  SwerveModuleConfig, 
  SwerveModuleLocation, 
  PoseAlignmentConstants,
  HeadingAlignmentConstants,
  RelativePositionControlModuleConstants,
  RelativePositionControlModuleConfig,
  ButtonControllerConfig,
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
    
    TRANSLATION_MAX_VELOCITY: units.meters_per_second = 5.74
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 720.0

    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorFreeSpeed = lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex],
      drivingMotorReduction = lib.constants.Drive.SWERVE_MODULE_GEAR_RATIOS[SwerveModuleGearKit.High],
      drivingMotorCurrentLimit = 80,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1.0, 0, 0),
      turningMotorAbsoluteEncoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder #TODO: update if/when V2 installed
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

    TARGET_POSE_ALIGNMENT_CONSTANTS = PoseAlignmentConstants(
      translationPID = PID(2.0, 0, 0),
      translationMaxVelocity = 1.5, #TODO: configure real value
      translationMaxAcceleration = 0.75, #TODO: configure real value
      translationPositionTolerance = 0.025, #TODO: configure real value
      rotationPID = PID(2.0, 0, 0), #TODO: configure real value
      rotationMaxVelocity = 720.0, #TODO: configure real value
      rotationMaxAcceleration = 360.0, #TODO: configure real value
      rotationPositionTolerance = 0.5
    )

    TARGET_HEADING_ALIGNMENT_CONSTANTS = HeadingAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    DRIFT_CORRECTION_CONSTANTS = HeadingAlignmentConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationPositionTolerance = 0.5
    )

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

  class Turret:
    TURRET_CONFIG = RelativePositionControlModuleConfig("Turret", 13, False, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorRelativeEncoderPositionConversionFactor = 360.0 / 21.0,
      motorPID = PID(0.025, 0, 0.0025),
      motorOutputRange = Range(-0.5, 0.5), # TODO: update to 100% output range and adjust velocity/acceleration/PID values in relation
      motorFeedForwardGains  = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionCruiseVelocity = 40000.0,
      motorMotionMaxAcceleration = 80000.0,
      motorMotionAllowedProfileError = 0.25,
      motorSoftLimitForward = 170.0,
      motorSoftLimitReverse = -160.0,
      motorHomingSpeed = 0.1,
      motorHomedPosition = -170
    ))

    INPUT_LIMIT: units.percent = 0.5

  class Launcher:
    pass

  class Elevator:
    pass

  class Feeder:
    pass

  class Indexer:
    pass

  class Intake:
    pass

  class Climber:
    pass

class Services:
  class Localization:
    VISION_MAX_POSE_AMBIGUITY: units.percent = 0.2
    VISION_MAX_ESTIMATED_POSE_DELTA: units.meters = 3.0
    VISION_ESTIMATE_MULTI_TAG_STANDARD_DEVIATIONS: tuple[units.meters, units.meters, units.radians] = (0.05, 0.05, units.degreesToRadians(2.5))
    VISION_ESTIMATE_SINGLE_TAG_STANDARD_DEVIATIONS: tuple[units.meters, units.meters, units.radians] = (0.3, 0.3, units.degreesToRadians(15.0))

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kUSB1
  
  class Pose:
    # TODO: configure for installed cameras
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        name = "Front",
        transform = Transform3d(Translation3d(0.107311, -0.050843, 0.264506), Rotation3d(0.001834, -0.569486, -0.027619)),
        stream = "http://10.28.81.6:1182/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1182/?action=stream" #TODO: configure real value

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1
  HOMING_BUTTON_CONFIG = ButtonControllerConfig(name = "Homing", channel = 0)

class Game:
  class Robot:
    TYPE = RobotType.Competition
    NAME: str = "2026-Robot" #TODO: configure selected robot name

  class Commands:
    AUTO_ALIGNMENT_TIMEOUT: units.seconds = 1.5 #TODO: configure real value

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    BOUNDS = (Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    class Targets:
      # TODO: properly configure and tune all target poses (rough/temp placeholders for now)
      TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
        Alliance.Red: {
          Target.Hub: Pose3d(11.918, 4.032, 1.263, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.CornerLeft: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.CornerRight: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.TowerLeft: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.TowerRight: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.TrenchLeft: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.TrenchRight: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.Outpost: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.Depot: Pose3d(14.000, 4.032, 0, Rotation3d(Rotation2d.fromDegrees(-90)))
        },
        Alliance.Blue: {
          Target.Hub: Pose3d(4.623, 4.032, 1.263, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.CornerLeft: Pose3d(0.280, 7.790, 0, Rotation3d(Rotation2d.fromDegrees(-45))),
          Target.CornerRight: Pose3d(0.280, 0.280, 0, Rotation3d(Rotation2d.fromDegrees(45))),
          Target.TowerLeft: Pose3d(1.385, 4.350, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.TowerRight: Pose3d(1.385, 3.150, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.TrenchLeft: Pose3d(3.664, 6.535, 0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.TrenchRight: Pose3d(3.664, 1.600, 0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.Outpost: Pose3d(0.280, 0.650, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.Depot: Pose3d(0.350, 5.125, 0, Rotation3d(Rotation2d.fromDegrees(0)))
        }
      }
