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
  FollowerModuleConfig,
  FollowerModuleConstants,
  RobotType,
  Alliance, 
  PID,
  Range,
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
  VelocityControlModuleConfig,
  VelocityControlModuleConstants,
  ButtonControllerConfig,
  PoseSensorConfig,
  ObjectSensorConfig,
  DistanceSensorConfig,
  BinarySensorConfig
)
from core.classes import Target, TargetLaunchSpeed
import lib.constants

_aprilTagFieldLayout = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2026-rebuilt-andymark.json')

class Subsystems:
  class Drive:
    BUMPER_LENGTH: units.meters = units.inchesToMeters(31.0)
    BUMPER_WIDTH: units.meters = units.inchesToMeters(37.0)
    WHEEL_BASE: units.meters = units.inchesToMeters(20.5)
    TRACK_WIDTH: units.meters = units.inchesToMeters(26.5)

    _drivingMotorModel = MotorModel.NEOVortex
    _swerveModuleGearKit = SwerveModuleGearKit.High
    
    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorFreeSpeed = lib.constants.Motors.MOTOR_FREE_SPEEDS[_drivingMotorModel],
      drivingMotorReduction = lib.constants.Drive.SWERVE_MODULE_GEAR_RATIOS[_swerveModuleGearKit],
      drivingMotorCurrentLimit = 80,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1.0, 0, 0),
      turningMotorAbsoluteEncoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2
    )

    SWERVE_MODULE_CONFIGS: tuple[SwerveModuleConfig, ...] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -90, Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, 180, Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, 90, Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), _swerveModuleConstants)
    )

    DRIVE_KINEMATICS = SwerveDrive4Kinematics(*(c.translation for c in SWERVE_MODULE_CONFIGS))

    TRANSLATION_MAX_VELOCITY: units.meters_per_second = lib.constants.Drive.SWERVE_MODULE_FREE_SPEEDS[_drivingMotorModel][_swerveModuleGearKit] * 1.0
    ROTATION_MAX_VELOCITY: units.degrees_per_second = 720.0

    TARGET_POSE_ALIGNMENT_CONSTANTS = PoseAlignmentConstants(
      translationPID = PID(2.0, 0, 0),
      translationMaxVelocity = 3.0,
      translationPositionTolerance = 0.025,
      rotationPID = PID(3.0, 0, 0),
      rotationMaxVelocity = 720.0,
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

    PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()
    PATHPLANNER_CONTROLLER = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    INPUT_LIMIT_DEMO: units.percent = 0.5
    INPUT_RATE_LIMIT_DEMO: units.percent = 0.5

  class Intake:
    ARM_CONFIG = RelativePositionControlModuleConfig("Intake/Arm", 18, False, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 40,
      motorPID = PID(0.2, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionCruiseVelocity = 12000.0,
      motorMotionMaxAcceleration = 6000.0,
      motorMotionAllowedProfileError = 0.5,
      motorRelativeEncoderPositionConversionFactor = 1.0,
      motorSoftLimitForward = 7.2,
      motorSoftLimitReverse = 0.0,
      motorHomingSpeed = 0.1,
      motorHomedPosition = 0.0
    ))
        
    ROLLERS_CONFIG = VelocityControlModuleConfig("Intake/Rollers", 17, False, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80, 
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 6000.0,
    ))

    ARM_INTAKE_POSITION: float = 6.4
    ARM_DEFAULT_HOLD_SPEED: units.percent = 0.05
    ARM_INTAKE_HOLD_SPEED: units.percent = 0.6
    ROLLERS_SPEED: units.percent = 0.9

  class Hopper:
    INDEXER_CONFIG = VelocityControlModuleConfig("Hopper/Indexer", 14, True, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 40,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 6000.0
    ))

    ROLLER_CONFIG = VelocityControlModuleConfig("Hopper/Roller", 20, True, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 40,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEO]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 6000.0
    ))

    FEEDER_CONFIG = VelocityControlModuleConfig("Hopper/Feeder", 15, False, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 40,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEO]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 6000.0
    ))

    ELEVATOR_CONFIG = VelocityControlModuleConfig("Hopper/Elevator", 16, True, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 6000.0
    ))

    INDEXER_SPEED: units.percent = 0.9
    ROLLER_SPEED: units.percent = 0.9
    FEEDER_SPEED: units.percent = 0.9
    ELEVATOR_SPEED: units.percent = 0.9
    AGITATE_SPEED_RATIO: units.percent = 0.5

  class Turret:
    TURRET_CONFIG = RelativePositionControlModuleConfig("Turret", 13, False, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorRelativeEncoderPositionConversionFactor = 360.0 / 21.0,
      motorPID = PID(0.025, 0, 0.0025),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains  = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionCruiseVelocity = 120000.0,
      motorMotionMaxAcceleration = 60000.0,
      motorMotionAllowedProfileError = 0.25,
      motorSoftLimitForward = 320.0,
      motorSoftLimitReverse = -10.0,
      motorHomingSpeed = 0.1,
      motorHomedPosition = -19.0
    ))

    WRAP_ANGLE_INPUT_RANGE = Range(-10, 350)

  class Launcher:
    LAUNCHER_CONFIG = VelocityControlModuleConfig("Launcher/Leader", 10, True, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 6000.0
    ))

    LAUNCHER_FOLLOWER_CONFIG = FollowerModuleConfig("Launcher/Follower", 11, 10, True, FollowerModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80
    ))

    ACCELERATOR_CONFIG = VelocityControlModuleConfig("Launcher/Accelerator", 12, False, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 6000.0
    ))

    ACCELERATOR_SPEED_RATIO: units.percent = 1.0
    LAUNCHER_TRANSFORM = Transform3d(units.inchesToMeters(-4.75), units.inchesToMeters(7.875), units.inchesToMeters(25.3375), Rotation3d())

    TARGET_SPEEDS: tuple[TargetLaunchSpeed, ...] = (
      TargetLaunchSpeed(1.0, 0.38),
      TargetLaunchSpeed(2.0, 0.40),
      TargetLaunchSpeed(2.4, 0.44),
      TargetLaunchSpeed(3.3, 0.48),
      TargetLaunchSpeed(4.6, 0.54),
      TargetLaunchSpeed(5.4, 0.58),
      TargetLaunchSpeed(6.0, 0.62)
    )

  class Climber:
    CLIMB_CONFIG = RelativePositionControlModuleConfig("Climb", 19, True, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorPID = PID(1.0, 0, 0),
      motorOutputRange = Range(-0.8, 0.95),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionCruiseVelocity = 6000.0,
      motorMotionMaxAcceleration = 3000.0,
      motorMotionAllowedProfileError = 0.5,
      motorRelativeEncoderPositionConversionFactor = 1.0,
      motorSoftLimitForward = 100.0,
      motorSoftLimitReverse = 0.0,
      motorHomingSpeed = 0.1,
      motorHomedPosition = 0.0
    ))

    CLIMBER_UP_POSITION = 0.0
    CLIMBER_DOWN_POSITION = 90.0

class Services:
  class Localization:
    VISION_MAX_POSE_AMBIGUITY: units.percent = 0.2
    VISION_ESTIMATE_MULTI_TAG_STANDARD_DEVIATIONS: tuple[units.meters, units.meters, units.radians] = (0.05, 0.05, units.degreesToRadians(5.0))
    VISION_ESTIMATE_SINGLE_TAG_STANDARD_DEVIATIONS: tuple[units.meters, units.meters, units.radians] = (0.3, 0.3, units.degreesToRadians(15.0))

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kUSB1
  
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        name = "FrontLeft",
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(9.75), y = units.inchesToMeters(12.75), z = units.inchesToMeters(10.25)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-25.0), yaw = units.degreesToRadians(50.0))
        ),
        stream = "http://10.28.81.6:1186/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
      PoseSensorConfig(
        name = "FrontRight",
        transform = Transform3d(
        Translation3d(x = units.inchesToMeters(9.75), y = units.inchesToMeters(-12.75), z = units.inchesToMeters(12.0)),
        Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-25.0), yaw = units.degreesToRadians(-50.0))
      ),
        stream = "http://10.28.81.7:1184/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
      PoseSensorConfig(
        name = "RearLeft",
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(-9.75), y = units.inchesToMeters(12.75), z = units.inchesToMeters(10.25)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-30.0), yaw = units.degreesToRadians(135.0))
        ),
        stream = "http://10.28.81.6:1184/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
      PoseSensorConfig(
        name = "RearRight",
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(-9.75), y = units.inchesToMeters(-12.75), z = units.inchesToMeters(10.25)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-30.0), yaw = units.degreesToRadians(-135.0))
        ),
        stream = "http://10.28.81.7:1182/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      )
    )

  class Object:
    OBJECT_SENSOR_CONFIG = ObjectSensorConfig(
      name = "Fuel", 
      transform = Transform3d(Translation3d(units.inchesToMeters(-11.0), units.inchesToMeters(-4.0), units.inchesToMeters(24.0)), Rotation3d(0, units.degreesToRadians(25.0), units.degreesToRadians(0))),
      stream = "http://10.28.81.6:1182/?action=stream",
      objectHeight = units.inchesToMeters(5.71)
    )

  class Distance:
    HOPPER_SENSOR_CONFIG = DistanceSensorConfig(
      name = "Hopper", 
      channel = 0, 
      pulseWidthConversionFactor = 2.0, 
      minTargetDistance = 0, 
      maxTargetDistance = 800 # TODO: tune max target distance for inside hopper walls
    ) 
    INDEXER_SENSOR_CONFIG = DistanceSensorConfig(
      name = "Indexer",
      channel = 1, 
      pulseWidthConversionFactor = 0.75, 
      minTargetDistance = 0, 
      maxTargetDistance = 500 # TODO: tune max target distance for inside hopper walls
    )

  class Binary:
    FEEDER_SENSOR_CONFIG = BinarySensorConfig(name = "Feeder", channel = 4)
    ELEVATOR_SENSOR_CONFIG = BinarySensorConfig(name = "Elevator", channel = 2)

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1182/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1
  HOMING_BUTTON_CONFIG = ButtonControllerConfig(name = "Homing", channel = 3)

class Game:
  class Robot:
    TYPE = RobotType.Competition
    NAME: str = "Rosetta Stone"

  class Commands:
    AUTO_ALIGNMENT_TIMEOUT: units.seconds = 2.0
    LAUNCHER_READY_TIMEOUT: units.seconds = 1.5

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    BOUNDS = (Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
        Alliance.Red: {
          Target.Hub: Pose3d(11.917, 4.038, 1.263, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.Shuttle: Pose3d(15.040, 6.570, 0.0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.TrenchLeft: Pose3d(13.040, 1.170, 0.0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.TrenchRight: Pose3d(13.040, 6.895, 0.0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.TowerLeft: Pose3d(14.965, 3.770, 0.0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.TowerRight: Pose3d(14.965, 4.770, 0.0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.CornerLeft: Pose3d(16.040, 0.420, 0.0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.Outpost: Pose3d(16.040, 7.395, 0.0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.ClimbLeft: Pose3d(15.435, 3.220, 0.0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.ClimbRight: Pose3d(15.520, 5.450, 0.0, Rotation3d(Rotation2d.fromDegrees(180)))
        },
        Alliance.Blue: {
          Target.Hub: Pose3d(4.623, 4.032, 1.263, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.Shuttle: Pose3d(1.500, 1.500, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          Target.TrenchLeft: Pose3d(3.500, 6.900, 0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.TrenchRight: Pose3d(3.500, 1.175, 0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.TowerLeft: Pose3d(1.575, 4.300, 0, Rotation3d(Rotation2d.fromDegrees(90))),
          Target.TowerRight: Pose3d(1.575, 3.300, 0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.CornerLeft: Pose3d(0.500, 7.650, 0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.Outpost: Pose3d(0.500, 0.675, 0, Rotation3d(Rotation2d.fromDegrees(-90))),
          Target.ClimbLeft: Pose3d(1.105, 4.850, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.ClimbRight: Pose3d(1.020, 2.620, 0, Rotation3d(Rotation2d.fromDegrees(0))),
        }
      }
