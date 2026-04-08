import wpilib
from wpimath import units
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from rev import SparkLowLevel, AbsoluteEncoderConfig
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.path import FlippingUtil
from lib import logger, utils
from lib.classes import (
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
  FollowerModuleConfig,
  FollowerModuleConstants,
  ButtonControllerConfig,
  PoseSensorConfig,
  BinarySensorConfig,
  DistanceSensorConfig
)
from core.classes import Target, TargetLaunchMetric
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
      translationPID = PID(3.0, 0, 0),
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
    ARM_CONFIG = RelativePositionControlModuleConfig("Intake/Arm", 18, True, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorPID = PID(0.2, 0, 0),
      motorOutputRange = Range(-1.0, 0.3),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionCruiseVelocity = 48000.0,
      motorMotionMaxAcceleration = 24000.0,
      motorMotionAllowedProfileError = 0.5,
      motorRelativeEncoderPositionConversionFactor = 1.8 / 1.0, # TODO: update to use actual 45/1 reduction and calibrate closed loop values
      motorSoftLimitForward = 27.0,
      motorSoftLimitReverse = 0,
      motorHomingSpeed = 0.3,
      motorHomedPosition = 0
    ))
        
    ROLLERS_CONFIG = VelocityControlModuleConfig("Intake/Rollers", 17, True, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80, 
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 12000.0,
      motorMotionMaxAcceleration = 24000.0,
      motorVelocityConversionFactor = 2.0 / 1.0 
    ))

    ARM_RETRACT_POSITION: float = 0
    ARM_INTAKE_POSITION: float = 27.0
    ARM_AGITATE_RANGE = Range(0.1, 0.9)
    ARM_AGITATE_RANGE_MIN_RATIO: units.percent = 0.75
    ARM_AGITATE_TIME: units.seconds = 1.2
    ROLLERS_INTAKE_SPEED: units.percent = 1.0
    ROLLERS_AGITATE_SPEED: units.percent = 0.2

  class Hopper:
    INDEXER_CONFIG = VelocityControlModuleConfig("Hopper/Indexer", 14, True, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0, 
      motorMotionMaxAcceleration = 6000.0,
      motorVelocityConversionFactor = 3.0 / 1.0
    ))

    ELEVATOR_CONFIG = VelocityControlModuleConfig("Hopper/Elevator", 16, False, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 12000.0,
      motorMotionMaxAcceleration = 12000.0,
      motorVelocityConversionFactor = 3.0 / 1.0
    ))

    INDEXER_SPEED: units.percent = 1.0
    INDEXER_REVERSE_SPEED: units.percent = 0.5
    ELEVATOR_SPEED: units.percent = 1.0
    ELEVATOR_REVERSE_SPEED: units.percent = 0.5

  class Turret:
    TURRET_CONFIG = RelativePositionControlModuleConfig("Turret", 13, False, RelativePositionControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorRelativeEncoderPositionConversionFactor = 360.0 / 21.0,
      motorPID = PID(0.02, 0, 0.002),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains  = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionCruiseVelocity = 40000.0, 
      motorMotionMaxAcceleration = 80000.0,
      motorMotionAllowedProfileError = 0.25,
      motorSoftLimitForward = 300.0,
      motorSoftLimitReverse = -10.0,
      motorHomingSpeed = 0.1,
      motorHomedPosition = -19.45
    ))

    WRAP_ANGLE_INPUT_RANGE = Range(-10, 350)

  class Launcher:
    LAUNCHER_LEADER_CONFIG = VelocityControlModuleConfig("Launcher/Leader", 10, True, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 12000.0,
      motorVelocityConversionFactor = 1.0
    ))

    LAUNCHER_FOLLOWER_CONFIG = FollowerModuleConfig("Launcher/Follower", 11, 10, True, FollowerModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = LAUNCHER_LEADER_CONFIG.constants.motorCurrentLimit
    ))

    LAUNCHER_ACCELERATOR_CONFIG = VelocityControlModuleConfig("Launcher/Accelerator", 12, False, VelocityControlModuleConstants(
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorPID = PID(0.0001, 0, 0),
      motorOutputRange = Range(-1.0, 1.0),
      motorFeedForwardGains = FeedForwardGains(velocity = 12.0 / lib.constants.Motors.MOTOR_FREE_SPEEDS[MotorModel.NEOVortex]),
      motorMotionMaxVelocity = 6000.0,
      motorMotionMaxAcceleration = 12000.0,
      motorVelocityConversionFactor = 1.0
    ))

    LAUNCHER_TRANSFORM = Transform3d(units.inchesToMeters(-4.75), units.inchesToMeters(7.875), units.inchesToMeters(25.3375), Rotation3d())

class Services:
  class Localization:
    VISION_MAX_TARGET_AMBIGUITY: units.percent = 0.2
    VISION_MAX_TARGET_REPROJECTION_ERROR: float = 1.0
    VISION_MAX_TARGET_DISTANCE: units.meters = 5.0
    VISION_MAX_POSE_CHANGE: units.meters = 1.5
    VISION_STDDEV_XY_COEFF: float = 0.1
    VISION_STDDEV_Z_COEFF: float = 0.5
    VISION_STDDEV_TARGET_AMBIGUITY_SCALE_FACTOR: float = 15.0
    VISION_STDDEV_TARGET_REPROJECTION_ERROR_SCALE_FACTOR: float = 2.5

  class Targeting:
    TARGET_LAUNCH_METRICS: tuple[TargetLaunchMetric, ...] = (
      TargetLaunchMetric(distance = 2.0, speed = 0.40, time = 0.95),
      TargetLaunchMetric(distance = 2.5, speed = 0.43, time = 1.05),
      TargetLaunchMetric(distance = 3.0, speed = 0.46, time = 1.15),
      TargetLaunchMetric(distance = 3.5, speed = 0.49, time = 1.25),
      TargetLaunchMetric(distance = 4.0, speed = 0.52, time = 1.35),
      TargetLaunchMetric(distance = 4.5, speed = 0.55, time = 1.45),
      TargetLaunchMetric(distance = 5.0, speed = 0.58, time = 1.55),
      TargetLaunchMetric(distance = 6.0, speed = 0.64, time = 1.75),
      TargetLaunchMetric(distance = 7.0, speed = 0.70, time = 1.95),
      TargetLaunchMetric(distance = 8.0, speed = 0.76, time = 2.05),
      TargetLaunchMetric(distance = 9.0, speed = 0.82, time = 2.25)
    )
    LOCALIZATION_LATENCY_COMPENSATION: units.seconds = 0.04
    VELOCITY_COMPENSATION_THRESHOLD: units.meters_per_second = 0.1
    FUEL_LAUNCH_DRAG_COEFFICIENT: float = 0.2

class Sensors: 
  class Gyro:
    class NAVX2:
      COM_TYPE = AHRS.NavXComType.kUSB1
  
  class Pose:
    POSE_SENSOR_CONFIGS: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        name = "FrontLeft",
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(2.75), y = units.inchesToMeters(13.5), z = units.inchesToMeters(10.25)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-26.0), yaw = units.degreesToRadians(55.0))
        ),
        stream = "http://10.28.81.6:1186/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
      PoseSensorConfig(
        name = "FrontRight",
        transform = Transform3d(
        Translation3d(x = units.inchesToMeters(1.25), y = units.inchesToMeters(-14.0), z = units.inchesToMeters(8.75)),
        Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-20.2), yaw = units.degreesToRadians(-90.0))
      ),
        stream = "http://10.28.81.7:1184/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
      PoseSensorConfig(
        name = "RearLeft",
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(-9.75), y = units.inchesToMeters(12.75), z = units.inchesToMeters(10.25)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-32.5), yaw = units.degreesToRadians(135.0))
        ),
        stream = "http://10.28.81.6:1182/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      ),
      PoseSensorConfig(
        name = "RearRight",
        transform = Transform3d(
          Translation3d(x = units.inchesToMeters(-9.75), y = units.inchesToMeters(-12.75), z = units.inchesToMeters(10.25)),
          Rotation3d(roll = units.degreesToRadians(0), pitch = units.degreesToRadians(-32.5), yaw = units.degreesToRadians(-160.0))
        ),
        stream = "http://10.28.81.7:1182/?action=stream",
        aprilTagFieldLayout = _aprilTagFieldLayout
      )
    )

  class Binary:
    INDEXER_SENSOR_CONFIG = BinarySensorConfig(
      name = "Indexer", 
      channel = 2
    )

  class Distance:
    HOPPER_SENSOR_CONFIG = DistanceSensorConfig(
      name = "Hopper", 
      channel = 1, 
      pulseWidthConversionFactor = 2.0, 
      minTargetDistance = 0, 
      maxTargetDistance = 650
    )

class Cameras:
  DRIVER_STREAM = "http://10.28.81.6:1184/?action=stream"

class Controllers:
  DRIVER_CONTROLLER_PORT: int = 0
  OPERATOR_CONTROLLER_PORT: int = 1
  INPUT_DEADBAND: units.percent = 0.1
  HOMING_BUTTON_CONFIG = ButtonControllerConfig(name = "Homing", channel = 0)

class Game:
  class Robot:
    TYPE = RobotType.Competition
    NAME: str = "Rosetta Stone"

  class Commands:
    LAUNCHER_READY_TIMEOUT: units.seconds = 1.25
    TURRET_HEADING_LAUNCH_TOLERANCE: units.degrees = 5.0

  class Field:
    LENGTH = _aprilTagFieldLayout.getFieldLength()
    WIDTH = _aprilTagFieldLayout.getFieldWidth()
    BOUNDS = (Translation2d(0, 0), Translation2d(LENGTH, WIDTH))

    class Targets:
      TARGETS: dict[Alliance, dict[Target, Pose3d]] = {
        Alliance.Blue: {
          Target.Hub: Pose3d(4.625, 4.030, 1.263, Rotation3d(Rotation2d.fromDegrees(0))), 
          Target.ShuttleLeft: Pose3d(1.200, 6.800, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.ShuttleRight: Pose3d(1.200, 1.200, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.ScoreLeft: Pose3d(2.750, 6.600, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          Target.ScoreRight: Pose3d(2.750, 1.400, 0, Rotation3d(Rotation2d.fromDegrees(180))), 
          Target.BumpLeftIn: Pose3d(3.100, 5.700, 0, Rotation3d(Rotation2d.fromDegrees(-135))),
          Target.BumpLeftOut: Pose3d(6.100, 5.700, 0, Rotation3d(Rotation2d.fromDegrees(-45))),
          Target.BumpRightIn: Pose3d(3.100, 2.400, 0, Rotation3d(Rotation2d.fromDegrees(135))),
          Target.BumpRightOut: Pose3d(6.100, 2.400, 0, Rotation3d(Rotation2d.fromDegrees(45))),
        },
        Alliance.Red: {}
      }

      for target in TARGETS[Alliance.Blue]:
        redTargetPose = FlippingUtil.flipFieldPose(TARGETS[Alliance.Blue][target].toPose2d())
        TARGETS[Alliance.Red][target] = Pose3d(redTargetPose.X(), redTargetPose.Y(), TARGETS[Alliance.Blue][target].Z(), Rotation3d(redTargetPose.rotation()))
