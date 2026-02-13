from commands2 import cmd
from wpilib import DriverStation, SmartDashboard
from lib import logger, utils
from lib.controllers.xbox import XboxController
from lib.controllers.button import ButtonController
from lib.sensors.gyro_navx2 import Gyro_NAVX2
from lib.sensors.pose import PoseSensor
from lib.sensors.object import ObjectSensor
from core.commands.auto import Auto
from core.commands.game import Game
from core.subsystems.drive import Drive
from core.subsystems.intake import Intake
from core.subsystems.hopper import Hopper
from core.subsystems.turret import Turret
from core.subsystems.launcher import Launcher
from core.subsystems.climber import Climber
from core.services.localization import Localization
from core.services.match import Match
from core.services.lights import Lights
from core.classes import Target
import core.constants as constants

class RobotCore:
  def __init__(self) -> None:
    self._initSensors()
    self._initSubsystems()
    self._initServices()
    self._initCommands()
    self._initControllers()
    self._initTriggers()
    self._initTelemetry()
    utils.addRobotPeriodic(self._periodic)

  def _initSensors(self) -> None:
    self.gyro = Gyro_NAVX2(constants.Sensors.Gyro.NAVX2.COM_TYPE)
    self.poseSensors = tuple(PoseSensor(c) for c in constants.Sensors.Pose.POSE_SENSOR_CONFIGS)
    self.objectSensor = ObjectSensor(constants.Sensors.Object.OBJECT_SENSOR_CONFIG)

  def _initSubsystems(self) -> None:
    self.drive = Drive(self.gyro.getHeading)
    self.intake = Intake()
    self.hopper = Hopper()
    self.turret = Turret()
    self.launcher = Launcher()
    self.climber = Climber()
    
  def _initServices(self) -> None:
    self.localization = Localization(self.gyro.getHeading, self.drive.getModulePositions, self.poseSensors, self.objectSensor)
    self.match = Match()
    self.lights = Lights(lambda: self.isHomed(), lambda: self.localization.hasValidVisionTarget(), lambda: self.game.isLaunchReady())

  def _initCommands(self) -> None:
    self.game = Game(self)
    self.auto = Auto(self)

  def _initControllers(self) -> None:
    DriverStation.silenceJoystickConnectionWarning(not utils.isCompetitionMode())
    self.driver = XboxController(constants.Controllers.DRIVER_CONTROLLER_PORT, constants.Controllers.INPUT_DEADBAND)
    self.operator = XboxController(constants.Controllers.OPERATOR_CONTROLLER_PORT, constants.Controllers.INPUT_DEADBAND)
    self.homingButton = ButtonController(constants.Controllers.HOMING_BUTTON_CONFIG)

  def _initTriggers(self) -> None:
    self._setupDriver()
    self._setupOperator()
    self.homingButton.pressed().debounce(0.5).whileTrue(cmd.parallel(self.intake.resetToHome(), self.turret.resetToHome()))

  def _setupDriver(self) -> None:
    self.drive.setDefaultCommand(self.drive.drive(self.driver.getLeftY, self.driver.getLeftX, self.driver.getRightX))
    self.driver.leftStick().whileTrue(self.drive.lockSwerveModules())
    self.driver.rightStick().whileTrue(self.game.alignRobotToTargetHeading(Target.Hub))
    self.driver.leftTrigger().whileTrue(self.intake.retract())
    self.driver.rightTrigger().whileTrue(self.game.runIntake())
    self.driver.leftBumper().whileTrue(self.game.alignRobotToTargetPose(Target.TrenchLeft))
    self.driver.rightBumper().whileTrue(self.game.alignRobotToTargetPose(Target.TrenchRight))
    # self.driver.povUp().whileTrue(cmd.none()) # TODO: climb up sequence
    # self.driver.povDown().whileTrue(cmd.none()) # TODO: climb down sequence
    self.driver.povLeft().whileTrue(self.game.alignRobotToTargetPose(Target.TowerLeft))
    self.driver.povRight().whileTrue(self.game.alignRobotToTargetPose(Target.TowerRight))
    # self.driver.a().whileTrue(cmd.none())
    self.driver.b().whileTrue(self.game.alignRobotToTargetPose(Target.CornerRight))
    # self.driver.y().whileTrue(cmd.none())
    self.driver.x().whileTrue(self.game.alignRobotToTargetPose(Target.CornerLeft))
    # self.driver.start().whileTrue(cmd.none())
    self.driver.back().debounce(0.5).whileTrue(self.gyro.reset())

  def _setupOperator(self) -> None:
    # self.operator.leftStick().whileTrue(cmd.none())
    # self.operator.rightStick().whileTrue(cmd.none())
    self.operator.leftTrigger().whileTrue(self.hopper.agitate())
    self.operator.rightTrigger().whileTrue(self.game.runLauncher(Target.Hub)) # TODO: FOR INITIAL TESTING ONLY
    self.operator.leftBumper().whileTrue(self.game.runHopper()) # TODO: FOR INITIAL TESTING ONLY
    # self.operator.rightBumper().whileTrue(cmd.none())
    # self.operator.povDown().whileTrue(cmd.none())
    # self.operator.povUp().whileTrue(cmd.none())
    # self.operator.povRight().whileTrue(cmd.none())
    # self.operator.povLeft().whileTrue(cmd.none())
    # self.operator.a().whileTrue(cmd.none())
    # self.operator.b().whileTrue(cmd.none())
    # self.operator.y().whileTrue(cmd.none())
    # self.operator.x().whileTrue(cmd.none())
    self.operator.start().whileTrue(self.turret.resetToHome())
    self.operator.back().whileTrue(self.intake.resetToHome())
    pass

  def _initTelemetry(self) -> None:
    SmartDashboard.putString("Game/Robot/Type", constants.Game.Robot.TYPE.name)
    SmartDashboard.putString("Game/Robot/Name", constants.Game.Robot.NAME)
    SmartDashboard.putNumber("Game/Field/Length", constants.Game.Field.LENGTH)
    SmartDashboard.putNumber("Game/Field/Width", constants.Game.Field.WIDTH)
    SmartDashboard.putNumber("Robot/Drive/Length", constants.Subsystems.Drive.BUMPER_LENGTH)
    SmartDashboard.putNumber("Robot/Drive/Width", constants.Subsystems.Drive.BUMPER_WIDTH)
    SmartDashboard.putString("Robot/Cameras/Driver", constants.Cameras.DRIVER_STREAM)
    SmartDashboard.putStringArray("Robot/Sensors/Pose/Names", tuple(c.name for c in constants.Sensors.Pose.POSE_SENSOR_CONFIGS))

  def _periodic(self) -> None:
    self._updateTelemetry()

  def disabledInit(self) -> None:
    self.reset()

  def autoInit(self) -> None:
    self.reset()

  def autoExit(self) -> None: 
    self.gyro.resetRobotToField(self.localization.getRobotPose())

  def teleopInit(self) -> None:
    self.reset()

  def testInit(self) -> None:
    self.reset()

  def simulationInit(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.drive.reset()

  def isHomed(self) -> bool:
    return self.intake.isHomed() and self.turret.isHomed()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Status/IsHomed", self.isHomed())
