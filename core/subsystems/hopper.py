from commands2 import Subsystem, Command
from wpilib import SmartDashboard, Timer
from lib import logger, utils
import core.constants as constants
from lib.components.velocity_control_module import VelocityControlModule

from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex, FeedbackSensor, ResetMode, PersistMode

class Hopper(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Hopper

    self._indexer = SparkFlex(self._constants.INDEXER_CONFIG.motorCANId, self._constants.INDEXER_CONFIG.constants.motorType)
    self._indexerMotorConfig = SparkBaseConfig()
    (self._indexerMotorConfig
      .smartCurrentLimit(self._constants.INDEXER_CONFIG.constants.motorCurrentLimit)
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .inverted(self._constants.INDEXER_CONFIG.isInverted))
    (self._indexerMotorConfig.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1))
    utils.setSparkConfig(self._indexer.configure(self._indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))
    self._relativeEncoder = self._indexer.getEncoder()
    self._relativeEncoder.setPosition(0)

    # self._indexer = VelocityControlModule(self._constants.INDEXER_CONFIG)
    self._roller = VelocityControlModule(self._constants.ROLLER_CONFIG)
    self._feeder = VelocityControlModule(self._constants.FEEDER_CONFIG)
    self._elevator = VelocityControlModule(self._constants.ELEVATOR_CONFIG)

    self._indexerTimer = Timer()
  
  def periodic(self) -> None:
    self._updateTelemetry()

  def run_(self) -> Command:
    return self.runEnd(
      lambda: [
        self._indexerTimer.advanceIfElapsed(1.0),
        self._indexer.set(self._constants.INDEXER_SPEED if self._indexerTimer.get() < 0.9 else 0),
        self._roller.setSpeed(self._constants.ROLLER_SPEED),
        self._feeder.setSpeed(self._constants.FEEDER_SPEED),
        self._elevator.setSpeed(self._constants.ELEVATOR_SPEED)
      ],
      lambda: self.reset()
    ).beforeStarting(lambda: self._indexerTimer.restart()).withName("Hopper:Run")

  def agitate(self) -> Command:
    return self.runEnd(
      lambda: [
        self._indexer.set(-self._constants.INDEXER_SPEED * self._constants.AGITATE_SPEED_RATIO),
        self._roller.setSpeed(-self._constants.ROLLER_SPEED * self._constants.AGITATE_SPEED_RATIO),
        self._feeder.setSpeed(-self._constants.FEEDER_SPEED * self._constants.AGITATE_SPEED_RATIO),
        self._elevator.setSpeed(-self._constants.ELEVATOR_SPEED * self._constants.AGITATE_SPEED_RATIO)
      ],
      lambda: self.reset()
    ).withName("Hopper:Agitate")
  
  def isRunning(self) -> bool:
    return self._indexer.get() != 0 and self._roller.getSpeed() != 0 and self._feeder.getSpeed() != 0 and self._elevator.getSpeed() != 0

  def reset(self) -> None:
    self._indexer.stopMotor()
    self._roller.reset()
    self._feeder.reset()
    self._elevator.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hopper/IsRunning", self.isRunning())