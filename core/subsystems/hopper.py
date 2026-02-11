from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from lib import logger, utils
import core.constants as constants
from lib.components.velocity_control_module import VelocityControlModule

class Hopper(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Hopper

    self._indexer = VelocityControlModule(self._constants.INDEXER_CONFIG)
    self._feeder = VelocityControlModule(self._constants.FEEDER_CONFIG)
    self._elevator = VelocityControlModule(self._constants.ELEVATOR_CONFIG)
  
  def periodic(self) -> None:
    self._updateTelemetry()

  def run_(self) -> Command:
    return self.runEnd(
      lambda: [
        self._indexer.setSpeed(self._constants.INDEXER_SPEED),
        self._feeder.setSpeed(self._constants.FEEDER_SPEED),
        self._elevator.setSpeed(self._constants.ELEVATOR_SPEED)
      ],
      lambda: self.reset()
    ).withName("Hopper:Run")

  def agitate(self) -> Command:
    return self.runEnd(
      lambda: [
        self._indexer.setSpeed(self._constants.INDEXER_AGITATE_SPEED),
        self._feeder.setSpeed(self._constants.FEEDER_AGITATE_SPEED),
        self._elevator.setSpeed(self._constants.ELEVATOR_AGITATE_SPEED)
      ],
      lambda: self.reset()
    ).withName("Hopper:Agitate")
  
  def isRunning(self) -> bool:
    return self._indexer.getSpeed() != 0 and self._feeder.getSpeed() != 0 and self._elevator.getSpeed() != 0

  def reset(self) -> None:
    self._indexer.reset()
    self._feeder.reset()
    self._elevator.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hopper/IsRunning", self.isRunning())