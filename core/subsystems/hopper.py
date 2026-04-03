from typing import Callable
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
    self._elevator = VelocityControlModule(self._constants.ELEVATOR_CONFIG)

    SmartDashboard.putNumber("Robot/Hopper/Indexer/SpeedOverride", self._constants.INDEXER_SPEED)

  def periodic(self) -> None:
    self._updateTelemetry()

  def run_(self, isEnabled: Callable[[], bool]) -> Command:
    return self.runEnd(
      lambda: [
        indexerSpeedOverride := SmartDashboard.getNumber("Robot/Hopper/Indexer/SpeedOverride", 0),
        self._indexer.setSpeed((self._constants.INDEXER_SPEED if indexerSpeedOverride == 0 else indexerSpeedOverride) if isEnabled() else 0),
        self._elevator.setSpeed(self._constants.ELEVATOR_SPEED if isEnabled() else 0)
      ],
      lambda: self.reset()
    )
  
  def reverse(self) -> Command:
    return self.runEnd(
      lambda: [
        self._indexer.setSpeed(-self._constants.INDEXER_REVERSE_SPEED),
        self._elevator.setSpeed(-self._constants.ELEVATOR_REVERSE_SPEED)
      ],
      lambda: self.reset()
    )
  
  def isRunning(self) -> bool:
    return self._indexer.getSpeed() != 0 and self._elevator.getSpeed() != 0

  def reset(self) -> None:
    self._indexer.reset()
    self._elevator.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hopper/IsRunning", self.isRunning())