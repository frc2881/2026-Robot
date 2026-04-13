from typing import Callable
from commands2 import Subsystem, Command, cmd
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

    self._isReversing: bool = False
    self._isRunning: bool = False

  def periodic(self) -> None:
    self._updateState()
    self._updateTelemetry()

  def _updateState(self) -> None:
    if self._isReversing:
      self._indexer.setSpeed(-self._constants.INDEXER_REVERSE_SPEED)
      self._elevator.setSpeed(-self._constants.ELEVATOR_REVERSE_SPEED)
    elif self._isRunning:
      self._indexer.setSpeed(self._constants.INDEXER_SPEED)
      self._elevator.setSpeed(self._constants.ELEVATOR_SPEED)
    else:
      self.reset()

  def run_(self, isEnabled: Callable[[], bool]) -> Command:
    return cmd.runEnd(
      lambda: setattr(self, "_isRunning", isEnabled()),
      lambda: setattr(self, "_isRunning", False)
    )
  
  def reverse(self) -> Command:
    return cmd.startEnd(
      lambda: setattr(self, "_isReversing", True),
      lambda: setattr(self, "_isReversing", False)
    )

  def isRunning(self) -> bool:
    return self._isRunning

  def reset(self) -> None:
    self._indexer.reset()
    self._elevator.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hopper/IsRunning", self.isRunning())