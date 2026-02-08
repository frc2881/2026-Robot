from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
import core.constants as constants
from lib.components.velocity_control_module import VelocityControlModule

class Indexer(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Indexer

    self._motor = VelocityControlModule(self._constants.MOTOR_CONFIG)

  def agitate(self, speed: units.percent):
    return cmd.repeatingSequence(
      cmd.runOnce(lambda: self._motor.setSpeed(self._constants.INDEXER_AGITATE_FAST_SPEED)),
      cmd.waitSeconds(self._constants.INDEXER_AGITATE_FAST_TIME),
      cmd.runOnce(lambda: self._motor.setSpeed(self._constants.INDEXER_AGITATE_SLOW_SPEED)),
      cmd.waitSeconds(self._constants.INDEXER_AGITATE_SLOW_TIME)
    ).finallyDo(
      lambda: self._motor.reset()
    ).withName("Indexer:Agitate")

  def runIndexer(self) -> Command:
    return self.startEnd(
      lambda: self._motor.setSpeed(self._constants.INDEXER_SPEED),
      lambda: self._motor.reset()
      ).withName("Indexer:RunIndexer")
  
  def periodic(self) -> None:
    self._updateTelemetry()

  def reset(self) -> None:
    self._motor.reset()

  def _updateTelemetry(self) -> None:
    pass