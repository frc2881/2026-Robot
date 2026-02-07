from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
import core.constants as constants
from lib.components.velocity_control_module import VelocityControlModule

class Feeder(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Feeder

    self._motor = VelocityControlModule(self._constants.MOTOR_CONFIG)

  def setSpeed(self, speed: units.percent) -> None:
    self._motor.setSpeed(speed)

  def periodic(self) -> None:
    self._updateTelemetry()

  def reset(self) -> None:
    self._motor.reset()

  def _updateTelemetry(self) -> None:
    pass