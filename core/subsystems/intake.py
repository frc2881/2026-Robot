from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
import core.constants as constants

class Intake(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake

    # TODO: Create a Rollers Velocity Control Module, Vortex, CAN 17
    # TODO: Create an Arm Position Control Module, Neo/Vortex (ask build), CAN 18

  def periodic(self) -> None:
    self._updateTelemetry()

    # TODO: Create functions to run Rollers both directions using SmartMotion Velocity Control

    # TODO: Create function to move Arm up and down using the joystick

    # TODO: Create function to set Arm to a position

  def reset(self) -> None:
    # TODO: Stop motor on Rollers and Arm
    pass

  def _updateTelemetry(self) -> None:
    # TODO: Add in SmartDashboard calls for the most important data (Rollers velocity, Arm position, etc.)
    pass