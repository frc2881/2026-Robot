from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from lib import logger, utils
import core.constants as constants
from lib.components.relative_position_control_module import RelativePositionControlModule

class Climber(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Climber
    self._climber = RelativePositionControlModule(self._constants.CLIMB_CONFIG)

  def up(self) -> Command:
    return self.run(
      lambda: self._climber.setPosition(self._constants.CLIMB_UP_POSITION)
    ).withName("Climb:Up")
  
  def down(self) -> Command:
    return self.run(
      lambda: self._climber.setPosition(self._constants.CLIMB_DOWN_POSITION)
    ).withName("Climb:Down")

  def periodic(self) -> None:
    self._updateTelemetry()

  def reset(self) -> None:
    lambda: self._climber.reset()

  def _updateTelemetry(self) -> None:
    pass