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
    return self.runEnd(
      lambda: self._climber.setPosition(self._constants.CLIMBER_UP_POSITION),
      lambda: self._climber.reset()
    ).withName("Climber:Up")
  
  def down(self) -> Command:
    return self.runEnd(
      lambda: self._climber.setPosition(self._constants.CLIMBER_DOWN_POSITION),
      lambda: self._climber.reset()
    ).withName("Climber:Down")

  def periodic(self) -> None:
    self._updateTelemetry()

  def resetToHome(self) -> Command:
    return self._climber.resetToHome(self).withName("Climber:ResetToHome")

  def isHomed(self) -> bool:
    return self._climber.isHomed()

  def reset(self) -> None:
    lambda: self._climber.reset()

  def _updateTelemetry(self) -> None:
    pass