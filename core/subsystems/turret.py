from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.components.relative_position_control_module import RelativePositionControlModule
import core.constants as constants

class Turret(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Turret

    self._turret = RelativePositionControlModule(self._constants.TURRET_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def setHeading(self, heading: units.degrees) -> Command:
    return self.run(
      lambda: self._turret.setPosition(heading)
    )
  
  def getHeading(self) -> units.degrees:
    return self._turret.getPosition()

  def alignToTargetHeading(self, getTargetHeading: Callable[[], units.degrees]) -> Command:
    return self.run(
      lambda: self._turret.setPosition(getTargetHeading())
    )

  def isAlignedToTargetHeading(self) -> bool:
    return self._turret.isAtTargetPosition()
  
  def isAtSoftLimit(self) -> bool:
    return self._turret.isAtSoftLimit()

  def resetToHome(self) -> Command:
    return self._turret.resetToHome(self).withName("Turret:ResetToHome")

  def isHomed(self) -> bool:
    return self._turret.isHomed()

  def reset(self) -> None:
    self._turret.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Turret/IsAlignedToTargetHeading", self.isAlignedToTargetHeading())
    SmartDashboard.putNumber("Robot/Turret/Heading", self.getHeading())