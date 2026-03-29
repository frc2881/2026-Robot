from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from lib import logger, utils
from lib.components.relative_position_control_module import RelativePositionControlModule
from lib.components.velocity_control_module import VelocityControlModule
import core.constants as constants

class Intake(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake

    self._arm = RelativePositionControlModule(self._constants.ARM_CONFIG)
    self._rollers = VelocityControlModule(self._constants.ROLLERS_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def run_(self) -> Command:
    return self.runEnd(
      lambda: [
        lambda: self._arm.setPosition(self._constants.ARM_INTAKE_POSITION),
        lambda: self._rollers.setSpeed(self._constants.ROLLERS_INTAKE_SPEED if self._arm.getPosition() > self._constants.ARM_INTAKE_POSITION * 0.9 else 0)
      ],
      lambda: self.reset()
    ).withName("Intake:Run")

  def retract(self) -> Command:
    return self.runEnd(
      lambda: [ 
        self._arm.setPosition(self._constants.ARM_RETRACT_POSITION),
        self._rollers.setSpeed(0)
      ],
      lambda: self.reset()
    ).withName("Intake:Retract")

  def isExtended(self) -> bool:
    return self._arm.getPosition() > self._constants.ARM_INTAKE_POSITION * 0.9
  
  def isRunning(self) -> bool:
    return self._rollers.getSpeed() != 0
  
  def resetToHome(self) -> Command:
    return self._arm.resetToHome(self).withName("Intake:ResetToHome")

  def isHoming(self) -> bool:
    return self._arm.isHoming()

  def isHomed(self) -> bool:
    return self._arm.isHomed()

  def reset(self) -> None:
    self._arm.reset()
    self._rollers.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Intake/IsExtended", self.isExtended())
    SmartDashboard.putBoolean("Robot/Intake/IsRunning", self.isRunning())