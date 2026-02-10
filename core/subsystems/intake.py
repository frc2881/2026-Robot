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
    self._roller = VelocityControlModule(self._constants.ROLLER_CONFIG)

    self.setDefaultCommand(self.hold())

  def periodic(self) -> None:
    self._updateTelemetry()

  def hold(self) -> Command:
    return self.run(
      lambda: self._arm.setSpeed(self._constants.ARM_HOLD_SPEED if self._arm.getPosition() > 1 else -self._constants.ARM_HOLD_SPEED)
    ).withName("Intake:HoldPosition")
  
  def extend(self) -> Command:
    return self.run(
      lambda: self._arm.setPosition(self._constants.ARM_INTAKE_POSITION)
    ).withName("Intake:Extend")
  
  def retract(self) -> Command:
    return self.run(
      lambda: self._arm.setPosition(0)
    ).withName("Intake:Extend")

  def activate(self) -> Command:
    return self.runEnd(
      lambda: [
        self._arm.setPosition(self._constants.ARM_INTAKE_POSITION),
        self._roller.setSpeed(self._constants.ROLLER_INTAKE_SPEED)
      ],
      lambda: self._roller.reset()
    ).withName("Intake:Activate")

  def isAtTargetPosition(self) -> bool:
    return self._arm.isAtTargetPosition()
  
  def isExtended(self) -> bool:
    return self._arm.isAtTargetPosition() and self._arm.getPosition() > 1
  
  def isRunning(self) -> bool:
    return self._roller.getSpeed() > 0
  
  def resetToHome(self) -> Command:
    return self._arm.resetToHome(self).withName("Intake:ResetToHome")

  def isHomed(self) -> bool:
    return self._arm.isHomed()

  def reset(self) -> None:
    lambda: self._arm.reset()
    lambda: self._roller.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Intake/IsExtended", self.isExtended())
    SmartDashboard.putBoolean("Robot/Intake/IsRunning", self.isRunning())