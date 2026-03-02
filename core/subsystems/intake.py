import math
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from wpimath import units
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

    self.setDefaultCommand(self.hold())

  def periodic(self) -> None:
    self._updateTelemetry()

  def run_(self) -> Command:
    return self.startEnd(
      lambda: [
        self._arm.setSpeed(self._constants.ARM_INTAKE_HOLD_SPEED),
        self._rollers.setSpeed(self._constants.ROLLERS_SPEED)
      ],
      lambda: self.reset()
    ).beforeStarting(self._extend()).withName("Intake:Run")

  def hold(self) -> Command:
    return self.runEnd(
      lambda: self._runHold(),
      lambda: self.reset()
    ).withName("Intake:Hold")
  
  def _runHold(self) -> None:
    if math.isclose(self._arm.getPosition(), 0, abs_tol = 1.0):
      self._arm.setSpeed(-self._constants.ARM_DEFAULT_HOLD_SPEED)
    if math.isclose(self._arm.getPosition(), self._constants.ARM_INTAKE_POSITION, abs_tol = 1.0):
      self._arm.setSpeed(self._constants.ARM_DEFAULT_HOLD_SPEED)

  def _extend(self) -> Command:
    return (
      self.run(lambda: self._arm.setPosition(self._constants.ARM_INTAKE_POSITION))
      .withTimeout(0.2)
      .withName("Intake:Extend")
    )
  
  def retract(self, level: units.percent = 1.0) -> Command:
    return self.run(
      lambda: self._arm.setPosition(self._constants.ARM_INTAKE_POSITION - (self._constants.ARM_INTAKE_POSITION * level))
    ).withName("Intake:Retract")

  def isExtended(self) -> bool:
    return self._arm.getPosition() > self._constants.ARM_INTAKE_POSITION * 0.8
  
  def isRunning(self) -> bool:
    return self._rollers.getSpeed() != 0
  
  def resetToHome(self) -> Command:
    return self._arm.resetToHome(self).withName("Intake:ResetToHome")

  def isHomed(self) -> bool:
    return self._arm.isHomed()

  def reset(self) -> None:
    self._arm.reset()
    self._rollers.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Intake/IsExtended", self.isExtended())
    SmartDashboard.putBoolean("Robot/Intake/IsRunning", self.isRunning())