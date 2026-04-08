from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard, Timer
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

    self._isIntaking: bool = False
    self._isAgitating: bool = False
    self._isRetracting: bool = False

    self._agitationTimer = Timer()

  def periodic(self) -> None:
    self._updateState()
    self._updateTelemetry()

  def _updateState(self) -> None:
    if self._isIntaking:
      self._arm.setPosition(self._constants.ARM_INTAKE_POSITION)
      self._rollers.setSpeed(self._constants.ROLLERS_INTAKE_SPEED if self._arm.getPosition() > self._constants.ARM_INTAKE_POSITION * 0.9 else 0)
    elif self._isRetracting:
      self._arm.setPosition(self._constants.ARM_RETRACT_POSITION)
      self._rollers.setSpeed(0)
    elif self._isAgitating:
      self._agitationTimer.advanceIfElapsed(self._constants.ARM_AGITATE_TIME)
      self._arm.setPosition(
        self._constants.ARM_INTAKE_POSITION * 
        (range.min if self._agitationTimer.get() < self._constants.ARM_AGITATE_TIME / 2 else range.max)
      )
      self._rollers.setSpeed(self._constants.ROLLERS_AGITATE_SPEED)
    else:
      if not self.isHoming():
        self.reset()

  def run_(self) -> Command:
    return cmd.startEnd(
      lambda: setattr(self, "_isIntaking", True),
      lambda: setattr(self, "_isIntaking", False)
    )
  
  def retract(self) -> Command:
    return cmd.startEnd(
      lambda: setattr(self, "_isRetracting", True),
      lambda: setattr(self, "_isRetracting", False)
    )

  def agitate(self) -> Command:
    return cmd.startEnd(
      lambda: setattr(self, "_isAgitating", True),
      lambda: setattr(self, "_isAgitating", False)
    ).beforeStarting(lambda: self._agitationTimer.restart())

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