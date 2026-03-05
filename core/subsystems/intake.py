import math
from commands2 import Subsystem, Command
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

    self._armAgitatePatternTimer = Timer()

  def periodic(self) -> None:
    self._updateTelemetry()

  def run_(self) -> Command:
    return self.startEnd(
      lambda: [
        self._arm.setPosition(self._constants.ARM_INTAKE_POSITION),
        self._rollers.setSpeed(self._constants.ROLLERS_INTAKE_SPEED)
      ],
      lambda: self.reset()
    ).withName("Intake:Run")

  def agitate(self) -> Command:
    return self.runEnd(
      lambda: [
        self._armAgitatePatternTimer.advanceIfElapsed(1.0),
        self._arm.setPosition(
          self._constants.ARM_INTAKE_POSITION * (
            self._constants.ARM_AGITATE_RANGE.max 
            if self._armAgitatePatternTimer.get() < 0.75 else 
            self._constants.ARM_AGITATE_RANGE.min
          )
        ),
        self._rollers.setSpeed(self._constants.ROLLERS_AGITATE_SPEED)
      ],
      lambda: self.reset()
    ).beforeStarting(lambda: self._armAgitatePatternTimer.restart()).withName("Intake:Agitate")
  
  def retract(self) -> Command:
    return self.startEnd(
      lambda: self._arm.setPosition(self._constants.ARM_RETRACT_POSITION),
      lambda: self.reset()
    ).withName("Intake:Retract")

  def isExtended(self) -> bool:
    return self._arm.getPosition() > self._constants.ARM_INTAKE_POSITION * 0.9
  
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