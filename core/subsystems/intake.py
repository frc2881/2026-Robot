from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
import core.constants as constants
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.relative_position_control_module import RelativePositionControlModule

class Intake(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake

    self._rollersMotor = VelocityControlModule(self._constants.ROLLERS_MOTOR_CONFIG)
    self._armMotor = RelativePositionControlModule(self._constants.ARM_MOTOR_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def runRollersForward(self, speed: units.percent) -> Command:
    return self.startEnd(
      lambda: self._rollersMotor.setSpeed(self._constants.ROLLER_SPEED_FORWARD),
      lambda: self._rollersMotor.reset()
    ).withName("Intake:RunRollersForward")

  def runRollersBackward(self, speed: units.percent) -> None:
    return self.startEnd(
      lambda: self._rollersMotor.setSpeed(self._constants.ROLLER_SPEED_BACKWARD),
      lambda: self._rollersMotor.reset()
    ).withName("Intake:RunRollersBackward")
    
  def setArmSpeed(self, getInput: Callable[[], units.percent]) -> Command:
    return self.runEnd(
      lambda: self._armMotor.setSpeed(getInput() * self._constants.ARM_INPUT_LIMIT),
      lambda: self._armMotor.reset()
    ).withName("Intake:SetArmSpeed")
  
  def setArmPosition(self, position: float):
    self._armMotor.setPosition(position)

  def reset(self) -> None:
    lambda: self._armMotor.reset()
    lambda: self._rollersMotor.reset()

  def _updateTelemetry(self) -> None:
    pass