from commands2 import cmd
from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
import core.constants as constants
from lib.classes import Position
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.relative_position_control_module import RelativePositionControlModule

class Intake(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Intake

    self._rollersMotor = VelocityControlModule(self._constants.ROLLERS_MOTOR_CONFIG)
    self._armMotor = RelativePositionControlModule(self._constants.ARM_MOTOR_CONFIG)

    self._armPostion = Position.Unknown
    self._isArmAlignedToPosition = False

  def periodic(self) -> None:
    self._updateTelemetry()

  def runRollersForward(self) -> Command:
    return self.startEnd(
      lambda: self._rollersMotor.setSpeed(self._constants.ROLLER_SPEED_FORWARD),
      lambda: self._rollersMotor.reset()
    ).withName("Intake:RunRollersForward")

  def runRollersBackward(self) -> None:
    return self.startEnd(
      lambda: self._rollersMotor.setSpeed(self._constants.ROLLER_SPEED_BACKWARD),
      lambda: self._rollersMotor.reset()
    ).withName("Intake:RunRollersBackward")
    
  def setArmSpeed(self, getInput: Callable[[], units.percent]) -> Command:
    return self.runEnd(
      lambda: self._armMotor.setSpeed(getInput() * self._constants.ARM_INPUT_LIMIT),
      lambda: self._armMotor.reset()
    ).withName("Intake:SetArmSpeed")

  def setArmPosition(self, position: Position) -> Command:
    return self.startEnd(
      lambda: [
        self._resetPositionAlignment(),
        self._armMotor.setPosition(self._constants.ARM_POSITION_UP if position == Position.Up else self._constants.ARM_POSITION_DOWN)
      ],
      lambda: self._armMotor.reset()
    ).until(
      lambda: self._armMotor.isAtTargetPosition()
    ).andThen(
      cmd.runOnce(lambda: setattr(self, "_armPosition", position))
    ).withName("Intake:SetArmPosition")
  
  def toggleArmPosition(self) -> Command:
    return cmd.either(
      self.setArmPosition(Position.Up), 
      self.setArmPosition(Position.Down), 
      lambda: self._armPosition != Position.Up
    ).withName("Intake:ToggleArmPosition")

  def runIntake(self) -> Command:
    return self.setArmPosition(Position.Down).andThen(
      self.runRollersForward()
    ).withName("Intake:RunIntake")

  def _resetPositionAlignment(self) -> None:
    self._armPosition = Position.Unknown
    self._isArmAlignedToPosition = False

  def isAlignedToPosition(self) -> bool:
    return self._isArmAlignedToPosition
  
  def resetArmToHome(self) -> Command:
    return self._armMotor.resetToHome(self).withName("Intake:ResetArmToHome")

  def isArmHomed(self) -> bool:
    return self._armMotor.isHomed()

  def reset(self) -> None:
    lambda: self._armMotor.reset()
    lambda: self._rollersMotor.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Intake/IsAlignedToPosition", self.isAlignedToPosition())