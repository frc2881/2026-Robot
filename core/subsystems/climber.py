from typing import Callable
from commands2 import Subsystem, Command
from rev import PersistMode, ResetMode, SparkBaseConfig, SparkFlex, SparkLowLevel, SparkMax
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
import core.constants as constants
from lib.components.follower_module import FollowerModule
from lib.components.relative_position_control_module import RelativePositionControlModule

class Climber(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Climber

    self._rotatorMotor = RelativePositionControlModule(self._constants.ROTATOR_MOTOR_CONFIG)
    self._rotatorFollowerMotor = FollowerModule(self._constants.ROTATOR_FOLLOWER_CONFIG)

    self._deployerMotor = RelativePositionControlModule(self._constants.DEPLOYER_MOTOR_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def setRotatorPosition(self, position: float):
    self._rotatorMotor.setPosition(position)

  def setDeployerPosition(self, position: float):
    self._deployerMotor.setPosition(position)

  def reset(self) -> None:
    self._rotatorMotor.reset()
    self._deployerMotor.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Climber/Rotator/Position", self._rotatorMotor.getPosition())
    SmartDashboard.putNumber("Robot/Climber/Deployer/Position", self._deployerMotor.getPosition())
    