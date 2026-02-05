from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
import core.constants as constants
from lib.components.follower_module import FollowerModule
from lib.components.velocity_control_module import VelocityControlModule

class Launcher(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

    self._flywheelMotor = VelocityControlModule(self._constants.FLYWHEEL_MOTOR_CONFIG)
    self._flywheelFollower = FollowerModule(self._constants.FLYWHEEL_FOLLOWER_CONFIG)

    self._acceleratorMotor = VelocityControlModule(self._constants.ACCELERATOR_MOTOR_CONFIG)

  def periodic(self) -> None:
    self._updateTelemetry()

  def setFlywheelSpeed(self, speed: units.percent):
    self._flywheelMotor.setSpeed(speed)

  def setAcceleratorSpeed(self, speed: units.percent):
    self._acceleratorMotor.setSpeed(speed)

  def reset(self) -> None:
    self._flywheelMotor.reset()
    self._acceleratorMotor.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Launcher/Flywheel/Velocity", self._flywheelMotor.getVelocity())
    SmartDashboard.putNumber("Robot/Launcher/Accelerator/Velocity", self._acceleratorMotor.getVelocity())