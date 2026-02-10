from commands2 import Subsystem, Command
from lib import logger, utils
from lib.classes import MotorIdleMode
from lib.components.velocity_control_module import VelocityControlModule
from lib.components.follower_module import FollowerModule
import core.constants as constants

class Launcher(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

    self._launcher = VelocityControlModule(self._constants.LAUNCHER_CONFIG)
    self._launcherFollower = FollowerModule(self._constants.LAUNCHER_FOLLOWER_CONFIG)
    self._accelerator = VelocityControlModule(self._constants.ACCELERATOR_CONFIG)

    self._launcher.setIdleMode(MotorIdleMode.Coast)
    self._launcherFollower.setIdleMode(MotorIdleMode.Coast)
    self._accelerator.setIdleMode(MotorIdleMode.Coast)

  def periodic(self) -> None:
    self._updateTelemetry()

  def activate(self) -> Command:
    return self.runEnd(
      lambda: [
        self._launcher.setSpeed(1.0),
        self._accelerator.setSpeed(1.0 * .75)
      ],
      lambda: self.reset()
    ).withName("Launcher:Activate")
  
  def isAtTargetSpeed(self) -> bool:
    return self._accelerator.isAtTargetSpeed() and self._launcher.isAtTargetSpeed()

  def reset(self) -> None:
    self._launcher.reset()
    self._accelerator.reset()

  def _updateTelemetry(self) -> None:
    pass