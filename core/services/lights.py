from typing import Callable
from wpilib import DriverStation
from lib.classes import RobotState
from lib.controllers.lights import LightsController
from lib import logger, utils
from core.classes import LightsMode

class Lights():
  def __init__(
      self,
      isHomed: Callable[[], bool],
      hasValidVisionTarget: Callable[[], bool]
    ) -> None:
    self._isHomed = isHomed
    self._hasValidVisionTarget = hasValidVisionTarget

    self._lightsController = LightsController()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateLights()

  def _updateLights(self) -> None:
    if not DriverStation.isDSAttached():
      self._lightsController.setMode(LightsMode.RobotNotConnected)
      return
    if utils.getRobotState() == RobotState.Disabled:
      if not self._isHomed():
        self._lightsController.setMode(LightsMode.RobotNotHomed)
        return
      if not self._hasValidVisionTarget():
        self._lightsController.setMode(LightsMode.VisionNotReady)
        return
    else:
      pass
    self._lightsController.setMode(LightsMode.Default)