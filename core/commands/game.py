from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
from core.classes import Target
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

  def alignRobotToTargetPose(self, target: Target) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.localization.getTargetPose(target))
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName(f'Game:AlignRobotToTargetPose:{ target.name }')
    )
  
  def _isRobotAlignedToTargetPose(self) -> bool:
    return self._robot.drive.isAlignedToTargetPose()

  def alignRobotToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.drive.alignToTargetHeading(self._robot.localization.getRobotPose, lambda: self._robot.localization.getTargetPose(target))
      .withName(f'Game:AlignRobotToTargetHeading:{ target.name }')
    )
  
  def _isRobotAlignedToTargetHeading(self) -> bool:
    return self._robot.drive.isAlignedToTargetHeading()

  def alignTurretToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.turret.alignToTargetHeading(self._robot.localization.getRobotPose, lambda: self._robot.localization.getTargetPose(target))
      .withName(f'Game:AlignTurretToTargetHeading:{ target.name }')
    )

  def _isTurretAlignedToTargetHeading(self) -> bool:
    return self._robot.turret.isAlignedToTargetHeading() and not self._robot.turret.isAtSoftLimit() # TODO: validate conditions for setting turret position relative to reaching soft limit

  def isLaunchReady(self) -> bool:
    return self._isTurretAlignedToTargetHeading() # TODO: add all other sensor/subsystem readiness validation checks

  def alignRobotToNearestFuel(self) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.localization.getObjectsPose())
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .onlyIf(lambda: self._robot.localization.getObjectsCount() >= 5)
      .withName(f'Game:AlignRobotToNearestFuel')
    )

  def rumbleControllers(
    self, 
    mode: ControllerRumbleMode = ControllerRumbleMode.Both, 
    pattern: ControllerRumblePattern = ControllerRumblePattern.Short
  ) -> Command:
    return cmd.parallel(
      self._robot.driver.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Operator),
      self._robot.operator.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Driver)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName(f'Game:RumbleControllers:{ mode.name }:{ pattern.name }')
