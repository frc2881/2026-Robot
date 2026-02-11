from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
from core.classes import Target
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(self, robot: "RobotCore") -> None:
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
      self._robot.drive.alignToTargetHeading(self._robot.localization.getRobotPose, lambda: self._robot.localization.getTargetHeading(target))
      .withName(f'Game:AlignRobotToTargetHeading:{ target.name }')
    )
  
  def _isRobotAlignedToTargetHeading(self) -> bool:
    return self._robot.drive.isAlignedToTargetHeading()

  def alignTurretToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.turret.alignToTargetHeading(lambda: self._robot.localization.getTargetHeading(target))
      .withName(f'Game:AlignTurretToTargetHeading:{ target.name }')
    )

  def _isTurretAlignedToTargetHeading(self) -> bool:
    return self._robot.turret.isAlignedToTargetHeading()

  def _isLaunchReady(self) -> bool: # TODO: add other launch readiness validation checks (sensors indicating fuel in proper locations, etc.)
    return (
      self._isTurretAlignedToTargetHeading() and
      self._robot.launcher.isAtTargetSpeed()
    ) 

  def runIntake(self) -> Command:
    return (
      self._robot.intake.run_()
      .withName("Game:RunIntake")
    )
  
  def runHopper(self) -> Command: # TODO: temporary command for manual testing - will be removed and integrated into single fuel scoring command with validation checks
    return (
      self._robot.hopper.run_()
      .withName("Game:RunHopper")
    )

  def runLauncher(self, target: Target) -> Command: # TODO: temporary command for manual testing - will be removed and integrated into single fuel scoring command with validation checks
    return (
      self._robot.launcher.run_(lambda: self._robot.localization.getTargetDistance(Target.Hub, constants.Subsystems.Turret.TURRET_TRANSFORM))
      .withName(f'Game:RunLauncher:{ target.name }')
    )

  def alignRobotToNearestFuel(self) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.localization.getObjectsPose())
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .onlyIf(lambda: self._robot.localization.getObjectsCount() >= 5) # TODO: make a constant for and validate minimum fuel count to target if we use this feature on the robot
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
