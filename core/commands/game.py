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
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.localization.getTargetPose(target).toPose2d())
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName(f'Game:AlignRobotToTargetPose:{ target.name }')
    )
  
  def alignRobotToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.drive.alignToTargetHeading(self._robot.localization.getRobotPose, lambda: self._robot.localization.getTargetPose(target).toPose2d())
      .withName(f'Game:AlignRobotToTargetHeading:{ target.name }')
    )
  
  def alignRobotToNearestFuel(self) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, self._robot.localization.getObjectsPose)
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .onlyIf(lambda: self._robot.localization.getObjectsCount() >= 10) # TODO: make a constant for and validate minimum fuel count to target IF we use this feature on the robot
      .withName(f'Game:AlignRobotToNearestFuel')
    )

  def alignTurretToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.turret.alignToTargetHeading(self._robot.localization.getRobotPose, lambda: self._robot.localization.getTargetPose(target).toPose2d())
      .withName(f'Game:AlignTurretToTargetHeading:{ target.name }')
    )
  
  def scoreFuel(self) -> Command:
    return (
      self.alignTurretToTargetHeading(Target.Hub)
      .alongWith(
        self._robot.launcher.run_(self._robot.localization.getRobotPose, lambda: self._robot.localization.getTargetPose(Target.Hub)),
        cmd.waitUntil(lambda: self._robot.launcher.isAtTargetSpeed()).withTimeout(constants.Game.Commands.LAUNCHER_READY_TIMEOUT).andThen(self._robot.hopper.run_())
        # TODO: add sensor-based logic to detect hopper fuel level and slowly moved intake arm up to funnel fuel into the indexer as level decreases
      ).withName("Game:ScoreFuel")
    )
  
  def runIntake(self) -> Command:
    return (
      self._robot.intake.run_()
      .withName("Game:RunIntake")
    )

  def climbUp(self) -> Command:
    return (
      self._robot.climber.up()
      .withName("Game:ClimbUp")
    )
  
  def climbDown(self) -> Command:
    return (
      self._robot.climber.down()
      .withName("Game:ClimbDown")
    )

  def isLaunchReady(self) -> bool:
    return (
      self._robot.turret.isAlignedToTargetHeading() and
      self._robot.launcher.isAtTargetSpeed()
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
