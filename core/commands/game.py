from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
from core.classes import Target, FuelLevel
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class Game:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

  def alignRobotToTargetPose(self, target: Target) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target))
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName(f'Game:AlignRobotToTargetPose:{ target.name }')
    )

  def alignRobotToNearestTargetPose(self, targets: list[Target]) -> Command:
    return (
      self._robot.drive.alignToTargetPose(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getNearestTargetPose(targets))
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName(f'Game:AlignRobotToNearestTargetPose')
    )

  def alignRobotToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.drive.alignToTargetHeading(self._robot.localization.getRobotPose, lambda: self._robot.targeting.getTargetPose(target))
      .withName(f'Game:AlignRobotToTargetHeading:{ target.name }')
    )

  def alignRobotToNearestBump(self) -> Command:
    return (
      self.alignRobotToNearestTargetPose([Target.BumpLeftIn, Target.BumpLeftOut, Target.BumpRightIn, Target.BumpRightOut])
    )

  def alignTurretToTargetHeading(self, target: Target) -> Command:
    return (
      self._robot.turret.setHeading(lambda: self._robot.targeting.getLaunchHeading(target))
      .withName(f'Game:AlignTurretToTargetHeading:{ target.name }')
    )
  
  def launchFuel(self, target: Target) -> Command:
    return (
      self.alignTurretToTargetHeading(target)
      .alongWith(
        self._robot.launcher.run_(lambda: self._robot.targeting.getLaunchSpeed(target)),
        cmd.waitUntil(lambda: self._robot.launcher.isAtTargetSpeed()).withTimeout(constants.Game.Commands.LAUNCHER_READY_TIMEOUT)
        .andThen(self._robot.hopper.run_())
      )
      .withName(f'Game:LaunchFuel:{ target.name }')
    )
  
  def runIntake(self) -> Command:
    return (
      self._robot.intake.run_()
      .withName("Game:RunIntake")
    )
  
  def retractIntake(self) -> Command:
    return (
      self._robot.intake.retract()
      .withName("Game:RetractIntake")
    )

  def getFuelLevel(self) -> FuelLevel:
    level = self._robot.hopperSensor.getDistance()
    if utils.isValueInRange(level, 0, constants.Sensors.Distance.HOPPER_FUEL_LEVEL_FULL):
      return FuelLevel.Full
    if level <= constants.Sensors.Distance.HOPPER_FUEL_LEVEL_MID: 
      return FuelLevel.Mid
    if level <= constants.Sensors.Distance.HOPPER_FUEL_LEVEL_LOW: 
      return FuelLevel.Low
    return FuelLevel.Empty

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
