from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from wpimath import units
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
        .andThen(self._robot.hopper.run_(lambda: utils.isValueWithinTolerance(self._robot.turret.getHeading(), self._robot.turret.getTargetHeading(), constants.Game.Commands.TURRET_HEADING_LAUNCH_TOLERANCE)))
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
  
  def agitateIntake(self) -> Command:
    return (
      self._robot.intake.agitate()
      .withName("Game:AgitateIntake")
    )
  
  def agitateRobot(self) -> Command:
    return (
      self._robot.drive.drive(lambda: 0.2, lambda: 0, lambda: 0).withTimeout(0.05).andThen(self._robot.drive.reset())
      .withName("Game:AgitateRobot")
    )
  
  def reverseHopper(self) -> Command:
    return (
      self._robot.hopper.reverse()
      .withName("Game:ReverseHopper")
    )

  def getFuelLevel(self) -> FuelLevel:
    distance = self._robot.hopperSensor.getDistance()
    if utils.isValueWithinRange(distance, 0, 150):
      return FuelLevel.Full
    if utils.isValueWithinRange(distance, 0, 250): 
      return FuelLevel.Mid
    if utils.isValueWithinRange(distance, 0, 450) or self._robot.indexerSensor.hasTarget(): 
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
