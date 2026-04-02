from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath import units
from wpimath.geometry import Transform2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance
from core.classes import Target
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class AutoPath(Enum):
  BL_NZ_LP_SF = auto()
  BL_NZ_J_D_SF = auto()
  BR_NZ_LP_SF = auto()
  BR_NZ_J_SF = auto()
  TR_OP_SF = auto()
  TL_DP_SF = auto()

class Auto:
  def __init__(self, robot: "RobotCore") -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }
    self._auto = cmd.none()

    AutoBuilder.configure(
      self._robot.localization.getRobotPose, 
      self._robot.localization.resetRobotPose,
      self._robot.drive.getChassisSpeeds, 
      self._robot.drive.setChassisSpeeds, 
      constants.Subsystems.Drive.PATHPLANNER_CONTROLLER,
      constants.Subsystems.Drive.PATHPLANNER_ROBOT_CONFIG,
      lambda: utils.getAlliance() == Alliance.Red,
      self._robot.drive
    )

    self._autos = SendableChooser()
    self._autos.setDefaultOption("None", cmd.none)
    
    self._autos.addOption("[Bump Left] + Loop", self.auto_BL_NZ_LP_SF)
    self._autos.addOption("[Bump Left] + J + Depot", self.auto_BL_NZ_J_D_SF)
    self._autos.addOption("[Bump Right] + Loop", self.auto_BR_NZ_LP_SF)
    self._autos.addOption("[Bump Right] + J", self.auto_BR_NZ_J_SF)
    self._autos.addOption("[Trench Left] + Depot", self.auto_TL_DP_SF)
    self._autos.addOption("[Trench Right] + Outpost", self.auto_TR_OP_SF)

    self._autos.onChange(lambda auto: self.set(auto()))
    SmartDashboard.putData("Robot/Auto", self._autos)

  def get(self) -> Command:
    return self._auto
  
  def set(self, auto: Command) -> None:
    self._auto = auto
  
  def _reset(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.resetOdom(self._paths.get(path).getPathPoses()[0].transformBy(Transform2d(0, 0, self._paths.get(path).getInitialHeading())))
      .andThen(cmd.waitSeconds(0.1))
    )
  
  def _move(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.followPath(self._paths.get(path))
      .deadlineFor(logger.log_(f'Auto:Move:{path.name}'))
    )
  
  def _intake(self) -> Command:
    return (
      self._robot.game.runIntake()
      .deadlineFor(logger.log_("Auto:Intake"))
    )
  
  def _score(self) -> Command:
    return (
      (self._robot.game.launchFuel(Target.Hub).deadlineFor(self._robot.game.retractIntake()))
      .deadlineFor(logger.log_("Auto:Score"))
    )

  def auto_BL_NZ_LP_SF(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BL_NZ_LP_SF).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:[BL]_NZ_LP_SF")
  
  def auto_BL_NZ_J_D_SF(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BL_NZ_J_D_SF).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:[BL]_NZ_ST_SF") 

  def auto_BR_NZ_LP_SF(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BR_NZ_LP_SF).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:[BR]_NZ_LP_SF")
  8
  def auto_BR_NZ_J_SF(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BR_NZ_J_SF).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:[BR]_NZ_ST_SF")
  
  def auto_TR_OP_SF(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.TR_OP_SF).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:[TR]_OP_SF")
  
  def auto_TL_DP_SF(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.TL_DP_SF).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:[TL]_DP_SF")
  