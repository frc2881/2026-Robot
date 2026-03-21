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
  BL_NZ_ST_SF = auto()
  BR_NZ_LP_SF = auto()
  BR_NZ_ST_SF = auto()
  TR_OP_SF = auto()
  TL_DP_SF = auto()
  BL_LV_NZ = auto()
  BR_LV_NZ = auto()

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
    
    self._autos.addOption("[BLeft]_Loop", self.auto_BL_NZ_LP_SF_LV)
    self._autos.addOption("[BLeft]_Straight", self.auto_BL_NZ_ST_SF_LV)
    self._autos.addOption("[BRight]_Loop", self.auto_BR_NZ_LP_SF_LV)
    self._autos.addOption("[BRight]_Straight", self.auto_BR_NZ_ST_SF_LV)
    self._autos.addOption("[TL]_Depot", self.auto_TL_DP_SF)
    self._autos.addOption("[TR]_Outpost", self.auto_TR_OP_SF)

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
      (self._robot.game.launchFuel(Target.Hub)
      .deadlineFor(self._robot.game.agitateIntake())
      .deadlineFor(logger.log_("Auto:Score")))
    )

  def auto_BL_NZ_LP_SF_LV(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BL_NZ_LP_SF).deadlineFor(self._intake()),
      self._score()# .until(lambda: utils.getMatchTime() <= constants.Game.Commands.AUTO_NZ_LEAVE_MATCHTIME),
      # self._move(AutoPath.BL_LV_NZ).deadlineFor(self._intake())
    ).withName("Auto:[BL]_NZ_LP_SF_LV")
  
  def auto_BL_NZ_ST_SF_LV(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BL_NZ_ST_SF).deadlineFor(self._intake()),
      self._score()# .until(lambda: utils.getMatchTime() <= constants.Game.Commands.AUTO_NZ_LEAVE_MATCHTIME),
      # self._move(AutoPath.BL_LV_NZ).deadlineFor(self._intake())
    ).withName("Auto:[BL]_NZ_ST_SF_LV") 

  def auto_BR_NZ_LP_SF_LV(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BR_NZ_LP_SF).deadlineFor(self._intake()),
      self._score()# .until(lambda: utils.getMatchTime() <= constants.Game.Commands.AUTO_NZ_LEAVE_MATCHTIME),
      # self._move(AutoPath.BR_LV_NZ).deadlineFor(self._intake())
    ).withName("Auto:[BR]_NZ_LP_SF_LV")
  8
  def auto_BR_NZ_ST_SF_LV(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BR_NZ_ST_SF).deadlineFor(self._intake()),
      self._score()# .until(lambda: utils.getMatchTime() <= constants.Game.Commands.AUTO_NZ_LEAVE_MATCHTIME),
      # self._move(AutoPath.BR_LV_NZ).deadlineFor(self._intake())
    ).withName("Auto:[BR]_NZ_ST_SF_LV")
  
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
  