from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Transform2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance
from core.classes import Target
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class AutoPath(Enum):
  BPL_NZN_LPL_DPO = auto()
  BPL_NZN_LPR_DPO = auto()
  BPL_NZN_CTR_DPO = auto()
  BPR_NZN_LPL = auto()
  BPR_NZN_LPR = auto()
  HUB_DPO = auto()

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
    
    self._autos.addOption("Bump Left > Loop Left > Depot", self.auto_BPL_NZN_LPL_DPO)
    self._autos.addOption("Bump Left > Loop Right > Depot", self.auto_BPL_NZN_LPR_DPO)
    self._autos.addOption("Bump Left > Center > Depot", self.auto_BPL_NZN_CTR_DPO)
    self._autos.addOption("Bump Right > Loop Left", self.auto_BPR_NZN_LPL)
    self._autos.addOption("Bump Right > Loop Right", self.auto_BPR_NZN_LPR)
    self._autos.addOption("Hub > Depot", self.auto_HUB_DPO)

    self._autos.onChange(lambda auto: self.set(auto()))
    SmartDashboard.putData("Robot/Auto", self._autos)

  def get(self) -> Command:
    return self._auto
  
  def set(self, auto: Command) -> None:
    self._auto = auto
    SmartDashboard.putString("Robot/Auto/command", auto.getName().replace("Auto:", ""))
  
  def _reset(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.resetOdom(self._paths.get(path).getPathPoses()[0].transformBy(Transform2d(0, 0, self._paths.get(path).getInitialHeading())))
      .andThen(cmd.waitSeconds(0.1))
    ).deadlineFor(logger.log_("Auto:Reset"))
  
  def _move(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.followPath(self._paths.get(path))
    ).deadlineFor(logger.log_(f'Auto:Move:{path.name}'))
  
  def _intake(self) -> Command:
    return (
      cmd.waitSeconds(1.2).andThen(self._robot.game.runIntake())
    ).deadlineFor(logger.log_("Auto:Intake"))
  
  def _score(self) -> Command:
    return (
      (self._robot.game.launchFuel(Target.Hub)
       .deadlineFor(
          self._robot.game.agitateIntake(),
          cmd.waitSeconds(3.0).andThen(self._robot.game.agitateRobot())
      ))
    ).deadlineFor(logger.log_("Auto:Score"))

  def auto_BPL_NZN_LPL_DPO(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BPL_NZN_LPL_DPO).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:BPL_NZN_LPL_DPO")

  def auto_BPL_NZN_LPR_DPO(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BPL_NZN_LPR_DPO).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:BPL_NZN_LPR_DPO")
  
  def auto_BPL_NZN_CTR_DPO(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BPL_NZN_CTR_DPO).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:BPL_NZN_CTR_DPO")

  def auto_BPR_NZN_LPL(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BPR_NZN_LPL).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:BPR_NZN_LPL")

  def auto_BPR_NZN_LPR(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BPR_NZN_LPR).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:BPR_NZN_LPR")

  def auto_HUB_DPO(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.HUB_DPO).deadlineFor(self._intake()),
      self._score()
    ).withName("Auto:HUB_DPO")
  