from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Transform2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.events import EventTrigger
from lib import logger, utils
from lib.classes import Alliance
from core.classes import Target
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class AutoPath(Enum):
  BL_NZ_SCL = auto()
  BR_NZ_SCR = auto()
  TR_OP_SCR = auto()
  TL_DP_SCL = auto()

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
    
    self._autos.addOption("[BL]_NZ_SCL", self.auto_BL_NZ_SCL)
    self._autos.addOption("[BR]_NZ_SCR", self.auto_BR_NZ_SCR)
    self._autos.addOption("[TR]_OP_SCR", self.auto_TR_OP_SCR)
    self._autos.addOption("[TL]_DP_SCL", self.auto_TL_DP_SCL)

    self._autos.onChange(lambda auto: self.set(auto()))
    SmartDashboard.putData("Robot/Auto", self._autos)

    EventTrigger("RunIntake").whileTrue(self._robot.game.runIntake())

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
  
  def _score(self) -> Command:
    return self._robot.game.scoreFuel()
  
  def _climb(self, target: Target) -> Command:
    return self._robot.game.alignRobotToTargetPose(target) # TODO: add drive on to tower and climb up with X seconds left in auto

  def auto_BL_NZ_SCL(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BL_NZ_SCL),
      self._score()
    ).withName("Auto:BL_NZ_SCL")

  def auto_BR_NZ_SCR(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BR_NZ_SCR),
      self._score()
    ).withName("Auto:BR_NZ_SCR")
  
  def auto_TR_OP_SCR(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.TR_OP_SCR),
      self._score()
    ).withName("Auto:TR_OP_SCR")
  
  def auto_TL_DP_SCL(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.TL_DP_SCL),
      self._score()
    ).withName("Auto:TL_DP_SCL")
  