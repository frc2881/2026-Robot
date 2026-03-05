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
  BL_NZ_SF = auto()
  BR_NZ_SF = auto()
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
    
    self._autos.addOption("[BL]_NZ_SF", self.auto_BL_NZ_SF)
    self._autos.addOption("[BL]_NZ_SF_CL", self.auto_BL_NZ_SF_CL)
    self._autos.addOption("[BR]_NZ_SF", self.auto_BR_NZ_SF)
    self._autos.addOption("[BR]_NZ_SF_CR", self.auto_BR_NZ_SF_CR)
    self._autos.addOption("[TL]_DP_SF", self.auto_TL_DP_SF)
    self._autos.addOption("[TL]_DP_SF_CL", self.auto_TL_DP_SF_CL)
    self._autos.addOption("[TR]_OP_SF", self.auto_TR_OP_SF)
    self._autos.addOption("[TR]_OP_SF_CR", self.auto_TR_OP_SF_CR)

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
  
  def _runIntake(self) -> Command:
    return (
      self._robot.game.runIntake()
      .deadlineFor(logger.log_("Auto:RunIntake"))
    )
  
  def _scoreFuel(self) -> Command:
    return (
      self._robot.game.launchFuel(Target.Hub)
      .deadlineFor(self._robot.game.agitateIntake())
      .deadlineFor(logger.log_("Auto:ScoreFuel"))
    )
  
  def _alignClimb(self, target: Target) -> Command:
    return (
      self._robot.game.alignRobotToClimb(target)
      .deadlineFor(logger.log_("Auto:AlignClimb"))
    )
  
  def _climb(self) -> Command:
    return (
      cmd.waitUntil(lambda: utils.getMatchTime() <= 2.0)
      .andThen(self._robot.game.climbUp().withTimeout(2.0))
      .deadlineFor(logger.log_("Auto:Climb"))
    )

  def auto_BL_NZ_SF(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BL_NZ_SF).deadlineFor(cmd.waitSeconds(1.25).andThen(self._runIntake())),
      self._scoreFuel()
    ).withName("Auto:[BL]_NZ_SF")

  def auto_BL_NZ_SF_CL(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BL_NZ_SF).deadlineFor(cmd.waitSeconds(1.25).andThen(self._runIntake())),
      self._alignClimb(Target.ClimbLeft),
      self._scoreFuel().alongWith(self._climb())
    ).withName("Auto:[BL]_NZ_SF_CL")

  def auto_BR_NZ_SF(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BR_NZ_SF).deadlineFor(cmd.waitSeconds(1.25).andThen(self._runIntake())),
      self._scoreFuel()
    ).withName("Auto:[BR]_NZ_SF")
  
  def auto_BR_NZ_SF_CR(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BR_NZ_SF).deadlineFor(cmd.waitSeconds(1.25).andThen(self._runIntake())),
      self._alignClimb(Target.ClimbRight),
      self._scoreFuel().deadlineFor(self._climb())
    ).withName("Auto:[BR]_NZ_SF_CR")
  
  def auto_TR_OP_SF(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.TR_OP_SF).deadlineFor(self._runIntake().withTimeout(1.0)),
      self._scoreFuel()
    ).withName("Auto:[TR]_OP_SF")
  
  def auto_TR_OP_SF_CR(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.TR_OP_SF).deadlineFor(self._runIntake().withTimeout(1.0)),
      cmd.waitSeconds(2.0),
      self._alignClimb(Target.ClimbRight),
      self._scoreFuel().deadlineFor(self._climb())
    ).withName("Auto:[TR]_OP_SF_CR")
  
  def auto_TL_DP_SF(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.TL_DP_SF).deadlineFor(self._runIntake()),
      self._scoreFuel()
    ).withName("Auto:[TL]_DP_SF")
  
  def auto_TL_DP_SF_CL(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.TL_DP_SF).deadlineFor(self._runIntake()),
      self._alignClimb(Target.ClimbLeft),
      self._scoreFuel().deadlineFor(self._climb())
    ).withName("Auto:[TL]_DP_SF_CL")
  