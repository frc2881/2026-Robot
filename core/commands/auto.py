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
import core.constants as constants
if TYPE_CHECKING: from core.robot import RobotCore

class AutoPath(Enum):
  Start_TRL_D = auto()
  Start_TRR_O = auto()
  Start_BL_D = auto()
  Start_BR_O = auto()
  BL_NL = auto()
  BR_NR = auto()
  NL_BL = auto()
  NR_BR = auto()
  Intake_NL = auto()
  Intake_NR = auto()
  Score_BL_TWL = auto()
  Score_BR_TWR = auto()
  Score_D_TWL = auto()
  Score_O_TWR = auto()
  Score_O_BR = auto()
  Score_D_BL = auto()

  BL_NZ_SCL = auto()

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
    
    self._autos.addOption("[TrenchL_D_C]", self.auto_TrenchL_D_C)
    self._autos.addOption("[BumpL_D_C]", self.auto_BumpL_D_C)
    self._autos.addOption("[BumpL_NL_C]", self.auto_BumpL_NL_C)
    self._autos.addOption("[TrenchL_D_BL]", self.auto_TrenchL_D_BL)
    self._autos.addOption("[BumpL_D_BL]", self.auto_BumpL_D_BL)
    self._autos.addOption("[BumpL_NL_BL]", self.auto_BumpL_NL_BL)

    self._autos.addOption("[TrenchR_O_C]", self.auto_TrenchR_O_C)
    self._autos.addOption("[BumpR_O_C]", self.auto_BumpR_O_C)
    self._autos.addOption("[BumpR_NR_C]", self.auto_BumpR_NR_C)
    self._autos.addOption("[TrenchR_O_BR]", self.auto_TrenchR_O_BR)
    self._autos.addOption("[BumpR_O_BR]", self.auto_BumpR_O_BR)
    self._autos.addOption("[BumpR_NR_BR]", self.auto_BumpR_NR_BR)

    self._autos.addOption("BL_NZ_SCL", self.auto_BL_NZ_SCL)

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

  def auto_BL_NZ_SCL(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BL_NZ_SCL)
    ).withName("Auto:BL_NZ_SCL")

  def auto_TrenchL_D_C(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._move(AutoPath.Start_TRL_D)
        # INTAKE
      ),
      self._move(AutoPath.Score_D_TWL)
      # SCORE
      # ADD CLIMB SEQUENCE
    ).withName("Auto:[TrenchL_D_C]")
  
  def auto_BumpL_D_C(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._move(AutoPath.Start_BL_D)
        # INTAKE
      ),
      self._move(AutoPath.Score_D_TWL)
      # SCORE
      # ADD CLIMB SEQUENCE
    ).withName("Auto:[BumpL_D_C]")
  
  def auto_BumpL_NL_C(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BL_NL),
      cmd.parallel(
        self._move(AutoPath.Intake_NL)
        # INTAKE
      ),
      self._move(AutoPath.NL_BL),
      self._move(AutoPath.Score_BL_TWL)
      # SCORE
      # ADD CLIMB SEQUENCE
    ).withName("Auto:[BumpL_NL_C]")
  
  def auto_TrenchL_D_BL(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._move(AutoPath.Start_TRL_D)
        # INTAKE
      ),
      self._move(AutoPath.Score_D_BL)
      # SCORE
    ).withName("Auto:[TrenchL_D_BL]")
  
  def auto_BumpL_D_BL(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._move(AutoPath.Start_BL_D)
        # INTAKE
      ),
      self._move(AutoPath.Score_D_BL)
      # SCORE
    ).withName("Auto:[BumpL_D_BL]")
  
  def auto_BumpL_NL_BL(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BL_NL),
      cmd.parallel(
        self._move(AutoPath.Intake_NL)
        # INTAKE
      ),
      self._move(AutoPath.NL_BL),
      # SCORE
    ).withName("Auto:[BumpL_NL_BL]")

  def auto_TrenchR_O_C(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._move(AutoPath.Start_TRR_O)
        # INTAKE
      ),
      self._move(AutoPath.Score_O_TWR)
      # SCORE
      # ADD CLIMB SEQUENCE
    ).withName("Auto:[TrenchR_O_C]")
  
  def auto_BumpR_O_C(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._move(AutoPath.Start_BR_O)
        # INTAKE
      ),
      self._move(AutoPath.Score_O_TWR)
      # SCORE
      # ADD CLIMB SEQUENCE
    ).withName("Auto:[BumpR_O_C]")
  
  def auto_BumpR_NR_C(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BR_NR),
      cmd.parallel(
        self._move(AutoPath.Intake_NR)
        # INTAKE
      ),
      self._move(AutoPath.NR_BR),
      self._move(AutoPath.Score_BR_TWR)
      # SCORE
      # ADD CLIMB SEQUENCE
    ).withName("Auto:[BumpR_NR_C]")
  
  def auto_TrenchR_O_BR(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._move(AutoPath.Start_TRR_O)
        # INTAKE
      ),
      self._move(AutoPath.Score_O_BR)
      # SCORE
    ).withName("Auto:[TrenchR_O_BR]")
  
  def auto_BumpR_O_BR(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._move(AutoPath.Start_BR_O)
        # INTAKE
      ),
      self._move(AutoPath.Score_O_BR)
      # SCORE
    ).withName("Auto:[BumpR_O_BR]")
  
  def auto_BumpR_NR_BR(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BR_NR),
      cmd.parallel(
        self._move(AutoPath.Intake_NR)
        # INTAKE
      ),
      self._move(AutoPath.NR_BR),
      # SCORE
    ).withName("Auto:[BumpR_NR_BR]")


