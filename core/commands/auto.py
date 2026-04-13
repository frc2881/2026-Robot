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
  BUMP_LEFT_LOOP = auto()
  BUMP_LEFT_CENTER = auto()
  BUMP_RIGHT_LOOP = auto()
  DEPOT = auto()
  DEPOT_RIGHT = auto()
  HUB = auto()
  CUSTOM = auto()

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
    self._autos.setDefaultOption("0: None", cmd.none)
    
    self._autos.addOption("1: Bump Left > Loop > Depot", self.auto_BUMP_LEFT_LOOP_DEPOT)
    self._autos.addOption("2: Bump Left > Center > Depot", self.auto_BUMP_LEFT_CENTER_DEPOT)
    self._autos.addOption("3: Bump Right > Loop", self.auto_BUMP_RIGHT_LOOP)
    self._autos.addOption("4: Hub > Depot", self.auto_HUB_DEPOT)
    self._autos.addOption("5: Custom", self.auto_CUSTOM)

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
  
  def auto_BUMP_LEFT_LOOP_DEPOT(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BUMP_LEFT_LOOP).deadlineFor(
        cmd.waitSeconds(1.25).andThen(self._robot.game.runIntake().deadlineFor(self._robot.game.setTurretHeading(190.0)))
      ),
      self._robot.game.launchFuel(Target.Hub).deadlineFor(
        self._move(AutoPath.DEPOT_RIGHT).deadlineFor(self._robot.game.runIntake()),
        self._robot.game.agitateIntake(),
        cmd.waitSeconds(1.5).andThen(self._robot.game.agitateHopper())
      )
    ).withName("Auto:BUMP_LEFT_LOOP_DEPOT")

  def auto_BUMP_LEFT_CENTER_DEPOT(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BUMP_LEFT_CENTER).deadlineFor(
        cmd.waitSeconds(2.5).andThen(self._robot.game.runIntake().deadlineFor(self._robot.game.setTurretHeading(190.0)))
      ),
      self._robot.game.launchFuel(Target.Hub).deadlineFor(
        self._move(AutoPath.DEPOT_RIGHT).deadlineFor(self._robot.game.runIntake()),
        self._robot.game.agitateIntake(),
        cmd.waitSeconds(1.5).andThen(self._robot.game.agitateHopper())
      )
    ).withName("Auto:BUMP_LEFT_CENTER_DEPOT")

  def auto_BUMP_RIGHT_LOOP(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.BUMP_RIGHT_LOOP).deadlineFor(
        cmd.waitSeconds(1.25).andThen(self._robot.game.runIntake().deadlineFor(self._robot.game.setTurretHeading(75.0)))
      ),
      self._robot.game.launchFuel(Target.Hub).deadlineFor(
        self._robot.game.agitateIntake(),
        cmd.waitSeconds(1.5).andThen(self._robot.game.agitateHopper())
      )
    ).withName("Auto:BUMP_RIGHT_LOOP")

  def auto_HUB_DEPOT(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.HUB).deadlineFor(
        cmd.waitSeconds(0.5).andThen(self._robot.game.runIntake().deadlineFor(self._robot.game.setTurretHeading(105.0)))
      ),
      self._move(AutoPath.DEPOT).deadlineFor(self._robot.game.runIntake()),
      self._robot.game.launchFuel(Target.Hub).deadlineFor(
        self._robot.game.agitateIntake()
      )
    ).withName("Auto:HUB_DEPOT")
  
  def auto_CUSTOM(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.CUSTOM).deadlineFor(
        self._robot.game.setTurretHeading(180.0)
      ),
      self._robot.game.launchFuel(Target.Hub).deadlineFor(
        self._robot.game.agitateIntake()
      )
    ).withName("Auto:CUSTOM")
