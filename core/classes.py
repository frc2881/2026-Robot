from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units

class Target(Enum):
  Hub = auto()
  TowerLeft = auto()
  TowerRight = auto()
  TrenchLeft = auto()
  TrenchRight = auto()
  CornerLeft = auto()
  CornerRight = auto()
  Outpost = auto()
  Depot = auto()

@dataclass(frozen=True, slots=True)
class TargetLaunchSpeed:
  distance: units.meters
  speed: units.percent

class MatchState(Enum):
  Stopped = auto()
  Auto = auto()
  Transition = auto()
  Shift1 = auto()
  Shift2 = auto()
  Shift3 = auto()
  Shift4 = auto()
  EndGame = auto()

class HubState(Enum):
  Inactive = auto()
  Active = auto()

class LightsMode(Enum):
  Default = auto()
  RobotNotConnected = auto()
  RobotNotHomed = auto()
  VisionNotReady = auto()
  LaunchReady = auto()
