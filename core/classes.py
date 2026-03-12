from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units

class Target(Enum):
  Hub = auto()
  Shuttle = auto()
  ScoreLeft = auto()
  ScoreRight = auto()
  BumpLeftIn = auto()
  BumpLeftOut = auto()
  BumpRightIn = auto()
  BumpRightOut = auto()
  

@dataclass(frozen=True, slots=True)
class TargetLaunchMetric:
  distance: units.meters
  speed: units.percent
  time: units.seconds

class FuelLevel(IntEnum):
  Empty = 0
  Low = 1
  Mid = 2
  Full = 3

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
  RobotIsHoming = auto()
  HubStateActive = auto()
  HubStateActiveEnding = auto()
  HubStateInactive = auto()
  HubStateInactiveEnding = auto()
