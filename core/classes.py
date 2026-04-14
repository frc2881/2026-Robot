from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units

class Target(Enum):
  Hub = auto()
  ShuttleLeft = auto()
  ShuttleRight = auto()
  ScoreLeft = auto()
  ScoreRight = auto()
  BumpLeftInOut = auto()
  BumpLeftOutIn = auto()
  BumpRightInOut = auto()
  BumpRightOutIn = auto()
  
@dataclass(frozen=True, slots=True)
class TargetLaunchMetric:
  distance: units.meters
  speed: units.percent
  time: units.seconds

@dataclass(frozen=False, slots=True)
class TargetLaunchInfo:
  distance: units.meters = 0
  speed: units.percent = 0
  heading: units.degrees = 0
  isDistanceValid: bool = False
  isHeadingValid: bool = False

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
