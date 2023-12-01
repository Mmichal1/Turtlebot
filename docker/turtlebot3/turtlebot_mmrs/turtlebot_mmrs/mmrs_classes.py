from dataclasses import dataclass
from shapely.geometry import Point
from enum import Enum, unique


@unique
class TriggerType(Enum):
    SIGNAL_TRIGGER = 1.05
    STOP_TRIGGER = 0.5


@dataclass
class TriggerData:
    position: Point
    trigger_type: TriggerType
    restricted_area_id: int
