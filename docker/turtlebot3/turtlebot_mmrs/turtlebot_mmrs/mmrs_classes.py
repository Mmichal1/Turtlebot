from typing import List
from dataclasses import dataclass
from shapely.geometry import Point
from enum import Enum, unique, auto


@unique
class TriggerType(Enum):
    SIGNAL_TRIGGER = 1.05
    STOP_TRIGGER = 0.5


@dataclass
class TriggerData:
    position: Point
    trigger_type: TriggerType
    restricted_area_id: int


class TaskStatus(Enum):
    ONGOING = auto()
    CANCELLED = auto()
    COMPLETED = auto()


class UniqueQueue:
    def __init__(self):
        self.queue = []

    def enqueue(self, item):
        if item in self.queue:
            raise ValueError(f"Robot '{item}' already in the queue")
        self.queue.append(item)

    def dequeue(self):
        if self.is_empty():
            raise IndexError("Dequeue from empty queue")
        return self.queue.pop(0)

    def is_empty(self):
        return len(self.queue) == 0

    def size(self):
        return len(self.queue)
