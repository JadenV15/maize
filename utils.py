#!/usr/bin/env python3

import math
import time
from enum import IntEnum
from typing import Union

from shapely.geometry import Point

__all__ = ['Numeric', 'Direction', 'Speed', 'Color', 'wait', 'shift_heading']

Numeric = Union[int, float] # type hint any number

class Direction(IntEnum):
    """The four cardinal directions, whose values correspond to clockwise rotation relative to the positive y-axis"""
    NORTH = 0
    EAST = 90
    SOUTH = 180
    WEST = 270

class Speed(IntEnum):
    """Different motor speeds, to be used with SpeedPercent"""
    SLOW = 10
    MEDIUM = 20
    FAST = 30

# Copied from source
class Color(IntEnum):
    NONE = 0
    BLACK = 1
    BLUE = 2
    GREEN = 3
    YELLOW = 4
    RED = 5
    WHITE = 6
    BROWN = 7

def wait():
    """Sleep a short amount of time between movements, to prevent jamming"""
    time.sleep(400 / 1000)

def shift_heading(point: Point, degrees: Numeric, amount: Numeric = 1) -> Point:
    """Shift a point by <amount> along <degrees> bearing"""
    assert 0 <= degrees < 360
    rad = math.radians(degrees)
    return Point(
        point.x + amount * math.sin(rad),
        point.y + amount * math.cos(rad)
    )