#!/usr/bin/env python3

import math
import time

from shapely.geometry import Point

from constants import *
from button import backspace_pressed

__all__ = [
    'wait',
    'shift',
]


def wait(time_ms: Numeric = WAIT_TIME_MS, check_pressed: bool = True):
    """Sleep a short amount of time between movements, to prevent jamming. Also check if exit requested (via button press)"""
    time.sleep(time_ms / 1000)

    if check_pressed and backspace_pressed():
        raise ExitRequestedError


def shift(point: Point, degrees: Numeric, amount: Numeric = 1) -> Point:
    """Shift a point by <amount> along <degrees> bearing"""
    assert 0 <= degrees < 360

    # special cases: avoid trigonometry to avoid small errors like 10.000000001
    if degrees == Direction.NORTH:
        return Point(point.x, point.y + amount)
    elif degrees == Direction.EAST:
        return Point(point.x + amount, point.y)
    elif degrees == Direction.SOUTH:
        return Point(point.x, point.y - amount)
    elif degrees == Direction.WEST:
        return Point(point.x - amount, point.y)
    
    rad = math.radians(degrees)
    return Point(
        point.x + amount * math.sin(rad),
        point.y + amount * math.cos(rad)
    )




