#!/usr/bin/env python3

import math
import time

from shapely.geometry import Point

from constants import *
from button import enter_pressed

__all__ = [
    'wait',
    'shift',
    'signed_distance_to'
]



def wait(time_ms: Numeric = WAIT_TIME_MS, check_pressed: bool = True):
    """Sleep a short amount of time between movements, to prevent jamming. Also check if exit requested (via button press)"""
    time.sleep(time_ms / 1000)

    if check_pressed and enter_pressed():
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


def signed_distance_to(start: Point, target: Point, direction: Direction) -> Numeric:
    """How far is it from <start> to <target> along <direction>?"""
    delta_x = target.x - start.x
    delta_y = target.y - start.y

    if direction == Direction.NORTH:
        return delta_y
    if direction == Direction.EAST:
        return delta_x
    if direction == Direction.SOUTH:
        return -delta_y
    if direction == Direction.WEST:
        return -delta_x

    raise ValueError(direction)


