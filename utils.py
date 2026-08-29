#!/usr/bin/env python3

import math
import time

from shapely.geometry import Point

from constants import Numeric, Direction

__all__ = [
    'wait',
    'shift',
    'rotate'
]


def wait():
    """Sleep a short amount of time between movements, to prevent jamming"""
    time.sleep(400 / 1000)


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


def rotate(degrees: Numeric, amount: Numeric, clockwise: bool = True) -> Numeric:
    """Rotate a heading (<degrees>) by <amount> in a <clockwise> direction"""
    assert 0 <= degrees < 360

    amount *= 1 if clockwise else -1
    return (degrees + amount) % 360 # return [0, 360)


