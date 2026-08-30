#!/usr/bin/env python3

from typing import Union
from enum import Enum, IntEnum

from shapely.geometry import Point

from ev3dev2.wheel import Wheel, EV3Tire # PART NUMBER 44309
from ev3dev2.motor import (
    Motor,
    OUTPUT_A,
    OUTPUT_B,
    OUTPUT_C,
    OUTPUT_D
)
from ev3dev2.sensor import (
    INPUT_1,
    INPUT_2,
    INPUT_3,
    INPUT_4
)

# tyoe hints

Numeric = Union[int, float] # type hint any number


# enums

class Direction(IntEnum):
    """The four cardinal directions, whose values correspond to clockwise rotation relative to the positive y-axis"""
    NORTH = 0
    EAST = 90
    SOUTH = 180
    WEST = 270

    def reverse(self) -> 'Direction':
        """Flip this direction, i.e. add 180deg and cap to [0, 360)"""
        return type(self)((self + 180) % 360)

class Speed(IntEnum):
    """Different motor speeds, to be used with SpeedPercent"""
    SLOW = 10
    MEDIUM = 20
    FAST = 30

# Copied from source
class Color(IntEnum):
    """The different colour values returned by ev3"""
    NONE = 0
    BLACK = 1
    BLUE = 2
    GREEN = 3
    YELLOW = 4
    RED = 5
    WHITE = 6
    BROWN = 7

class TileType(Enum):
    """Types of tile surfaces. These members are all mutually exclusive
    Start: silver reflective tile (with no victim)
    Normal: white tile (with no victim)
    Nogo: black tile (with no victim)
    Harmed victim: red square (on white tile)
    Unharmed victim: green square (on white tile)
    """
    START = 0
    NORMAL = 1
    NOGO = 2
    HARMED_VICTIM = 3
    UNHARMED_VICTIM = 4

class MovementType(Enum):
    """Types of movement, as outline in the doc"""
    ADVANCE = 0
    BACKTRACK = 1
    ADVANCE_LEFT = 2
    ADVANCE_RIGHT = 3
    BACKTRACK_LEFT = 4 # opposite of ADVANCE_LEFT
    BACKTRACK_RIGHT = 5 # opposite of ADVANCE_RIGHT


# MAZE:

TILE_WIDTH = 290
VICTIM_WIDTH = 50

TILE_HALF_WIDTH = TILE_WIDTH / 2


# ROBOT CALIBRATION
# TODO


WHEEL_ROTATION_RATIO = 1 # what i coded it to rotate divided by what it actually rotated by
WHEEL_DRIVING_RATIO = 150 / 142 # what i coded it to move forward divided by what it actually travelled
US_NINETY_DEGREES = 99 # TODO

# robot movements:
STEP_2_ROTATION = 35
STEP_3_DISTANCE = 340 # diagonal distance
STEP_5_DISTANCE = 205 # extra distance from half-width. increase this to shorten step 5 distance


# ROBOT:
# all coordinates are relative to the robot origin

# TODO: appropriate default speed
DEFAULT_SPEED = Speed.SLOW

# TODO: finalise measurements
ROBOT_WIDTH = 135
ROBOT_HEIGHT = 182

LEFT_WHEEL_PIN = OUTPUT_A
RIGHT_WHEEL_PIN = OUTPUT_B
WHEEL_TYPE = EV3Tire
WHEEL_MIDPOINT_GAP = 98 # measured: 88 # TODO
WHEEL_MIDPOINT_GAP_MIDPOINT = Point(0, -28)
WHEEL_POLARITY = Motor.POLARITY_NORMAL

US_PIN = INPUT_3
US_MOTOR_PIN = OUTPUT_C
US_MOTOR_MIDPOINT = Point(0, -52)
US_MOTOR_POLARITY = Motor.POLARITY_NORMAL
US_MOTOR_SPEED = Speed.SLOW

CS_PIN = INPUT_4
CS_MIDPOINT = Point(0, 0) # technically 'ctrpoint'

LEFT_TS_PIN = INPUT_1
LEFT_TS_ENDPOINT = Point(- ROBOT_WIDTH / 2, - ROBOT_HEIGHT + 28)
RIGHT_TS_PIN = INPUT_2
RIGHT_TS_ENDPOINT = Point(ROBOT_WIDTH / 2, - ROBOT_HEIGHT + 28)

DISTANCE_BETWEEN_ORIGIN_AND_TURNING_ORIGIN = CS_MIDPOINT.distance(WHEEL_MIDPOINT_GAP_MIDPOINT)

# used for wall detection
'''
|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|   T
|               |   | ← this distance
|   ____.____   |   ⊥
|  |         |  |
|  |         |  |
|__|_________|__|
   |         |
    ‾‾‾‾‾‾‾‾‾
If the detected distance is <= this, then there is a wall in front of the US.
If the distance is > this, there is open space in front of US.
'''
MAX_WALL_DETECTION_DISTANCE = TILE_HALF_WIDTH

# used for silver tile detection
REFLECTED_LIGHT_THRESHOLD = 75