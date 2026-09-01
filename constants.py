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


# exc
class ExitRequestedError(Exception):
    """The Robot Handler has pressed a button requesting to terminate the program"""
    pass


# enums

class Direction(IntEnum):
    """The four cardinal directions, whose values correspond to clockwise rotation relative to the positive y-axis"""
    NORTH = 0
    EAST = 90
    SOUTH = 180
    WEST = 270

    def rotate(self, amount: Numeric, clockwise: bool = True) -> Numeric:
        """Rotate this direction by <amount> in a <clockwise> direction"""
        assert 0 <= self < 360

        amount *= 1 if clockwise else -1
        return (self + amount) % 360 # return [0, 360)

    def reverse(self) -> 'Direction':
        """Flip this direction, i.e. add 180deg and wrap to [0, 360)"""
        return type(self)((self + 180) % 360)

    @classmethod
    def from_heading(cls, degrees: Numeric) -> 'Direction':
        """Convert a heading to *nearest* direction"""
        assert 0 <= degrees < 360
        # cool: min takes any iterable, including enums
        return min(
            cls,
            key=lambda direction: abs((degrees - direction + 180) % 360 - 180)
        )


class Speed(IntEnum):
    """Different motor speeds, to be used with SpeedPercent"""
    SLOW = 10
    MEDIUM = 25
    FAST = 40


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


# this is because my table at home is black. remember to switch these back on comp day TODO
NORMAL_TILE_COLOR = Color.BLACK # Color.WHITE
NOGO_TILE_COLOR = Color.WHITE # Color.BLACK

HARMED_VICTIM_COLOR = Color.RED
UNHARMED_VICTIM_COLOR = Color.GREEN


# SLEEP
WAIT_TIME_MS = 150
# BUTTONS
BUTTON_POLL_INTERVAL_MS = 300
# Drive wall
DRIVE_WALL_POLL_INTERVAL_MS = 100 # how often to read from US
# Tile type
TILE_TYPE_POLL_INTERVAL_MS = 100 # how often to read from CS


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
STEP_5_A_DISTANCE = TILE_HALF_WIDTH / 2 # TODO - see docstring. either 1/2 or 1/3, I think.
ADVANCE_MVT_DISTANCE = TILE_HALF_WIDTH / 2 # TODO - see robot.advance(). similar to above

# drive until hit wall:
DRIVE_WALL_MAX_DISTANCE = 2 * TILE_WIDTH # worst case, if stop condition never met
DRIVE_WALL_EXTRA_DISTANCE = 10 # TODO - extra distance to ensure heading is aligned

# ROBOT:
# all coordinates are relative to the robot origin

# TODO: appropriate default speeds
DEFAULT_STRAIGHT_SPEED = Speed.MEDIUM
DEFAULT_TURNING_SPEED = Speed.SLOW

# TODO: finalise measurements
ROBOT_WIDTH = 135
ROBOT_HEIGHT = 182

LEFT_WHEEL_PIN = OUTPUT_A
RIGHT_WHEEL_PIN = OUTPUT_D
WHEEL_TYPE = EV3Tire
WHEEL_MIDPOINT_GAP = 98 # measured: 88 # TODO
WHEEL_MIDPOINT_GAP_MIDPOINT = Point(0, -28)
WHEEL_POLARITY = Motor.POLARITY_NORMAL

US_PIN = INPUT_3
US_MOTOR_PIN = OUTPUT_B
US_MOTOR_MIDPOINT = Point(0, -52)
US_MOTOR_POLARITY = Motor.POLARITY_NORMAL
US_MOTOR_SPEED = Speed.MEDIUM # TODO

CS_PIN = INPUT_4
CS_MIDPOINT = Point(0, 0) # technically 'ctrpoint'

LEFT_TS_PIN = INPUT_1
LEFT_TS_ENDPOINT = Point(- ROBOT_WIDTH / 2, - ROBOT_HEIGHT + 28)
RIGHT_TS_PIN = INPUT_2
RIGHT_TS_ENDPOINT = Point(ROBOT_WIDTH / 2, - ROBOT_HEIGHT + 28)

DISTANCE_BETWEEN_ORIGIN_AND_TURNING_ORIGIN = CS_MIDPOINT.distance(WHEEL_MIDPOINT_GAP_MIDPOINT)

# used for wall detection
MAX_WALL_DETECTION_DISTANCE = TILE_HALF_WIDTH + 30 # TODO - added a buffer
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

# used for silver tile detection
REFLECTED_LIGHT_THRESHOLD = 75