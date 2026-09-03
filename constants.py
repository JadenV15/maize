#!/usr/bin/env python3

# i hate this underscore convention. maybe python should have js-style exports?
from typing import Union as _Union
from enum import Enum as _Enum, IntEnum as _IntEnum

from shapely.geometry import Point as _Point

from ev3dev2.wheel import EV3Tire as _EV3Tire # PART NUMBER 44309
from ev3dev2.motor import (
    Motor,
    OUTPUT_A as _A,
    OUTPUT_B as _B,
    OUTPUT_C as _C,
    OUTPUT_D as _D
)
from ev3dev2.sensor import (
    INPUT_1 as _1,
    INPUT_2 as _2,
    INPUT_3 as _3,
    INPUT_4 as _4
)



# ==== TYPING ====

Numeric = _Union[int, float]



# ==== EXCEPTIONS ====

class ExitRequestedError(Exception):
    """The Robot Handler requests to terminate the program"""
    pass



# ==== ENUMS ====

class Direction(_IntEnum):
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


class Speed(_IntEnum):
    """Different motor speeds, to be used with SpeedPercent"""
    SLOW = 10
    MEDIUM = 25
    FAST = 40


class Color(_IntEnum):
    """The different colour values returned by ev3"""
    NONE = 0
    BLACK = 1
    BLUE = 2
    GREEN = 3
    YELLOW = 4
    RED = 5
    WHITE = 6
    BROWN = 7


class TileType(_Enum):
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



# ==== MAZE ====

# TODO: revert to original
NORMAL_TILE_COLOR = Color.WHITE
NOGO_TILE_COLOR = Color.BLACK
HARMED_VICTIM_COLOR = Color.RED
UNHARMED_VICTIM_COLOR = Color.GREEN

# to calibrate

TILE_WIDTH = 290 # +/- 15%
VICTIM_WIDTH = 50

TILE_HALF_WIDTH = TILE_WIDTH / 2



# ==== ROBOT ====

# TODO: finalise measurements
ROBOT_WIDTH = 120
ROBOT_HEIGHT = 182

# to calibrate

EAST_WEST_WALL_US_DISTANCE = 110 # expected/'perfect' distances to walls

WHEEL_ROTATION_RATIO = 1 / 1 # programmed : actual
WHEEL_DRIVING_RATIO = 1 / 1 # programmed : actual
US_NINETY_DEGREES = 100 # TODO

DEFAULT_STRAIGHT_SPEED = Speed.MEDIUM
DEFAULT_TURNING_SPEED = Speed.SLOW
US_MOTOR_SPEED = Speed.FAST

WHEEL_MIDPOINT_GAP = 98 # measured: 88

MAX_WALL_DETECTION_DISTANCE = EAST_WEST_WALL_US_DISTANCE + 40 # buffer

REFLECTED_LIGHT_THRESHOLD = 95 # silver >= this

# config

LEFT_WHEEL_PIN = _A
RIGHT_WHEEL_PIN = _D
WHEEL_TYPE = _EV3Tire
WHEEL_MIDPOINT_GAP_MIDPOINT = _Point(0, -28)
WHEEL_POLARITY = Motor.POLARITY_NORMAL

US_PIN = _3
US_MOTOR_PIN = _B
US_MOTOR_MIDPOINT = _Point(0, -52)
US_MOTOR_POLARITY = Motor.POLARITY_NORMAL

CS_PIN = _4
CS_MIDPOINT = _Point(0, 0) # the robot origin

LEFT_TS_PIN = _1
RIGHT_TS_PIN = _2

DISTANCE_BETWEEN_ORIGIN_AND_TURNING_ORIGIN = CS_MIDPOINT.distance(WHEEL_MIDPOINT_GAP_MIDPOINT)



# ==== ROBOT MOVEMENT ====

# to calibrate

WAIT_TIME_MS = 100

NOGO_DETECTION_BUFFER = 40 # buffer

ADVANCE_A_DISTANCE = TILE_HALF_WIDTH + NOGO_DETECTION_BUFFER
ADVANCE_B_DISTANCE = TILE_HALF_WIDTH - NOGO_DETECTION_BUFFER
ADVANCE_CALIBRATE_IGNORE_OFFSET = 7
ADVANCE_CALIBRATE_BUFFER = 15
ADVANCE_CALIBRATE_DEGREES = 15

STEP_2_ROTATION = 36
STEP_3_DISTANCE = 210
STEP_4_ROTATION = 57 # roughly 90 - STEP_2_ROTATION
STEP_5_DISTANCE = 140
STEP_5_A_DISTANCE = STEP_5_DISTANCE - TILE_HALF_WIDTH + NOGO_DETECTION_BUFFER
STEP_5_B_DISTANCE = TILE_HALF_WIDTH - NOGO_DETECTION_BUFFER

DRIVE_WALL_MAX_DISTANCE = 2 * TILE_WIDTH # worst case, if stop condition never met
DRIVE_WALL_EXTRA_DISTANCE = 5 # extra distance
DRIVE_WALL_POLL_INTERVAL_MS = 150



# ==== INDICATION / ANNOUNCEMENT ====

LED_BRIGHTNESS = 100 # percent [0, 100]

LED_START_COLOR = 'YELLOW'
LED_NOGO_COLOR = 'YELLOW'
LED_HARMED_COLOR = 'RED'
LED_UNHARMED_COLOR = 'GREEN'
LED_FINISH_COLOR = 'YELLOW'



# ==== BUTTONS ==== #

BUTTON_POLL_INTERVAL_MS = 300


__all__ = [name for name in globals() if not name.startswith('_')] # type: ignore
