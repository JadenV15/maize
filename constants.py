#!/usr/bin/env python3

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

from utils import *


# MAZE:

TILE_WIDTH = 300 # check rules - i think 290?
VICTIM_WIDTH = 50


# ROBOT:

DEFAULT_SPEED = Speed.SLOW

ROBOT_WIDTH = 135
ROBOT_HEIGHT = 190

LEFT_WHEEL_PIN = OUTPUT_A
RIGHT_WHEEL_PIN = OUTPUT_B
WHEEL_TYPE = EV3Tire
WHEEL_MIDPOINT_GAP = 98 # measured: 88
WHEEL_MIDPOINT_GAP_MIDPOINT = Point(0, -28)
WHEEL_POLARITY = Motor.POLARITY_NORMAL
WHEEL_SPEED = Speed.SLOW

US_PIN = INPUT_3
US_MOTOR_PIN = OUTPUT_C
US_MOTOR_MIDPOINT = Point(0, -52)
US_MOTOR_POLARITY = Motor.POLARITY_NORMAL
US_MOTOR_SPEED = Speed.SLOW

US_NINETY_DEGREES = 97

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
MAX_WALL_DETECTION_DISTANCE = TILE_WIDTH / 2