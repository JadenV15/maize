#!/usr/bin/env python3

from contextlib import contextmanager
import math
from typing import List, Optional, Union

from shapely.geometry import Point

from ev3dev2.motor import (
    SpeedPercent,
    MediumMotor,
    MoveDifferential
)
from ev3dev2.sensor.lego import (
    Sensor,
    TouchSensor,
    ColorSensor,
    UltrasonicSensor
)

from constants import *
from utils import *

__all__ = ['Robot']



class Robot:
    """Our beloved Robot, built by Shaurya. Provides all basic physical movement"""


    def init(self):
        """Initialise the robot. The robot may move."""

        # use MoveDifferential to control both wheels at once
        self._drive = MoveDifferential(LEFT_WHEEL_PIN, RIGHT_WHEEL_PIN, WHEEL_TYPE, WHEEL_MIDPOINT_GAP)
        self._drive.set_polarity(WHEEL_POLARITY)
        # start odometry. 90deg points to the positive y-axis
        self._drive.odometry_start(theta_degrees_start=90)

        self._us = UltrasonicSensor(US_PIN)
        # medium motor, not large
        self._us_motor = MediumMotor(US_MOTOR_PIN)
        self._us_motor.polarity = US_MOTOR_POLARITY
        # keep track of US direction. initially pointing north
        self._us_rel_direction = Direction.NORTH

        self._cs = ColorSensor(CS_PIN)

        # dual touch sensors
        self._ts_map = {
            Direction.WEST: (LEFT_TS_ENDPOINT, TouchSensor(LEFT_TS_PIN)),
            Direction.EAST: (RIGHT_TS_ENDPOINT, TouchSensor(RIGHT_TS_PIN))
        }


    def shutdown(self):
        """Shutdown the robot"""
        self._drive.odometry_stop()


    # geometry utils


    @property
    def turning_origin(self) -> Point:
        r"""The turning origin of the robot relative to the global origin.
        Turning origin is the point marked '@' in the diagram.
        The robot rotates around this point, NOT the actual origin (CS).
              N
    |=========*=========|  # CS
    |                   |
    |  |      @      |  |  # wheels and wheel axle midpoint
    |                   |
    |         *         |  # US
    |                   |
    |                   |
    |                   |
   //                   \\ # touch sensors
    |                   |
    |=========|=========|
              S
        """
        # the turning origin is what's measured by odometry.
        # we can simply read the odometry coords
        return Point(self._drive.x_pos_mm, self._drive.y_pos_mm)


    @turning_origin.setter
    def turning_origin(self, origin: Point):
        """Calibrate the turning origin to <origin>.
        Note: this does nothing physically.
        It just shifts the odometry system to <origin>.
        """
        self._drive.x_pos_mm, self._drive.y_pos_mm = origin.x, origin.y


    @property
    def origin(self) -> Point:
        r"""The origin of the robot relative to the global origin
        Origin is the point marked '@' in the diagram.
        "Origin" is synonymous with "robot origin" "front midpoint" and "CS midpoint"
              N
    |=========@=========|  # CS
    |                   |
    |  |      *      |  |  # wheels and wheel axle midpoint
    |                   |
    |         *         |  # US
    |                   |
    |                   |
    |                   |
   //                   \\ # touch sensors
    |                   |
    |=========|=========|
              S
        """
        # as you can see in the diagram
        # the origin is north of the turning origin
        # so we just need to shift it along <heading>
        return shift(self.turning_origin, self.heading, DISTANCE_BETWEEN_ORIGIN_AND_TURNING_ORIGIN)


    @origin.setter
    def origin(self, origin: Point):
        """Calibrate the origin to <origin>.
        Note: this does nothing physically.
        """
        # shift backwards along <heading> to get turning origin 
        self.turning_origin = shift(origin, self.heading, - DISTANCE_BETWEEN_ORIGIN_AND_TURNING_ORIGIN)


    @property
    def heading(self) -> Numeric:
        """The heading of the robot relative to global north. This is 0deg initially
                    N
                    |
                    |
                    |
                    |
      W --------|===|===|-------- E
                |   |   |
                |   |   |
                |   |   |
                |   |   |
                |===|===|
                    S
        """
        # EV3 theta is measured CCW from +X (East); we convert it to our
        # heading system, measured CW from North.
        value = (90 - math.degrees(self._drive.theta)) % 360
        # there was an edge case where smth like 0.00001 % 360 became near 360, causing approx. [0, 360] instead of [0, 360)
        return 0 if value < 0 or math.isclose(value, 360, abs_tol=0.1) else value


    @heading.setter
    def heading(self, degrees: Numeric):
        """Calibrate the heading to <degrees>
        Note: this does nothing physically.
        """
        # inverse of the above: convert our heading back to EV3 theta.
        assert 0 <= degrees < 360
        self._drive.theta = math.radians((90 - degrees) % 360)


    def log(self):
        print(
            'Origin: {}\n'
            'Turning origin: {}\n'
            'Heading: {}\n'
            'US Direction: {}\n'.format(
                self.origin,
                self.turning_origin,
                self.heading,
                self.us_rel_direction,
            )
        )


    # motion


    def turn_to(self, degrees: Numeric, clockwise: Optional[bool] = None, speed: Speed = DEFAULT_TURNING_SPEED):
        """Turn to an absolute global heading.
        <degrees> must be in [0, 360)
        If <clockwise> not specified, choose the shortest length, other follow <clockwise>
        """
        assert 0 <= degrees < 360
        # find signed amount to turn by
        delta = (degrees - self.heading + 180) % 360 - 180
        # clockwise override
        if clockwise is True and delta < 0:
            delta += 360
        elif clockwise is False and delta > 0:
            delta -= 360

        # account for correction
        corrected_delta = delta * WHEEL_ROTATION_RATIO
        offset = delta - corrected_delta
        
        self._drive.turn_degrees(
            SpeedPercent(speed),
            corrected_delta,
        )

        self.heading += offset

        # TODO: should we treat the requested angle as the source of truth?
        #self.calibrate_heading(degrees)


    def turn_by(self, degrees: Numeric, clockwise: bool = True, speed: Speed = DEFAULT_TURNING_SPEED):
        """Turn by <degrees> <clockwise>
        <degrees> must be in [0, 360)
        """
        assert degrees >= 0
        # For some reason, turn_degrees accepts negative degrees, not negative speed
        degrees *= 1 if clockwise else -1

        # account for correction
        corrected_degrees = degrees * WHEEL_ROTATION_RATIO
        offset = degrees - corrected_degrees

        self._drive.turn_degrees(SpeedPercent(speed), corrected_degrees)

        self.heading += offset


    def drive(self, distance: Numeric, forward: bool = True, speed: Speed = DEFAULT_STRAIGHT_SPEED):
        """Drive <forward> by <distance>"""
        assert distance >= 0

        # account for correction
        # here its easier to just manually take charge of
        # self._drive.x_pos_mm, self._drive.y_pos_mm
        current_pos = self.origin
        delta = distance * (1 if forward else -1)
        new_pos = shift(current_pos, self.heading, delta)

        # account for correction
        corrected_distance = distance * WHEEL_DRIVING_RATIO
        self._drive.on_for_distance(SpeedPercent((1 if forward else -1) * speed), corrected_distance)

        # override the current coordinates
        self.origin = new_pos


    @contextmanager
    def driving(self, forward: bool = True, speed: Speed = DEFAULT_STRAIGHT_SPEED):
        """Start driving <forward> at <speed>"""
        # account for correction
        initial_pos = self.origin

        motor_speed = SpeedPercent((1 if forward else -1) * speed)
        self._drive.on(motor_speed, motor_speed)

        try:
            yield
        finally:
            self._drive.off()

            # account for correction
            distance = initial_pos.distance(self.origin)
            actual_distance = distance / WHEEL_DRIVING_RATIO

            new_pos = shift(initial_pos, self.heading, actual_distance)
            self.origin = new_pos


    @property
    def is_moving(self):
        """Check if the motors are moving normally.
        "moving normally" means running, not stalled, not overloaded
        NOTE: this DOESN'T WORK (always True) for our robot because the wheels keep spinning after the robot is blocked. Avoid this
        NOTE: i'm tempted to just remove this method
        """
        return self._drive.is_running and not self._drive.is_stalled and not self._drive.is_overloaded


    # ultrasonic


    @property
    def us_rel_direction(self) -> Direction:
        """The current relative direction (relative to the robot's frame) that the US is pointed to"""
        return self._us_rel_direction


    @property
    def us_distance(self) -> Numeric:
        """The distance detected by US"""
        return self._us.distance_centimeters * 10


    def us_turn_to(self, direction: Direction):
        """Turn to a relative direction"""
        assert direction != Direction.SOUTH
        assert self._us_rel_direction != Direction.SOUTH

        # dont turn if already there
        if self._us_rel_direction == direction:
            return

        if direction in (Direction.EAST, Direction.WEST):
            # because our robot has a guard preventing the US turning outside of 90deg left and right
            # so we can just turn until detect hit guard
            self._us_motor.on(SpeedPercent((-1 if direction == Direction.WEST else 1) * US_MOTOR_SPEED))
            self._us_motor.wait_until_not_moving()
            self._us_motor.off()

        elif direction == Direction.NORTH:
            # move from the left/right, to the center (north)
            self._us_motor.on_for_degrees(SpeedPercent((1 if self._us_rel_direction == Direction.WEST else -1) * US_MOTOR_SPEED), degrees=US_NINETY_DEGREES)

        self._us_rel_direction = direction


    def us_calibrate_to_north(self):
        """Turn to exact north (involves an extra turn)"""
        self._us_motor.on(US_MOTOR_SPEED)
        # turn to left/right
        self._us_motor.wait_until_not_moving()
        self._us_motor.off()
        wait() # wait for motor to stop moving before turning sensor to north
        
        self._us_motor.on_for_degrees(SpeedPercent(-1 * US_MOTOR_SPEED), degrees=US_NINETY_DEGREES)
        self._us_rel_direction = Direction.NORTH


    @property
    def is_open(self) -> bool:
        """Is there a wall in front of the robot?
        NOTE: we assume the robot is centred in a tile (so equal distance on either side)
        """
        return self.us_distance > MAX_WALL_DETECTION_DISTANCE


    def lookaround(self) -> List[Direction]:
        """Look to the (relative) north and east and west with the US, and return which of those directions are open (no wall)
        Instead of naively turning the sensor to each direction in turn, we try to minimise the amount of time / number of moves
        """
        directions = []

        # save me some typing. this helper turns to a direction and adds it to the list if its open
        def add_if_open(d: Direction):
            self.us_turn_to(d)
            if self.is_open:
                directions.append(d)

        # first look to the current direction (no turning required)
        current = self.us_rel_direction
        add_if_open(current)

        # visit each of the other two directions in an efficient order
        if current == Direction.NORTH:
            add_if_open(Direction.WEST)
            wait()
            add_if_open(Direction.EAST)

        elif current == Direction.EAST:
            add_if_open(Direction.NORTH)
            wait()
            add_if_open(Direction.WEST)

        elif current == Direction.WEST:
            add_if_open(Direction.NORTH)
            wait()
            add_if_open(Direction.EAST)

        else:
            raise Exception

        # TODO: do we restore to NORTH?

        return directions


    # colour!


    @property
    def color(self) -> Color:
        """The current colour at the CS"""
        return self._cs.color # type: ignore


    @property
    def tile_type(self) -> TileType:
        """What type of tile or victim is under the robot origin based on the color?"""
        col = self._cs.color

        # check light reflection first
        if self._cs.reflected_light_intensity >= REFLECTED_LIGHT_THRESHOLD:
            return TileType.START
        
        elif col == NORMAL_TILE_COLOR:
            return TileType.NORMAL
        
        elif col == NOGO_TILE_COLOR:
            return TileType.NOGO
        
        elif col == UNHARMED_VICTIM_COLOR:
            return TileType.UNHARMED_VICTIM
        
        elif col == HARMED_VICTIM_COLOR:
            return TileType.HARMED_VICTIM
        
        else:
            # lets hope its just a glitch
            return TileType.NORMAL


    # touch sensor


    def is_touching_rel_direction(self, direction: Direction) -> bool: # relative direction
        """Check whether the robot has been touched (pressed) on <direction> side"""
        return bool(self._ts_map[direction][1].is_pressed)


    def get_touching_rel_points(self) -> List[Point]:
        """Get a list of all currently touched points"""
        return [point for point, touch in self._ts_map.values() if touch.is_pressed]

