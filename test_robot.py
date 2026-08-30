#!/usr/bin/env python3

from constants import *
from utils import *

from robot import Robot

class Tester:
    def __init__(self, robot: Robot):
        self._robot = robot


    def calibrate_to_north(self):
        self._robot.heading = 0


    def calibrate_to_global_origin(self):
        self._robot.origin = Point(0, 0)


    def calibrate_to_zero(self):
        """REMEMBER TO CALL THIS AFTER EVERY TEST
        IF YOU MOVE THE ROBOT BACK WITH YOUR HANDS
        Don't make my fucking mistake and run multiple tests consectively
        and wonder why things are suddenly breaking.
        """
        self._robot.heading = 0
        self._robot.origin = Point(0, 0)



    # basic tests - can the robot do these things?


    def test_relative_turning(self):
        """Expect a 90deg clockwise turn on the spot."""
        self._robot.turn_by(90)
        self._robot.log()


    def test_absolute_turning(self):
        """Expect a turn ccw, then a turn exactly North"""
        self._robot.turn_by(45, clockwise=False)
        wait()
        self._robot.turn_to(Direction.NORTH)
        self._robot.log()


    def test_multiple_absolute_turning(self):
        """Expect multiple turns, then a turn exactly West"""
        print(self._robot.heading)
        self._robot.turn_by(45, clockwise=False)
        print(self._robot.heading)
        wait()
        self._robot.turn_by(80)
        print(self._robot.heading)
        wait()
        self._robot.turn_by(30, clockwise=False)
        print(self._robot.heading)
        wait()
        self._robot.turn_to(Direction.WEST)
        print(self._robot.heading)
        self._robot.log()


    def test_driving_straight(self):
        """Expect a 10cm drive backward then forward"""
        self._robot.drive(100, forward=False)
        self._robot.drive(100, forward=True)
        self._robot.log()


    def test_us_distance(self): # position robot to face a wall
        """Expect a small us_distance"""
        print(self._robot.us_distance)


    def test_us_turning(self):
        """Expect an absolute turn to East"""
        self._robot.us_turn_to(Direction.EAST)
        self._robot.log()


    def test_us_calibration(self):
        """Expect an absolute turn to North"""
        self._robot.us_calibrate_to_north()
        self._robot.log()


    def test_color(self):
        """Expect accurate color reading"""
        print(self._robot.color)


    def test_type(self):
        """Expect accurate tile/victim type detection"""
        print(self._robot.tile_type)


    # calibration tests





if __name__ == '__main__':
    input('Press enter to begin tests')

    robot = Robot()
    robot.init()

    try:
        tester = Tester(robot)
        while True:
            string = input('Enter expr (or nothing to exit): ')
            if not string: break
            eval(string, globals())

    finally:
        robot.shutdown()
