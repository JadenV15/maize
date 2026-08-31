#!/usr/bin/env python3

from robot import Robot
from map import Map
from solver import Solver

if __name__ == '__main__':
    input('enter> ')

    robot = Robot()
    robot.init()

    try:
        solver = Solver(robot, Map(), test_initial_only=True, test_without_calibration=True)

        robot.log()

        # THINGS TO TEST:
        # 

        robot.log()

    finally:
        robot.shutdown()
