#!/usr/bin/env python3

from robot import Robot
from map import Map
from solver import Solver

if __name__ == '__main__':
    input('enter> ')

    robot = Robot()
    robot.init()

    try:
        solver = Solver(robot, Map())

        robot.log()
        solver.advance_right()
        #robot.drive(150)
        robot.log()

    finally:
        robot.shutdown()
