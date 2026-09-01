#!/usr/bin/env python3

# FOLLOW THE PSEUDOCODE

# NOTE: remove all the debug stuff!
# NOTE: ensure no initial debris under the bot!

from constants import *
from utils import *

from robot import Robot
from map import Map
from solver import Solver


def main():
    # create the Robot
    robot = Robot()
    robot.init()

    try:
        map = Map()
        solver = Solver(robot, map)

        solver.solve()

    finally:
        robot.shutdown()


if __name__ == '__main__':
    input('Press enter to start...') #TODO :remove

    main()

