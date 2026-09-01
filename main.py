#!/usr/bin/env python3

"""Main entry point. This performs the slow imports, then stands by to run the main dfs"""

# NOTE: remove all the debug stuff!
# NOTE: ensure no initial debris under the bot!

import time

start_time = time.perf_counter()

print('Starting imports...')

from constants import *
from utils import *

from robot import Robot
from map import Map
from solver import Solver

from indicate import indicate_fuck_up
from button import *

print('Imported in {:.4f}'.format(time.perf_counter() - start_time))

# main func

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

# main loop

while True:
    print('Waiting for signal...')
    wait_for_enter_press()

    print('Started')
    try:
        main()

    except ExitRequestedError:
        print('Stopped')
        reset_backspace()
        continue # wait for Robot Handler to press enter again to restart

    except Exception as e:
        print('Error:')
        print(e)
        indicate_fuck_up()
        continue # try again, i guess?



