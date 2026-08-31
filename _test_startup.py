#!/usr/bin/env python3

"""This is to diagnose why it takes 1 minute to run `python3 -u main.py` on brick.
(well more importantly, _where_ it spends that 1 minute, and how to mitigate that)
"""

print('script::start')

import time

last_time = time.perf_counter()

print('import::constants::start')
from constants import *
current_time = time.perf_counter()
print('import::constants::end - total: {:.4f}s'.format(current_time - last_time))

print()

print('import::utils::start')
from utils import *
current_time = time.perf_counter()
print('import::utils::end - total: {:.4f}s'.format(current_time - last_time))

print()

print('import::robot::start')
from robot import *
current_time = time.perf_counter()
print('import::robot::end - total: {:.4f}s'.format(current_time - last_time))

print()

print('import::map::start')
from map import *
current_time = time.perf_counter()
print('import::map::end - total: {:.4f}s'.format(current_time - last_time))

print()

print('import::solver::start')
from solver import *
current_time = time.perf_counter()
print('import::solver::end - total: {:.4f}s'.format(current_time - last_time))

print()

print('robot::start')
robot = Robot()
robot.init()

robot.log()

robot.shutdown()

current_time = time.perf_counter()
print('robot::end - total: {:.4f}s'.format(current_time - last_time))

print()

current_time = time.perf_counter()
print('script::end - total: {:.4f}s'.format(current_time - last_time))

