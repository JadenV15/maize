#!/usr/bin/env python3

"""This is to diagnose why it takes 1 minute to run `python3 -u main.py` on brick.
(well more importantly, _where_ it spends that 1 minute, and how to mitigate that)
"""

import time


input('script::start> ')

# Overall script timer starts after the user presses Enter.
script_start = time.perf_counter()


print('import::constants::start')
section_start = time.perf_counter()

from constants import *

section_end = time.perf_counter()
print('import::constants::end - total: {:.4f}s'.format(
    section_end - section_start
))

print()


print('import::utils::start')
section_start = time.perf_counter()

from utils import *

section_end = time.perf_counter()
print('import::utils::end - total: {:.4f}s'.format(
    section_end - section_start
))

print()


print('import::robot::start')
section_start = time.perf_counter()

from robot import *

section_end = time.perf_counter()
print('import::robot::end - total: {:.4f}s'.format(
    section_end - section_start
))

print()


print('import::map::start')
section_start = time.perf_counter()

from map import *

section_end = time.perf_counter()
print('import::map::end - total: {:.4f}s'.format(
    section_end - section_start
))

print()


print('import::solver::start')
section_start = time.perf_counter()

from solver import *

section_end = time.perf_counter()
print('import::solver::end - total: {:.4f}s'.format(
    section_end - section_start
))

print()


print('robot::start')
section_start = time.perf_counter()

robot = Robot()
robot.init()

robot.log()

robot.shutdown()

section_end = time.perf_counter()
print('robot::end - total: {:.4f}s'.format(
    section_end - section_start
))

print()


script_end = time.perf_counter()
print('script::end - total: {:.4f}s'.format(
    script_end - script_start
))

