#!/usr/bin/env python3

"""Main entry point. This performs the slow imports, then stands by"""

import time

start_time = time.perf_counter()

print('Starting imports...')

from constants import ExitRequestedError
from indicate import indicate_fuck_up
from button import *

from main import main

print('Imported in {:.4f}'.format(time.perf_counter() - start_time))

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


