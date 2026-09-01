#!/usr/bin/env python3

from typing import Dict

from ev3dev2.led import Leds

from constants import *
from utils import wait

__all__ = [
    'indicate_fuck_up',
    'indicate_start',
    'indicate_nogo_tile',
    'indicate_harmed_victim',
    'indicate_unharmed_victim',
    'indicate_finish',
    'indicate_final_counts'
]


# Helpers


leds = Leds()


def set_color(color: str):
    leds.set_color('LEFT', color, pct=LED_BRIGHTNESS / 100) # type: ignore
    leds.set_color('RIGHT', color, pct=LED_BRIGHTNESS / 100) # type: ignore


def flash_color(color: str, time_ms: int):
    leds.animate_flash(color, sleeptime=LED_FLASH_INTERVAL_MS / 1000, duration=int(time_ms / 1000), block=True)


def clear_color():
    leds.all_off()




def indicate_fuck_up():
    """Indicate (with a bunch of flashing lights and sounds) that the robot has been FUBAR"""
    print('something went wrong')
    raise Exception


def indicate_start():
    """Pause for 1 second, indicating that the robot is starting"""
    set_color(LED_START_COLOR)
    wait(1000)
    clear_color()


def indicate_nogo_tile():
    """Pause for 1 second, indicating that the robot has hit a black tile and will be reversing"""
    set_color(LED_NOGO_COLOR)
    wait(1000)
    clear_color()


def indicate_harmed_victim():
    """Pause for 1 second, indicating that the robot has discovered a red victim"""
    flash_color(LED_HARMED_COLOR, 1000)
    clear_color()


def indicate_unharmed_victim():
    """Pause for 1 second, indicating the robot has discovered a green victim"""
    flash_color(LED_UNHARMED_COLOR, 1000)
    clear_color()


def indicate_finish():
    """Pause for 1 second, indicating that the robot has finished"""
    set_color(LED_FINISH_COLOR)
    wait(1000)
    clear_color()


def indicate_final_counts(statistics: Dict[TileType, int]):
    """Announce the final figures / statistics"""
    # TODO
    print(statistics)
    ...


