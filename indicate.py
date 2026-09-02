#!/usr/bin/env python3

from typing import Dict

from ev3dev2.led import Leds
from ev3dev2.sound import Sound

from constants import *
from utils import wait


sound = Sound()

leds = Leds()


# Helpers


def start_beep():
    sound.play_tone(frequency=1000, duration=500 / 1000, play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE)


def set_color(color: str):
    leds.set_color('LEFT', color, pct=LED_BRIGHTNESS / 100) # type: ignore
    leds.set_color('RIGHT', color, pct=LED_BRIGHTNESS / 100) # type: ignore


def flash_color(color: str, time_ms: int):
    leds.animate_flash(color, sleeptime=200 / 1000, duration=int(time_ms / 1000), block=True)


def clear_color():
    leds.all_off()




def indicate_fuck_up():
    """Indicate (with a bunch of flashing lights and sounds) that the robot has been FUBAR"""
    print('something went wrong') # TODO
    raise Exception


def indicate_start():
    """Pause for 1 second, indicating that the robot is starting"""
    start_beep()
    set_color(LED_START_COLOR)
    wait(1000)
    clear_color()


def indicate_nogo_tile():
    """Pause for half a second, indicating that the robot has hit a black tile and will be reversing"""
    start_beep()
    set_color(LED_NOGO_COLOR)
    wait(500)
    clear_color()


def indicate_harmed_victim():
    """Pause for 1 second, indicating that the robot has discovered a red victim"""
    start_beep()
    flash_color(LED_HARMED_COLOR, 1000)
    clear_color()


def indicate_unharmed_victim():
    """Pause for 1 second, indicating the robot has discovered a green victim"""
    start_beep()
    flash_color(LED_UNHARMED_COLOR, 1000)
    clear_color()


def indicate_finish():
    """Pause for 1 second, indicating that the robot has finished"""
    start_beep()
    set_color(LED_FINISH_COLOR)
    wait(1000)
    clear_color()


def indicate_final_counts(statistics: Dict[TileType, int]):
    """Announce the final figures / statistics"""
    # for now:
    print(statistics)

    def announce():
        sound.speak('{} red victims'.format(statistics[TileType.HARMED_VICTIM]))
        wait(300)
        sound.speak('{} green victims'.format(statistics[TileType.UNHARMED_VICTIM]))
        wait(300)
        sound.speak('{} black tiles'.format(statistics[TileType.NOGO]))

    announce()
    wait(1000)
    sound.speak('Repeating')
    announce()


__all__ = [name for name in globals() if name.startswith('indicate')] # type: ignore

