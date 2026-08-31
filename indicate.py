#!/usr/bin/env python3

from typing import Dict

from constants import TileType


def indicate_fuck_up():
    """Indicate (with a bunch of flashing lights and sounds) that the robot has been FUBAR"""
    print('something went wrong')
    ...

def indicate_start():
    """Pause for 1 second, indicating that the robot is starting"""
    print('start')
    ...

def indicate_black_tile():
    """(without pausing) indicate the robot has hit a black tile and will be reversing"""
    print('black')
    ...

def indicate_harmed_victim():
    """Pause for 1 second, indicating that the robot has discovered a red victim"""
    print('harmed')
    ...

def indicate_unharmed_victim():
    """Pause for 1 second, indicating the robot has discovered a green victim"""
    print('unharmed')
    ...

def indicate_finish():
    """(without pausing) indicate that the robot has finished"""
    print('finish')
    ...

def indicate_final_counts(statistics: Dict[TileType, int]):
    """Announce the final figures / statistics"""
    print(statistics)
    ...

