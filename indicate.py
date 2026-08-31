#!/usr/bin/env python3


def indicate_start():
    """Pause for 1 second, indicating that the robot is starting"""
    print('start')
    ...

def indicate_finish():
    """Pause for 1 second, indicating that the robot has finished"""
    print('finish')
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


