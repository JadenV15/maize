#!/usr/bin/env python3

import time
from threading import Thread

from ev3dev2.button import Button

from constants import BUTTON_POLL_INTERVAL_MS

__all__ = [
    'enter_pressed',
    'reset_enter',
    'wait_for_enter_press'
]

btn = Button()

_has_enter_pressed = False # type: bool

def enter_pressed() -> bool:
    """Has the enter button been pressed?"""
    return _has_enter_pressed

def on_enter_press(state):
    global _has_enter_pressed
    if state:
        _has_enter_pressed = True

btn.on_enter = on_enter_press # type: ignore

def poll_enter():
    while True:
        btn.process()
        time.sleep(BUTTON_POLL_INTERVAL_MS / 1000)

# bg thread
# this thread runs for the entirety of the program
t = Thread(target=poll_enter, daemon=True)
t.start()

def reset_enter():
    """Reset enter_pressed() to false"""
    global _has_enter_pressed
    _has_enter_pressed = False

def wait_for_enter_press():
    """Wait until enter button is pressed"""
    btn.wait_for_pressed(['enter'])

