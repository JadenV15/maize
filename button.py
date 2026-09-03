#!/usr/bin/env python3

import time
from threading import Thread

from ev3dev2.button import Button

from constants import BUTTON_POLL_INTERVAL_MS

__all__ = [
    'backspace_pressed',
    'reset_backspace',
    'wait_for_enter_press'
]

btn = Button()

_has_backspace_pressed = False # type: bool

def backspace_pressed() -> bool:
    """Has the backspace button been pressed?"""
    return _has_backspace_pressed

def on_backspace_press(state):
    global _has_backspace_pressed
    if state:
        _has_backspace_pressed = True

btn.on_backspace = on_backspace_press # type: ignore

def poll_backspace():
    while True:
        btn.process()
        time.sleep(BUTTON_POLL_INTERVAL_MS / 1000)

# bg thread
# this thread runs for the entirety of the program
t = Thread(target=poll_backspace, daemon=True)
t.start()

def reset_backspace():
    """Reset backspace_pressed() to false"""
    global _has_backspace_pressed
    _has_backspace_pressed = False

def wait_for_enter_press():
    """Wait until enter button is pressed"""
    btn.wait_for_pressed(['enter'])

