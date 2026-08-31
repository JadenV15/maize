#!/usr/bin/env python3

import time
from threading import Thread

from ev3dev2.button import Button

from constants import BUTTON_POLL_INTERVAL

__all__ = ['has_backspace_pressed', 'wait_for_enter_press']

btn = Button()

has_backspace_pressed = False # type: bool
"""Has the backspace button been pressed?"""

def on_backspace_press(state):
    global has_backspace_pressed
    if state:
        has_backspace_pressed = True

btn.on_backspace = on_backspace_press # type: ignore

def poll_backspace():
    while True:
        btn.process()
        time.sleep(BUTTON_POLL_INTERVAL)

# bg thread
t = Thread(target=poll_backspace, daemon=True)
t.start()

def wait_for_enter_press():
    """Wait until enter button is pressed"""
    btn.wait_for_pressed(['enter'])

