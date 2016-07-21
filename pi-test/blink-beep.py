#!/usr/bin/env python

import time
from WideBoyAStar import WideBoyAStar

wideboy = WideBoyAStar()

while 1:
    wideboy.set_leds(0, 0, 0)
    time.sleep(0.5)
    wideboy.set_leds(1, 0, 0)
    time.sleep(0.5)
    wideboy.play_notes("A16")
