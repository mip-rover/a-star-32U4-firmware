#!/usr/bin/env python

import time
from WideBoyAStar import WideBoyAStar

wideboy = WideBoyAStar()


def wait_for_button(lm_speed, rm_speed):
    led_state = False;
    while not wideboy.is_button_b_pushed():
        time.sleep(.016)
        led_state = not led_state
        left_count, right_count, left_speed, right_speed, left_motor, right_motor = wideboy.get_motor_state()
#        print('motor left: {:d}, right: {:d}'.format(left_motor, right_motor))
#        print('speed left: {:d}, right: {:d}'.format(left_speed, right_speed))
#        print('count left: {:d}, right: {:d}'.format(left_count, right_count))
        wideboy.set_leds(led_state, led_state, led_state)
        wideboy.set_motor_speeds(lm_speed, rm_speed)
    wideboy.play_notes("A4")


while 1:
    print('stop')
    wideboy.clear_motor_counts()
    wait_for_button(0, 0)

    print('left')
    wideboy.clear_motor_counts()
    wait_for_button(100, 0)

    print('right')
    wideboy.clear_motor_counts()
    wait_for_button(0, 100)

    print('forward')
    wideboy.clear_motor_counts()
    wait_for_button(100, 100)

    print('backward')
    wideboy.clear_motor_counts()
    wait_for_button(-100, -100)

