#!/usr/bin/env python3
'''Hello to the world from ev3dev.org'''

import os
import sys
import time

from ev3dev2.led import Leds
from ev3dev2.motor import (OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, LargeMotor,
                           MediumMotor, MoveTank, SpeedPercent)
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor

# state constants
ON = True
OFF = False 

# Initiate motors
tank = MoveTank(OUTPUT_B, OUTPUT_C)
rotate = MediumMotor(OUTPUT_A)

def debug_print(*args, **kwargs):
    '''Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def reset_console():
    '''Resets the console to the default state'''
    print('\x1Bc', end='')


def set_cursor(state):
    '''Turn the cursor on or off'''
    if state:
        print('\x1B[?25h', end='')
    else:
        print('\x1B[?25l', end='')


def set_font(name):
    '''Sets the console font
    A full list of fonts can be found with `ls /usr/share/consolefonts`
    '''
    os.system('setfont ' + name)


def main():
    '''The main function of our program'''
    for x in range(5):
        print('Rotating...')
        rotate.on_for_rotations(SpeedPercent(50), 5)
        tank.on_for_rotations(50, 50, 5)
    # set the console just how we want it
    #reset_console()
    #set_cursor(OFF)
    #set_font('Lat15-Terminus24x12')
#
    ## print something to the screen of the device
    #print('Hello World!')
#
    ## print something to the output panel in VS Code
    #debug_print('Hello VS Code!')

    # wait a bit so you have time to look at the display before the program
    # exits

if __name__ == '__main__':
    main()