from pybricks.ev3devices import (ColorSensor, GyroSensor, InfraredSensor,
                                 Motor, TouchSensor, UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import ImageFile, SoundFile
from pybricks.parameters import Button, Color, Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import DataLog, StopWatch, wait


class Movement:

    def drive_forwards(ev3: EV3Brick, robot: DriveBase):

        robot.straight(1000)
        ev3.speaker.beep()

    def turn_right_180(ev3: EV3Brick, robot: DriveBase):
        robot.turn(180)
        ev3.speaker.beep()
