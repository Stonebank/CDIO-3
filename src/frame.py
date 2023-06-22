import math

# Class for the colored frames places on the robot
class Frame:

    def __init__(self, x, y, box=None):
        self.x = x
        self.y = y
        self.box = box
