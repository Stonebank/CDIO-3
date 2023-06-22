import math

# Robot class. Containg location of the green and blue frames
class Robot:

    def __init__(self, greenFrame, blueFrame, box=None):
        self.greenFrame = greenFrame
        self.blueFrame = blueFrame
        self.box =  box
