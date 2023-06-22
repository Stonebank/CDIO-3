import math

# Class for balls. Containing position of the ball
class Ball:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        if isinstance(other, Ball):
            return self.x == other.x and self.y == other.y
        return False
