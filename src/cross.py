import math

# Class for the cross. Containing position, rectangle around cross, and offsets
class Cross:

    def __init__(self, x, y, rect_h, rect_w, rect_x, rect_y, offsets):
        self.x = x
        self.y = y
        self.rect_h = rect_h
        self.rect_w = rect_w
        self.rect_x = rect_x
        self.rect_y = rect_y
        self.offsets = offsets
