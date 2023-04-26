import math


class Ball:

    def __init__(self, x, y, radius, index):
        self.x = x
        self.y = y
        self.radius = radius
        self.index = index

    def distance_to(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        distance_in_pixels = int(math.sqrt(dx * dx + dy * dy))
        return distance_in_pixels / 4.2

    def angle_to(self, other):
        angle = math.degrees(math.atan2(other.y - self.y, other.x - self.x))
        if angle < 0:
            angle += 360
        return int(angle)
