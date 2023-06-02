from enum import Enum


class States(Enum): 
    CALCULATE = 0 
    ROTATE_FAST = 1 
    ROTATE_SLOW = 2
    COLLECTING = 4
    COLLECTING_NEAR_WALLS = 5
    SCORING = 6