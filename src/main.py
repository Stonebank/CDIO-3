import sys
import cv2
import numpy as np
import keyboard
from ball import Ball
from detection import *

from frameprovider import FrameTransformer
from remotecontrol import Remote


class Main:

    robotX, robotY, robotWidth, robotHeight = 0, 0, 0, 0

    def __init__(self):
        # Connect to robot
        # remote = Remote()
        # Set video input
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        self.ft = FrameTransformer()
        frameCount = 0

        keyboard.add_hotkey('m', lambda: (
            self.toggleManMode()))
        keyboard.add_hotkey('q', lambda: (
            cap.release(),
            cv2.destroyAllWindows(), sys.exit()))

        while True:
            frameCount += 1
            ret, frame = cap.read()
            transformed = self.ft.transform(frame, frameCount)
            transformed = frame if transformed is None else transformed

            robot = detectRobot(transformed)
            blueFrame = detectBlueFrame(transformed)
            orangeBall = detectOrangeBall(transformed, robot)
            balls = detectBalls(transformed, robot)
            if balls and robot:
                closestBall = balls[0]
                closestDistance = getDistance(
                    robot.x, robot.y, closestBall.x, closestBall.y)
                for ball in balls:
                    distance = getDistance(robot.x, robot.y, ball.x, ball.y)
                    if distance < closestDistance:
                        closestBall = ball
                        closestDistance = distance
                if blueFrame is not None:
                    drawLine(transformed, blueFrame[0], blueFrame[1],
                         closestBall.x, closestBall.y, ball=closestBall, robot=robot, blueframe=blueFrame)
                    

            cv2.imshow("Transformed", transformed)
            cv2.imshow("Board", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # if (frameCount%20==0):
            #     cv2.imwrite("robot.png", transformed)
        cap.release()
        cv2.destroyAllWindows()

    # Toggle for manual corner selection
    def toggleManMode(self):
        if (not self.ft.manMode):
            self.ft.manMode = True
            cv2.namedWindow("Board")
            cv2.setMouseCallback("Board", self.ft.get_point)
            print("MANUAL MODE - input corners")
        elif (self.ft.manMode):
            self.ft.manMode = False
            self.ft.selectCount = 0
            print("AUTO MODE")


Main()
