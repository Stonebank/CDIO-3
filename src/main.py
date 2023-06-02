import sys

import cv2
import keyboard
import numpy as np

from ball import Ball
from detection import *
from frameProvider import FrameTransformer
from remoteControl import Remote


class Main:

    robotX, robotY, robotWidth, robotHeight = 0, 0, 0, 0

    robot = None
    blueFrame = None
    closetsBall = None

    dAngle = 0
    dDistance = 0

    def __init__(self):
        # Connect to robot
        remote = Remote()
        # Set video input
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.ft = FrameTransformer()
        frameCount = 0

        keyboard.add_hotkey('m', lambda: (
            self.toggleManMode()))
        keyboard.add_hotkey('q', lambda: (
            cap.release(),
            cv2.destroyAllWindows(), sys.exit()))
        
        keyboard.add_hotkey("s", lambda: (
            remote.eject_balls()
        ))
        
        keyboard.add_hotkey('r', lambda: (
                self.rotateUntilZero(remote=remote), 
                remote.consume_balls(),
                self.goForwardUntilZero(remote=remote),
            ))
        
        keyboard.add_hotkey('w', lambda: (
            remote.stop_tank(),
            remote.stop_balls_mec())
        )

        keyboard.add_hotkey('p', lambda: (
                self.getIntoPositionToScore(remote=remote)
            ))

        while True:
            frameCount += 1
            ret, frame = cap.read()
            transformed = self.ft.transform(frame, frameCount)
            transformed = frame if transformed is None else transformed

            robot = detectRobot(transformed)
            blueFrame = detectBlueFrame(transformed)
            self.robot = robot
            orangeBall = detectOrangeBall(transformed, robot)
            balls = detectBalls(transformed, robot)
            if balls and robot:
                self.closetsBall = balls[0]
                closestDistance = getDistance(
                    self.robot.x, self.robot.y, self.closetsBall.x, self.closetsBall.y)
                for ball in balls:
                    distance = getDistance(robot.x, robot.y, ball.x, ball.y)
                    if distance < closestDistance:
                        self.closetsBall = ball
                        closestDistance = distance
                if blueFrame is not None:
                    self.blueFrame = blueFrame
                    #self.dAngle = getAngle(robot=self.robot, blueframe=self.blueFrame, ball=self.closetsBall)
                    self.dDistance = closestDistance
                    drawLine(transformed, blueFrame[0], blueFrame[1],
                         self.closetsBall.x, self.closetsBall.y, ball=self.closetsBall, robot=self.robot, blueframe=self.blueFrame)
                    

            cv2.imshow("Transformed", transformed)
            cv2.imshow("Board", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # if (frameCount%20==0):
            #     cv2.imwrite("robot.png", transformed)
        cap.release()
        cv2.destroyAllWindows(),

    def rotateUntilZero(self, remote):
        self.dAngle = getAngle(robot=self.robot, ball=self.closetsBall, blueframe=self.blueFrame)
        while self.dAngle != 0:
            remote.tank_turn_degrees(self.dAngle)
            self.dAngle = getAngle(robot=self.robot, ball=self.closetsBall, blueframe=self.blueFrame)
            print("updating ", self.dAngle)
            
    def goForwardUntilZero(self, remote):
        self.dDistance = getDistance(self.robot.x, self.robot.y, self.closetsBall.x, self.closetsBall.y)
        remote.go_forward_distance(self.dDistance)

    def getIntoPositionToScore(self, remote):

        # Set variables
        goal0 = Ball
        goal1 = Ball
        goal0OfSet = Ball
        goal1OfSet = Ball

        goal0.x, goal0.y = self.ft.goals[0]
        goal1.x, goal1.y = self.ft.goals[1]
        frameSizeX = goal1.x
        goalOffset = frameSizeX*0.05
        goal0OfSet.x = (goal0.x + goalOffset)
        goal0OfSet.y = goal0.y
        goal1OfSet.x = (goal1.x - goalOffset)
        goal1OfSet.y = goal1.y

        
        if (getDistance(self.robot.x, self.robot.y, goal0.x, goal0.y,) < getDistance(self.robot.x, self.robot.y, goal1.x, goal1.y,)):
            #go to goal0
            # turn to face goal off set 
            self.dAngle = getAngle(robot=self.robot, ball=goal0OfSet, blueframe=self.blueFrame)
            while self.dAngle > 2:
                remote.tank_turn_degrees(self.dAngle)
                self.dAngle = getAngle(robot=self.robot, ball=goal0OfSet, blueframe=self.blueFrame)
                print("updating ", self.dAngle)
            # drive to off set
            self.dDistance = getDistance(self.robot.x, self.robot.y, goal0OfSet.x, goal0OfSet.y)
            remote.go_forward_distance(self.dDistance)
            # turn to face goal
            self.dAngle = getAngle(robot=self.robot, ball=goal1, blueframe=self.blueFrame)
            while self.dAngle > 2:
                remote.tank_turn_degrees(self.dAngle)
                self.dAngle = getAngle(robot=self.robot, ball=goal1, blueframe=self.blueFrame)
                print("updating ", self.dAngle)
        else:
            #go to goal1
            self.dAngle = getAngle(robot=self.robot, ball=goal1OfSet, blueframe=self.blueFrame)
            while self.dAngle > 2:
                remote.tank_turn_degrees(self.dAngle)
                self.dAngle = getAngle(robot=self.robot, ball=goal1OfSet, blueframe=self.blueFrame)
                print("updating ", self.dAngle)

            self.dDistance = getDistance(self.robot.x, self.robot.y, goal1OfSet.x, goal1OfSet.y)
            remote.go_forward_distance(self.dDistance)

            self.dAngle = getAngle(robot=self.robot, ball=goal0, blueframe=self.blueFrame)
            while self.dAngle > 2:
                remote.tank_turn_degrees(self.dAngle)
                self.dAngle = getAngle(robot=self.robot, ball=goal0, blueframe=self.blueFrame)
                print("updating ", self.dAngle) 

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
