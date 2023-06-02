import sys

import cv2
import keyboard
import numpy as np

from goalClass import Goal
from ball import Ball
from detection import *
from frameProvider import FrameTransformer
from remoteControl import Remote


class Main:

    goal0 = Goal
    goal1 = Goal
    goal0OfSet = Goal
    goal1OfSet = Goal

    goal0OfSet.x = 1
    goal0OfSet.y = 1
    goal1OfSet.x = 1
    goal1OfSet.y = 1

    goal0.x = 1
    goal0.y = 1
    goal1.x = 1
    goal1.y = 1

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
            
            #print("Transformed shape: ", transformed.shape)
            #print("Frame shape: ", frame.shape)
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
                    drawLine(transformed, robot.x, robot.y,
                         self.goal0.x, self.goal0.y, ball=self.goal0, robot=self.robot, blueframe=self.blueFrame)
                    

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
            remote.tank_turn_degrees(self.dAngle, 10)
            self.dAngle = getAngle(robot=self.robot, ball=self.closetsBall, blueframe=self.blueFrame)
            print("updating ", self.dAngle)
            
    def goForwardUntilZero(self, remote):
        self.dDistance = getDistance(self.robot.x, self.robot.y, self.closetsBall.x, self.closetsBall.y)
        remote.go_forward_distance(self.dDistance, 50)

    def getIntoPositionToScore(self, remote):

        # Set variables

        
        
        
        self.goal0.x, self.goal0.y = self.ft.goal1[0], self.ft.goal1[1]
        #self.goal1.x, self.goal1.y = 750, 250
        
        self.goal0OfSet.x = (self.goal0.x + 75)
        self.goal0OfSet.y = self.goal0.y
        #self.goal1OfSet.x = (self.goal1.x - 75)
        #self.goal1OfSet.y = self.goal1.y

        
        if (True):
            # trying to score on goal0
            # turn to face goal off set 
            angle = getAngle(robot=self.robot, ball=self.goal0OfSet, blueframe=self.blueFrame)
            remote.tank_turn_degrees(angle, 10)
            angle = getAngle(robot=self.robot, ball=self.goal0OfSet, blueframe=self.blueFrame)
            while angle != 0:
                remote.tank_turn_degrees(angle, 4)
                angle = getAngle(robot=self.robot, ball=self.goal0OfSet, blueframe=self.blueFrame)
                print("updating ", angle)
            # drive to goal off set
            distance = getDistance(self.robot.x, self.robot.y, self.goal0OfSet.x, self.goal0OfSet.y)
            
            
            remote.go_forward_distance(distance-20, 100)
            angle = getAngle(robot=self.robot, ball=self.goal0OfSet, blueframe=self.blueFrame)
            remote.tank_turn_degrees(angle, 10)
            angle = getAngle(robot=self.robot, ball=self.goal0OfSet, blueframe=self.blueFrame)
            while angle != 0:
                remote.tank_turn_degrees(angle, 4)
                angle = getAngle(robot=self.robot, ball=self.goal0OfSet, blueframe=self.blueFrame)
                print("updating ", angle)
                
            

            # turn to face goal
            angle = getAngle(robot=self.robot, ball=self.goal0, blueframe=self.blueFrame)
            remote.tank_turn_degrees(angle, 15)
            angle = getAngle(robot=self.robot, ball=self.goal0, blueframe=self.blueFrame)
            while angle != 0:
                remote.tank_turn_degrees(angle, 2)
                angle = getAngle(robot=self.robot, ball=self.goal0, blueframe=self.blueFrame)
                print("updating ", angle)
        else:
            # trying to score on goal1
            # turn to face goal off set 
            self.dAngle = getAngle(robot=self.robot, ball=self.goal1OfSet, blueframe=self.blueFrame)
            while self.dAngle > 2 or self.dAngle < -2:
                remote.tank_turn_degrees(self.dAngle)
                self.dAngle = getAngle(robot=self.robot, ball=self.goal1OfSet, blueframe=self.blueFrame)
                print("updating ", self.dAngle)
            # drive to goal off set
            distance = getDistance(self.robot.x, self.robot.y, self.goal1OfSet.x, self.goal1OfSet.y)
            remote.go_forward_distance(distance)
            # turn to face goal
            self.dAngle = getAngle(robot=self.robot, ball=self.goal1, blueframe=self.blueFrame)
            while self.dAngle > 2 or self.dAngle < -2:
                remote.tank_turn_degrees(self.dAngle)
                self.dAngle = getAngle(robot=self.robot, ball=self.goal1, blueframe=self.blueFrame)
                print("updating ", self.dAngle) 

    # Toggle for manual corner selection
    def toggleManMode(self):
        if (not self.ft.manMode):
            self.ft.manMode = True
            cv2.namedWindow("Board")
            cv2.setMouseCallback("Board", self.ft.get_point)
            cv2.namedWindow("Transformed")
            cv2.setMouseCallback("Transformed", self.ft.get_goal)
            print("MANUAL MODE - input corners")
        elif (self.ft.manMode):
            self.ft.manMode = False
            self.ft.selectCount = 0
            print("AUTO MODE")


Main()
