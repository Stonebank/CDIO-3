import subprocess
import sys
import time

import cv2
import keyboard
import numpy as np

from ball import Ball
from detection import *
from frameProvider import FrameTransformer
from goalClass import Goal
from remoteControl import Remote


class Main:
    robotX, robotY, robotWidth, robotHeight = 0, 0, 0, 0

    robot = None
    blueFrame = None
    closestBall = None
    crossPosition = [0, 0]
    crossRadius = 50
    cross = None

    showGoal = False
    showClosestBall = True

    def __init__(self):
        # Connect to robot
        self.remote = Remote()
        # Set video input
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.ft = FrameTransformer()
        frameCount = 0
        
        keyboard.add_hotkey('m', lambda: (
            self.toggleManMode()))
        keyboard.add_hotkey('q', lambda: (
            self.remote.stop_tank(),
            self.remote.stop_balls_mec(),
            cap.release(),
            cv2.destroyAllWindows(), sys.exit()))

        keyboard.add_hotkey('s', lambda: (
            self.remote.go_forward_distance(-50, 50)))

        keyboard.add_hotkey('e', lambda: (
            self.remote.eject_balls()))

        keyboard.add_hotkey("b", lambda: (self.consumeClosestBall()))

        keyboard.add_hotkey('g', lambda: (self.score()))

        while True:
            frameCount += 1
            ret, frame = cap.read()
            transformed = self.ft.transform(frame, frameCount)
            transformed = frame if transformed is None else transformed
            robot = detectRobot(transformed)
            blueFrame = detectBlueFrame(transformed)
            if (blueFrame is not None):
                self.blueFrame = blueFrame
            if (robot is not None):
                self.robot = robot
            # orangeBall = detectOrangeBall(transformed, robot)
            cross = detectCross(transformed)
            if (cross is not None):
                self.cross = cross
            balls = detectBalls(transformed, robot)

            if self.robot and self.blueFrame is not None:
                if balls:
                    self.closestBall = balls[0]
                    closestDistance = getDistance(
                        self.blueFrame[0], self.blueFrame[1], self.closestBall.x, self.closestBall.y)
                    for ball in balls:
                        distance = getDistance(
                            self.blueFrame[0], self.blueFrame[1], ball.x, ball.y)
                        if distance < closestDistance:
                            self.closestBall = ball
                            closestDistance = distance
                    if (self.showClosestBall):
                        drawLine(transformed, self.blueFrame[0], self.blueFrame[1],
                                 self.closestBall.x, self.closestBall.y, object=self.closestBall, robot=self.robot, blueframe=self.blueFrame)

                else:
                    self.closestBall = None
                
                if (self.showGoal):
                    drawLine(transformed, self.robot.x, self.robot.y,
                             self.ft.goal.x, self.ft.goal.y, object=self.ft.goal, robot=self.robot, blueframe=self.blueFrame)
           
            cv2.imshow("Transformed", transformed)
            cv2.imshow("Board", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows(),

    def consumeClosestBall(self):
        self.showClosestBall = True
        self.showGoal = False
        while (self.closestBall is not None):
            ball = self.closestBall
            print("Consuming closest ball")
            # Check for cross intercept
            self.goAroundCross(ball)

            offsetX = ball.x
            offsetY = ball.y

            yUpper = 460
            yLower = 40
            xUpper = 710
            xLower = 40

            if ball.y < yLower:
                offsetY = ball.y + 100
                ball = Ball(ball.x-10, ball.y-10, 7, 10)
                print("ball upper side")

            if ball.x < xLower:
                offsetX = ball.x + 100
                ball = Ball(ball.x-10, ball.y-5, 7, 10)
                print("ball left side")

            if ball.y > yUpper:
                offsetY = ball.y - 100
                ball = Ball(ball.x, ball.y+15, 7, 10)
                print("ball lower side")

            if ball.x > xUpper:
                offsetX = ball.x - 100
                ball = Ball(ball.x+30, ball.y, 7, 10)
                print("ball right side")

            if offsetX != ball.x or offsetY != ball.y:
                if offsetX != ball.x and offsetY != ball.y:
                    # Upper left corner
                    if ball.y < yLower and ball.x < xLower:
                        print("Upper left")
                        offsetX = ball.x + 60
                        offsetY = ball.y + 200
                    # Lower left corner
                    elif ball.y > yUpper and ball.x < xLower:
                        print("Lower left")
                        offsetX = ball.x + 60
                        offsetY = ball.y - 200
                        # ball = Ball(ball.x, ball.y, 7, 10)
                    # Upper right corner
                    elif ball.y < yLower and ball.x > xUpper:
                        print("Upper right")
                        offsetX = ball.x - 100
                        offsetY = ball.y + 200
                    # Lower right corner
                    elif ball.y > yUpper and ball.x > xUpper:
                        print("Lower right")
                        offsetY = ball.y - 20
                        offsetX = ball.x - 200
                        ball = Ball(ball.x+2, ball.y-5, 7, 10)
                offset = Ball(offsetX, offsetY, 7, 10)
                self.driveToObject(offset, True)

            ballInsideCross = ball.x > self.cross.x-(self.cross.width/2) and ball.x < self.cross.x+(
                self.cross.width/2) and ball.y > self.cross.y-(self.cross.height/2) and ball.y < self.cross.y+(self.cross.height/2)
            # If ball inside cross
            if (ballInsideCross):
                # Ball upper left
                if (ball.x < self.cross.x and ball.y < self.cross.y):
                    offset = Goal(self.cross.x-(self.cross.width/2)-50,
                                  self.cross.y-(self.cross.height/2))
                    self.driveToObject(offset, False)
                # Ball upper right
                elif (ball.x > self.cross.x and ball.y < self.cross.y):
                    offset = Goal(self.cross.x+(self.cross.width/2)+50,
                                  self.cross.y-(self.cross.height/2))
                    self.driveToObject(offset, False)
                # Ball lower left
                elif (ball.x < self.cross.x and ball.y > self.cross.y):
                    offset = Goal(self.cross.x-(self.cross.width/2)-50,
                                  self.cross.y+(self.cross.height/2))
                    self.driveToObject(offset, False)
                # Ball lower right
                elif (ball.x > self.cross.x and ball.y > self.cross.y):
                    offset = Goal(self.cross.x+(self.cross.width/2)+50,
                                  self.cross.y+(self.cross.height/2))
                    self.driveToObject(offset, False)

            self.remote.consume_balls()

            self.driveToObject(ball, False)
            # self.rotateUntilZero(ball)

            # self.goForwardUntilZero(ball, False)
            time.sleep(2)

            if offsetX != ball.x or offsetY != ball.y or ballInsideCross:
                self.remote.go_forward_distance(-20, 80)

            self.remote.stop_balls_mec()
            self.remote.stop_tank()
        self.score()

    def goAroundCross(self, object):
        if (self.cross is not None and lineIntersectsCross(self.robot, object, self.cross)):
            # while (lineIntersectsCross(self.robot, object, self.cross)):
            print("Cross intercept")
            # Make path around cross
            # Robot upper left corner
            if (self.robot.x < self.cross.x and self.robot.y < self.cross.y):
                offset = Goal(self.cross.x/2, self.cross.y/2)
                self.driveToObject(offset, True)
                
                # Ball upper right
                if (object.x > self.cross.x and object.y < self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                    self.driveToObject(offset, True)
                # Ball lower left
                elif (object.x < self.cross.x and object.y > self.cross.y):
                    offset = Goal(self.cross.x/2, self.cross.y + ((500-self.cross.y)/2))
                    self.driveToObject(offset, True)
                # Ball lower right
                elif (object.x > self.cross.x and object.y > self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                    self.driveToObject(offset, True)
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                    self.driveToObject(offset, True)

                print("robot upper left")
            
            # Robot lower left corner
            elif (self.robot.x < self.cross.x and self.robot.y > self.cross.y):
                offset = Goal((self.cross.x/2), self.cross.y +((500-self.cross.y)/2))
                self.driveToObject(offset, True)
                
                # Ball upper left
                if (object.x < self.cross.x and object.y < self.cross.y):
                    offset = Goal(self.cross.x/2, self.cross.y/2)
                    self.driveToObject(offset, True)
                # Ball upper right
                elif (object.x > self.cross.x and object.y < self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                    self.driveToObject(offset, True)
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                    self.driveToObject(offset, True)
                # Ball lower right
                elif (object.x > self.cross.x and object.y > self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                    self.driveToObject(offset, True)

                print("robot lower left")
            # Robot upper right corner
            elif (self.robot.x > self.cross.x and self.robot.y < self.cross.y):
                offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                self.driveToObject(offset, True)

                # Ball upper left
                if (object.x < self.cross.x and object.y < self.cross.y):
                    offset = Goal(self.cross.x/2, self.cross.y/2)
                    self.driveToObject(offset, True)
                # Ball lower right
                elif (object.x > self.cross.x and object.y > self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                    self.driveToObject(offset, True)
                # Ball lower left
                elif (object.x < self.cross.x and object.y > self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                    self.driveToObject(offset, True)
                    offset = Goal(self.cross.x/2, self.cross.y + ((500-self.cross.y)/2))
                    self.driveToObject(offset, True)

                print("robot upper right")
            # Robot lower right corner
            elif (self.robot.x > self.cross.x and self.robot.y > self.cross.y):
                offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                self.driveToObject(offset, True)

                # Ball upper left
                if (object.x < self.cross.x and object.y < self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                    self.driveToObject(offset, True)
                    offset = Goal(self.cross.x/2, self.cross.y/2)
                    self.driveToObject(offset, True)
                # Ball upper right
                elif (object.x > self.cross.x and object.y < self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                    self.driveToObject(offset, True)
                # Ball lower left
                elif (object.x < self.cross.x and object.y > self.cross.y):
                    offset = Goal(self.cross.x/2, self.cross.y+((500-self.cross.y)/2))
                    self.driveToObject(offset, True)

                print("robot lower left")

    def score(self):
        self.showGoal = True
        self.showClosestBall = False
        self.getIntoPositionToScore()
        self.remote.eject_balls()
        time.sleep(3)
        self.remote.stop_tank(),
        self.remote.stop_balls_mec()

    def rotateUntilZero(self, ball):
        angle = getAngle(robot=self.robot, object=ball,
                         blueframe=self.blueFrame)
        self.remote.tank_turn_degrees(angle, 15)
        angle = getAngle(robot=self.robot, object=ball,
                         blueframe=self.blueFrame)
        count = 0
        while (angle > 0 or angle < 0) and count < 5:
            self.remote.tank_turn_degrees(angle, 3)
            angle = getAngle(robot=self.robot, object=ball,
                             blueframe=self.blueFrame)
            count = count + 1
        if (count == 20):
            print("Rotate give up")

    def driveToObject(self, object, useGreenPlate):
        angle = getAngle(robot=self.robot, object=object,
                         blueframe=self.blueFrame)
        self.remote.tank_turn_degrees(angle, 15)
        distance = getDistance(
            self.robot.x if useGreenPlate else self.blueFrame[0], self.robot.y if useGreenPlate else self.blueFrame[1], object.x, object.y)
        smallestDistance = distance

        while (distance > 6 and distance < smallestDistance+2):
            angle = getAngle(robot=self.robot, object=object,
                             blueframe=self.blueFrame)
            leftSpeed = 50
            rightSpeed = 50
            if (angle > 0):
                rightSpeed = rightSpeed - (angle/1.8)
            elif (angle < 0):
                leftSpeed = leftSpeed - ((-angle)/1.8)
             
            self.remote.tank.on(leftSpeed, rightSpeed)
            distance = getDistance(
                self.robot.x if useGreenPlate else self.blueFrame[0], self.robot.y if useGreenPlate else self.blueFrame[1], object.x, object.y)
            if (distance < smallestDistance):
                smallestDistance = distance
        self.remote.stop_tank()

    def goForwardUntilZero(self, ball, useGreenPlate):

        distance = getDistance(
            self.robot.x if useGreenPlate else self.blueFrame[0], self.robot.y if useGreenPlate else self.blueFrame[1], ball.x, ball.y)
        if (distance > 40):
            self.remote.go_forward_distance(distance-25, 65)
            self.rotateUntilZero(ball)

        distance = getDistance(
            self.robot.x if useGreenPlate else self.blueFrame[0], self.robot.y if useGreenPlate else self.blueFrame[1], ball.x, ball.y)
        if (distance > 0):
            self.remote.go_forward_distance(distance, 50)

    def getIntoPositionToScore(self):

        goal = self.ft.goal

        # Calculate offset
        offset = 50
        if (goal.x > 300):
            offset = -50

        goalOffset = Goal(goal.x + offset, goal.y)

        self.goAroundCross(goalOffset)

        self.driveToObject(goalOffset, False)
        
        
        # face the goal
        self.rotateUntilZero(goal)

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
