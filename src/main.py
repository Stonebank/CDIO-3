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
    robot = None
    closestBall = None
    cross = None

    showGoal = False
    showClosestBall = True

    balls = None

    previousFrames = []

    def __init__(self):
        # Connect to robot
        #self.remote = Remote()
        # Set video input
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        self.ft = FrameTransformer()
        frameCount = 0
        
        keyboard.add_hotkey('m', lambda: (
            self.toggleManMode()))
        keyboard.add_hotkey('q', lambda: (
            self.remote.stop_tank(),
            time.sleep(0.5),
            self.remote.stop_balls_mec(),
            time.sleep(0.5),
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
            if (len(self.previousFrames) < 10) :
                self.previousFrames.append(transformed)
            else : 
                self.previousFrames.pop(0)
                self.previousFrames.append(transformed)

            robot = detectRobot(transformed)
            # if (robot is not None and robot.greenFrame is not None and robot.blueFrame is not None):
            self.robot = robot
            orangeBall = detectOrangeBall(transformed, robot)
            
            self.balls = detectBalls(self.previousFrames, self.robot)

            cross = detectCross(transformed)
            if (cross is not None):
                self.cross = cross

            if self.robot is not None:
                
                # robotBox = self.robot.box
                # # Draw a line on the image
                # cv2.line(transformed, tuple(robotBox[0]), tuple(robotBox[1]), (0, 255, 0), 3)
                # cv2.line(transformed, tuple(robotBox[0]), tuple(robotBox[2]), (0, 255, 0), 3)
                # cv2.line(transformed, tuple(robotBox[1]), tuple(robotBox[3]), (0, 255, 0), 3)
                # cv2.line(transformed, tuple(robotBox[2]), tuple(robotBox[3]), (0, 255, 0), 3)

                cv2.drawContours(transformed, [self.robot.greenFrame.box], 0, (0, 255, 0), 3)
                cv2.drawContours(transformed, [self.robot.blueFrame.box], 0, (0, 255, 0), 3)
                if self.balls:
                    if len(self.balls) == 5 and orangeBall is not None:
                        self.closestBall = orangeBall
                        closestDistance = getDistance(
                            self.robot.blueFrame.x, self.robot.blueFrame.y, self.closestBall.x, self.closestBall.y)
                    else:
                        self.closestBall = self.balls[0]
                        closestDistance = getDistance(
                            self.robot.blueFrame.x, self.robot.blueFrame.y, self.closestBall.x, self.closestBall.y)
                        for ball in self.balls:
                            distance = getDistance(
                                self.robot.blueFrame.x, self.robot.blueFrame.y, ball.x, ball.y)
                            if distance < closestDistance:
                                self.closestBall = ball
                                closestDistance = distance
                    if (self.showClosestBall):
                        drawLine(transformed, self.robot.blueFrame.x, self.robot.blueFrame.y,
                                 self.closestBall.x, self.closestBall.y, robot=self.robot) 

                else:
                    self.closestBall = None
                
                if (self.showGoal):
                    drawLine(transformed, self.robot.greenFrame.x, self.robot.greenFrame.y,
                             self.ft.goal.x, self.ft.goal.y, robot=self.robot)
           
            cv2.imshow("Transformed", transformed)
            cv2.imshow("Board", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows(),

    def consumeClosestBall(self):
        self.showClosestBall = True
        self.showGoal = False
        count = 0
        scoring_count = 0
        while (self.closestBall is not None):
            print(len(self.balls))
            if (count % 3 == 0):
                self.remote.tank.gyro.calibrate()
            if(len(self.balls) == 4 and scoring_count == 0):
                self.score()
                scoring_count += 1
                continue
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
                ball = Ball(ball.x-10, ball.y-10)
                print("ball upper side")

            if ball.x < xLower:
                offsetX = ball.x + 100
                ball = Ball(ball.x-15, ball.y-5)
                print("ball left side")

            if ball.y > yUpper:
                offsetY = ball.y - 100
                ball = Ball(ball.x, ball.y+25)
                print("ball lower side")

            if ball.x > xUpper:
                offsetX = ball.x - 100
                ball = Ball(ball.x+30, ball.y)
                print("ball right side")

            if offsetX != ball.x or offsetY != ball.y:
                if offsetX != ball.x and offsetY != ball.y:
                    # Upper left corner
                    if ball.y < yLower and ball.x < xLower:
                        print("Upper left")
                        offsetX = ball.x + 60
                        offsetY = ball.y + 200
                        ball = Ball(ball.x-5, ball.y)
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
                        #offsetY = ball.y - 20
                        #offsetX = ball.x - 200
                        ball = Ball(ball.x+2, ball.y-5)
                offset = Ball(offsetX, offsetY)
                self.driveToObject(offset, True)

            ballInsideCross = ball.x > self.cross.x-(self.cross.width/2) and ball.x < self.cross.x+(
                self.cross.width/2) and ball.y > self.cross.y-(self.cross.height/2) and ball.y < self.cross.y+(self.cross.height/2)
            # If ball inside cross
            if (ballInsideCross):
                closestOffset = self.cross.offsets[0]
                closestOffsetDist = 999999
                for offset in self.cross.offsets :
                    offsetDist = getDistance(ball.x, ball.y, offset[0], offset[1])
                    if offsetDist < closestOffsetDist :
                        closestOffset = offset
                        closestOffsetDist = offsetDist
                offset = Ball(closestOffset[0], closestOffset[1]) 
                self.driveToObject(offset, False, 20)
               
            self.remote.consume_balls()
            
            self.driveToObject(ball, False, 20 if ballInsideCross else 50)
            # self.rotateUntilZero(ball)

            # self.goForwardUntilZero(ball, False)
            time.sleep(2)

            if offsetX != ball.x or offsetY != ball.y or ballInsideCross:
                self.remote.go_forward_distance(-20, 80)
            
            self.remote.stop_balls_mec()
            self.remote.stop_tank()

            
            
            count += 1
        self.score()

    
    def goAroundCross(self, object):
        if (self.cross is not None and lineIntersectsCross(self.robot.greenFrame, object, self.cross)):
            # while (lineIntersectsCross(self.robot, object, self.cross)):
            print("Cross intercept")
            # Make path around cross
            # Robot upper left corner
            if (self.robot.greenFrame.x < self.cross.x and self.robot.greenFrame.y < self.cross.y):
                offset = Goal(self.cross.x/2, self.cross.y/2)
                self.driveToObject(offset, True)
                
                # Ball upper right
                if (object.x > self.cross.x and object.y < self.cross.y and lineIntersectsCross(self.robot.greenFrame, object, self.cross)):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                    self.driveToObject(offset, True)
                # Ball lower left
                elif (object.x < self.cross.x and object.y > self.cross.y and lineIntersectsCross(self.robot.greenFrame, object, self.cross)):
                    offset = Goal(self.cross.x/2, self.cross.y + ((500-self.cross.y)/2))
                    self.driveToObject(offset, True)
                # Ball lower right
                elif (object.x > self.cross.x and object.y > self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                    self.driveToObject(offset, True)
                    if (lineIntersectsCross(self.robot.greenFrame, object, self.cross)) :
                        offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                        self.driveToObject(offset, True)

                print("robot upper left")
            
            # Robot lower left corner
            elif (self.robot.greenFrame.x < self.cross.x and self.robot.greenFrame.y > self.cross.y):
                offset = Goal((self.cross.x/2), self.cross.y +((500-self.cross.y)/2))
                self.driveToObject(offset, True)
                
                # Ball upper left
                if (object.x < self.cross.x and object.y < self.cross.y and lineIntersectsCross(self.robot.greenFrame, object, self.cross)):
                    offset = Goal(self.cross.x/2, self.cross.y/2)
                    self.driveToObject(offset, True)
                # Ball upper right
                elif (object.x > self.cross.x and object.y < self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                    self.driveToObject(offset, True)
                    if (lineIntersectsCross(self.robot.greenFrame, object, self.cross)) :
                        offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                        self.driveToObject(offset, True)
                # Ball lower right
                elif (object.x > self.cross.x and object.y > self.cross.y and lineIntersectsCross(self.robot.greenFrame, object, self.cross)):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                    self.driveToObject(offset, True)

                print("robot lower left")
            # Robot upper right corner
            elif (self.robot.greenFrame.x > self.cross.x and self.robot.greenFrame.y < self.cross.y):
                offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                self.driveToObject(offset, True)

                # Ball upper left
                if (object.x < self.cross.x and object.y < self.cross.y and lineIntersectsCross(self.robot.greenFrame, object, self.cross)):
                    offset = Goal(self.cross.x/2, self.cross.y/2)
                    self.driveToObject(offset, True)
                # Ball lower right
                elif (object.x > self.cross.x and object.y > self.cross.y and lineIntersectsCross(self.robot.greenFrame, object, self.cross)):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                    self.driveToObject(offset, True)
                # Ball lower left
                elif (object.x < self.cross.x and object.y > self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                    self.driveToObject(offset, True)
                    if (lineIntersectsCross(self.robot.greenFrame, object, self.cross)) :
                        offset = Goal(self.cross.x/2, self.cross.y + ((500-self.cross.y)/2))
                        self.driveToObject(offset, True)

                print("robot upper right")
            # Robot lower right corner
            elif (self.robot.greenFrame.x > self.cross.x and self.robot.greenFrame.y > self.cross.y):
                offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y+((500-self.cross.y)/2))
                self.driveToObject(offset, True)

                # Ball upper left
                if (object.x < self.cross.x and object.y < self.cross.y):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                    self.driveToObject(offset, True)
                    if (lineIntersectsCross(self.robot.greenFrame, object, self.cross)) :
                        offset = Goal(self.cross.x/2, self.cross.y/2)
                        self.driveToObject(offset, True)
                # Ball upper right
                elif (object.x > self.cross.x and object.y < self.cross.y and lineIntersectsCross(self.robot.greenFrame, object, self.cross)):
                    offset = Goal(self.cross.x+((750-self.cross.x)/2), self.cross.y/2)
                    self.driveToObject(offset, True)
                # Ball lower left
                elif (object.x < self.cross.x and object.y > self.cross.y and lineIntersectsCross(self.robot.greenFrame, object, self.cross)):
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
        angle = getAngle(robot=self.robot, object=ball)
        self.remote.tank_turn_degrees(angle, 15)
        angle = getAngle(robot=self.robot, object=ball)
        count = 0
        while (angle > 0 or angle < 0) and count < 5:
            self.remote.tank_turn_degrees(angle, 3)
            angle = getAngle(robot=self.robot, object=ball)
            count = count + 1
        if (count == 20):
            print("Rotate give up")

    def driveToObject(self, object, useGreenPlate, speed=50):
        angle = getAngle(robot=self.robot, object=object)
        self.remote.tank_turn_degrees(angle, 15)
        distance = getDistance(
            self.robot.greenFrame.x if useGreenPlate else self.robot.blueFrame.x, self.robot.greenFrame.y if useGreenPlate else self.robot.blueFrame.y, object.x, object.y)
        smallestDistance = distance

        while (self.robot is not None and distance > 8 and distance < smallestDistance+2):
            angle = getAngle(robot=self.robot, object=object)
            leftSpeed = speed
            rightSpeed = speed
            if (angle > 0):
                rightSpeed = rightSpeed - (angle/1.8)
            elif (angle < 0):
                leftSpeed = leftSpeed - ((-angle)/1.8)
             
            self.remote.tank.on(leftSpeed, rightSpeed)
            distance = getDistance(
                self.robot.greenFrame.x if useGreenPlate else self.robot.blueFrame.x, self.robot.greenFrame.y if useGreenPlate else self.robot.blueFrame.y, object.x, object.y)
            if (distance < smallestDistance):
                smallestDistance = distance
        self.remote.stop_tank()

    def goForwardUntilZero(self, ball, useGreenPlate):

        distance = getDistance(
            self.robot.greenFrame.x if useGreenPlate else self.robot.blueFrame.x, self.robot.greenFrame.y if useGreenPlate else self.robot.blueFrame.y, ball.x, ball.y)
        if (distance > 40):
            self.remote.go_forward_distance(distance-25, 65)
            self.rotateUntilZero(ball)

        distance = getDistance(
            self.robot.greenFrame.x if useGreenPlate else self.robot.blueFrame.x, self.robot.greenFrame.y if useGreenPlate else self.robot.blueFrame.y, ball.x, ball.y)
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
