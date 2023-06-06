import sys

import cv2
import keyboard
import numpy as np

from ball import Ball
from detection import *
from frameProvider import FrameTransformer
from goalClass import Goal
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
    crossPosition = [0,0]
    crossRadius = 50

    dAngle = 0
    dDistance = 0

    def __init__(self):
        # Connect to robot
        self.remote = Remote()
        # Set video input
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.ft = FrameTransformer()
        frameCount = 0

        # Import undistor matrix
        cameraMatrix = None
        dist = None
        rvecs = None
        tvecs = None

        # Load saved data
        data = np.load('calibrationvars.npz')

        cameraMatrix = data['arr_0']
        dist = data['arr_1']
        rvecs = data['arr_2']
        tvecs = data['arr_3']

        h, w = cap.shape[:2]
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

        keyboard.add_hotkey('m', lambda: (
            self.toggleManMode()))
        keyboard.add_hotkey('q', lambda: (
            self.remote.stop_tank(),
            self.remote.stop_balls_mec(),
            cap.release(),
            cv2.destroyAllWindows(), sys.exit()))
        
        keyboard.add_hotkey("b", lambda: (
            self.consumeClosestBall()
        ))
        
        keyboard.add_hotkey('g', lambda: (
                self.score()
            ))
        
      

        while True:
            frameCount += 1
            ret, frame = cap.read()
            dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
            #frame = cv2.imread('src/bane.jpg')
            transformed = self.ft.transform(dst frameCount)
            transformed = frame if transformed is None else transformed
            print("Transformed shape: ", transformed.shape)
            self.crossPosition = self.ft.getCross(transformed)
            cv2.circle(transformed, (self.crossPosition[0], self.crossPosition[1]), self.crossRadius, (0, 0, 255), 2)
            robot = detectRobot(transformed)
            blueFrame = detectBlueFrame(transformed)
            self.robot = robot
            orangeBall = detectOrangeBall(transformed, robot)
            balls = detectBalls(transformed, robot)
            #delete this circle later its just to get the 
            #cv2.circle(transformed, ( int (transformed.shape[1]/2), int (transformed.shape[0]/2)), int(50), (0, 255, 0), 2)
            
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
            cv2.imshow("Board", dst)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # if (frameCount%20==0):
            #     cv2.imwrite("robot.png", transformed)
        cap.release()
        cv2.destroyAllWindows(),

    def consumeClosestBall(self) : 
        ball = self.closetsBall
        self.rotateUntilZero(ball)
        self.remote.consume_balls()
        self.goForwardUntilZero(ball)

    def score(self) : 
        self.getIntoPositionToScore()
        self.remote.eject_balls()
        self.remote.stop_tank(),
        self.remote.stop_balls_mec()


    def rotateUntilZero(self, ball):
        angle = getAngle(robot=self.robot, ball=ball, blueframe=self.blueFrame)
        self.remote.tank_turn_degrees(angle, 20)
        angle = getAngle(robot=self.robot, ball=ball, blueframe=self.blueFrame)
        count = 0
        while angle != 0 and count < 20:
            self.remote.tank_turn_degrees(angle, 3)
            angle = getAngle(robot=self.robot, ball=ball, blueframe=self.blueFrame)
            count = count + 1
            
    def goForwardUntilZero(self, ball):
        point1 = [self.robot.x, self.robot.y]
        point2 = [ball.x, ball.y]
        circle_center = [250, 375]
        
        if is_line_crossing_circle(point1, point2, self.crossPosition, self.crossRadius) :
            return False

        distance = getDistance(self.robot.x, self.robot.y, ball.x, ball.y)
        if (distance > 10) :
            self.remote.go_forward_distance(distance-10, 80)
            self.rotateUntilZero(ball)
        else :
            self.remote.go_forward_distance(distance, 80)
        
        distance = getDistance(self.robot.x, self.robot.y, ball.x, ball.y)
        if (distance > 0) :
            self.remote.go_forward_distance(distance, 40)

    def getIntoPositionToScore(self):
        # Set variables
        self.goal0.x, self.goal0.y = self.ft.goal1[0], self.ft.goal1[1]

        self.goal0OfSet.x = (self.goal0.x + 50)
        self.goal0OfSet.y = self.goal0.y
        
        # face the goal offset
        self.rotateUntilZero(self.goal0OfSet)
        # drive to goal offset
        self.goForwardUntilZero(self.goal0OfSet) 
        # face the goal 
        self.rotateUntilZero(self.goal0)

    def goAroundCross(self):
        if self.robot.y > 250: #go up
            print("a")
        else: #go down
            print("a")
                    

            
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
