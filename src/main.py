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

    robotX, robotY, robotWidth, robotHeight = 0, 0, 0, 0

    robot = None
    blueFrame = None
    closetsBall = None
    crossPosition = [0,0]
    crossRadius = 50

    dAngle = 0
    dDistance = 0
    
    showGoal = False
    showClosestBall = True

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
        data = np.load('ressources\calibrationvars.npz')

        cameraMatrix = data['arr_0']
        dist = data['arr_1']
        rvecs = data['arr_2']
        tvecs = data['arr_3']

        h, w = (500, 750) 
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

        keyboard.add_hotkey('m', lambda: (
            self.toggleManMode()))
        keyboard.add_hotkey('q', lambda: (
            self.remote.stop_tank(),
            self.remote.stop_balls_mec(),
            cap.release(),
            cv2.destroyAllWindows(), sys.exit()))
        
        keyboard.add_hotkey("b", lambda: (self.consumeClosestBall()))
        
        keyboard.add_hotkey('g', lambda: (self.score()))

        while True:
            frameCount += 1
            ret, frame = cap.read()
            dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
            #frame = cv2.imread('src/bane.jpg')
            transformed = self.ft.transform(dst, frameCount)
            transformed = frame if transformed is None else transformed
            #print("Transformed shape: ", transformed.shape)
            self.crossPosition = self.ft.getCross(transformed)
            cv2.circle(transformed, (self.crossPosition[0], self.crossPosition[1]), self.crossRadius, (0, 0, 255), 2)
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
                   # self.goAroundCross()
                    #self.dAngle = getAngle(robot=self.robot, blueframe=self.blueFrame, ball=self.closetsBall)
                    self.dDistance = closestDistance
                    if (self.showClosestBall) :
                        drawLine(transformed, blueFrame[0], blueFrame[1],
                             self.closetsBall.x, self.closetsBall.y, object=self.closetsBall, robot=self.robot, blueframe=self.blueFrame)
                    if (self.showGoal) :
                        drawLine(transformed, robot.x, robot.y,
                            self.ft.goal.x, self.ft.goal.y, object=self.ft.goal, robot=self.robot, blueframe=self.blueFrame)
            #if (frameCount%20==0):
            #    self.findChessBoard(transformed)       

            cv2.imshow("Transformed", transformed)
            cv2.imshow("Board", dst)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        cap.release()
        cv2.destroyAllWindows(),

    def consumeClosestBall(self) : 
        ball = self.closetsBall

        offsetX = ball.x
        offsetY = ball.y

        if ball.y < 50 :
            offsetY = ball.y + 80
            print("offset 1")
            
        if ball.x < 50 :
            offsetX = ball.x + 80
            print("offset 2")

        if ball.y > 450 :
            offsetY = ball.y - 80
            print("offset 3")

        if ball.x > 700 :
            offsetX = ball.x - 80
            print("offset 4")
        
        if offsetX != ball.x or offsetY != ball.y:
            offset = Ball(offsetX, offsetY, 7, 10)
            self.rotateUntilZero(offset)
            self.goForwardUntilZero(offset)

        self.rotateUntilZero(ball)
        self.remote.consume_balls()
        self.goForwardUntilZero(ball)

    def score(self) : 
        self.showGoal = True
        self.getIntoPositionToScore()
        self.remote.eject_balls()
        self.remote.stop_tank(),
        self.remote.stop_balls_mec()


    def rotateUntilZero(self, ball):
        point1 = [self.robot.x, self.robot.y]
        point2 = [ball.x, ball.y]

        if is_line_crossing_circle(point1, point2, self.crossPosition, self.crossRadius) :
            #self.goAroundCross(ball)
            return False

        angle = getAngle(robot=self.robot, object=ball, blueframe=self.blueFrame)
        self.remote.tank_turn_degrees(angle, 15)
        angle = getAngle(robot=self.robot, object=ball, blueframe=self.blueFrame)
        count = 0
        while angle != 0 and count < 10:
            self.remote.tank_turn_degrees(angle, 3)
            angle = getAngle(robot=self.robot, object=ball, blueframe=self.blueFrame)
            count = count + 1
            
    def goForwardUntilZero(self, ball):

        distance = getDistance(self.robot.x, self.robot.y, ball.x, ball.y)
        if (distance > 30) :
            self.remote.go_forward_distance(distance-15, 70)
            self.rotateUntilZero(ball)
        
        distance = getDistance(self.robot.x, self.robot.y, ball.x, ball.y)
        if (distance > 0) :
            self.remote.go_forward_distance(distance, 35)

    def getIntoPositionToScore(self):
        
        goal = self.ft.goal

        # Calculate offset
        offset = 70
        if (goal.x > 300) :
            offset = -70

        goalOffset = Goal(goal.x + offset, goal.y)
        
        # face the goal offset
        self.rotateUntilZero(goalOffset)
        # drive to goal offset
        self.goForwardUntilZero(goalOffset) 
        # face the goal 
        self.rotateUntilZero(goal)

    def goAroundCross(self, endpoint): # this function needs to ends with an rotateUntilZero(endpoint)
        tempangle = self.calculate_angle(self.robot.x, self.robot.y, self.blueFrame[0], self.blueFrame[1])
        #tempCrossPosition = Goal(self.crossPosition[0], self.crossPosition[1])
        #angle = getAngle(robot=self.robot, ball=tempCrossPosition, blueframe=self.blueFrame)
        # Test if we can drive around cross
        if self.crossPosition[0] > 150 and self.crossPosition[0] < 600 and self.crossPosition[1] > 150 and self.crossPosition[1] < 350:
            
            # point to get close to cross
            tempRobot = [self.robot.x, self.robot.y]
            tempDriveToCross = Goal(self.createVektorBetweenTwoPoint(tempRobot, self.crossPosition, 15))

            # turn towards cross
            angle = getAngle(robot=self.robot, object=tempDriveToCross, blueframe=self.blueFrame)
            self.remote.tank_turn_degrees(angle, 15)
            angle = getAngle(robot=self.robot, object=tempDriveToCross, blueframe=self.blueFrame)
            count = 0
            while angle != 0 and count < 10:
                self.remote.tank_turn_degrees(angle, 3)
                angle = getAngle(robot=self.robot, object=tempDriveToCross, blueframe=self.blueFrame)
                count = count + 1

            self.goForwardUntilZero(tempDriveToCross)

            tempDriveAround1 = Goal(self.robot.x, self.robot.y)
            tempDriveAround2 = Goal(self.robot.x, self.robot.y)
            print("a")
            
        else:
            print("b")

        self.rotateUntilZero(endpoint)

    def createVektorBetweenTwoPoint(robot, cross, length):
        x = robot[0] - cross[0]
        y = robot[1] - cross[1]

        c = math.sqrt(x**2 + y**2)

        x = x/c
        y = y/c

        x = x * length
        y = y * length

        return (x,y)

    def calculate_angle(self, x1, y1, x2, y2):
        # Calculate the differences in x and y coordinates
        dx = x2 - x1
        dy = y2 - y1
        
        # Calculate the angle using arctan2 function
        angle_rad = math.atan2(dy, dx)
        
        # Convert the angle from radians to degrees
        angle_deg = math.degrees(angle_rad)
        
        # Ensure the angle is within the range of 0 to 360 degrees
        if angle_deg < 0:
            angle_deg += 360

        return angle_deg
                           

    def findChessBoard(self, frame):
        chessboardSize = (12,8)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((chessboardSize[0]*chessboardSize[1],3), np.float32)
        objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)
        axisBoxes = np.float32([ [0,0,0], [0,3,0], [3,3,0], [3,0,0], 
                        [0,0,-3], [0,3,-3], [3,3,-3], [3,0,-3] ])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)
        cv2.drawChessboardCorners(frame, chessboardSize, corners, ret)
                    

            
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
