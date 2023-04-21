import sys
import threading
import cv2
import numpy as np
import keyboard

from frameProvider import FrameTransformer
from remoteControl import Remote


class Main:

    robotX, robotY, robotWidth, robotHeight = 0, 0, 0, 0

    def __init__(self):
        # Connect to robot
        remote = Remote()
        # Set video input
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.ft = FrameTransformer()
        frameCount = 0

        # Set hotkeys
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
            self.findRobot(transformed)
            self.findBalls(transformed)

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
            print("AUTO MODE")

    def findBalls(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        lower_white = np.array([200, 200, 200], dtype="uint8")
        upper_white = np.array([255, 255, 255], dtype="uint8")

        whiteMask = cv2.inRange(frame, lower_white, upper_white)

        contours, hierarchy = cv2.findContours(
            whiteMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:

            (x, y, w, h) = cv2.boundingRect(contour)
            radius = w/2
            if (radius > 9 or radius < 6):
                continue
            # filter if the robotX, robotY, robotWidth, robotHeight is in the ball area
            if x > self.robotX and x < self.robotX + self.robotWidth and y > self.robotY and y < self.robotY + self.robotHeight:
                continue
            cv2.putText(frame, "x: {}, y: {}, r: {}".format(int(x+w/2), int(y+h/2),
                        int(w/2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(frame, (int(x + w / 2), int(y + h/2)),
                       int(w/2), (0, 255, 0), 2)

    def findRobot(self, frame):
        # range of rectangle green color
        lower_green = np.array([30, 42, 42])
        upper_green = np.array([93, 255, 255])
        # green color mask / convert frame to HSV color
        greenMask = cv2.inRange(cv2.cvtColor(
            frame, cv2.COLOR_BGR2HSV), lower_green, upper_green)

        # get the green rectangle
        greenRectangle = cv2.bitwise_and(frame, frame, mask=greenMask)

        # convert to gray
        gray = cv2.cvtColor(greenRectangle, cv2.COLOR_BGR2GRAY)

        # catch the green rectangle
        # threshold get a image out of a gray image
        ret, threshGreen = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)

        # green rectangle contours
        # contours recognize the shape
        rectangleContours, hierarchy = cv2.findContours(
            threshGreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #  cheack for big Green area
        if len(rectangleContours) > 0:
            maxGreenArea = max(rectangleContours, key=cv2.contourArea)
            robotX, robotY, robotWidth, robotHeight = cv2.boundingRect(
                maxGreenArea)
            self.robotX = robotX - 70
            self.robotY = robotY - 70
            self.robotWidth = robotWidth + 130
            self.robotHeight = robotHeight + 130
            cv2.rectangle(frame, (self.robotX, self.robotY), (self.robotX +
                          self.robotWidth, self.robotY+self.robotHeight), (0, 255, 0), 3)


Main()
