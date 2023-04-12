import sys
import threading
import cv2
import numpy as np

from frameProvider import FrameTransformer


class Main:
    def __init__(self):
        # Set video input
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        ft = FrameTransformer()
        frameCount = 0

        while True:
            frameCount += 1
            ret, frame = cap.read()
            transformed = ft.transform(frame, frameCount)
            transformed = frame if transformed is None else transformed
            self.findRobot(transformed)
            self.findBalls(transformed)
            if cv2.waitKey(1) == ord('m'):
                if (not ft.manMode):
                    ft.manMode = True
                    cv2.namedWindow("Board")
                    cv2.setMouseCallback("Board", ft.get_point)
                    print("MANUAL MODE - input corners")
                elif (ft.manMode):
                    ft.manMode = False
                    print("AUTO MODE")
            elif cv2.waitKey(1) & 0xFF == ord('q'):
                break
            cv2.imshow("Transformed", transformed)
            cv2.imshow("Board", frame)

        cv2.destroyAllWindows()
        sys.exit()

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
            cv2.putText(frame, "x: {}, y: {}, r: {}".format(int(x+w/2), int(y+h/2),
                        int(w/2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(frame, (int(x + w / 2), int(y + h/2)),
                       int(w/2), (0, 255, 0), 2)
    
    def findRobot(self, frame):
        # range of rectangle green color 
        lower_green = np.array([30, 42, 42])
        upper_green = np.array([93, 255, 255])
        # green color mask / convert frame to HSV color
        greenMask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lower_green, upper_green)
        
        # get the green rectangle 
        greenRectangle = cv2.bitwise_and(frame, frame, mask=greenMask)
        
        # convert to gray
        gray = cv2.cvtColor(greenRectangle, cv2.COLOR_BGR2GRAY)
        
        # catch the green rectangle
        #threshold get a image out of a gray image
        ret, threshGreen = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)
        
        # green rectangle contours 
        #contours recognize the shape 
        rectangleContours, hierarchy = cv2.findContours(threshGreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #  cheack for big Green area
        if len(rectangleContours) > 0:
            maxGreenArea = max(rectangleContours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(maxGreenArea)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
        


Main()
