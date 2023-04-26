import cv2
import numpy as np
import math
from ball import Ball

def detect_balls(frame):

    balls = []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    lower_white = np.array([200, 200, 200], dtype="uint8")
    upper_white = np.array([255, 255, 255], dtype="uint8")

    white_mask = cv2.inRange(frame, lower_white, upper_white)

    contours, hierarchy = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        (x, y, w, h) = cv2.boundingRect(contour)
        radius = w / 2
        if radius > 9 or radius < 7:
            continue
        balls.append(Ball(x + radius / 2, y + radius / 2, radius, len(balls)))

        #cv2.putText(frame, str(len(balls)), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.circle(frame, (int(x + radius), int(y + radius)), int(radius), (0, 255, 0), 2)

    return balls

def detect_robot(frame):

    lower_green = np.array([30, 42, 42])
    upper_green = np.array([93, 255, 255])

    greenMask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lower_green, upper_green)
    greenRectangle = cv2.bitwise_and(frame, frame, mask=greenMask)

    gray = cv2.cvtColor(greenRectangle, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        greenArea = max(contours, key=cv2.contourArea)
        robotX, robotY, robotW, robotH = cv2.boundingRect(greenArea)
        rX = robotX - 70
        rY = robotY - 70
        rW = robotW + 130
        rH = robotH + 130
        if robotX > rX and robotX < rX + rW and robotY > rY and robotY < rY + rH:
            cv2.rectangle(frame, (rX, rY), (rX + rW, rY + rH), (0, 255, 0), 2)

def detect_blue_frame(frame):
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    blue_mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), lower_blue, upper_blue)
    blue_rectangle = cv2.bitwise_and(frame, frame, mask=blue_mask)
    gray = cv2.cvtColor(blue_rectangle, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        cv2.drawContours(frame, [box], 0, (0, 255, 0), 3)

def draw_line(frame, header, neighbour):
    x1, y1 = int(header.x + header.radius / 2), int(header.y + header.radius / 2)
    x2, y2 = int(neighbour.x + neighbour.radius / 2), int(neighbour.y + neighbour.radius / 2)
    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

    midpoint = ((x1 + x2) // 2, (y1 + y2) // 2 - 50)

    cv2.putText(frame, "Angle: {:.2f}".format(header.angle_to(neighbour)), midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, "Distance: {:.2f} cm".format(header.distance_to(neighbour)), (midpoint[0], midpoint[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def main():

    frame = cv2.imread("robot.png")

    balls = detect_balls(frame)

    header = balls[0]
    neighbour = balls[1]

    draw_line(frame, header, neighbour)

    detect_robot(frame)
    detect_blue_frame(frame)

    cv2.imshow("frame", frame)
    cv2.waitKey(0)


if __name__ == "__main__":
    main()


    