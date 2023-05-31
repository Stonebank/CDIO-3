import cv2
import numpy as np
import math
from ball import Ball
from robot import Robot


def detectBalls(frame, robot):

    balls = []

    hsv_values = {'hmin': 0, 'smin': 0, 'vmin': 218,
                  'hmax': 179, 'smax': 67, 'vmax': 255}

    hmin, smin, vmin = hsv_values['hmin'], hsv_values['smin'], hsv_values['vmin']
    hmax, smax, vmax = hsv_values['hmax'], hsv_values['smax'], hsv_values['vmax']

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_range = np.array([hmin, smin, vmin])
    upper_range = np.array([hmax, smax, vmax])

    mask = cv2.inRange(hsv, lower_range, upper_range)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:
                (x, y, w, h) = cv2.boundingRect(contour)
                radius = int(w / 2)
                if x > robot.x and x < robot.x + robot.width and y > robot.y and y < robot.y + robot.height:
                    continue
                cv2.circle(frame, (int(x + w / 2), int(y + h / 2)),
                           int(max(w, h) / 2), (0, 255, 0), 2)
                balls.append(Ball(x + radius / 2, y +
                             radius / 2, radius, len(balls)))

    return balls


def detectRobot(frame):

    lower_green = np.array([30, 42, 42])
    upper_green = np.array([93, 255, 255])

    greenMask = cv2.inRange(cv2.cvtColor(
        frame, cv2.COLOR_BGR2HSV), lower_green, upper_green)
    greenRectangle = cv2.bitwise_and(frame, frame, mask=greenMask)

    gray = cv2.cvtColor(greenRectangle, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        greenArea = max(contours, key=cv2.contourArea)
        robotX, robotY, robotW, robotH = cv2.boundingRect(greenArea)

        rect = cv2.minAreaRect(greenArea)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # Compute the center of the box
        center = np.mean(box, axis=0)

        # Scale the box vertices along the x and y axis separately
        scale_factor_x = 4
        scale_factor_y = 2
        scaled_box = np.zeros_like(box)
        scaled_box[:, 0] = center[0] + scale_factor_x * (box[:, 0] - center[0])
        scaled_box[:, 1] = center[1] + scale_factor_y * (box[:, 1] - center[1])

        rX = robotX - 70
        rY = robotY - 70
        rW = robotW + 130
        rH = robotH + 130
        if robotX > rX and robotX < rX + rW and robotY > rY and robotY < rY + rH:
            # Draw the scaled box on the image
            cv2.drawContours(frame, [np.int0(scaled_box)], 0, (0, 255, 0), 3)
        return Robot(center[0], center[1], robotH, robotW)


def detectBlueFrame(frame):
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    blue_mask = cv2.inRange(cv2.cvtColor(
        frame, cv2.COLOR_BGR2HSV), lower_blue, upper_blue)
    blue_rectangle = cv2.bitwise_and(frame, frame, mask=blue_mask)
    gray = cv2.cvtColor(blue_rectangle, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        cv2.drawContours(frame, [box], 0, (0, 255, 0), 3)
        # Compute the center of the box
        center = np.mean(box, axis=0)
        return center


def drawLine(frame, x1, y1, x2, y2):
    x1, y1 = int(x1), int(y1)
    x2, y2 = int(x2), int(y2)

    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

    midpoint = ((x1 + x2) // 2, (y1 + y2) // 2 - 50)

    cv2.putText(frame, "Angle: {:.2f}".format(getAngle(
        x1, y1, x2, y2)), midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, "Distance: {:.2f} cm".format(getDistance(x1, y1, x2, y2)), (
        midpoint[0], midpoint[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def getDistance(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    distance_in_pixels = int(math.sqrt(dx * dx + dy * dy))
    return distance_in_pixels / 4.2


def getAngle(x1, y1, x2, y2):
    angle = math.degrees(math.atan2(y2 - y1, x2 - x1))
    if angle < 0:
        angle += 360
    return int(angle)
