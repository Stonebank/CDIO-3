import math

import cv2
import numpy as np

from ball import Ball
from robot import Robot


def detectOrangeBall(frame, robot):

    orange_ball_hsv_values = {'hmin': 16, 'smin': 98,
                              'vmin': 191, 'hmax': 21, 'smax': 255, 'vmax': 255}

    hmin, smin, vmin = orange_ball_hsv_values['hmin'], orange_ball_hsv_values['smin'], orange_ball_hsv_values['vmin']
    hmax, smax, vmax = orange_ball_hsv_values['hmax'], orange_ball_hsv_values['smax'], orange_ball_hsv_values['vmax']

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
                if radius > 9 or radius < 7:
                    continue
                if robot and x > robot.x and x < robot.x + robot.width and y > robot.y and y < robot.y + robot.height:
                    continue
                cv2.circle(frame, (int(x + w / 2), int(y + h / 2)),
                           int(max(w, h) / 2), (0, 255, 0), 2)
                return Ball(x + radius / 2, y +
                            radius / 2, radius, 0)
    return None


def detectBalls(frame, robot):

    balls = []

    hsv_values = {'hmin': 0, 'smin': 0, 'vmin': 220,
                  'hmax': 179, 'smax': 111, 'vmax': 255}

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
                if radius > 9 or radius < 7:
                    continue
                if robot and x > robot.x and x < robot.x + robot.width and y > robot.y and y < robot.y + robot.height:
                    continue
                cv2.circle(frame, (int(x + w / 2), int(y + h / 2)),
                           int(max(w, h) / 2), (0, 255, 0), 2)
                balls.append(Ball(x + radius / 2, y +
                             radius / 2, radius, len(balls)))

    return balls


def detectRobot(frame):

    hsv_values = {'hmin': 27, 'smin': 0, 'vmin': 0,
                  'hmax': 107, 'smax': 255, 'vmax': 255}
    

    hmin, smin, vmin = hsv_values['hmin'], hsv_values['smin'], hsv_values['vmin']
    hmax, smax, vmax = hsv_values['hmax'], hsv_values['smax'], hsv_values['vmax']

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_range = np.array([hmin, smin, vmin])
    upper_range = np.array([hmax, smax, vmax])

    mask = cv2.inRange(hsv, lower_range, upper_range)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        cv2.drawContours(frame, [box], 0, (0, 255, 0), 3)
        center = np.mean(box, axis=0)
        robotH = math.sqrt((box[0][0] - box[1][0]) **
                           2 + (box[0][1] - box[1][1]) ** 2)
        robotW = math.sqrt((box[1][0] - box[2][0]) **
                           2 + (box[1][1] - box[2][1]) ** 2)
        
        return Robot(center[0], center[1], robotH, robotW)


def detectBlueFrame(frame):

    hsv_values = {'hmin': 110, 'smin': 33, 'vmin': 0,
                  'hmax': 161, 'smax': 232, 'vmax': 192}

    hmin, smin, vmin = hsv_values['hmin'], hsv_values['smin'], hsv_values['vmin']
    hmax, smax, vmax = hsv_values['hmax'], hsv_values['smax'], hsv_values['vmax']

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_range = np.array([hmin, smin, vmin])
    upper_range = np.array([hmax, smax, vmax])

    mask = cv2.inRange(hsv, lower_range, upper_range)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        cv2.drawContours(frame, [box], 0, (0, 255, 0), 3)
        center = np.mean(box, axis=0)

        return center


def drawLine(frame, x1, y1, x2, y2, robot, object, blueframe):
    x1, y1 = int(x1), int(y1)
    x2, y2 = int(x2), int(y2)

    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

    midpoint = ((x1 + x2) // 2, (y1 + y2) // 2 - 50)

    cv2.putText(frame, "Angle: {:.2f}".format(getAngle(
        robot, object, blueframe)), midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, "Distance: {:.2f} cm".format(getDistance(x1, y1, x2, y2)), (
        midpoint[0], midpoint[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def getDistance(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    distance_in_pixels = int(math.sqrt(dx * dx + dy * dy))
    return distance_in_pixels / 4.2


def getAngle(robot, object, blueframe):
    a = [robot.x - blueframe[0], robot.y - blueframe[1]]
    b = [robot.x - object.x, robot.y - object.y]
    angle = np.math.atan2(np.linalg.det([a, b]), np.dot(a, b))
    angle = np.degrees(angle)
    return int(angle)


def is_line_crossing_circle(point1, point2, circle_center, circle_radius):
    distance = math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    direction_vector = [(point2[0] - point1[0]) / distance, (point2[1] - point1[1]) / distance]
    endpoint_vector = [point1[0] - circle_center[0], point1[1] - circle_center[1]]
    dot_product = direction_vector[0] * endpoint_vector[0] + direction_vector[1] * endpoint_vector[1]
    closest_point = [circle_center[0] + dot_product * direction_vector[0], circle_center[1] + dot_product * direction_vector[1]]
    closest_distance = math.sqrt((closest_point[0] - circle_center[0])**2 + (closest_point[1] - circle_center[1])**2)
    
    if closest_distance <= circle_radius:
        return True
    else:
        return False
