import math

import cv2
import cvzone
import numpy as np

from ball import Ball
from cross import Cross
from frame import Frame
from robot import Robot


# The purpose of this python file is to detect the different objects on the field
# using hsv color codes.
# Source: OpenCV (https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html)


# Setting the hsv color values for the different objects on the field
hsvWhiteBall = {'hmin': 0, 'smin': 0, 'vmin': 198, 'hmax': 179, 'smax': 52, 'vmax': 255}
hsvOrangeBall = {'hmin': 11, 'smin': 56, 'vmin': 197, 'hmax': 169, 'smax': 255, 'vmax': 255}
hsvCross = {'hmin': 0, 'smin': 174, 'vmin': 211, 'hmax': 179, 'smax': 255, 'vmax': 255}
hsvGreen = {'hmin': 52, 'smin': 18, 'vmin': 0, 'hmax': 98, 'smax': 255, 'vmax': 255}
hsvBlue = {'hmin': 98, 'smin': 61, 'vmin': 0, 'hmax': 159, 'smax': 255, 'vmax': 255}

rectanglesCross = []


def detectOrangeBall(frame, robot):

    global hsvOrangeBall

    hmin, smin, vmin = hsvOrangeBall['hmin'], hsvOrangeBall['smin'], hsvOrangeBall['vmin']
    hmax, smax, vmax = hsvOrangeBall['hmax'], hsvOrangeBall['smax'], hsvOrangeBall['vmax']

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    cv2.GaussianBlur(hsv, (5, 5), 0)

    lower_range = np.array([hmin, smin, vmin], dtype=np.uint8)
    upper_range = np.array([hmax, smax, vmax], dtype=np.uint8)

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
                # if robot and x > robot.x and x < robot.x + robot.width and y > robot.y and y < robot.y + robot.height:
                #     continue
                cv2.circle(frame, (int(x + w / 2), int(y + h / 2)),
                           int(max(w, h) / 2), (0, 255, 0), 2)
                return Ball(x + radius / 2, y +
                            radius / 2)
    return None


def detectBalls(previousFrames, robot):

    global hsvWhiteBall

    hmin, smin, vmin = hsvWhiteBall['hmin'], hsvWhiteBall['smin'], hsvWhiteBall['vmin']
    hmax, smax, vmax = hsvWhiteBall['hmax'], hsvWhiteBall['smax'], hsvWhiteBall['vmax']

    filteredBalls = []

    for frame in previousFrames :
        balls = []
 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        cv2.GaussianBlur(hsv, (5, 5), 0)

        lower_range = np.array([hmin, smin, vmin], dtype=np.uint8)
        upper_range = np.array([hmax, smax, vmax], dtype=np.uint8)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.inRange(hsv, lower_range, upper_range)
        mask = cv2.erode(mask, kernel)
        mask = cv2.dilate(mask, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 20:
                    (x, y, w, h) = cv2.boundingRect(contour)
                    radius = int(w / 2)
                    if (radius < 11 and radius >= 3) or (radius > 13 and radius < 19):
                        # if robot is not None and is_point_inside_rectangle((x,y), robot.box)==True:
                        #     continue
                        cv2.circle(frame, (int(x + w / 2), int(y + h / 2)),
                                int(max(w, h) / 2), (0, 255, 0), 2)
                        balls.append(Ball(x + radius / 2, y +
                                    radius / 2))
                    
        if (len(balls) > 0) :
            for ball in filteredBalls :
                if ball not in balls :
                    filteredBalls.remove(ball)
        else :
            filteredBalls = balls

    return balls


def detectRobot(frame):

    global hsvGreen

    hmin, smin, vmin = hsvGreen['hmin'], hsvGreen['smin'], hsvGreen['vmin']
    hmax, smax, vmax = hsvGreen['hmax'], hsvGreen['smax'], hsvGreen['vmax']

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_range = np.array([hmin, smin, vmin], dtype=np.uint8)
    upper_range = np.array([hmax, smax, vmax], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_range, upper_range)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        center = np.mean(box, axis=0)
        # robotH = math.sqrt((box[0][0] - box[1][0]) **
        #                    2 + (box[0][1] - box[1][1]) ** 2)
        # robotW = math.sqrt((box[1][0] - box[2][0]) **
        #                    2 + (box[1][1] - box[2][1]) ** 2)

        blueFrame = detectBlueFrame(frame)
        if blueFrame is not None :
            robot = Robot(Frame(center[0], center[1], box), blueFrame)
            robot.box = getRobotBox(robot)
            return robot

def getRobotBox(robot) :
        if robot.greenFrame is not None and robot.blueFrame is not None:
        
            # Define the distance between the two lines
            distance = 40

            # Calculate the slope of the line
            s = (robot.blueFrame.x - robot.greenFrame.x)
            #print(s)
            if s==0 or s==0.0 :
                s = 0.01
            
            slope = (robot.blueFrame.y - robot.greenFrame.y) / s
            # print(slope)

            # Calculate the angle of the line in radians
            angle = np.arctan(slope)

            # Calculate the x and y offsets for the parallel line
            x_offset = distance * np.sin(angle)
            y_offset = distance * np.cos(angle)

            # New points for the parallel line
            pt1 = (int(robot.greenFrame.x - x_offset), int(robot.greenFrame.y + y_offset))
            pt2 = (int(robot.blueFrame.x - x_offset), int(robot.blueFrame.y + y_offset))
            pt3 = (int(robot.greenFrame.x + x_offset), int(robot.greenFrame.y - y_offset))
            pt4 = (int(robot.blueFrame.x + x_offset), int(robot.blueFrame.y - y_offset))

            # Define the length to extend the line by
            # length = 50

            # # Calculate the x and y distances to extend the line by
            # dx = length / (2 * (1 + slope**2))**0.5
            # dy = slope * dx

            # # # Calculate the new points
            # pt1 = (int(pt1[0] - dx), int(pt1[1] - dy))
            # pt2 = (int(pt2[0] + dx), int(pt2[1] + dy))
            # pt3 = (int(pt3[0] - dx), int(pt3[1] - dy))
            # pt4 = (int(pt4[0] + dx), int(pt4[1] + dy))

            box = np.array([pt1, pt2, pt3, pt4])

            return box

# TODO - Not done
def is_point_inside_rectangle(point, rectangle):
    
    # Step 1: Define the coordinates of the rectangle's vertices
    # print(rectangle)
    x1,y1 = rectangle[0]
    x2,y2 = rectangle[1]
    x3,y3 = rectangle[2]
    x4,y4 = rectangle[3]
    D1 = (x2 - x1) * (point[1] - y1) - (point[0] - x1) * (y2 - y1)
    D2 = (x3 - x2) * (point[1] - y2) - (point[0] - x2) * (y3 - y2)
    D3 = (x4 - x3) * (point[1] - y3) - (point[0] - x3) * (y4 - y3)
    D4 = (x1 - x4) * (point[1] - y4) - (point[0] - x4) * (y1 - y4)
    print("D1: ",D1)
    print("D2: ",D2)
    print("D3: ",D3)
    print("D4: ",D4)
    if (D1 < 0 and D2 > 0 and D3 > 0 and D4 < 0) :
        return True
    return False

def detectBlueFrame(frame):

    global hsvBlue
    
    hmin, smin, vmin = hsvBlue['hmin'], hsvBlue['smin'], hsvBlue['vmin']
    hmax, smax, vmax = hsvBlue['hmax'], hsvBlue['smax'], hsvBlue['vmax']

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_range = np.array([hmin, smin, vmin], dtype=np.uint8)
    upper_range = np.array([hmax, smax, vmax], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_range, upper_range)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        center = np.mean(box, axis=0)

        return Frame(center[0], center[1], box)


def drawLine(frame, x1, y1, x2, y2, robot):
    x1, y1 = int(x1), int(y1)
    x2, y2 = int(x2), int(y2)

    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

    midpoint = ((x1 + x2) // 2, (y1 + y2) // 2 - 50)

    # cvzone.putTextRect(frame, "Angle: {:.2f}".format(getAngle(
    #     robot, object, blueframe)), (50,50))

    cv2.putText(frame, "Angle: {:.2f}".format(getAngle(
        robot, Ball(x2, y2))), midpoint, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, "Distance: {:.2f} cm".format(getDistance(x1, y1, x2, y2)), (
        midpoint[0], midpoint[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


def getDistance(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    distance_in_pixels = int(math.sqrt(dx * dx + dy * dy))
    return distance_in_pixels / 4.2


def getAngle(robot, object):
    a = [robot.greenFrame.x - robot.blueFrame.x, robot.greenFrame.y - robot.blueFrame.y]
    b = [robot.greenFrame.x - object.x, robot.greenFrame.y - object.y]
    angle = np.math.atan2(np.linalg.det([a, b]), np.dot(a, b))
    angle = np.degrees(angle)
    return int(angle)

# Check if the line is crossing the circle around the cross
# Source: https://mathworld.wolfram.com/Circle-LineIntersection.html
def is_line_crossing_circle(point1, point2, circle_center, circle_radius):
    distance = math.sqrt((point2[0] - point1[0]) **
                         2 + (point2[1] - point1[1])**2)
    direction_vector = [(point2[0] - point1[0]) / distance,
                        (point2[1] - point1[1]) / distance]
    endpoint_vector = [point1[0] - circle_center[0],
                       point1[1] - circle_center[1]]
    dot_product = direction_vector[0] * endpoint_vector[0] + \
        direction_vector[1] * endpoint_vector[1]
    closest_point = [circle_center[0] + dot_product * direction_vector[0],
                     circle_center[1] + dot_product * direction_vector[1]]
    closest_distance = math.sqrt(
        (closest_point[0] - circle_center[0])**2 + (closest_point[1] - circle_center[1])**2)

    if closest_distance <= circle_radius:
        return True
    else:
        return False

# Source: OpenCV,  https://www.geeksforgeeks.org/drawing-a-cross-on-an-image-with-opencv/
def detectCross(frame) :
    
    global hsvCross

    hmin, smin, vmin = hsvCross['hmin'], hsvCross['smin'], hsvCross['vmin']
    hmax, smax, vmax = hsvCross['hmax'], hsvCross['smax'], hsvCross['vmax']

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_range = np.array([hmin, smin, vmin], dtype=np.uint8)
    upper_range = np.array([hmax, smax, vmax], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_range, upper_range)
    
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:
                (x, y, w, h) = cv2.boundingRect(contour)
                if ( x > 50 and x < 700 and y > 50 and y < 450) :
                    
                    # largest_contour = max(contours, key=cv2.contourArea)
                    # rect = cv2.minAreaRect(largest_contour)
                    rect = cv2.minAreaRect(contour)

                    global rectanglesCross
                    rectanglesCross.append(rect)
                    if (len(rectanglesCross) > 10) :
                        rectanglesCross.pop(0)
                    
                    average_rect = getAverageRect()
                    
                    box = cv2.boxPoints(average_rect)
                    box = np.int0(box)

                    center = np.mean(box, axis=0)

                    offsets = [None]*4
                    if (box is not None and center is not None) :
                        # Drawing cross lines
                        line_length = 120  # Adjust this value to change the length of the lines
                        angle = average_rect[2]  # Angle of rotation
                        rad_angle = math.radians(angle)  # Convert angle to radians
                        cos_val = math.cos(rad_angle)  # Cosine of angle
                        sin_val = math.sin(rad_angle)  # Sine of angle
                        x1 = int(center[0] - line_length * sin_val)  # Starting x-coordinate of line
                        y1 = int(center[1] + line_length * cos_val)  # Starting y-coordinate of line
                        x2 = int(center[0] + line_length * sin_val)  # Ending x-coordinate of line
                        y2 = int(center[1] - line_length * cos_val)  # Ending y-coordinate of line
                        offsets[0] = (x1, y1)
                        offsets[1] = (x2, y2)
                        # cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)  # Draw first line
                        x1 = int(center[0] - line_length * cos_val)  # Starting x-coordinate of line
                        y1 = int(center[1] - line_length * sin_val)  # Starting y-coordinate of line
                        x2 = int(center[0] + line_length * cos_val)  # Ending x-coordinate of line
                        y2 = int(center[1] + line_length * sin_val)  # Ending y-coordinate of line
                        offsets[2] = (x1, y1)
                        offsets[3] = (x2, y2)
                        # cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)  # Draw second line

                        # Drawing box around cross
                        # rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(largest_contour)
                        rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(contour)
                        rect_x -= rect_w // 2  # Move top-left corner left by half the width
                        rect_y -= rect_h // 2  # Move top-left corner up by half the height
                        rect_w *= 2  # Double the width
                        rect_h *= 2  # Double the height
                        # cv2.rectangle(frame, (rect_x, rect_y), (rect_x+rect_w, rect_y+rect_h), (0, 255, 0), 3)
                        # for offset in offsets :
                        #     cv2.circle(center=offset, thickness=10, radius=0, img=frame, color=(0, 0, 255))
                        # if (rect_h > 148 and rect_h < 175 and rect_w > 148 and rect_w < 175) :
                        return Cross(center[0], center[1], rect_h, rect_w, rect_x, rect_y, offsets)

def drawCross(frame, cross) : 
    cv2.line(frame, (cross.offsets[0][0], cross.offsets[0][1]), (cross.offsets[1][0], cross.offsets[1][1]), (0, 255, 0), 3)  # Draw first line
    cv2.line(frame, (cross.offsets[2][0], cross.offsets[2][1]), (cross.offsets[3][0], cross.offsets[3][1]), (0, 255, 0), 3)  # Draw second line
    cv2.rectangle(frame, (cross.rect_x, cross.rect_y), (cross.rect_x+cross.rect_w, cross.rect_y+cross.rect_h), (0, 255, 0), 3)

def getAverageRect() : 
    sumA1 = 0
    sumA2 = 0
    sumB1 = 0
    sumB2 = 0
    sumC = 0
    global rectanglesCross
    for rect in rectanglesCross :
        sumA1 += rect[0][0]
        sumA2 += rect[0][1]
        sumB1 += rect[1][0]
        sumB2 += rect[1][1]
        sumC += rect[2]
    
    return ((sumA1/len(rectanglesCross), sumA2/len(rectanglesCross)), (sumB1/len(rectanglesCross), sumB2/len(rectanglesCross)), sumC/len(rectanglesCross))


def lineIntersectsCross(robot, ball, cross):
    half_w = cross.rect_w / 2
    half_h = cross.rect_h / 2
    top_left = (int(cross.x - half_w), int(cross.y - half_h))
    top_right = (int(cross.x + half_w), int(cross.y - half_h))
    bottom_left = (int(cross.x - half_w), int(cross.y + half_h))
    bottom_right = (int(cross.x + half_w), int(cross.y + half_h))
    
    sides = [(top_left, top_right), (top_left, bottom_left),
             (bottom_left, bottom_right), (top_right, bottom_right)]
    
    for side in sides:
        intersection = line_intersection(robot, ball, side[0], side[1])
        if intersection is not None:
            if intersection[0] >= min(side[0][0], side[1][0]) and intersection[0] <= max(side[0][0], side[1][0]) and \
               intersection[1] >= min(side[0][1], side[1][1]) and intersection[1] <= max(side[0][1], side[1][1]):
                return True
    return False

def line_intersection(robot, ball, side1, side2):
    x1, y1 = robot.x, robot.y
    x2, y2 = ball.x, ball.y
    x3, y3 = side1
    x4, y4 = side2
    
    det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    
    if det == 0:
        return None
    
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det
    
    if t >= 0 and t <= 1 and u >= 0 and u <= 1:
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        return (x, y)
    
    return None