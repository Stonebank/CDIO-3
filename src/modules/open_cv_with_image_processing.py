import cv2
import numpy as np

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Convert the frame to grayscale and apply a Gaussian blur
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (11, 11), 0)

    # Apply threshold to the image
    threshold = cv2.threshold(blurred, 220, 255, cv2.THRESH_BINARY)[1]

    # Find contours of white objects in the thresholded image
    contours, hierarchy = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Identify contours that match the shape and size of a ping pong ball
    ping_pong_ball_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / perimeter ** 2
        if area > 50 and circularity > 0.7:
            ping_pong_ball_contours.append(contour)

    # Draw circles around the identified ping pong balls in the original frame
    for contour in ping_pong_ball_contours:
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        cv2.circle(frame, center, radius, (0, 255, 0), 2)

    cv2.imshow('Ping pong detection', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
