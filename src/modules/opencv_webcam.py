import cv2
import numpy as np

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

lower_red = np.array([0, 0, 200], dtype = "uint8") 

upper_red= np.array([100, 100, 255], dtype = "uint8")

lower_white = np.array([200, 200, 200], dtype = "uint8") 

upper_white= np.array([255, 255, 255], dtype = "uint8")

while True:

    ret, frame = cap.read()
    
    if not ret:
        print("Failed to capture frame from webcam")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    whiteMask = cv2.inRange(frame, lower_white, upper_white)

    redMask = cv2.inRange(frame, lower_red, upper_red)

    detected_output_red = cv2.bitwise_and(frame, frame, mask = redMask) 

    detected_output_white = cv2.bitwise_and(frame, frame, mask = whiteMask) 

    cv2.imshow("red color detection", detected_output_red) 

    cv2.imshow("white color detection", detected_output_white) 
    
    '''cv2.imshow("Blur", gray)

    bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 51, 10)

    cv2.imshow("Bw", bw)

    circles = cv2.HoughCircles(bw, cv2.HOUGH_GRADIENT, 2, 10, param1=50, param2=30, minRadius=5, maxRadius=14)

    print(circles)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for circle in circles:

            if circle[2] < 15:

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                cv2.imshow("hsv", hsv)

                lowerOrange = np.array([0, 128, 128])
                upperOrange = np.array([255, 200, 255])
                maskOrange = cv2.inRange(hsv, lowerOrange, upperOrange)

                lowerWhite = np.array([0, 0, 220])
                upperWhite = np.array([255, 30, 255])
                maskWhite = cv2.inRange(hsv, lowerWhite, upperWhite)

                mask = cv2.bitwise_or(maskOrange, maskWhite)

                cv2.imshow("White and orange color mask", mask)

                masked = cv2.bitwise_and(frame, frame, mask=mask)

                cv2.imshow("Bitwise mask", masked)

                x, y, r = circle[0], circle[1], circle[2]

                cropped = masked[y - r:y + r, x - r:x + r]
                mean = cv2.mean(cropped)

                if mean[2] > 50:
                    cv2.putText(frame, str(x - r) + " " + str(y - r), (x - r, y - r - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
                    cv2.rectangle(frame, (x - r, y - r), (x + r, y + r), (0, 255, 0), 2)'''

    cv2.imshow("Ping pong detection", frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
