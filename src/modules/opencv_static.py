import cv2
import numpy as np

path = "screenshot.png"
image = cv2.imread(path)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

lower_white = np.array([200, 200, 200], dtype = "uint8") 
upper_white = np.array([255, 255, 255], dtype = "uint8")

whiteMask = cv2.inRange(image, lower_white, upper_white)

contours, hierarchy = cv2.findContours(whiteMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    (x, y, w, h) = cv2.boundingRect(contour)
    cv2.putText(image, "x: {}, y: {}, r: {}".format(int(x+w/2), int(y+h/2), int(w/2)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.circle(image, (int(x + w/ 2 ), int(y + h/2)), int(w/2), (0, 255, 0), 2)

cv2.imshow("Ping pong balls detection", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
