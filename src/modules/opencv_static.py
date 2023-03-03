import cv2
import numpy as np

path = r"screenshot.png"

image = cv2.imread("screenshot.png")

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

cv2.imshow('image', image)

lower_red = np.array([0, 0, 200], dtype = "uint8") 

upper_red= np.array([100, 100, 255], dtype = "uint8")

mask = cv2.inRange(image, lower_red, upper_red)

detected_output = cv2.bitwise_and(image, image, mask =  mask) 

cv2.imshow("red color detection", detected_output) 

cv2.waitKey(0) 

cv2.destroyAllWindows()
