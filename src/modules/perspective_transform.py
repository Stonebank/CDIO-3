import cv2
import numpy as np

# Set video input
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

# Define lower/upper color field for red
lower_red = np.array([0, 0, 200], dtype = "uint8") 
upper_red= np.array([100, 100, 255], dtype = "uint8")

# Set default x,y values for corners
upper_left = [99999, 99999]
upper_right = [0, 99999]
lower_left = [99999, 0]
lower_right = [0, 0]

while True:
    # Webcam as image soruce
    # ret, frame = cap.read()
    # if not ret:
    #     print("Failed to capture frame from webcam")
    #     break

    # File as image source
    frame = cv2.imread("bane.png")

    # Frame dimensions
    height = frame.shape[0]
    width = frame.shape[1]
   
    redMask = cv2.inRange(frame, lower_red, upper_red)

    # Filter out everything that is not red
    wall = frame.copy()
    wall[np.where(redMask==0)] = 0
    
    # Convert to grey
    wall = cv2.cvtColor(wall, cv2.COLOR_BGR2GRAY)
  
    # Find corners
    dest = cv2.cornerHarris(np.float32(wall), 2, 5, 0.1)
    dest = cv2.dilate(dest, None)
    # frame[dest > 0.01 * dest.max()]=[0, 0, 255]

    # Iterate through the detected corners, and use the appropiate ones
    # TODO should be optimized in the future
    thresh = 0.1*dest.max()
    if lower_right[0] == 0 :
        for j in range(0, dest.shape[0]):
            for i in range(0, dest.shape[1]):
                if(dest[j,i] > thresh):
                    if i < upper_left[0] and i < width/2 and j < upper_left[1] and j < height/2 :
                        upper_left[0] = i
                        upper_left[1] = j
                    if i > upper_right[0] and i > width/2 and j < upper_right[1] and j < height/2 :
                        upper_right[0] = i
                        upper_right[1] = j
                    if i < lower_left[0] and i < width/2 and j > lower_left[1] and j > height/2 :
                        lower_left[0] = i
                        lower_left[1] = j
                    if i > lower_right[0] and i > width/2 and j > lower_right[1] and j > height/2 :
                        lower_right[0] = i
                        lower_right[1] = j

    # Coordinates for the corners                
    oldCoordinates = np.float32([upper_left, upper_right,
                      lower_left, lower_right])
    # New coordinates for the corners
    newCoordinates = np.float32([[0, 0], [width, 0],
                       [0, height], [width, height]])
    
    # Transform image
    matrix = cv2.getPerspectiveTransform(oldCoordinates, newCoordinates)
    transformed = cv2.warpPerspective(frame, matrix, (width, height))

    # Create circles to indicate detected corners 
    frame = cv2.circle(frame, tuple(upper_left), 20, (255, 0, 0), 2)
    frame = cv2.circle(frame, tuple(upper_right), 20, (255, 0, 0), 2)
    frame = cv2.circle(frame, tuple(lower_left), 20, (255, 0, 0), 2)
    frame = cv2.circle(frame, tuple(lower_right), 20, (255, 0, 0), 2)

    cv2.imshow("Board", frame) 

    cv2.imshow("Transformed", transformed) 
    
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
