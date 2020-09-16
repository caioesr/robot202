import cv2
import numpy as np
import matplotlib.pyplot as plt

cap = cv2.VideoCapture('line_following.mp4')



while (cap.isOpened()):
    
    # Reading the image and getting it to gray scale
    ret, frame = cap.read()
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
    # Declaring the polygon that will be cut from the image
    width = frame.shape[1]
    height = frame.shape[0]
    
    polygon = np.array([[0, height / 2], [0, 0], [width, 0], [width, height / 2], [width / 2, height / 2 - 75]])
    polygon.astype('int32')
    
    # Cleaning the image from other colors than white
    for i in range(height):
        for j in range(width):
            if frame_gray[i][j] < 240:
                frame_gray[i][j] = 0
    
    # Filling the gray image with the polygon
    frame_gray = cv2.fillPoly(frame_gray, [polygon], 0)
    
    # Hough Lines
    frame_cunny = cv2.Canny(frame_gray, 20, 350)
    lines = cv2.HoughLinesP(frame_cunny, 1, np.pi / 180, 0)
    print(len(lines))
    
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)
    
    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()