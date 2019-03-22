import cv2
import numpy as np


cap = cv2.VideoCapture(0)
while True:
    r, frame = cap.read()
    #img = cv2.imread('ring.jpg',0)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img = cv2.medianBlur(frame,5)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    
    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,200,
                                param1=50,param2=40,minRadius=150,maxRadius=0)
    try:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
    except:
        pass
    
    cv2.imshow('detected circles',cimg)
    k = cv2.waitKey(1)
    if k == 0xFF & ord("q"):
        break
cv2.destroyAllWindows()
cap.release()