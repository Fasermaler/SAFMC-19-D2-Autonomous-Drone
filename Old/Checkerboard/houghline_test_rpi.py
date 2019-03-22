
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

import time
import numpy as np
from PIL import Image


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(1280, 720))
 
# allow the camera to warmup
time.sleep(0.1)
count = 0
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,100,150,apertureSize = 3)

    lines = cv2.HoughLines(edges,1,np.pi/180,180, None, 0, 0)
    print(lines)
    for i in range(0, len(lines)):
        for rho,theta in lines[i]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1400*(-b))
            y1 = int(y0 + 1400*(a))
            x2 = int(x0 - 1400*(-b))
            y2 = int(y0 - 1400*(a))

            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

    #2.imshow('edges',edges)
    cv2.imshow('houghlines',img)

    rawCapture.truncate(0)
    k = cv2.waitKey(1)
    if k == 0xFF & ord("q"):
        break
cv2.destroyAllWindows()

