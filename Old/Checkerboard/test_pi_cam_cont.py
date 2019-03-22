
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
#import timestamp
import time
import numpy as np
from PIL import Image


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1920, 1080)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(1920, 1080))

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
# allow the camera to warmup
time.sleep(0.1)
count = 0
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
 
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
	if ret == True:
		objpoints.append(objp)
		corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
		imgpoints.append(corners2)
		image = cv2.drawChessboardCorners(image, (7,6), corners2,ret)
		cv2.imshow('img',image)
		cv2.waitKey(500)
	else:
		cv2.imshow('img', image)
	key = cv2.waitKey(1)
	if key == 27: # exit on ESC
		break
cv2.destroyAllWindows()
cap.release()