
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

import time
import numpy as np
from PIL import Image


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1920, 1080)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(1920, 1080))
 
# allow the camera to warmup
time.sleep(0.1)
count = 0

mtx = np.array([[  1.31960797e+03,   0.00000000e+00,   1.00916350e+03],
       [  0.00000000e+00,   1.35786213e+03,   6.36593029e+02],
       [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
dist = np.array([[ -3.86818203e-01,   2.81274627e+00,  -2.46548663e-02,
         -1.66810086e-05,  -1.20151138e+01]])
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	img = frame.array
	h,  w = img.shape[:2]
	newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
	dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
 
	# show the frame
	cv2.imshow("preview", dst)
	#img2 = Image.fromarray(frame, 'RGB')
	#img2.show()
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
	# elif key == ord('c'):
	# 	filename = "image" + str(count) + ".jpg"
	# 	cv2.imwrite(filename, image)
	# 	count += 1
cv2.destroyAllWindows()

