
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

import time
import numpy as np
from PIL import Image


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1648, 928)
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(1648, 928))

out = cv2.VideoWriter(str(time.time()) + ".avi",cv2.VideoWriter_fourcc('M','J','P','G'), 10, (412, 232))
 
# allow the camera to warmup
time.sleep(0.1)
count = 0
start = time.time()
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	if time.time() - start > 30:
		break
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
	image = cv2.pyrDown(cv2.pyrDown(image))

 	out.write(image)
	# show the frame
	#cv2.imshow("preview", image)
	#img2 = Image.fromarray(frame, 'RGB')
	#img2.show()
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	#if key == ord("q"):
	#	break
	# elif key == ord('c'):
	# 	filename = "image" + str(count) + ".jpg"
	# 	cv2.imwrite(filename, image)
	# 	count += 1
out.release()
#cv2.destroyAllWindows()

