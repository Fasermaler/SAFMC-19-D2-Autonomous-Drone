
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import timestamp
import time
import numpy as np
from PIL import Image


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (426, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(426, 240))
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array
 
	# show the frame
	cv2.imshow("preview", image)
	#img2 = Image.fromarray(frame, 'RGB')
	#img2.show()
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
cv2.destroyAllWindows()
cap.release()