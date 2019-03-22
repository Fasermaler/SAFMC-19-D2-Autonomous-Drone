
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

import time
import numpy as np
from PIL import Image


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1080, 720)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(1080, 720))
 
# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	img = frame.array
	b_channel = np.array(img[:,:,0]).astype('float')
    g_channel = np.array(img[:,:,1]).astype('float')
    r_channel = np.array(img[:,:,2]).astype('float')
    cv2.imshow('b_chan', b_channel)
#    cv2.imshow('g_chan', g_channel)
#    cv2.imshow('r_chan', r_channel)
    bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
    img_rec_green = np.divide(g_channel, bgr_channel)
    np.floor(img_rec_green)
    img_rec_green = img_rec_green * 255
    img_rec_green = np.floor(img_rec_green).astype('uint8')
 
	# show the frame
	cv2.imshow("preview", img)
	cv2.imshow("rec_green", img_rec_green)
	#img2 = Image.fromarray(frame, 'RGB')
	#img2.show()
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

cv2.destroyAllWindows()

