
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import imutils
from PIL import Image


def image_convert_to_perc_green(img):
	b_channel = np.array(img[:,:,0]).astype('float')
	g_channel = np.array(img[:,:,1]).astype('float')
	r_channel = np.array(img[:,:,2]).astype('float')
	bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
	img_rec_green = np.divide(g_channel, bgr_channel)
	img_rec_green = img_rec_green * 255
	img_rec_green = np.floor(img_rec_green).astype('uint8')
	return img_rec_green

def image_grab_green_channel(img):
	g = img.copy()
	# set blue and red channels to 0
	g[:, :, 0] = 0
	g[:, :, 2] = 0
	g = cv2.cvtColor(g, cv2.COLOR_BGR2GRAY)
	return g

def image_grab_green_hsv(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, (36, 25, 25), (70, 255,255))

	imask = mask>0
	green = np.zeros_like(img, np.uint8)
	green[imask] = img[imask]
	green = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
	return green




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
	height, width = image.shape[:2]
	centre = (int(width/2), int(height/2))
	output = image.copy()
	cv2.circle(output, centre, 3, (255, 255, 255), -1)
	#image = image_grab_green(image)
	#image = image_convert_to_perc_green(image)
	image = image_grab_green_hsv(image)
	#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(image, (27, 27), 0)
	thresh = cv2.threshold(blurred, 75, 255, cv2.THRESH_BINARY)[1]
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	# loop over the contours
	try:
		#for c in cnts:
			# compute the center of the contour
		c = cnts[0]
		M = cv2.moments(c)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
	 
		# draw the contour and center of the shape on the image
		cv2.drawContours(output, [c], -1, (0, 255, 0), 2)
		cv2.circle(output, (cX, cY), 3, (255, 255, 255), -1)
		cv2.putText(output, "center", (cX - 20, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
		cv2.line(output, centre, (cX, cY), (255,0,0), 2)
		dX = cX - centre[0] 
		dY = centre[1] - cY
		cv2.putText(output, ("(" + str(dX) + ", " + str(dY) + " )"), (centre[0] - 20, centre[1] - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
	except:
		pass
	 
	# show the frame
	cv2.imshow("thresh", thresh)
	cv2.imshow("preview", output)
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