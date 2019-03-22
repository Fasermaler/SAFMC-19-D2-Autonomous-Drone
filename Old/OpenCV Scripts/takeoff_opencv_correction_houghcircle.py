
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
	#green = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
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
	output = image.copy()
	#image = image_grab_green(image)
	#image = image_convert_to_perc_green(image)
	image = image_grab_green_hsv(image)
	#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blur = cv2.medianBlur(image,15 )
	image = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)


	# loop over the contours

	circles = cv2.HoughCircles(image,cv2.HOUGH_GRADIENT,1,80,
                            param1=50,param2=25,minRadius=0,maxRadius=0)
	print(circles)
	 
	# ensure at least some circles were found
	if circles is not None:
		# convert the (x, y) coordinates and radius of the circles to integers
		circles = np.round(circles[0, :]).astype("int")
	 
		# loop over the (x, y) coordinates and radius of the circles
		for (x, y, r) in circles:
			# draw the circle in the output image, then draw a rectangle
			# corresponding to the center of the circle
			cv2.circle(output, (x, y), r, (0, 255, 0), 4)
			cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

	 
	# show the frame
	cv2.imshow("preview", output)
	cv2.imshow("blur", blur)
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