import numpy as np
#import cv2
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import time
import datetime
import math

import RPi.GPIO as GPIO

#imports for dk
import sys 

from dronekit import connect, VehicleMode
from pymavlink import mavutil
#from transforms3d import euler

#setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(18,GPIO.OUT)
GPIO.output(18,GPIO.LOW)

#setup dk params
takeOffAlt = 0.5
takeOffGain = 1
homeAlt = None #home altitude is set by take-off command
hoverThrust = 0.4

if hoverThrust > 0.5:
	hoverThrust = 0.5

armed = False
stage = 0
takenOff = False
landed = False

############################################################################################################

def userOW():
	rcThrottle = vehicle.channels['3']
	if rcThrottle > 1000:
		return (rcThrottle - 990)/2000
	else:
		return 999

############################################################################################################

def send_attitude_target(r,p,h,thrust,duration):
	
	userOWVal = userOW()
	if not userOWVal == 999:
#		print "overwriting!"
#		r = 0
#		p = 0
#		h = 0
#		thrust = userOWVal
#		print "new thrust is ", thrust
#		duration = 1
		vehicle.mode = VehicleMode("STABILIZE")

		while not vehicle.mode.name=='STABILIZE':
			print "CHANGING MODES..."
			time.sleep(1)


		sys.exit(0)

	q = euler.euler2quat(r,p,h)
	msg = vehicle.message_factory.set_attitude_target_encode(
		0,
		0,0,
		0b00000000,
		q,
		0,0,0,thrust)
	
	for x in range(0,duration):
		vehicle.send_mavlink(msg)
		vehicle.flush()
		time.sleep(1)
		#print q

############################################################################################################

def takeoff():
	
	global homeAlt	

	if homeAlt is None:
		homeAlt = vehicle.location.global_relative_frame.alt
		print "Altitude zeroed at: ", homeAlt
	currentAlt = vehicle.location.global_relative_frame.alt
	thrust = (currentAlt - (homeAlt + takeOffAlt)) * takeOffGain

	if thrust > 1.0:
		thrust = 1.0
	elif thrust < -1.0:
		thrust = -1.0
	thrust = hoverThrust - (hoverThrust * thrust)	

	print "takeoff thrust ", thrust, " at alt ", currentAlt 

	send_attitude_target(0,0,0,thrust,1)

	print "takeoff check ", abs(currentAlt - (homeAlt + takeOffAlt))

	if abs(currentAlt - (homeAlt + takeOffAlt)) < 0.1:
		print "reached"
		return True
	else:
		return False

############################################################################################################

def land():

	if vehicle.location.global_relative_frame.alt - homeAlt > 0:
		thrust = (vehicle.location.global_relative_frame.alt - homeAlt) * takeOffGain
		if thrust > 1:
			thrust = 1
		elif thrust < -1:
			thrust = -1
		thrust = hoverThrust - (hoverThrust * thrust)
		print "landing thrust ", thrust

		send_attitude_target(0,0,0,thrust,1)
		return False
	else:
		send_attitude_target(0,0,0,0,5)
		return True

############################################################################################################

#setup capturing params
imgWidth = 640 #320
imgHeight = 480 #240

#setup search params
searchGain = 1.5
searchX = 0
searchY = 0
searchSizeY = imgHeight
searchSizeX = imgWidth

#setup video file output
#fourcc = cv2.VideoWriter_fourcc(*'X264')
#out = cv2.VideoWriter('output_'+str(datetime.datetime.now())+'.avi',fourcc,20.0,(imgWidth,imgHeight))

#setup pi camera
#camera = PiCamera()
#camera.resolution = (imgWidth,imgHeight)
#camera.shutter_speed = 20000
#camera.iso = 100
#rawCapture = PiRGBArray(camera, size=(imgWidth,imgHeight))

#point to targets
#mTarget = cv2.imread('female_target.png',0)
#fTarget = None

# Initiate SIFT detector
#sift = cv2.xfeatures2d.SIFT_create(nfeatures = 30)
#kp2, des2 = sift.detectAndCompute(mTarget,None)

#meanshift params
#useMeanshift = False
#termCrit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

#print len(kp2)

#setup dk 

if not armed:
	vehicle = connect('udp:127.0.0.1:14650', wait_ready=True)
	vehicle.mode = VehicleMode("GUIDED_NOGPS")
	vehicle.armed = True

	while not vehicle.mode.name=='GUIDED_NOGPS' and not vehicle.armed:
		print("Getting ready...")
		time.sleep(1)
	
	armed = True
	print("Armed")


while True:
	print(vehicle.armed)

vehicle.exit()

#for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#	frame = image.array
#	rawCapture.truncate(0)
#
#	if useMeanshift == False:
#
#		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#
#		roi = np.zeros(frame.shape[:2], dtype = np.uint8)
#		roi[searchY:searchY + searchSizeY, searchX: searchX + searchSizeX] = gray[searchY:searchY + searchSizeY, searchX: searchX + searchSizeX]
#
#		# find the keypoints and descriptors with SIFT
#		kp1, des1 = sift.detectAndCompute(gray,roi)
#
#		# FLANN parameters
#		FLANN_INDEX_KDTREE = 0
#		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 1)
#		search_params = dict(checks=50)   # or pass empty dictionary
#
#		flann = cv2.FlannBasedMatcher(index_params,search_params)
#
#		matches = flann.knnMatch(des1,des2,k=2)
#
#		# Need to draw only good matches, so create a mask
#		matchesMask = [[0,0] for i in xrange(len(matches))]
#
#		goodMatches = 0
#		height, width = frame.shape[:2]
#
#		minX = width
#		minY = height
#		maxX = 0
#		maxY = 0
#
#		# ratio test as per Lowe's paper
#		for i,(m,n) in enumerate(matches):
#		    if m.distance < 0.45*n.distance:
#		        matchesMask[i]=[1,0]
#		        goodMatches += 1
#		        if kp1[i].pt[0] < minX:
#		        	minX = kp1[i].pt[0]
#		        if kp1[i].pt[0] > maxX:
#		        	maxX = kp1[i].pt[0]
#		        if kp1[i].pt[1] < minY:
#		        	minY = kp1[i].pt[1]
#		        if kp1[i].pt[1] > maxY:
#		        	maxY = kp1[i].pt[1]
#		
#		percentageMatches = goodMatches*100/len(des2)
#		print "%d percentage matches" % percentageMatches
#
#		if percentageMatches > 0:		
#			midX = (minX + maxX) / 2
#			midY = (minY + maxY) / 2
#			searchSizeX = (maxX - minX) * searchGain
#			searchSizeY = (maxY - minY) * searchGain
#			searchX = midX - searchSizeX / 2
#			if searchX < 0:
#				searchX = 0
#			if searchX + searchSizeX > width:
#				searchSizeX = width - searchX
#			searchY = midY - searchSizeY / 2
#			if searchY < 0:
#				searchY = 0
#			if searchY + searchSizeY > height:
#				searchSizeY = height - searchY
#
#			if searchSizeY * searchX > 300:
#				#calculate meanshift params
#				meanshiftROI = frame[searchY:searchY+searchSizeY, searchX:searchX+searchSizeX]
#				meanshiftHSVROI = cv2.cvtColor(meanshiftROI, cv2.COLOR_BGR2HSV)
#				meanshiftMask = cv2.inRange(meanshiftHSVROI,np.array((0.,60.,32.)),np.array((180.,255.,255.)))
#				meanshiftROIHist = cv2.calcHist([meanshiftHSVROI],[0],meanshiftMask,[180],[0,180])
#				cv2.normalize(meanshiftROIHist,meanshiftROIHist,0,255,cv2.NORM_MINMAX)
#				trackWindow = (int(searchX), int(searchY), int(searchSizeX), int(searchSizeY))
#				useMeanshift = True
#
#				#activate IR LED
#				GPIO.output(18,GPIO.HIGH)
#				time.sleep (1)
#
#		else:
#			searchX = 0
#			searchY = 0
#			searchSizeY = imgHeight
#			searchSizeX = imgWidth
#	
#	else:
#		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#		dst =  cv2.calcBackProject([hsv],[0],meanshiftROIHist,[0,180],1)
#
#		ret, trackWindow = cv2.meanShift(dst,trackWindow,termCrit)
#
#		searchX, searchY, searchSizeX, searchSizeY = trackWindow
#	
#	#error terms
#	errDist = math.sqrt((searchX + searchSizeX/2 - imgWidth/2)**2 + (searchY + searchSizeY/2 - imgHeight/2)**2)
#	errAng = math.atan2(searchY + searchSizeY/2 - imgHeight/2, searchX + searchSizeX/2 - imgWidth/2)
#
#	if useMeanshift:
#
#		if not vehicle.armed:
#			print "disarmed!"
#			vehicle.armed = True
#			while not vehicle.armed:
#				print "waiting for arm..."
#				time.sleep(1)
#			print "arm ok"
#
#		pitchError = (searchX + searchSizeX/2 - imgWidth/2)/1000.0
#		rollError = (searchY + searchSizeY/2 - imgHeight/2)/-1000.0
#		print "pitch error is ", pitchError
#		print "roll error is ", rollError
#
#		#send_attitude_target(rollError,pitchError,0,0.52,1)
#
#	#annotate frame
#	cv2.rectangle(frame, (int(searchX),int(searchY)),(int(searchX + searchSizeX),int(searchY + searchSizeY)),(0,0,255),3)
#	cv2.line(frame, (int(imgWidth/2),int(imgHeight/2)), (int(searchX + searchSizeX/2) , int(searchY + searchSizeY/2)), (0,255,0),2)
#	cv2.putText(frame, str(rollError),(5,imgHeight-5),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),2,cv2.LINE_AA)
#	cv2.putText(frame, str(pitchError),(5,imgHeight-20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),2,cv2.LINE_AA)
#
#	cv2.imshow('matches',frame)
#	out.write(frame)
#	if cv2.waitKey(1) & 0xFF == ord('q'):
#		vehicle.mode = VehicleMode("STABILIZE")
#		while not vehicle.mode.name=='STABILIZE':
#			print "CHANGING MODES..."
#			time.sleep(1)
#		break
#		sys.exit(0)

sys.exit(0)
