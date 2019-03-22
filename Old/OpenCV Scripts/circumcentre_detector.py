

from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time
import random 
from fractions import Fraction
from PIL import Image
from math import cos
from sympy import Point, Polygon, pi


#cap = cv2.VideoCapture(0)
camera = PiCamera()
camera.resolution = (426, 240)
camera.framerate = 24
camera.exposure_mode = 'auto'
camera.exposure_compensation = -3
camera.drc_strength = 'off'
camera.still_stats = False

camera.awb_mode = 'off'
camera.awb_gains = (Fraction(167, 103), Fraction(27,16))

rawCapture = PiRGBArray(camera, size=(426, 240))


# allow the camera to warmup
time.sleep(0.1)



out = cv2.VideoWriter(str(time.time()) + ".avi",cv2.VideoWriter_fourcc('M','J','P','G'), 10, (426, 240))

  
for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    for i in range(5): # Clears the 5 frame buffer 
        frame = img.array
    height, width = frame.shape[:2]
    centre = (int(width/2), int(height/2))

    b_channel = np.array(frame[:,:,0]).astype('float')
    g_channel = np.array(frame[:,:,1]).astype('float')
    r_channel = np.array(frame[:,:,2]).astype('float')

    bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
    img_rec_red2 = np.subtract(r_channel,((b_channel + g_channel)/ 2))
    #img_rec_red2 = np.divide(r_channel, 255)
    img_rec_red2 = np.divide(img_rec_red2,255) 
    #img_rec_red2 = np.square(img_rec_red2)
    img_rec_red2[img_rec_red2 < 0.3] = 0
    dX, dY = 0,0

    trials = 1
    try:
        # Get the array of indices of detected pixels
        thresholded_array = np.argwhere(img_rec_red2 >= 0.3)
        thresholded_list = thresholded_array.tolist()
        print(thresholded_list)

        
        if len(thresholded_list) > trials*3:
        # sets the number of trials before averaging to get the centre
            
            total_centres_X = 0
            total_centres_Y = 0
            hoop_centre = (0,0)
            arr_len_3rd = int(len(thresholded_list) / 3)
            for i in range(trials):
                r1 = random.randrange(0, int(arr_len_3rd/2))

                #r2 = random.randrange(0, arr_len_3rd)
                # rerolls if the same number was rolled
                #while r2 == r1:
                r2 = random.randrange(arr_len_3rd, 2*arr_len_3rd)
                r3 = random.randrange(int(2.5*arr_len_3rd), len(thresholded_list))
                #while r3 == r1 or r3 == r2:
                #r3 = random.randrange(0, len(thresholded_list))
                print(thresholded_list[r1],thresholded_list[r2],thresholded_list[r3])
                current_centre = Polygon(thresholded_list[r1],thresholded_list[r2],thresholded_list[r3]).circumcenter
                print(current_centre)
                total_centres_X += int(current_centre.y)
                total_centres_Y += int(current_centre.x)
                cv2.circle(frame, (thresholded_list[r1][1], thresholded_list[r1][0]), 5, (0, 0, 255), -1)
                cv2.circle(frame, (thresholded_list[r2][1], thresholded_list[r2][0]), 5, (0, 0, 255), -1)
                cv2.circle(frame, (thresholded_list[r3][1], thresholded_list[r3][0]), 5, (0, 0, 255), -1)
                
            cX = int(total_centres_X / trials)
            cY = int(total_centres_Y / trials)

            print(cX,cY)
    except:
        print("no hoop detected")
     
    # put text and highlight the center
    try:
        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
        cv2.line(frame, centre, (cX, cY), (255,0,0), 2)
   
    #cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
        dX = cX - centre[0] 
        dY = centre[1] - cY
        cv2.putText(frame, ("(" + str(dX) + ", " + str(dY) + " )"), (centre[0] - 20, centre[1] - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        print('Velocities: ' + str(dX) + "," + str(dY))
    except:
        print("No centre detected")


    
    cv2.imshow('original', frame)
    #cv2.imshow('rec_red',img_rec_red)
    cv2.imshow('detected circles',img_rec_red2)
    out.write(frame)
    k = cv2.waitKey(1)
    rawCapture.truncate(0)
    if k == 0xFF & ord("q"):
        break
#cv2.destroyAllWindows()
#cap.release()

out.release()