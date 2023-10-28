import io
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time
from datetime import datetime
import capture_object as capimg
import gripper
import robotic_arm
import RPi.GPIO as GPIO # Import the RPi Library for GPIO pin control

print('initialize robotic arm...')
robotic_arm.calibrate_joints()
time.sleep(2)
gripper.check_gripper()

GPIO.setmode(GPIO.BOARD)# We want to use the physical pin number scheme
LED1=18
bright=50

GPIO.setup(LED1,GPIO.OUT) # LED1 will be an output pin
pwm1=GPIO.PWM(LED1,1000)  # activate PWM on LED so we can dim, use 1000 Hz
pwm1.start(1)

#Load haar cascade file
tomatoCascade = cv2.CascadeClassifier('/home/pi/Documents/tomato_stage15.xml') 

camera = PiCamera()
camera.resolution = (320, 240)#set camera resolution#
camera.framerate = 30 #set frame rate
rawCapture = PiRGBArray(camera, size = (320, 240))


# allow the camera to adjust to lighting/white balance
time.sleep(2)

def main():
    
    avg = None
    
    for f in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        ripe_flag = False
        pwm1.ChangeDutyCycle(bright)
        # grab the raw array representation of the image
        frame = f.array
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        gaussian_gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        # inittialize avg if it hasn't been done
        if avg is None:
            avg = gaussian_gray.copy().astype("float")
            rawCapture.truncate(0)
            continue
        
        # accumulate the weighted average between the current frame and
        # previous frames, then compute the difference between the current
        # frame and running average
        cv2.accumulateWeighted(gaussian_gray, avg, 0.05)
        frameDelta = cv2.absdiff(gaussian_gray, cv2.convertScaleAbs(avg))
        
        # coonvert the difference into binary & dilate the result to fill in small holes
        motion_thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
        motion_thresh = cv2.dilate(motion_thresh, None, iterations=2)
        
        # find contours or continuous white blobs in the image
        motion_contours, motion_hierarchy = cv2.findContours(motion_thresh.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        # find the index of the largest contour
        if len(motion_contours) > 0:
            motiondetect_areas = [cv2.contourArea(c) for c in motion_contours]
            motion_max_index = np.argmax(motiondetect_areas)
            motion_cnt=motion_contours[motion_max_index]   
            motiondetect_area = cv2.contourArea(motion_cnt)
            print('Motion is detected....')
        else:
            
            faces = tomatoCascade.detectMultiScale(gray)
            print (str(faces)+"  \n locations(s)\n")
            print (str(len(faces))+" tomato(es) detected\n")

            #extract foreground color in HSV
            lower_forecolor = np.array([0,79,40])
            upper_forecolor = np.array([255,255,200])
            
            # Threshold the HSV image to get only right foreground colors
            foreground_mask = cv2.inRange(hsv, lower_forecolor, upper_forecolor)
            fore_kernel = np.ones((5,5), np.uint8)
            
            #remove foreground noise
            fore_erotion = cv2.erode(foreground_mask, fore_kernel, iterations =1)        
            fore_opening = cv2.morphologyEx(fore_erotion, cv2.MORPH_OPEN, fore_kernel)
            fore_closing = cv2.morphologyEx(fore_opening , cv2.MORPH_CLOSE, fore_kernel)
            
            # find contours or continuous white blobs in the foreground image
            fore_contours, fore_hierarchy = cv2.findContours(fore_closing.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            
            # Bitwise-AND mask and original image
            foreground_result = cv2.bitwise_and(frame,frame, mask = fore_closing)
            
            # find the index of the largest foreground contour
            if len(fore_contours) > 0:
                fore_areas = [cv2.contourArea(c) for c in fore_contours]
                fore_max_index = np.argmax(fore_areas)
                fore_cnt=fore_contours[fore_max_index]   
                fore_area = cv2.contourArea(fore_cnt)
                
                # draw a bounding box/rectangle around the largest contour
                x,y,w,h = cv2.boundingRect(fore_cnt)
                cv2.rectangle(foreground_result,(x,y),(x+w,y+h),(0,255,0),2)
                fore_area = cv2.contourArea(fore_cnt)
                # add text to the frame
                cv2.putText(foreground_result, " Surface area =" + str(fore_area) , (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
            # define range of ripeness color in HSV
            lower_color = np.array([0,41,30])
            upper_color = np.array([16,255,255])
            # Threshold the HSV image to get only right colors
            mask = cv2.inRange(hsv, lower_color, upper_color)
            kernel = np.ones((5,5), np.uint8)

            #Remove Image Noise
            erotion = cv2.erode(mask, kernel, iterations =1)
            diletion = cv2.dilate(erotion, kernel, iterations =1)
            opening = cv2.morphologyEx(erotion, cv2.MORPH_OPEN, kernel)
            closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

            # find contours or continuous white blobs in the image
            contours, hierarchy = cv2.findContours(closing.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            
            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(frame,frame, mask = closing)
            
            # find the index of the largest contour
            if len(contours) > 0:
                areas = [cv2.contourArea(c) for c in contours]
                max_index = np.argmax(areas)
                cnt=contours[max_index]   

                # draw a bounding box/rectangle around the largest contour
                x,y,w,h = cv2.boundingRect(cnt)
                cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,0),2)
                ripe_area = cv2.contourArea(cnt)
                
                # add text to the frame
                cv2.putText(res, " Ripe area =" + str(ripe_area) , (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                notripe_area = fore_area - ripe_area
                # print area to the terminal
                print("ripe area =" + str(ripe_area))
                print("surface area = " + str(fore_area))
                print("not ripe area =" +str(notripe_area))
                if notripe_area < ripe_area:
                    print('tomato is ripe...')
                    print('prepare for pickikng...\n')
                    ripe_flag = True
            elif len(fore_contours) != 0 and len(contours) == 0:
                print('tomato not ripe')
            
            
                
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
        

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        
        if ripe_flag == True:
            pass
            time.sleep(5)
            robotic_arm.harvest()
       
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        
        
    cv2.destroyAllWindows()
    GPIO.cleanup()
    
if __name__ =='__main__':
    main()
