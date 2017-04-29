# -*- coding: utf-8 -*-
"""
Created on Sat Mar 04 15:58:52 2017

@author: yifangz
"""

#import IoRTcarL as iortl

#import sys, tty, termios
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import time
import glob
import cv2
import numpy as np
import matplotlib.pyplot as plt


cv2plt = lambda image: image[::1, ::1, ::-1]

def getch():
  import sys, tty, termios
  old_settings = termios.tcgetattr(0)
  new_settings = old_settings[:]
  new_settings[3] &= ~termios.ICANON
  try:
    termios.tcsetattr(0, termios.TCSANOW, new_settings)
    ch = sys.stdin.read(1)
  finally:
    termios.tcsetattr(0, termios.TCSANOW, old_settings)
  return ch




def findColors(image):
    tgt_x = 0;
    tgt_y = 0;
    tgt_w = 0;
    tgt_h = 0;
    
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
#    lower_red = np.array([0, 120, 120])
#    upper_red = np.array([50,255,180])
    lower_blue = np.array([0,100,100])
    upper_blue = np.array([40,255, 255])
    #lower_blue = np.array([100,105,105])
    #upper_blue = np.array([135,255, 255])
    lower_green = np.array([35,55,55])
    upper_green = np.array([50,255,255])
    
    blue_threshed = cv2.inRange(image, lower_blue, upper_blue)
    imgray = blue_threshed
    ret,thresh = cv2.threshold(imgray,10,255,0)
    kernel = np.ones((5,5),np.uint8)
    closed_thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    im,contoursblue,hierarchy = cv2.findContours(closed_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    green_threshed = cv2.inRange(image, lower_green, upper_green)
    imgray = green_threshed
    ret,thresh = cv2.threshold(imgray,10,255,0)
    im,contoursgreen,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#    red_threshed = cv2.inRange(image, lower_red, upper_red)
#    imgray = red_threshed
#    ret,thresh = cv2.threshold(imgray,10,255,0)
#    im,contoursred,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    
    image = cv2.cvtColor(image,cv2.COLOR_HSV2BGR)
    
    for contour in contoursblue:
        area = cv2.contourArea(contour)
        #print(area)
        if (area>1000):
            #rect = cv2.minAreaRect(contour)
            #center,dim, angle = rect
            #x,y = center
            x,y,w,h = cv2.boundingRect(contour)
#            if w/h >0.8 and w/h < 1.5:
#                (tgt_x,tgt_y),radius = cv2.minEnclosingCircle(contour)
#                center = (int(tgt_x),int(tgt_y))
#                radius = int(radius)
#                cv2.circle(image,center,radius,(0,255,0),2)
#                tgt_w = w;
#                tgt_h = h;
#            else:
            cv2.rectangle(image,(x,y),(x+w,y+h), (0,0,255),2)
            x = x+w/2
            y = y+h/2
#    
    for contour in contoursgreen:
        area = cv2.contourArea(contour)
        print(area)
        if (area>80):
            #rect = cv2.minAreaRect(contour)
            #center,dim, angle = rect
            #mx,my = center
            mx,my,mw,mh = cv2.boundingRect(contour)
            cv2.rectangle(image,(mx,my),(mx+mw,my+mh), (0,0,255),2)
            mx = mx+mw/2
            my = my+mh/2
#
#    for contour in contoursred:
#        area = cv2.contourArea(contour)
#        if (area>500):
#            x,y,w,h = cv2.boundingRect(contour)
#            cv2.rectangle(image,(x,y),(x+w,y+h), (0,0,255),2)
    return image, x, y, mx, my


    
    
    
    
image = cv2.imread('cvimages/capture.jpg')
image,x, y, mx, my = findColors(image)
print(mx-x)
print(y-my)
angle = np.arctan2(mx-x,y-my)
angle = np.degrees(angle)
angle = angle-90
if (angle<0):
    angle = angle+360
    
plt.figure()
plt.imshow(cv2plt(image))
#cv2.imshow("Images",image)
#time.sleep(2)
#cv2.destroyAllWindows();    
    
    
#camera = PiCamera()
#camera.resolution = (320, 240)
#camera.framerate = 30
#camera.hflip = True
#camera.vflip = True
#rawCapture = PiRGBArray(camera, size=(320, 240))
#
#display_window = cv2.namedWindow("Images")
#
## face detection
##face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
#
#time.sleep(1)
#
#num = 1
#img_list = glob.glob('./*.jpg')
#num = len(img_list)
#
#for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#    image = frame.array
#    image,tx,ty, tw, th = findColors(image);
#    
#    # face detection
#    #gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);
#    #faces = face_cascade.detectMultiScale(gray, 1.1, 5);
#    #for (x, y, w, h) in faces:
#    #    cv2.rectangle(image, (x,y), (x+w, y+h), (255, 0, 0), 2)
#    cv2.imshow("Images", image)
#    # when you need to store image, please use following command
#    #cv2.imwrite("image.jpg", image)
#    key = cv2.waitKey(1)
#
#    rawCapture.truncate(0)
#
#    if key == 27:
#        camera.close()
#        cv2.destroyAllWindows()
#        break
#    elif key == ord('c'):
#        name = 'capture%02d.jpg' % num
#        num = num + 1;
#        cv2.imwrite(name, image)
#
#    if tx < 100:
#        iortl.ccw(0.1)
#        time.sleep(0.2)
#    elif tx> 220:
#        iortl.cw(0.1)
#        time.sleep(0.2)
#
#    if tx>=100 and tx<220:
#        if tw<60: 
#            iortl.forward(0.1)
#            time.sleep(0.2)
#        elif tw>160:
#            iortl.backward(0.1)
#            time.sleep(0.2)  
#        else:
#            iortl.stop()
#            break
#      
#print("position reached")
#cv2.destroyAllWindows();


    
