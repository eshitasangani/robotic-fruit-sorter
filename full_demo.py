#!/usr/bin/env python3

# Python code for Multiple Color Detection + Integrated with Inverse Kinematics
import os
# os.system("sudo pigpiod")

import numpy as np 
import cv2 
import imutils
import ik
import time
import RPi.GPIO as GPIO
import sys
import pygame


webcam = cv2.VideoCapture(-1) 

MM_TO_PIXEL = 260 / 640

blue_flag = False
rasp_flag = False
lem_lim_flag = False
org_flag = False

detected_fruit = ""

# use this to quit the program
code_run = True
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(17, GPIO.IN, pull_up_down = GPIO.PUD_UP)

# external button to quit program and return to console
def callback_21(channel):
    global code_run 
    code_run = False
    sys.quit(0)

# button 17 on piTFT to shutdown pi
def callback_17(channel):
    global code_run 
    code_run = False
    os.system("sudo shutdown -h now")

GPIO.add_event_detect(21, GPIO.FALLING, callback=callback_21, bouncetime=300)
GPIO.add_event_detect(17, GPIO.FALLING, callback=callback_17, bouncetime=300)

# -------------------------------------- FRUIT MASKS ---------------------------------------------
_, imageFrame = webcam.read() 
hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 

## RASPBERRIES ##

raspberries_lower1 = np.array([155, 171, 99], np.uint8) 
raspberries_upper1 = np.array([175, 216, 227], np.uint8) 
raspb_mask1 = cv2.inRange(hsvFrame, raspberries_lower1, raspberries_upper1)

raspberries_lower2 = np.array([177, 195, 95], np.uint8) 
raspberries_upper2 = np.array([179, 255, 255], np.uint8) 
raspb_mask2 = cv2.inRange(hsvFrame, raspberries_lower2, raspberries_upper2)

raspb_mask1 = cv2.morphologyEx(raspb_mask1, cv2.MORPH_OPEN, kernel, iterations=2)  # erosion followed by dilation
raspb_mask1 = cv2.dilate(raspb_mask1, kernel, iterations=3)
res_rasp1 = cv2.bitwise_and(imageFrame, imageFrame, mask=raspb_mask1) 

raspb_mask2 = cv2.morphologyEx(raspb_mask2, cv2.MORPH_OPEN, kernel, iterations=2) 
raspb_mask2 = cv2.dilate(raspb_mask2, kernel, iterations=3)
res_rasp2 = cv2.bitwise_and(imageFrame, imageFrame, mask=raspb_mask2)

## BLUEBERRIES ##

blueberries_lower1 = np.array([90,50,110], np.uint8)
blueberries_upper1 = np.array([106,170,255], np.uint8)
bluber_mask1 = cv2.inRange(hsvFrame, blueberries_lower1, blueberries_upper1)

blueberries_lower2 = np.array([90,185,60], np.uint8)
blueberries_upper2 = np.array([110,255,195], np.uint8)
bluber_mask2 = cv2.inRange(hsvFrame, blueberries_lower2, blueberries_upper2)

blueberries_lower3 = np.array([110,160,40], np.uint8)
blueberries_upper3 = np.array([119,255,175], np.uint8)
bluber_mask3 = cv2.inRange(hsvFrame, blueberries_lower3, blueberries_upper3)

bluber_mask1 = cv2.morphologyEx(bluber_mask1, cv2.MORPH_OPEN, kernel, iterations=2)
bluber_mask1 = cv2.dilate(bluber_mask1,kernel,iterations=3)
res_bluber1 = cv2.bitwise_and(imageFrame, imageFrame, mask = bluber_mask1)

bluber_mask2 = cv2.morphologyEx(bluber_mask2, cv2.MORPH_OPEN, kernel, iterations=2) 
bluber_mask2 = cv2.dilate(bluber_mask2,kernel,iterations=3) 
res_bluber2 = cv2.bitwise_and(imageFrame, imageFrame, mask = bluber_mask2)

bluber_mask3 = cv2.morphologyEx(bluber_mask3, cv2.MORPH_OPEN, kernel, iterations=2) 
bluber_mask3 = cv2.dilate(bluber_mask3,kernel,iterations=3) 
res_bluber3 = cv2.bitwise_and(imageFrame, imageFrame, mask = bluber_mask3)

def detect_fruit(position):
    global blue_flag
    global rasp_flag
    global lem_lim_flag
    global org_flag

    global hsvFrame
    global raspberries_lower1
    global raspberries_upper1
    global blueberries_lower2
    global blueberries_upper2
    global imageFrame
    print("detecting fruit!")

    # --------------------------------------BLUEBERRY COLOR DETECTION---------------------------------------------
    _, imageFrame = webcam.read() 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 

    bluber_mask2 = cv2.inRange(hsvFrame, blueberries_lower2, blueberries_upper2)
    
    bluber_mask2 = cv2.morphologyEx(bluber_mask2, cv2.MORPH_OPEN, kernel, iterations=2) 
    bluber_mask2 = cv2.dilate(bluber_mask2,kernel,iterations=3) 
    res_bluber2 = cv2.bitwise_and(imageFrame, imageFrame, mask = bluber_mask2)

    ## contour to track blue 1 color
    contours_bluber, hierarchy_bluber = cv2.findContours(bluber_mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    for pic, contour in enumerate(contours_bluber): 
        area = cv2.contourArea(contour) 
        if(area > 300):
            print("found fruit blue") 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h
            if 0.25 < ratio < 2:
                print ("in ratio blue")
                x2 = x + int(w/2)
                y2 = y + int(h/2)
                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)
                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    print ("in abs blue")
                    blue_flag = True
                    return

    # # --------------------------------------RASPBERRY COLOR DETECTION---------------------------------------------
    _, imageFrame = webcam.read() 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 
    raspb_mask1 = cv2.inRange(hsvFrame, raspberries_lower1, raspberries_upper1)
    raspb_mask1 = cv2.morphologyEx(raspb_mask1, cv2.MORPH_OPEN, kernel, iterations=2)  # erosion followed by dilation
    raspb_mask1 = cv2.dilate(raspb_mask1, kernel, iterations=3)
    res_rasp1 = cv2.bitwise_and(imageFrame, imageFrame, mask=raspb_mask1) 

    ## contour to track red 1 color
    contours_rasp, hierarchy_rasp = cv2.findContours(raspb_mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    for pic, contour in enumerate(contours_rasp): 
        area = cv2.contourArea(contour) 
        if(area > 300):
            print("found fruit rasp") 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:
                print ("in ratio rasp")
                x2 = x + int(w/2)
                y2 = y + int(h/2)
                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)
                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    print ("in abs rasp")
                    rasp_flag = True
                    return


    # # --------------------------------------LEMON COLOR DETECTION---------------------------------------------
    _, imageFrame = webcam.read() 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 

    lemons_lower1 = np.array([35,35,40], np.uint8)
    lemons_upper1 = np.array([65,195,165], np.uint8)
    lem_mask1 = cv2.inRange(hsvFrame, lemons_lower1, lemons_upper1)

    lemons_lower2 = np.array([65,80,0], np.uint8) 
    lemons_upper2 = np.array([90,255,101], np.uint8) 
    lem_mask2 = cv2.inRange(hsvFrame, lemons_lower2, lemons_upper2) 

    lem_mask1 = cv2.morphologyEx(lem_mask1, cv2.MORPH_OPEN, kernel, iterations=2) 
    lem_mask1 = cv2.dilate(lem_mask1,kernel,iterations=3) 
    res_lem1 = cv2.bitwise_and(imageFrame, imageFrame, mask = lem_mask1) 
    
    lem_mask2 = cv2.morphologyEx(lem_mask2, cv2.MORPH_OPEN, kernel, iterations=2) 
    lem_mask2 = cv2.dilate(lem_mask2,kernel,iterations=3) 
    res_lem2 = cv2.bitwise_and(imageFrame, imageFrame, mask = lem_mask2)

    contours_lem, hierarchy = cv2.findContours(lem_mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours_lem): 
        area = cv2.contourArea(contour) 
        if(area > 300):
            print("found fruit lem") 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:
                print ("in ratio lem")
                x2 = x + int(w/2)
                y2 = y + int(h/2)
                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)
                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    print ("in abs lem")
                    lem_lim_flag = True
                    return

    # # --------------------------------------LIME COLOR DETECTION---------------------------------------------
    _, imageFrame = webcam.read() 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 

    limes_lower = np.array([20,85,100], np.uint8) 
    limes_upper = np.array([27,255,255], np.uint8) 
    lim_mask = cv2.inRange(hsvFrame, limes_lower, limes_upper)

    lim_mask = cv2.morphologyEx(lim_mask, cv2.MORPH_OPEN, kernel, iterations=2) 
    lim_mask = cv2.dilate(lim_mask,kernel,iterations=3) 
    res_lim = cv2.bitwise_and(imageFrame, imageFrame, mask = lim_mask)

    contours_lim, hierarchy = cv2.findContours(lim_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours_lim): 
        area = cv2.contourArea(contour) 
        if(area > 300):
            print("found fruit lim") 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:
                print ("in ratio lim")
                x2 = x + int(w/2)
                y2 = y + int(h/2)
                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)
                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    print ("in abs lim")
                    lem_lim_flag = True
                    return

    # # --------------------------------------ORANGE COLOR DETECTION---------------------------------------------
    _, imageFrame = webcam.read() 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 

    oranges_lower = np.array([0, 105, 160], np.uint8) 
    oranges_upper = np.array([5, 220, 255], np.uint8) 
    orng_mask = cv2.inRange(hsvFrame, oranges_lower, oranges_upper)

    orng_mask = cv2.morphologyEx(orng_mask, cv2.MORPH_OPEN, kernel, iterations=2) 
    orng_mask = cv2.dilate(orng_mask,kernel,iterations=3) 
    res_orng = cv2.bitwise_and(imageFrame, imageFrame, mask = orng_mask)

    ## contour to track orange color

    contours_org, hierarchy = cv2.findContours(orng_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours_org): 
        area = cv2.contourArea(contour) 
        if(area > 300):
            print("found fruit org") 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:
                print ("in ratio org")
                x2 = x + int(w/2)
                y2 = y + int(h/2)
                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)
                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    print ("in abs org")
                    org_flag = True
                    return

# Start a while loop 
while(code_run): 

    # Reading the video from the 
    _, imageFrame = webcam.read() 

    
    x = 168 
    y = 100
    position = (x, y)

    
    #functions for each fruit box
    detect_fruit(position) 
    if (rasp_flag):
        print('rasp')

        ik.raspberries()
        for i in range(5):
            _, imageFrame = webcam.read() 
            time.sleep(1)
        # print("image updated")
        rasp_flag = False
    elif (blue_flag):
        print('blue')

        ik.blueberries()
        for i in range(5):
            _, imageFrame = webcam.read() 
            time.sleep(1)
        blue_flag = False
    elif (lem_lim_flag):
        print('lem/lim')

        ik.lemons()
        for i in range(5):
            _, imageFrame = webcam.read() 
            time.sleep(1)
        lem_lim_flag = False
    elif (org_flag):
        print('orange')
        ik.oranges()
        for i in range(5):
            _, imageFrame = webcam.read() 
            time.sleep(1)
        org_flag = False
    
cv2.destroyAllWindows() 
GPIO.cleanup()
