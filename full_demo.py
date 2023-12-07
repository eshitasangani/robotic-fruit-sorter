#!/usr/bin/env python3

# Python code for Multiple Color Detection + Integrated with Inverse Kinematics

import numpy as np 
import cv2 
import imutils
import ik
import time

webcam = cv2.VideoCapture(-1) 

MM_TO_PIXEL = 260 / 640

blue_flag = False
rasp_flag = False
lem_flag = False
lim_flag = False
org_flag = False

# --------------------------------------RASPBERRY COLOR DETECTION---------------------------------------------

def detect_raspberry(imageFrame, position):
    global rasp_flag
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 

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

    ## contour to track red 1 color

    contours, hierarchy = cv2.findContours(raspb_mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    counter = 0
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:

                M = cv2.moments(contour)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position

                x2 = x + int(w/2)
                y2 = y + int(h/2)

                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)

                cv2.rectangle(imageFrame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                text = "x:" + str(robot_x) + ", y:" + str(robot_y)
                cv2.putText(imageFrame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
                cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle

                # cv2.putText(imageFrame, "Raspberry", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))

                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    color = imageFrame[cy, cx]
                    # print("Detected Raspberry Color at Position {}: {}".format(position, color))
                    cv2.putText(imageFrame, "Raspberry", (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    ## put in raspberry box here
                    rasp_flag = True
                    return

    ## contour to track red 2 color
    contours, hierarchy = cv2.findContours(raspb_mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    counter = 0
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:

                M = cv2.moments(contour)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position

                x2 = x + int(w/2)
                y2 = y + int(h/2)

                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)

                cv2.rectangle(imageFrame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                text = "x:" + str(robot_x) + ", y:" + str(robot_y)
                cv2.putText(imageFrame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    color = imageFrame[cy, cx]
                    # print("Detected Raspberry Color at Position {}: {}"x.format(position, color))
                    cv2.putText(imageFrame, "Raspberry", (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    ## put in raspberry box here
                    rasp_flag = True
                    return

                cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
                cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle


# --------------------------------------BLUEBERRY COLOR DETECTION---------------------------------------------

def detect_blueberry(imageFrame, position):
    global blue_flag
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

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

    ## contour to track blue 1 color

    contours, hierarchy = cv2.findContours(bluber_mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    counter = 0
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:

                M = cv2.moments(contour)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position

                x2 = x + int(w/2)
                y2 = y + int(h/2)

                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)

                cv2.rectangle(imageFrame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                text = "x:" + str(robot_x) + ", y:" + str(robot_y)
                cv2.putText(imageFrame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
                cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle

                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    color = imageFrame[cy, cx]
                    # print("Detected Blueberry at Position {}: {}".format(position, color))
                    cv2.putText(imageFrame, "Blueberry", (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    ## put in blueberry box here
                    blue_flag = True
                    return

    ## contour to track blue 2 color
    contours, hierarchy = cv2.findContours(bluber_mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    counter = 0
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:

                M = cv2.moments(contour)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position

                x2 = x + int(w/2)
                y2 = y + int(h/2)

                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)

                cv2.rectangle(imageFrame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                text = "x:" + str(robot_x) + ", y:" + str(robot_y)
                cv2.putText(imageFrame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    color = imageFrame[cy, cx]
                    # print("Detected Blueberry at Position {}: {}"x.format(position, color))
                    cv2.putText(imageFrame, "Blueberry", (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    ## put in blueberry box here
                    blue_flag = True
                    return

                cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
                cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle

    ## contour to track blue 3 color
    contours, hierarchy = cv2.findContours(bluber_mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    counter = 0
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:

                M = cv2.moments(contour)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position

                x2 = x + int(w/2)
                y2 = y + int(h/2)

                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)

                cv2.rectangle(imageFrame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                text = "x:" + str(robot_x) + ", y:" + str(robot_y)
                cv2.putText(imageFrame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    color = imageFrame[cy, cx]
                    # print("Detected Blueberry at Position {}: {}"x.format(position, color))
                    cv2.putText(imageFrame, "Blueberry", (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    ## put in blueberry box here
                    blue_flag = True
                    return

                cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
                cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle


# --------------------------------------LEMON COLOR DETECTION---------------------------------------------

# lines are yellow, lemons are green 

def detect_lemons(imageFrame, position):
    global lem_flag
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

    ## contour to track lemon 1 color

    contours, hierarchy = cv2.findContours(lem_mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    counter = 0
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:

                M = cv2.moments(contour)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position

                x2 = x + int(w/2)
                y2 = y + int(h/2)

                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)

                cv2.rectangle(imageFrame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                text = "x:" + str(robot_x) + ", y:" + str(robot_y)
                cv2.putText(imageFrame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
                cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle

                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    color = imageFrame[cy, cx]
                    # print("Detected Lemon Color at Position {}: {}".format(position, color))
                    cv2.putText(imageFrame, "Lemon", (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    ## put in lemon + limes box here
                    lem_flag = True
                    return

    ## contour to track lemon 2 color
    contours, hierarchy = cv2.findContours(lem_mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    counter = 0
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:

                M = cv2.moments(contour)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position

                x2 = x + int(w/2)
                y2 = y + int(h/2)

                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)

                cv2.rectangle(imageFrame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                text = "x:" + str(robot_x) + ", y:" + str(robot_y)
                cv2.putText(imageFrame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    color = imageFrame[cy, cx]
                    # print("Detected Lemon Color at Position {}: {}"x.format(position, color))
                    cv2.putText(imageFrame, "Lemon", (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    ## put in lemon + limes box here // set flag? 
                    lim_flag = True
                    return

                cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
                cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle

# --------------------------------------LIMES COLOR DETECTION---------------------------------------------

def detect_limes(imageFrame, position):
    global lim_flag
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 

    limes_lower = np.array([20,85,100], np.uint8) 
    limes_upper = np.array([27,255,255], np.uint8) 
    lim_mask = cv2.inRange(hsvFrame, limes_lower, limes_upper)

    lim_mask = cv2.morphologyEx(lim_mask, cv2.MORPH_OPEN, kernel, iterations=2) 
    lim_mask = cv2.dilate(lim_mask,kernel,iterations=3) 
    res_lim = cv2.bitwise_and(imageFrame, imageFrame, mask = lim_mask)

    ## contour to track lime 1 color

    contours, hierarchy = cv2.findContours(lim_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    counter = 0
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:

                M = cv2.moments(contour)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position

                x2 = x + int(w/2)
                y2 = y + int(h/2)

                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)

                cv2.rectangle(imageFrame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                text = "x:" + str(robot_x) + ", y:" + str(robot_y)
                cv2.putText(imageFrame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
                cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle

                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    color = imageFrame[cy, cx]
                    # print("Detected Lime Color at Position {}: {}".format(position, color))
                    cv2.putText(imageFrame, "Lime", (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    ## put in lemon + limes box here 
                    lim_flag = True
                    return

# --------------------------------------ORANGE COLOR DETECTION---------------------------------------------

def detect_oranges(imageFrame, position):
    global org_flag
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 

    oranges_lower = np.array([0, 105, 160], np.uint8) 
    oranges_upper = np.array([5, 220, 255], np.uint8) 
    orng_mask = cv2.inRange(hsvFrame, oranges_lower, oranges_upper)

    orng_mask = cv2.morphologyEx(orng_mask, cv2.MORPH_OPEN, kernel, iterations=2) 
    orng_mask = cv2.dilate(orng_mask,kernel,iterations=3) 
    res_orng = cv2.bitwise_and(imageFrame, imageFrame, mask = orng_mask)

    ## contour to track orange color

    contours, hierarchy = cv2.findContours(orng_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    counter = 0
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        if(area > 300): 
            x, y, w, h = cv2.boundingRect(contour)
            ratio = float(w) / h

            if 0.25 < ratio < 2:

                M = cv2.moments(contour)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position

                x2 = x + int(w/2)
                y2 = y + int(h/2)

                robot_x = int(x2 * MM_TO_PIXEL)
                robot_y = int(y2 * MM_TO_PIXEL)

                cv2.rectangle(imageFrame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                text = "x:" + str(robot_x) + ", y:" + str(robot_y)
                cv2.putText(imageFrame, text, (x2 - 10, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
                cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle

                if abs(robot_x - position[0]) < 10 and abs(robot_y - position[1]) < 10:
                    color = imageFrame[cy, cx]
                    # print("Detected Orange Color at Position {}: {}".format(position, color))
                    cv2.putText(imageFrame, "Orange", (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                    ## put in oranges box here 
                    org_flag = True
                    return

## MOVEMENT FUNCITONS FROM IK.PY 




# ik.go_to_coor(10,100,50)  # center position 12/7

# time.sleep(0.1)

# ik.go_to_coor(10,205,10) # location to pick up the fruit 12/7

# ik.go_to_coor(20,-10,80) #raspberries 12/7

# ik.go_to_coor(50,60,80) # oranges 12/7 

# ik.go_to_coor(-30,-40,80) #lemons + limes 12/7

# ik.go_to_coor(-120,100,150) #blueberries 12/7


# Start a while loop 
while(1): 

    # Reading the video from the 
    _, imageFrame = webcam.read() 
    
    x = 168 
    y = 136
    position = (x, y)
    ## create a range of +- 15 in case it moves around

# for i in range(x - 15, x + 15): 

#     for j in range(y - 15, y + 15): 

        # position = (i, j) # adjusted function
    
        # functions for each fruit box
    
    detect_blueberry(imageFrame, position) 
    if blue_flag: 
        #blue movemnet funtion
        _, imageFrame = webcam.read() 
        ik.blueberries()
        time.sleep(3)
        _, imageFrame = webcam.read() 
        print('blue')
        blue_flag = False
        _, imageFrame = webcam.read() 
        time.sleep(3)

    detect_raspberry(imageFrame, position) 
    if rasp_flag: 
        #rasp movemnet funtion
        _, imageFrame = webcam.read() 
        ik.raspberries()
        time.sleep(3)
        _, imageFrame = webcam.read() 
        print('rasp')
        rasp_flag = False
        _, imageFrame = webcam.read() 
        time.sleep(3)
    
    detect_lemons(imageFrame, position) 
    if lem_flag: 
        #lemon movemnet funtion
        print('lemon')
        lem_flag = False
    
    detect_limes(imageFrame, position) 
    if lim_flag: 
        #limes movemnet funtion
        print('lim')
        lim_flag = False
    
    detect_oranges(imageFrame, position)
    if org_flag: 
        #orange movemnet funtion
        print('org')
        org_flag = False




    # detect_blueberry(imageFrame, (x, y))
    # detect_raspberry(imageFrame, (x, y))
    # detect_lemons(imageFrame, (x, y))
    # detect_limes(imageFrame, (x, y))
    # detect_oranges(imageFrame, (x, y))

    ## need to figure out how we want this quitting thing to work 
    cv2.imshow("Multiple Color Detection in Real-Time", imageFrame) 
    if cv2.waitKey(10) & 0xFF == ord('q'): 
        cap.release() 
        cv2.destroyAllWindows() 
        break
