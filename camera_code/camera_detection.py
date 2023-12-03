#!/usr/bin/env python3

# Python code for Multiple Color Detection 


import numpy as np 
import cv2 
import imutils


# Capturing video through webcam 
webcam = cv2.VideoCapture(-1) 

# fruits - blueberries, raspberries, lemons_limes, oranges

# Start a while loop 
while(1): 
	
	# Reading the video from the 
	# webcam in image frames 
	_, imageFrame = webcam.read() 

	# Convert the imageFrame in 
	# BGR(RGB color space) to 
	# HSV(hue-saturation-value) 
	# color space 
	hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

	# -------separating the different colors and shades into their own masks ------
	# -------- from lighter to darker shades-----------

	# Set range for raspberries (red color shades) and 
	# define mask -  
	raspberries_lower1 = np.array([155, 171, 99], np.uint8) 
	raspberries_upper1 = np.array([175, 216, 227], np.uint8) 
	raspb_mask1 = cv2.inRange(hsvFrame, raspberries_lower1, raspberries_upper1)

	# Set range for raspberries (red color shades) and 
	# define mask -  
	raspberries_lower2 = np.array([177, 195, 95], np.uint8) 
	raspberries_upper2 = np.array([179, 255, 255], np.uint8) 
	raspb_mask2 = cv2.inRange(hsvFrame, raspberries_lower2, raspberries_upper2) 

	# Set range for lemons (green color shades) and 
	# define mask - 
	lemons_lower1 = np.array([35,35,40], np.uint8) 
	lemons_upper1 = np.array([65,195,165], np.uint8) 
	lem_mask1 = cv2.inRange(hsvFrame, lemons_lower1, lemons_upper1)

	# Set range for lemons (green color shades) and 
	# define mask - 
	lemons_lower2 = np.array([65,80,0], np.uint8) 
	lemons_upper2 = np.array([90,255,101], np.uint8) 
	lem_mask2 = cv2.inRange(hsvFrame, lemons_lower2, lemons_upper2) 

	# Set range for limes (yellow color shades) and 
	# define mask - 
	limes_lower = np.array([20,85,100], np.uint8) 
	limes_upper = np.array([27,255,255], np.uint8) 
	lim_mask = cv2.inRange(hsvFrame, limes_lower, limes_upper)

	# Set range for blueberries (blue color shades) and 
	# define mask - 
	blueberries_lower1 = np.array([90,50,110], np.uint8) 
	blueberries_upper1 = np.array([106,170,255], np.uint8) 
	bluber_mask1 = cv2.inRange(hsvFrame, blueberries_lower1, blueberries_upper1) 

	# Set range for blueberries (blue color shades) and 
	# define mask - 
	blueberries_lower2 = np.array([90,185,60], np.uint8) 
	blueberries_upper2 = np.array([110,255,195], np.uint8) 
	bluber_mask2 = cv2.inRange(hsvFrame, blueberries_lower2, blueberries_upper2) 

	# Set range for blueberries (blue color shades) and 
	# define mask - 
	blueberries_lower3 = np.array([110,160,40], np.uint8) 
	blueberries_upper3 = np.array([119,255,175], np.uint8) 
	bluber_mask3 = cv2.inRange(hsvFrame, blueberries_lower3, blueberries_upper3) 

	# Set range for oranges (orange color shades) and 
	# define mask - 
	oranges_lower = np.array([0, 105, 160], np.uint8) 
	oranges_upper = np.array([5, 220, 255], np.uint8) 
	orng_mask = cv2.inRange(hsvFrame, oranges_lower, oranges_upper) 
	
	# Morphological Transform, Dilation 
	# for each color and bitwise_and operator 
	# between imageFrame and mask determines 
	# to detect only that particular color 
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) 
	
	# For red color shades (raspberries)
	raspb_mask1 = cv2.morphologyEx(raspb_mask1, cv2.MORPH_OPEN, kernel, iterations=2)  # erosion followed by dilation
	raspb_mask1 = cv2.dilate(raspb_mask1,kernel,iterations=3)
	res_rasp1 = cv2.bitwise_and(imageFrame, imageFrame, 
							mask = raspb_mask1) 

	raspb_mask2 = cv2.morphologyEx(raspb_mask2, cv2.MORPH_OPEN, kernel, iterations=2) 
	raspb_mask2 = cv2.dilate(raspb_mask2,kernel,iterations=3)
	res_rasp2 = cv2.bitwise_and(imageFrame, imageFrame, 
							mask = raspb_mask2)
	
	# For green color shades 
	lem_mask1 = cv2.morphologyEx(lem_mask1, cv2.MORPH_OPEN, kernel, iterations=2)
	lem_mask1 = cv2.dilate(lem_mask1,kernel,iterations=3)
	res_lem1 = cv2.bitwise_and(imageFrame, imageFrame, 
								mask = lem_mask1)

	lem_mask2 = cv2.morphologyEx(lem_mask2, cv2.MORPH_OPEN, kernel, iterations=2)
	lem_mask2 = cv2.dilate(lem_mask2,kernel,iterations=3)
	res_lem2 = cv2.bitwise_and(imageFrame, imageFrame, 
								mask = lem_mask2)

	# For yellow color shades 
	lim_mask = cv2.morphologyEx(lim_mask, cv2.MORPH_OPEN, kernel, iterations=2) 
	lim_mask = cv2.dilate(lim_mask,kernel,iterations=3)
	res_lim = cv2.bitwise_and(imageFrame, imageFrame, 
								mask = lim_mask) 
	
	# For blue color shades
	bluber_mask1 = cv2.morphologyEx(bluber_mask1, cv2.MORPH_OPEN, kernel, iterations=2)
	bluber_mask1 = cv2.dilate(bluber_mask1,kernel,iterations=3)
	res_bluber1 = cv2.bitwise_and(imageFrame, imageFrame, 
							mask = bluber_mask1)

	bluber_mask2 = cv2.morphologyEx(bluber_mask2, cv2.MORPH_OPEN, kernel, iterations=2)
	bluber_mask2 = cv2.dilate(bluber_mask2,kernel,iterations=3)
	res_bluber2 = cv2.bitwise_and(imageFrame, imageFrame, 
							mask = bluber_mask2)

	bluber_mask3 = cv2.morphologyEx(bluber_mask3, cv2.MORPH_OPEN, kernel, iterations=2)
	bluber_mask3 = cv2.dilate(bluber_mask3,kernel,iterations=3) 
	res_bluber3 = cv2.bitwise_and(imageFrame, imageFrame, 
							mask = bluber_mask3)

	# For orange color shades
	orng_mask = cv2.morphologyEx(orng_mask, cv2.MORPH_OPEN, kernel, iterations=2)
	orng_mask = cv2.dilate(orng_mask,kernel,iterations=3)
	res_orng = cv2.bitwise_and(imageFrame, imageFrame, 
							mask = orng_mask)


	# --------------red color shades---------------------
	# Creating contour to track red1 color 
	contours, hierarchy = cv2.findContours(raspb_mask1, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	counter = 0
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			M = cv2.moments(contour)
			cx = int(M['m10'] / M['m00']) # calculate X position
			cy = int(M['m01'] / M['m00']) # calculate Y position

			cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
			cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle

			cv2.putText(imageFrame, "Raspberry", (cx, cy), 
						cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
						(0, 0, 255))

	# Creating contour to track red2 color 
	contours, hierarchy = cv2.findContours(raspb_mask2, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	counter = 0
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300):  

			M = cv2.moments(contour)
			cx = int(M['m10'] / M['m00']) # calculate X position
			cy = int(M['m01'] / M['m00']) # calculate Y position

			cv2.drawContours(imageFrame, [contour], -1, (0, 0, 255), 3) #draw contours
			cv2.circle(imageFrame, (cx, cy), 7, (0, 0, 255), -1)  # draw circle

			cv2.putText(imageFrame, "Raspberry", (cx, cy), 
					cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
					(0, 0, 255))
			

	# --------------green color shades---------------------
	# Creating contour to track green1 color 
	contours, hierarchy = cv2.findContours(lem_mask1, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	counter = 0
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			M = cv2.moments(contour)
			cx = int(M['m10'] / M['m00']) # calculate X position
			cy = int(M['m01'] / M['m00']) # calculate Y position

			cv2.drawContours(imageFrame, [contour], -1, (0, 255, 0), 3) # draw contours
			cv2.circle(imageFrame, (cx, cy), 7, (0, 255, 0), -1)  # draw circle

			cv2.putText(imageFrame, "Lemon", (cx, cy), 
					cv2.FONT_HERSHEY_SIMPLEX, 
					1.0, (0, 255, 0)) 


	# Creating contour to track green2 color 
	contours, hierarchy = cv2.findContours(lem_mask2, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	counter = 0
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			M = cv2.moments(contour)
			cx = int(M['m10'] / M['m00']) # calculate X position
			cy = int(M['m01'] / M['m00']) # calculate Y position

			cv2.drawContours(imageFrame, [contour], -1, (0, 255, 0), 3) # draw contours
				
			cv2.circle(imageFrame, (cx, cy), 7, (0, 255, 0), -1)  # draw circle

			cv2.putText(imageFrame, "Lemon", (cx, cy), 
					cv2.FONT_HERSHEY_SIMPLEX, 
					1.0, (0, 255, 0))
	

	# --------------yellow color shades---------------------
	# Creating contour to track yellow color 
	contours, hierarchy = cv2.findContours(lim_mask, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	counter = 0
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			M = cv2.moments(contour)
			cx = int(M['m10'] / M['m00']) # calculate X position
			cy = int(M['m01'] / M['m00']) # calculate Y position

			cv2.drawContours(imageFrame, [contour], -1, (0, 255, 255), 3) #draw contours
			cv2.circle(imageFrame, (cx, cy), 7, (0, 255, 255), -1)  # draw circle

			cv2.putText(imageFrame, "Lime", (cx, cy), 
					cv2.FONT_HERSHEY_SIMPLEX, 
					1.0, (0, 255, 255)) 

	# --------------blue color shades---------------------
	# Creating contour to track blue1 color 
	contours, hierarchy = cv2.findContours(bluber_mask1, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 

	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			M = cv2.moments(contour)
			cx = int(M['m10'] / M['m00']) # calculate X position
			cy = int(M['m01'] / M['m00']) # calculate Y position

			cv2.drawContours(imageFrame, [contour], -1, (255, 0, 0), 3) #draw contours
			cv2.circle(imageFrame, (cx, cy), 7, (255, 0, 0), -1)  # draw circle

			cv2.putText(imageFrame, "Blueberry", (cx, cy), 
					cv2.FONT_HERSHEY_SIMPLEX, 
					1.0, (255, 0, 0)) 

	# Creating contour to track blue2 color 
	contours, hierarchy = cv2.findContours(bluber_mask2, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			M = cv2.moments(contour)
			cx = int(M['m10'] / M['m00']) # calculate X position
			cy = int(M['m01'] / M['m00']) # calculate Y position

			cv2.drawContours(imageFrame, [contour], -1, (255, 0, 0), 3) #draw contours
			cv2.circle(imageFrame, (cx, cy), 7, (255, 0, 0), -1)  # draw circle

			cv2.putText(imageFrame, "Blueberry", (cx, cy), 
					cv2.FONT_HERSHEY_SIMPLEX, 
					1.0, (255, 0, 0)) 


	# Creating contour to track blue3 color 
	contours, hierarchy = cv2.findContours(bluber_mask3, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			M = cv2.moments(contour)
			cx = int(M['m10'] / M['m00']) # calculate X position
			cy = int(M['m01'] / M['m00']) # calculate Y position

			cv2.drawContours(imageFrame, [contour], -1, (255, 0, 0), 3) #draw contours
			cv2.circle(imageFrame, (cx, cy), 7, (255, 0, 0), -1)  # draw circle

			cv2.putText(imageFrame, "Blueberry", (cx, cy), 
					cv2.FONT_HERSHEY_SIMPLEX, 
					1.0, (255, 0, 0)) 

	# --------------orange color shades---------------------
	# Creating contour to track orange color 
	contours, hierarchy = cv2.findContours(orng_mask, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			M = cv2.moments(contour)
			cx = int(M['m10'] / M['m00']) # calculate X position
			cy = int(M['m01'] / M['m00']) # calculate Y position

			cv2.drawContours(imageFrame, [contour], -1, (0, 165, 255), 3) #draw contours
			cv2.circle(imageFrame, (cx, cy), 7, (0, 165, 255), -1)  # draw circle

			cv2.putText(imageFrame, "Orange", (cx, cy), 
					cv2.FONT_HERSHEY_SIMPLEX, 
					1.0, (0, 165, 255)) ## change this
			
	# Program Termination 
	cv2.imshow("Multiple Color Detection in Real-Time", imageFrame) 
	if cv2.waitKey(10) & 0xFF == ord('q'): 
		cap.release() 
		cv2.destroyAllWindows() 
		break
