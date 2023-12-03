# from picamera import PiCamera
# import time

# camera = PiCamera()

# camera.start_preview()
# time.sleep(5)
# camera.capture('/home/pi/final-proj/image.jpg')
# camera.stop_preview()

# Python code for Multiple Color Detection 


import numpy as np 
import cv2 


# Capturing video through webcam 
webcam = cv2.VideoCapture(0) 

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

	# Set range for red color and 
	# define mask 
	red_lower = np.array([136, 87, 111], np.uint8) 
	red_upper = np.array([180, 255, 255], np.uint8) 
	red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 

	# Set range for green color and 
	# define mask 
	green_lower = np.array([25, 52, 72], np.uint8) 
	green_upper = np.array([102, 255, 255], np.uint8) 
	green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 

	# Set range for blue color and 
	# define mask 
	blue_lower = np.array([94, 80, 2], np.uint8) 
	blue_upper = np.array([120, 255, 255], np.uint8) 
	blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 
	
	# Morphological Transform, Dilation 
	# for each color and bitwise_and operator 
	# between imageFrame and mask determines 
	# to detect only that particular color 
	kernel = np.ones((5, 5), "uint8") 
	
	# For red color 
	# red_mask = cv2.dilate(red_mask, kernel) 
	res_red = cv2.bitwise_and(imageFrame, imageFrame, 
							mask = red_mask) 
	
	# For green color 
	# green_mask = cv2.dilate(green_mask, kernel) 
	res_green = cv2.bitwise_and(imageFrame, imageFrame, 
								mask = green_mask) 
	
	# For blue color 
	# blue_mask = cv2.dilate(blue_mask, kernel) 
	res_blue = cv2.bitwise_and(imageFrame, imageFrame, 
							mask = blue_mask) 

	# Creating contour to track red color 
	contours, hierarchy = cv2.findContours(red_mask, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour) 
			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(0, 0, 255), 2) 
			
			cv2.putText(imageFrame, "Red Colour", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
						(0, 0, 255))	 

	# Creating contour to track green color 
	contours, hierarchy = cv2.findContours(green_mask, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour) 
			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(0, 255, 0), 2) 
			
			cv2.putText(imageFrame, "Green Colour", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 
						1.0, (0, 255, 0)) 

	# Creating contour to track blue color 
	contours, hierarchy = cv2.findContours(blue_mask, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour) 
			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(255, 0, 0), 2) 
			
			cv2.putText(imageFrame, "Blue Colour", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 
						1.0, (255, 0, 0)) 
			
	# Program Termination 
	cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame) 
	if cv2.waitKey(10) & 0xFF == ord('q'): 
		cap.release() 
		cv2.destroyAllWindows() 
		break




"""

# --------------red color shades---------------------
	# Creating contour to track red1 color 
	contours, hierarchy = cv2.findContours(raspb_mask1, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(0, 0, 255), 2) 
			
			cv2.putText(imageFrame, "Raspberry", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
						(0, 0, 255))

	# Creating contour to track red2 color 
	contours, hierarchy = cv2.findContours(raspb_mask2, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(0, 0, 255), 2) 
			
			cv2.putText(imageFrame, "Raspberry", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
						(0, 0, 255))

	# --------------green color shades---------------------
	# Creating contour to track green1 color 
	contours, hierarchy = cv2.findContours(lem_mask1, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(0, 255, 0), 2) 
			
			cv2.putText(imageFrame, "Lemon", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 
						1.0, (0, 255, 0))

	# Creating contour to track green2 color 
	contours, hierarchy = cv2.findContours(lem_mask2, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(0, 255, 0), 2) 
			
			cv2.putText(imageFrame, "Lemon", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 
						1.0, (0, 255, 0))

	# --------------yellow color shades---------------------
	# Creating contour to track yellow color 
	contours, hierarchy = cv2.findContours(lim_mask, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour)

			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(0, 255, 255), 2) 
			
			cv2.putText(imageFrame, "Lime", (x, y), 
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
			x, y, w, h = cv2.boundingRect(contour) 

			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(255, 0, 0), 2) 
			
			cv2.putText(imageFrame, "Blueberry", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 
						1.0, (255, 0, 0)) 

	# Creating contour to track blue2 color 
	contours, hierarchy = cv2.findContours(bluber_mask2, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour) 

			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(255, 0, 0), 2) 
			
			cv2.putText(imageFrame, "Blueberry", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 
						1.0, (255, 0, 0)) 

	# Creating contour to track blue3 color 
	contours, hierarchy = cv2.findContours(bluber_mask3, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour) 

			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(255, 0, 0), 2) 

			# imageFrame = cv2.circle(imageFrame, center, 
			# 					radius,(255, 0, 0), 2)
			
			cv2.putText(imageFrame, "Blueberry", (x, y), 
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
			x, y, w, h = cv2.boundingRect(contour) 

			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(0, 165, 255), 2) 

			cv2.putText(imageFrame, "Orange", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 
						1.0, (0, 165, 255)) ## change this 
			
	# Program Termination 
	cv2.imshow("Multiple Color Detection in Real-Time", imageFrame) 
	if cv2.waitKey(10) & 0xFF == ord('q'): 
		cap.release() 
		cv2.destroyAllWindows() 
		break

"""


'''
def find_color(mask):
		"""
		Calculate moment and contour.

		Return contour, center x-position, center y-position
		"""
		cnts = cv2.findContours(mask, cv2.RETR_TREE, 
							cv2.CHAIN_APPROX_SIMPLE) # find contours from mask
		cnts = imutils.grab_contours(cnts)
		for c in cnts:
			area = cv2.contourArea(c) # find how big countour is
			if area > 300:       # only if countour is big enough, then
				M = cv2.moments(c)
				cx = int(M['m10'] / M['m00']) # calculate X position
				cy = int(M['m01'] / M['m00']) # calculate Y position
				return c, cx, cy

	fruits = {'raspb1': (raspb_mask2, 'Raspberry', (0, 0, 255)), 'raspb2': (raspb_mask2, 'Raspberry', (0, 0, 255)), 'lem1': (lem_mask1, 'Lemon', (0, 255, 0)), 
			'lem2': (lem_mask2, 'Lemon', (0, 255, 0)), 'lime': (lim_mask, 'Lime', (0, 255, 255)), 'bluber1': (bluber_mask1, 'Blueberry', (255, 0, 0)), 
			'bluber2': (bluber_mask2, 'Blueberry', (255, 0, 0)), 'bluber3': (bluber_mask3, 'Blueberry', (255, 0, 0)), 'orange': (orng_mask, 'Orange', (0, 165, 255))}

	while webcam.isOpened(): # main loop
		for type, (mask, name, color) in fruits.items(): # for each color in colors
			if find_color(mask):  # call find_color function above
				c, cx, cy = find_color(mask)
				cv2.drawContours(imageFrame, [c], -1, color, 3) #draw contours
				cv2.circle(imageFrame, (cx, cy), 7, color, -1)  # draw circle
				cv2.putText(imageFrame, name, (cx,cy), 
							cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1) # put text
		cv2.imshow("Frame: ", imageFrame) # show image
		if cv2.waitKey(1) == ord('q'):
			break

	cap.release()   #idk what it is
	cv2.destroyAllWindows() # close all windows opened by opencv
'''
