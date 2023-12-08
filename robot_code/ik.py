#!/usr/bin/env python3

#----------------------------------------------------------
# This file contains our code to implement inverse kinematics 
# on our MeArm system. 
# Our coordinate system is in millimeters (mm)
# Origin is right above the A0/ROTATE servo at the same 
# level as the A1/SHOULDER servo
# +x direction is pointing right (when looking at back of arm)
# +y direction is directly forward (when looking at back of arm)
# +z direction is up
#----------------------------------------------------------

import pigpio 
import time
import math
import numpy as np
#-------------- VARIABLES -----------#

GRIPPER = 26
GRIPPER_OPEN = 1400
GRIPPER_CLOSE = 2000

# shoulder: right servo (when gripper is facing away from you)
# outstretches the arm
# corresponds to a1
SHOULDER = 5 
SHOULDER_OUT = 2000 # 0 deg
SHOULDER_IN = 750 # 130 deg
SHOULDER_RANGE = 2.26893 # rad

# elbow: left servo (when gripper is facing away from you)
# moves the top part of arm up and down 
# corresponds to a2
ELBOW = 13 
ELBOW_BENT = 750    
ELBOW_OUT = 1700		 # 25 degrees from horizontal
ELBOW_RANGE = 1.8326 # rad

# corresponds to a0
ROTATE = 16
ROTATE_MAX = 2350 # far left (almost pi radians)
ROTATE_MIN = 700 # far right (0)
ROTATE_MID = 1500	# mid

pi = pigpio.pi()

pi.set_mode(GRIPPER,pigpio.OUTPUT)
pi.set_mode(ELBOW,pigpio.OUTPUT)
pi.set_mode(SHOULDER,pigpio.OUTPUT)
pi.set_mode(ROTATE,pigpio.OUTPUT)

curr_elbow = 0
curr_shoulder = 0
curr_gripper = 0
## MEASURED 
L1 = 105  # shoulder to elbow len
L2 = 85  # elbow to wrist len
L3 = 95  # len from wrist to hand plus base center to shoulder

#-------------- CALCULATIONS FOR MOVEMENT -----------#

# Get polar coords from cartesian ones
def cart2polar(a, b):

	# mag of cartesian coords
	# is math.hypot more accurate?
	# r = np.sqrt(a*a + b*b)
	r = math.hypot(a, b)
	# print("cart2polar r = " + str(r))
	
	# don't calculate zero mag
	if r == 0:
		return None, None
	
	c = a / r
	s = b / r

	# cap the values
	if s > 1:
		s = 1
	if c > 1:
		c = 1
	if s < -1:
		s = -1
	if c < -1:
		c = -1
	
	# calculate angle from 0 to pi
	# try math.acos() instead?
	# theta = np.arccos(c)
	theta = math.acos(c)

	if s < 0:
		theta *= -1

	return r, theta

# get angle from a triangle using cos rule
def cosangle(opp, adj1, adj2):
	den = 2 * adj1 * adj2
	if den == 0:
		print("den = 0")
		return None
	
	c = (adj1 * adj1 + adj2 * adj2 - opp * opp) / den

	if c > 1 or c < -1:
		print("c = " + str(c))
		return None
	
	theta = np.arccos(c)

	return theta

# solve for the servo angles
def solve(x, y, z):
	global L1
	global L2
	global L3

	r, th0 = cart2polar(y, x)
	if r == None:
		return None, None, None

	r -= L3

	R, ang_P = cart2polar(r, z)

	# print("r = " + str(r))
	# print("z = " + str(z))
	# print("post cart2polar R = " + str(R))

	C = cosangle(R, L1, L2)
	B = cosangle(L2, L1, R)

	if B == None:
		print("calculate B fail")
		# print("L2 = "+ str(L2))
		# print("L1 = " + str(L1))
		# print("R = " + str(R))
		# print("B = " + str(B))
		return None, None, None
	if C == None:
		print("calculate C fail")
		return None, None, None
	
	a0 = th0
	a1 = ang_P + B
	a2 = C + a1 - np.pi

	return a0, a1, a2

# turn off all the servos
def stop_servos():
	global ELBOW
	global SHOULDER
	global ROTATE

	pi.set_servo_pulsewidth(ELBOW, 0)
	pi.set_servo_pulsewidth(SHOULDER, 0)
	pi.set_servo_pulsewidth(ROTATE, 0)


# each servo has slightly different range so calibrate based on that
# angle given in radians
def ang2pulse(servo, angle):
	# pulses are in microseconds (us)
	# us per angle
	if servo == ROTATE:
		us_per_angle = (ROTATE_MAX - ROTATE_MIN) / (np.pi)
		pulse = ROTATE_MID - (us_per_angle * angle)
		if pulse > 2350: 
			pulse = 2350
		if pulse < 700:
			pulse = 700

	
	if servo == SHOULDER:
		us_per_angle = (SHOULDER_OUT-SHOULDER_IN) / SHOULDER_RANGE
		pulse = SHOULDER_OUT - (us_per_angle * angle)
		if pulse > 2000: 
			pulse = 2000
		if pulse < 750:
			pulse = 750


	if servo == ELBOW:
		us_per_angle = (ELBOW_OUT - ELBOW_BENT) / ELBOW_RANGE
		pulse = 1500 + (us_per_angle * angle)
		if pulse > 1700: 
			pulse = 1700
		if pulse < 750:
			pulse = 750

	return pulse

#--------------GRIPPER MOVEMENT -----------#

# open the gripper!
def open_gripper():
	global GRIPPER
	global GRIPPER_OPEN
	pi.set_servo_pulsewidth(GRIPPER, GRIPPER_OPEN)
	time.sleep(1)

# close the gripper!
def close_gripper():
	global GRIPPER
	global GRIPPER_CLOSE
	pi.set_servo_pulsewidth(GRIPPER, GRIPPER_CLOSE)
	time.sleep(1)

def stop_gripper():
	global GRIPPER
	pi.set_servo_pulsewidth(GRIPPER, 0)


#--------------SMOOTHNESS + MOVEMENT -----------#

# go to specified x y z coordinate
def go_to_coor(x, y, z):
	global ROTATE
	global SHOULDER
	global ELBOW

	a0, a1, a2 = solve(x, y, z)
	if a0 != None:
		pi.set_servo_pulsewidth(SHOULDER, ang2pulse(SHOULDER, a1))
		time.sleep(0.2)
		pi.set_servo_pulsewidth(ELBOW, ang2pulse(ELBOW, a2))
		time.sleep(0.2)
		pi.set_servo_pulsewidth(ROTATE, ang2pulse(ROTATE, a0))
		time.sleep(0.2)
		stop_servos()
		time.sleep(0.5)
	else: 
		print("SOLVE FAILED")

def go_to_coor_center(x, y, z):
	global ROTATE
	global SHOULDER
	global ELBOW

	a0, a1, a2 = solve(x, y, z)
	if a0 != None:
		pi.set_servo_pulsewidth(ROTATE, ang2pulse(ROTATE, a0))
		time.sleep(0.2)
		pi.set_servo_pulsewidth(SHOULDER, ang2pulse(SHOULDER, a1))
		time.sleep(0.2)
		pi.set_servo_pulsewidth(ELBOW, ang2pulse(ELBOW, a2))
		time.sleep(0.2)
		stop_servos()
		time.sleep(0.5)
	else: 
		print("SOLVE FAILED")

#--------------ARM LOCATIONS - CENTER + BOXES -------------#

def reset_to_center():
	# dont turn 
	time.sleep(0.1)
	go_to_coor_center(10,150,70)
	time.sleep(0.1)

def lemons():
	open_gripper()
	go_to_coor(35,240,60) # fruit location (z= 20 for floor, 50 for on mount)
	time.sleep(0.3)
	close_gripper()
	go_to_coor(-30,-40,80) # lemon lime box
	open_gripper()
	stop_gripper()
	time.sleep(0.1)
	reset_to_center()


def blueberries():
	open_gripper()
	go_to_coor(35,240,60) # fruit location (z= 20 for floor, 50 for on mount)
	time.sleep(0.3)
	close_gripper()
	go_to_coor(-120,100,150) #blueberry box coordination
	open_gripper()
	stop_gripper()
	time.sleep(0.1)
	reset_to_center()

def raspberries():
	open_gripper()
	go_to_coor(35,240,60) # fruit location
	time.sleep(0.3)
	close_gripper()
	go_to_coor(20,-10,80) #raspberry box coordination
	open_gripper()
	stop_gripper()
	time.sleep(0.1)
	reset_to_center()


def oranges():
	open_gripper()
	go_to_coor(35,240,60) # fruit location (z= 20 for floor, 50 for on mount)
	time.sleep(0.3)
	close_gripper()
	go_to_coor(50,60,80) # oranges box
	open_gripper()
	stop_gripper()
	time.sleep(0.1)
	reset_to_center()
