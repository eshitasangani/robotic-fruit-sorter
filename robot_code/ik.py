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
GRIPPER_OPEN = 1000
GRIPPER_CLOSE = 2250

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
ELBOW_BENT = 750 # -85
ELBOW_OUT = 2000 # 20
ELBOW_RANGE = 1.8326 # rad

# corresponds to a0
ROTATE = 16
ROTATE_MAX = 2250
ROTATE_MIN = 750
ROTATE_MID = 1500

pi = pigpio.pi()

pi.set_mode(GRIPPER,pigpio.OUTPUT)
pi.set_mode(ELBOW,pigpio.OUTPUT)
pi.set_mode(SHOULDER,pigpio.OUTPUT)
pi.set_mode(ROTATE,pigpio.OUTPUT)

curr_elbow = 0
curr_shoulder = 0
curr_gripper = 0

# were from github repo
# L1 = 80  # shoulder to elbow len
# L2 = 80  # elbow to wrist len
# L3 = 68  # len from wrist to hand plus base center to shoulder

## MEASURED 
L1 = 105  # shoulder to elbow len
L2 = 85  # elbow to wrist len
L3 = 95  # len from wrist to hand plus base center to shoulder

#-------------- CALCULATIONS FOR MOVEMENT -----------#

# Get polar coords from cartesian ones
def cart2polar(a, b):

	# mag of cartesian coords
	r = np.sqrt(a*a + b*b)
	print("cart2polar r = " + str(r))
	
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
	theta = np.arccos(c)

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

	print("r = " + str(r))
	print("z = " + str(z))
	print("post cart2polar R = " + str(R))

	C = cosangle(R, L1, L2)
	B = cosangle(L2, L1, R)

	if B == None:
		print("calculate B fail")
		print("L2 = "+ str(L2))
		print("L1 = " + str(L1))
		print("R = " + str(R))
		print("B = " + str(B))
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
	global GRIPPER
	global ROTATE

	pi.set_servo_pulsewidth(ELBOW, 0)
	pi.set_servo_pulsewidth(SHOULDER, 0)
	pi.set_servo_pulsewidth(GRIPPER, 0)
	pi.set_servo_pulsewidth(ROTATE, 0)

# each servo has slightly different range so calibrate based on that
# angle given in radians
def ang2pulse(servo, angle):
	# pulses are in microseconds (us)
	# us per angle
	if servo == ROTATE:
		us_per_angle = (ROTATE_MAX - ROTATE_MIN) / (np.pi)
		pulse = ROTATE_MID - (us_per_angle * angle)
		if pulse > 2250: 
			pulse = 2000
		if pulse < 750:
			pulse = 750

	
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
		if pulse > 2000: 
			pulse = 2000
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


#--------------SMOOTHNESS + MOVEMENT -----------#

# This function moves one servo slowly from its current position 
# to the next using acceleration and deaccleeration phases of each 
# servo limits

def move_to_pos(servo, goal):
    curr = pi.get_servo_pulsewidth(servo)
    time_step = 1.5  # how long each time step is
    num_steps = int(time_step / 0.001)  # step # with delay

    # Acceleration and deceleration parameters
    acceleration_steps = num_steps // 10  # ratio of acceleration steps
    deceleration_steps = acceleration_steps

    # if its already tehre, just stay there 
    goal = max(500, min(2500, goal))

    # Acceleration phase
    for i in range(acceleration_steps):
        step_size = (goal - curr) / (acceleration_steps - i)
        curr += step_size
        curr = max(500, min(2500, curr))  # Ensure pulsewidth stays within valid range
        pi.set_servo_pulsewidth(servo, int(curr))
        time.sleep(0.001)

    # Constant velocity phase
    for i in range(num_steps - 2 * acceleration_steps):
        curr = goal  # Set current position to goal directly
        curr = max(500, min(2500, curr))  # Ensure pulsewidth stays within valid range
        pi.set_servo_pulsewidth(servo, int(curr))
        time.sleep(0.001)

    # Deceleration phase
    for i in range(deceleration_steps):
        step_size = (goal - curr) / (deceleration_steps - i)
        curr += step_size
        curr = max(500, min(2500, curr))  # Ensure pulsewidth stays within valid range
        pi.set_servo_pulsewidth(servo, int(curr))
        time.sleep(0.001)

# go to specified x y z coordinate
def go_to_coor(x, y, z):
	global ROTATE
	global SHOULDER
	global ELBOW

	a0, a1, a2 = solve(x, y, z)
	if a0 != None:
		move_to_pos(ROTATE, ang2pulse(ROTATE, a0))
		move_to_pos(SHOULDER, ang2pulse(SHOULDER, a1))
		move_to_pos(ELBOW, ang2pulse(ELBOW, a2))

		# pi.set_servo_pulsewidth(ROTATE, ang2pulse(ROTATE, a0))
		# pi.set_servo_pulsewidth(SHOULDER, ang2pulse(SHOULDER, a1))
		# pi.set_servo_pulsewidth(ELBOW, ang2pulse(ELBOW, a2))

		time.sleep(0.5)
		stop_servos()
		time.sleep(0.5)
	else: 
		print("SOLVE FAILED")

#--------------ARM LOCATIONS - CENTER + BOXES -------------#

def reset_to_center():
	# dont turn 
	time.sleep(0.1)
	go_to_coor(10,100,50)
	time.sleep(0.1)

def lemons():
	# close left
	go_to_coor(-130,0,75) 
	# reset_to_center()

def blueberries():
	# far left
	go_to_coor(-250,0,75)
	# reset_to_center()

def raspberries():
	# close right
	go_to_coor(190,-50,75) 
	# reset_to_center()

def oranges():
	# far right
	go_to_coor(190,-170,75) 
	# reset_to_center()

#--------------TEST THE INVERSE KINEMATICS CODE!!-----------#
try:
	# initial initialization
	# go_to_coor(10,100,50)
	# reset_to_center()

	# lemons()
	# time.sleep(0.05)

	blueberries()
	# reset_to_center()
	# time.sleep(0.05)

	# raspberries()
	# reset_to_center()
	# time.sleep(0.05)

	# reset_to_center()
	# blueberries()

	# reset_to_center()
	# raspberries()

	# reset_to_center()
	# oranges()
	
finally:
	stop_servos()

#first left boundary box 
# go_to_coor(-130,0,75) 

#super far left
# go_to_coor(-250,0,75) 

#first right boundary box 
# go_to_coor(190,-50,75) 

#super far right
# go_to_coor(190,-170,75) 


# COORDINATES 
# go_to_coor(10, 200, -10) # middle 
# go_to_coor(10,100,50) # resting middle position


