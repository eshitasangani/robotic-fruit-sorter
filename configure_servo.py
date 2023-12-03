#!/usr/bin/env python3

import pigpio 
import time
import numpy as np

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

L1 = 80  # shoulder to elbow len
L2 = 80  # elbow to wrist len
L3 = 68  # len from wrist to hand plus base center to shoulder


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

# This function moves one servo slowly from its current position 
# to the next. 
## OLD VERSION 
def move_to_pos(servo, goal):
	curr = pi.get_servo_pulsewidth(servo)
	step=1
	if (curr > goal):
		dist = curr - goal
		for i in range(int(dist)):
			curr -= step
			if curr > 2250:
				curr = 2250
			if curr < 750:
				curr = 750
			pi.set_servo_pulsewidth(servo, curr)
			time.sleep(0.0005)
		
	else:
		dist = goal-curr
		for i in range(int(dist)):
			curr += step
			if curr > 2250:
				curr = 2250
			if curr < 750:
				curr = 750
			pi.set_servo_pulsewidth(servo, curr)
			time.sleep(0.0005)



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


def open_gripper():
	global GRIPPER
	global GRIPPER_OPEN
	pi.set_servo_pulsewidth(GRIPPER, GRIPPER_OPEN)
	time.sleep(1)

def close_gripper():
	global GRIPPER
	global GRIPPER_CLOSE
	pi.set_servo_pulsewidth(GRIPPER, GRIPPER_CLOSE)
	time.sleep(1)

def go_to_coor(x, y, z):
	global ROTATE
	global SHOULDER
	global ELBOW

	a0, a1, a2 = solve(x, y, z)
	if a0 != None:
		pi.set_servo_pulsewidth(ROTATE, ang2pulse(ROTATE, a0))
		pi.set_servo_pulsewidth(SHOULDER, ang2pulse(SHOULDER, a1))
		pi.set_servo_pulsewidth(ELBOW, ang2pulse(ELBOW, a2))
		time.sleep(1)
	else: 
		print("SOLVE FAILED")


# y max: 228
# 

try:
	# go_to_coor(10,100,50)
	# open_gripper()
	# go_to_coor(10, 200, -10)
	# close_gripper()
	# go_to_coor(10,100,50)
	open_gripper()
	close_gripper()

finally:
	stop_servos()

# close_gripper()
# while True:
# 	pi.set_servo_pulsewidth(GRIPPER, 2000)

	# move_to_pos(GRIPPER, GRIPPER_OPEN)
	# time.sleep(1)

	# move_to_pos(GRIPPER, GRIPPER_CLOSE)
	# time.sleep(1)

	
	# for n in range(3):
	# 	move_to_pos(ELBOW, ELBOW_BENT)
	# 	time.sleep(1)

	# 	move_to_pos(ELBOW, ELBOW_OUT)
	# 	time.sleep(1)

	# for n in range(3):
	# 	move_to_pos(SHOULDER, SHOULDER_OUT)
	# 	time.sleep(1)

	# 	move_to_pos(SHOULDER, SHOULDER_IN)
	# 	time.sleep(1)
	
	# pi.set_servo_pulsewidth(ELBOW, ang2pulse(ELBOW, -np.pi / 8))
	# move_to_pos(ROTATE, ang2pulse(ROTATE, -np.pi / 2))
	# stop_servos()


# finally:
# 	stop_servos()
