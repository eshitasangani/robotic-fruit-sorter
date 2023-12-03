#!/usr/bin/env python3

#----------------------------------------------------------
# This file is the integration of the inverse kinematics 
# of the arm and the machine learning model of the camera 
#----------------------------------------------------------

import pigpio 
import time
import cv2 
import imutils
import numpy as np
import camera_detection
import ik
import math
