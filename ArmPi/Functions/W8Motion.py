#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
import numpy as np

servo1 = 500 
servo2 = 500
servo3 = 500
servo4 = 500
servo5 = 500

# Set the servo positions with a use_time of 1000ms
Board.setBusServoPulse(1, servo1, 1000) # Servo 1
Board.setBusServoPulse(2, servo2, 1000) # Servo 2
Board.setBusServoPulse(3, servo3, 1000) # Servo 3
Board.setBusServoPulse(4, servo4, 1000) # Servo 4
Board.setBusServoPulse(5, servo5, 1000) # Servo 5

time.sleep(1)

# Change the servo positions
servo1 = 600
servo2 = 600
servo3 = 600
servo4 = 600
servo5 = 600

# Set the new servo positions with a use_time of 1000ms
Board.setBusServoPulse(1, servo1, 1000) # Servo 1
Board.setBusServoPulse(2, servo2, 1000) # Servo 2
Board.setBusServoPulse(3, servo3, 1000) # Servo 3
Board.setBusServoPulse(4, servo4, 1000) # Servo 4
Board.setBusServoPulse(5, servo5, 1000) # Servo 5

time.sleep(1)