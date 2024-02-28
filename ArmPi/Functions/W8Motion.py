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

Board.setBusServoPulse(1, servo1, 1000) 
Board.setBusServoPulse(2, servo2, 1000) 
Board.setBusServoPulse(3, servo3, 1000) 
Board.setBusServoPulse(4, servo4, 1000) 
Board.setBusServoPulse(5, servo5, 1000) 

time.sleep(1)

servo1 = 600


Board.setBusServoPulse(1, servo1, 1000) 
Board.setBusServoPulse(2, servo2, 1000) 
Board.setBusServoPulse(3, servo3, 1000) 
Board.setBusServoPulse(4, servo4, 1000) 
Board.setBusServoPulse(5, servo5, 1000) 

time.sleep(1)