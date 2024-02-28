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
 
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
 
AK = ArmIK()
servo1 = 500 
servo2 = 500
servo3 = 500
servo4 = 500
servo5 = 500