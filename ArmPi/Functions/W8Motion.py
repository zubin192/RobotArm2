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

AK = ArmIK()

servo1 = 700

# Initial position
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

# Mechanical arm movement thread
def move():
    global servo1
    while True:
        Board.setBusServoPulse(1, servo1 - 280, 500)  # Open the gripper
        time.sleep(0.8)
        Board.setBusServoPulse(1, servo1, 500)  # Close the gripper
        time.sleep(1)
        initMove()  # Return to the initial position
        time.sleep(1.5)

# Run the sub-thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

if __name__ == '__main__':
    initMove()
    while True:
        time.sleep(1)