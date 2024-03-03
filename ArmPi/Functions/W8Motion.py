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

servo1 = 500

def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

def moveArm(target_position):
    # Removed pitch, roll, and yaw parameters
    AK.setPitchRangeMoving(target_position, -30, -30, -90, 1500)

def openGripper():
    Board.setBusServoPulse(1, servo1 - 280, 500)  # Open the gripper

def closeGripper():
    Board.setBusServoPulse(1, servo1, 500)  # Close the gripper

if __name__ == '__main__':
    initMove()
    time.sleep(2)  # Wait for the arm to reach the initial position

    # Get the target position from the user
    x = float(input("Enter the x-coordinate: "))
    y = float(input("Enter the y-coordinate: "))
    z = float(input("Enter the z-coordinate: "))
    target_position = (x, y, z)

    # Move the arm to the specified position
    moveArm(target_position)

    # Open the gripper to pick up the block
    openGripper()

    # Wait for some time to simulate picking up the block
    time.sleep(2)

    # Close the gripper after picking up the block
    closeGripper()

    initMove()  # Move the arm back to the initial position