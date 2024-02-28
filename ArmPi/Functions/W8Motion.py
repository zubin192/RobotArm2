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
def move(target_position, pitch, roll, yaw):
    global servo1
    Board.setBusServoPulse(1, servo1 - 280, 500)  # Open the gripper
    time.sleep(0.8)
    Board.setBusServoPulse(1, servo1, 500)  # Close the gripper
    time.sleep(1)
    initMove()  # Return to the initial position
    time.sleep(1.5)

    # Move the end effector to the target position
    AK.setPitchRangeMoving(target_position, pitch, roll, yaw, 1500)
    time.sleep(1.5)

if __name__ == '__main__':
    initMove()
    while True:
        # Get the target position and the pitch, roll, and yaw angles from the terminal
        x = float(input("Enter the x-coordinate: "))
        y = float(input("Enter the y-coordinate: "))
        z = float(input("Enter the z-coordinate: "))
        target_position = (x, y, z)

        pitch = float(input("Enter the pitch angle: "))
        roll = float(input("Enter the roll angle: "))
        yaw = float(input("Enter the yaw angle: "))

        # Run the move function with the input values
        move(target_position, pitch, roll, yaw)