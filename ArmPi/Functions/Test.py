#!/usr/bin/python3
# coding=utf8

import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import threading
import numpy as np
import math
import HiwonderSDK.Board as Board
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

AK = ArmIK()

class RoboticArmMotionControl:
    def __init__(self):
        self._stop = False
        self._isRunning = False
        self._thread = threading.Thread(target=self._move)
        self._thread.setDaemon(True)

        # Initialize other necessary variables for motion control
        self._target_coordinates = None
        self._action_finish = True

        # Add other initialization steps here

    def start(self):
        self._stop = False
        self._isRunning = True
        self._thread.start()

    def stop(self):
        self._stop = True
        self._isRunning = False

    def set_target_coordinates(self, coordinates):
        self._target_coordinates = coordinates

    def _move(self):
        while True:
            if self._isRunning:
                if self._target_coordinates and self._action_finish:
                    self._action_finish = False
                    # Add logic for moving the arm to the target coordinates
                    x, y, z = self._target_coordinates
                    result = AK.setPitchRangeMoving((x, y, z), -90, -90, 0)  # Adjust parameters as needed
                    time.sleep(result[2] / 1000)  # The third item in the returned parameters is the time
                    # Once the action is completed, reset the target coordinates and set action_finish to True
                    self._target_coordinates = None
                    self._action_finish = True
            else:
                if self._stop:
                    # Add logic to return to initial position when stopped
                    self._stop = False
                    # Placeholder, replace with actual movement commands
                    time.sleep(0.1)
                time.sleep(0.01)

def main():
    # Initialize robotic arm motion control
    motion_controller = RoboticArmMotionControl()
    motion_controller.start()

    # Accept input from terminal for XYZ coordinates
    while True:
        try:
            x = float(input("Enter X coordinate: "))
            y = float(input("Enter Y coordinate: "))
            z = float(input("Enter Z coordinate: "))
            motion_controller.set_target_coordinates((x, y, z))
            time.sleep(2)  # Add a delay before accepting new coordinates
        except ValueError:
            print("Invalid input. Please enter numerical values.")

    # Uncomment the lines below if you want to stop the motion control after a certain time
    # time.sleep(60)  # Wait for 60 seconds
    # motion_controller.stop()

if __name__ == '__main__':
    main()
