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

class ArmController:
    def __init__(self):
        self.AK = ArmIK()
        self.servo1 = 500

    def initMove(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def moveArm(self, target_position):
        # Set pitch, roll, and yaw to fixed values
        self.AK.setPitchRangeMoving(target_position, -30, -30, -90, 1500)

    def openGripper(self):
        Board.setBusServoPulse(1, self.servo1 - 280, 500)

    def closeGripper(self):
        Board.setBusServoPulse(1, self.servo1, 500)

if __name__ == '__main__':
    controller = ArmController()
    controller.initMove()
    time.sleep(2)

    x = float(input("Enter the x-coordinate: "))
    y = float(input("Enter the y-coordinate: "))
    z = float(input("Enter the z-coordinate: "))
    target_position = (x, y, z)

    controller.moveArm(target_position)
    controller.openGripper()
    time.sleep(2)
    controller.closeGripper()
    controller.initMove()