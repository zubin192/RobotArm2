#!/usr/bin/python3
# coding=utf8

import sys
sys.path.append('/home/pi/ArmPi/')
import time
import HiwonderSDK.Board as Board
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

class RoboticArm:
    def __init__(self):
        self.servo1 = 500
        self.AK = ArmIK()

    def init_move(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), 0, -30, -90, 1500)

    def move_arm(self, target_position):
        return self.AK.setPitchRangeMoving(target_position, -90, -90, 0, 1500)

    def open_gripper(self):
        Board.setBusServoPulse(1, self.servo1 - 280, 500)

    def close_gripper(self):
        Board.setBusServoPulse(1, self.servo1, 500)

def main():
    robotic_arm = RoboticArm()
    robotic_arm.init_move()
    time.sleep(2)

    while True:
        x = float(input("Enter the x-coordinate to move to: "))
        y = float(input("Enter the y-coordinate to move to: "))
        z = float(input("Enter the z-coordinate to move to: "))
        target_position = (x, y, z)

        robotic_arm.move_arm(target_position)
        time.sleep(5)

if __name__ == '__main__':
    main()