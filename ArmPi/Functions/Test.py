#!/usr/bin/python3
# coding=utf8

import sys
sys.path.append('/home/pi/ArmPi/')
import time
import HiwonderSDK.Board as Board

class RoboticArm:
    def __init__(self):
        self.servo1 = 500

    def move_servo1(self, pulse):
        Board.setBusServoPulse(1, pulse, 500)

    def move_servo2(self, pulse):
        Board.setBusServoPulse(2, pulse, 500)

    def move_servo3(self, pulse):
        Board.setBusServoPulse(3, pulse, 500)

    def move_servo4(self, pulse):
        Board.setBusServoPulse(4, pulse, 500)

    def move_servo5(self, pulse):
        Board.setBusServoPulse(5, pulse, 500)

    def move_servo6(self, pulse):
        Board.setBusServoPulse(6, pulse, 500)

def main():
    robotic_arm = RoboticArm()

    # Example usage:
    robotic_arm.move_servo1(600)
    time.sleep(1)
    robotic_arm.move_servo2(600)
    time.sleep(1)
    robotic_arm.move_servo3(600)
    time.sleep(1)
    robotic_arm.move_servo4(600)
    time.sleep(1)
    robotic_arm.move_servo5(600)
    time.sleep(1)
    robotic_arm.move_servo6(600)
    time.sleep(1)

if __name__ == '__main__':
    main()