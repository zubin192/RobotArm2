#!/usr/bin/python3
# coding=utf8

import sys
import time
import keyboard
sys.path.append('/home/pi/ArmPi/')
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
        self.AK.setPitchRangeMoving((0, 10, 10), -30, 0, 90, 1500)

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

    print("Control the arm using the keyboard:")
    print("w/s: Increase/decrease x-coordinate")
    print("a/d: Increase/decrease y-coordinate")
    print("q/e: Increase/decrease z-coordinate")
    print("Press 'esc' to exit")

    x, y, z = 0, 0, 0

    def change_coordinates(e):
        nonlocal x, y, z
        if e.name == 'w': x += 100
        elif e.name == 's': x -= 100
        elif e.name == 'a': y += 100
        elif e.name == 'd': y -= 100
        elif e.name == 'q': z += 100
        elif e.name == 'e': z -= 100

        target_position = (x, y, z)
        robotic_arm.move_arm(target_position)

    keyboard.on_press_key("w", change_coordinates)
    keyboard.on_press_key("s", change_coordinates)
    keyboard.on_press_key("a", change_coordinates)
    keyboard.on_press_key("d", change_coordinates)
    keyboard.on_press_key("q", change_coordinates)
    keyboard.on_press_key("e", change_coordinates)

    keyboard.wait('esc')

if __name__ == '__main__':
    main()