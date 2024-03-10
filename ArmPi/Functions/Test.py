#!/usr/bin/python3
# coding=utf8

import sys
sys.path.append('/home/pi/ArmPi/')
import time
import threading
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
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

class RoboticArmMotionControl:
    def __init__(self):
        self._stop = False
        self._is_running = False
        self._thread = threading.Thread(target=self._move)
        self._thread.setDaemon(True)

        # Initialize other necessary variables for motion control
        self._target_coordinates = None
        self._target_location = None
        self._action_finish = True
        self.robotic_arm = RoboticArm()

    def start(self):
        self._stop = False
        self._is_running = True
        self._thread.start()

    def stop(self):
        self._stop = True
        self._is_running = False

    def set_target_coordinates(self, coordinates, target_location=None):
        self._target_coordinates = coordinates
        self._target_location = target_location

    def _move(self):
        while True:
            if self._is_running:
                if self._target_coordinates and self._action_finish:
                    self._action_finish = False
                    self.robotic_arm.open_gripper()
                    result = self.robotic_arm.move_arm(self._target_coordinates, -90, -90, 0)
                    if result is not None:
                        time.sleep(result[2] / 1000)
                    self.robotic_arm.close_gripper()
                    self._target_coordinates = None
                    self._action_finish = True
                elif self._target_location and self._action_finish:
                    self._action_finish = False
                    result = self.robotic_arm.move_arm(self._target_location, -90, -90, 0)
                    if result is not None:
                        time.sleep(result[2] / 1000)
                    self.robotic_arm.open_gripper()
                    time.sleep(1)
                    self.robotic_arm.close_gripper()
                    self.robotic_arm.init_move()
                    time.sleep(2)
                    self._target_location = None
                    self._action_finish = True
            else:
                if self._stop:
                    self._stop = False
                    self._is_running = False
                time.sleep(0.01)

def main():
    motion_controller = RoboticArmMotionControl()
    motion_controller.robotic_arm.init_move()
    time.sleep(2)

    motion_controller.start()

    x = float(input("Enter the x-coordinate to pick up: "))
    y = float(input("Enter the y-coordinate to pick up: "))
    z = float(input("Enter the z-coordinate to pick up: "))
    target_position = (x, y, z)

    motion_controller.stop()

if __name__ == '__main__':
    main()
