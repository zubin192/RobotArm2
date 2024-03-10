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

    def move_arm(self, target_position, pitch, roll, yaw):
        result = self.AK.setPitchRangeMoving(target_position, pitch, roll, yaw, 1500)
        if result is not None:
            return result
        else:
            return (0, 0, 0, 0)  # Return a default value if no result is returned

    def open_gripper(self):
        Board.setBusServoPulse(1, self.servo1 - 280, 500)

    def close_gripper(self):
        Board.setBusServoPulse(1, self.servo1, 500)

class RoboticArmMotionControl:
    def __init__(self):
        self._stop = False
        self._is_running = False
        self._thread = threading.Thread(target=self._move)
        self._thread.setDaemon(True)

        # Initialize other necessary variables for motion control
        self._target_coordinates = None
        self._target_angles = None
        self._action_finish = True
        self.robotic_arm = RoboticArm()

    def start(self):
        self._stop = False
        self._is_running = True
        self._thread.start()

    def stop(self):
        self._stop = True
        self._is_running = False

    def set_target_position(self, position, angles=None):
        self._target_coordinates = position
        self._target_angles = angles

    def _move(self):
        while True:
            if self._is_running:
                if self._target_coordinates and self._action_finish:
                    self._action_finish = False
                    self.robotic_arm.open_gripper()
                    result = self.robotic_arm.move_arm(self._target_coordinates, *self._target_angles)
                    time.sleep(result[3] / 1000)
                    self.robotic_arm.close_gripper()
                    self._target_coordinates = None
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

    x = float(input("Enter the X-coordinate to move to: "))
    y = float(input("Enter the Y-coordinate to move to: "))
    z = float(input("Enter the Z-coordinate to move to: "))
    target_position = (x, y, z)

    pitch = float(input("Enter the pitch angle: "))
    roll = float(input("Enter the roll angle: "))
    yaw = float(input("Enter the yaw angle: "))
    target_angles = (pitch, roll, yaw)

    motion_controller.set_target_position(target_position, target_angles)
    time.sleep(5)

    motion_controller.stop()

if __name__ == '__main__':
    main()
