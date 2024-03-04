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
        self.AK.setPitchRangeMoving(target_position, pitch, roll, yaw, 1500)

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
        self._target_release_coordinates = None
        self._action_finish = True
        self._gripped_object = False
        self.robotic_arm = RoboticArm()

    def start(self):
        self._stop = False
        self._is_running = True
        self._thread.start()

    def stop(self):
        self._stop = True
        self._is_running = False

    def set_target_coordinates(self, coordinates):
        self._target_coordinates = coordinates

    def set_target_release_coordinates(self, coordinates):
        self._target_release_coordinates = coordinates

    def _move(self):
        while True:
            if self._is_running:
                if self._target_coordinates and self._action_finish:
                    self._action_finish = False
                    x, y, z = self._target_coordinates
                    result = self.robotic_arm.move_arm((x, y, z), -90, -90, 0)
                    if result is not None:
                        time.sleep(result[2] / 1000)
                    self._target_coordinates = None
                    self._action_finish = True
            else:
                if self._stop:
                    self._stop = False
                    self._is_running = False
                time.sleep(0.01)


def main():
    # Create robotic arm motion control instance
    motion_controller = RoboticArmMotionControl()
    motion_controller.start()

    # Initialize arm movement
    motion_controller.robotic_arm.init_move()
    time.sleep(2)  # Wait for the arm to reach the initial position

    # Get the target position from the user
    x = float(input("Enter the x-coordinate to pick up the object: "))
    y = float(input("Enter the y-coordinate to pick up the object: "))
    z = float(input("Enter the z-coordinate to pick up the object: "))
    target_position = (x, y, z)

    # Move the arm to the specified position to pick up the object
    motion_controller.set_target_coordinates(target_position)

    # Wait until the object is gripped
    while not motion_controller._gripped_object:
        time.sleep(0.1)

    # Get the target release position from the user
    x_release = float(input("Enter the x-coordinate to release the object: "))
    y_release = float(input("Enter the y-coordinate to release the object: "))
    z_release = float(input("Enter the z-coordinate to release the object: "))
    target_release_position = (x_release, y_release, z_release)

    # Move the arm to the specified release position
    motion_controller.set_target_release_coordinates(target_release_position)

    # Stop the motion controller
    motion_controller.stop()

if __name__ == '__main__':
    main()
