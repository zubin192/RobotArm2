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
        return self.AK.setPitchRangeMoving(target_position, pitch, roll, yaw, 1500)

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

    def set_target_coordinates(self, coordinates):
        self._target_coordinates = coordinates

    def set_target_location(self, location):
        self._target_location = location

    def _move(self):
        while True:
            if self._is_running:
                if self._target_coordinates and self._action_finish:
                    self._action_finish = False
                    x, y, z = self._target_coordinates
                    # Open gripper before moving to pick up the object
                    self.robotic_arm.open_gripper()
                    result = self.robotic_arm.move_arm((x, y, z), -90, -90, 0)
                    if result is not None:
                        time.sleep(result[2] / 1000)
                    # Close gripper after picking up the object
                    self.robotic_arm.close_gripper()
                    self._target_coordinates = None
                    self._action_finish = True
                elif self._target_location and self._action_finish:
                    self._action_finish = False
                    x, y, z = self._target_location
                    result = self.robotic_arm.move_arm((x, y, z), -90, -90, 0)
                    if result is not None:
                        time.sleep(result[2] / 1000)
                    # Open gripper to release the object
                    self.robotic_arm.open_gripper()
                    time.sleep(1)  # Delay to ensure object is released
                    self.robotic_arm.close_gripper()
                    self._target_location = None
                    self._action_finish = True
            else:
                if self._stop:
                    self._stop = False
                    self._is_running = False
                time.sleep(0.01)

def main():
    # Create robotic arm motion control instance
    motion_controller = RoboticArmMotionControl()

    # Initialize arm movement
    motion_controller.robotic_arm.init_move()
    time.sleep(2)  # Wait for the arm to reach the initial position

    # Start the motion controller
    motion_controller.start()

    # Get the target position from the user
    x = float(input("Enter the x-coordinate to pick up: "))
    y = float(input("Enter the y-coordinate to pick up: "))
    z = float(input("Enter the z-coordinate to pick up: "))
    target_position = (x, y, z)

    # Set the target coordinates for picking up the object
    motion_controller.set_target_coordinates(target_position)

    # Wait for some time to allow the arm to pick up the object
    time.sleep(5)

    # Hardcoded target location to place the object
    target_location = (-15 + 0.5, 12 - 0.5, 1.5)

    # Set the target location to place the object
    motion_controller.set_target_location(target_location)

    # Wait for some time to allow the arm to place the object
    time.sleep(5)

    # Stop the motion controller
    motion_controller.stop()

if __name__ == '__main__':
    main()
