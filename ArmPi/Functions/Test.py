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

    def move_arm_to_xyz(self, target_position):
        # Calculate inverse kinematics solution for XYZ coordinates
        result = self.AK.setPitchTargetPos(target_position)
        if result is not None:
            time.sleep(result[2] / 1000)
            return True
        else:
            return False

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

    def set_target_coordinates(self, coordinates, target_location=None):
        self._target_coordinates = coordinates
        self._target_location = target_location

    def _move(self):
        while True:
            if self._is_running:
                if self._target_coordinates and self._action_finish:
                    self._action_finish = False
                    self.robotic_arm.open_gripper()
                    if self.robotic_arm.move_arm_to_xyz(self._target_coordinates):
                        self.robotic_arm.close_gripper()
                    self._target_coordinates = None
                    self._action_finish = True
                elif self._target_location and self._action_finish:
                    self._action_finish = False
                    if self.robotic_arm.move_arm_to_xyz(self._target_location):
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

    x = float(input("Enter the x-coordinate to point to: "))
    y = float(input("Enter the y-coordinate to point to: "))
    z = float(input("Enter the z-coordinate to point to: "))
    target_position = (x, y, z)

    motion_controller.set_target_coordinates(target_position)
    time.sleep(5)

    target_location = (-15 + 0.5, 12 - 0.5, 1.5)
    motion_controller.set_target_coordinates(None, target_location)
    time.sleep(5)

    motion_controller.stop()

if __name__ == '__main__':
    main()
