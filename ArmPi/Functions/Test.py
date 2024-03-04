import time
import threading
from Perception import Perception  # Assuming Perception class is saved in Perception.py
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board

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

        self._target_coordinates = None
        self._target_location = None
        self._action_finish = True
        self.robotic_arm = RoboticArm()
        self.perception = Perception()

    def start(self):
        self._stop = False
        self._is_running = True
        self._thread.start()

    def stop(self):
        self._stop = True
        self._is_running = False

    def _move(self):
        while True:
            if self._is_running:
                if self._target_coordinates and self._action_finish:
                    self._action_finish = False
                    x, y, _ = self._target_coordinates  # Ignore Z coordinate, always set to 0.5
                    self.robotic_arm.open_gripper()
                    result = self.robotic_arm.move_arm((x, y, 0.5), -90, -90, 0)
                    if result is not None:
                        time.sleep(result[2] / 1000)
                    self.robotic_arm.close_gripper()
                    self._target_coordinates = None
                    self._action_finish = True
                elif self._target_location and self._action_finish:
                    self._action_finish = False
                    x, y, _ = self._target_location  # Ignore Z coordinate, always set to 0.5
                    result = self.robotic_arm.move_arm((x, y, 0.5), -90, -90, 0)
                    if result is not None:
                        time.sleep(result[2] / 1000)
                    self.robotic_arm.open_gripper()
                    time.sleep(1)
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
    motion_controller.perception.start()
    motion_controller.perception.main_loop()

    # The following code will be executed after the perception loop exits
    # Assuming you have some way to get the target coordinates from Perception class
    # and set them using motion_controller.set_target_coordinates() method

if __name__ == '__main__':
    main()
