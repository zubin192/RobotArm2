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
        self.servo2 = 500
        self.AK = ArmIK()

    def init_move(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, self.servo2, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), 30, 30, 1500)

    def move_arm(self, target_position):
        return self.AK.setPitchRangeMoving(target_position, 90, 90, 0, 1500)

    def move_servo(self, servo, pulse):
        Board.setBusServoPulse(servo, pulse)

def main():
    robotic_arm = RoboticArm()
    robotic_arm.init_move()
    time.sleep(2)

    print("Control the servos using the keyboard:")
    print("w/s: Increase/decrease pulse width for servo 1")
    print("a/d: Increase/decrease pulse width for servo 2")
    print("Press 'esc' to exit")

    pulse1 = 500  # initial pulse width for servo 1
    pulse2 = 500  # initial pulse width for servo 2

    while True:
        if keyboard.is_pressed('w'): pulse1 += 10
        elif keyboard.is_pressed('s'): pulse1 -= 10
        elif keyboard.is_pressed('a'): pulse2 += 10
        elif keyboard.is_pressed('d'): pulse2 -= 10
        elif keyboard.is_pressed('esc'): break

        robotic_arm.move_servo(1, pulse1)
        robotic_arm.move_servo(2, pulse2)
        time.sleep(0.1)

if __name__ == '__main__':
    main()