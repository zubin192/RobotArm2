import sys
import time
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

    pulse1 = 500  # initial pulse width for servo 1
    pulse2 = 500  # initial pulse width for servo 2
    

    # Here you can add your own logic to control the servos
    # For example, you can use a loop, user input, sensor data, etc.

    robotic_arm.move_servo(1, pulse1)
    robotic_arm.move_servo(2, pulse2)

if __name__ == '__main__':
    main()