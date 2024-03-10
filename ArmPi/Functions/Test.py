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

    while True:
        servo = input("Enter the servo number (1-6) to move, or 'q' to quit: ")
        if servo == 'q':
            break

        pulse = int(input("Enter the pulse width for servo {}: ".format(servo)))

        if servo == '1':
            robotic_arm.move_servo1(pulse)
        elif servo == '2':
            robotic_arm.move_servo2(pulse)
        elif servo == '3':
            robotic_arm.move_servo3(pulse)
        elif servo == '4':
            robotic_arm.move_servo4(pulse)
        elif servo == '5':
            robotic_arm.move_servo5(pulse)
        elif servo == '6':
            robotic_arm.move_servo6(pulse)
        else:
            print("Invalid servo number. Please enter a number between 1 and 6.")

        time.sleep(1)

if __name__ == '__main__':
    main()