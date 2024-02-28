# Import the necessary libraries
import time
import HiwonderSDK.Board as Board

servo1 = 500 
servo2 = 500
servo3 = 500
servo4 = 500
servo5 = 500

Board.setBusServoPulse(1, servo1) # Servo 1
Board.setBusServoPulse(2, servo2) # Servo 2
Board.setBusServoPulse(3, servo3) # Servo 3
Board.setBusServoPulse(4, servo4) # Servo 4
Board.setBusServoPulse(5, servo5) # Servo 5

time.sleep(1)

servo1 = 600
servo2 = 600
servo3 = 600
servo4 = 600
servo5 = 600

Board.setBusServoPulse(1, servo1) # Servo 1
Board.setBusServoPulse(2, servo2) # Servo 2
Board.setBusServoPulse(3, servo3) # Servo 3
Board.setBusServoPulse(4, servo4) # Servo 4
Board.setBusServoPulse(5, servo5) # Servo 5

time.sleep(1)