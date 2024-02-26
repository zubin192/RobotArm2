#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibrati

class Perception:
    def __init__(self):
        self.target_colors = {'red': (0, 0, 255), 'blue': (255, 0, 0), 'green': (0, 255, 0)}
        self.is_running = False
        self.camera = cv2.VideoCapture(0)  # Initialize the camera

    def start(self):
        self.is_running = True

    def stop(self):
        self.is_running = False

    def find_blocks(self, frame):
        detected_blocks = []

        for color_name, color_value in self.target_colors.items():
            lower_bound = np.array(color_value, dtype=np.uint8)
            upper_bound = np.array(color_value, dtype=np.uint8)

            # Threshold the image to get only specified color
            mask = cv2.inRange(frame, lower_bound, upper_bound)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter contours based on area to eliminate noise
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 1000:  # Adjust the minimum area as needed
                    # Calculate centroid of the contour
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        detected_blocks.append((color_name, (cx, cy)))

        return detected_blocks

    def run(self):
        while self.is_running:
            ret, frame = self.camera.read()  # Capture frame from camera
            if not ret:
                break

            blocks = self.find_blocks(frame)
            
            # Draw rectangles around detected blocks and display coordinates
            for color, (cx, cy) in blocks:
                cv2.rectangle(frame, (cx - 20, cy - 20), (cx + 20, cy + 20), self.target_colors[color], 2)
                cv2.putText(frame, f'{color} ({cx}, {cy})', (cx - 20, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.target_colors[color], 2)

            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    perception = Perception()
    perception.start()
    perception.run()
