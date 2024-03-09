#!/usr/bin/python3
import cv2
import numpy as np

class Perception:
    def __init__(self):
        self.size = (640, 480)
        self.my_camera = cv2.VideoCapture(0)  # Assuming camera index 0
        self.drink_can_color_range = {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])}

    def detect_drink_can(self, frame):
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frame_hsv, self.drink_can_color_range['lower'], self.drink_can_color_range['upper'])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            contour_area_max = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour_area_max)
            return (x + w // 2, y + h // 2)
        else:
            return None

    def main_loop(self):
        while True:
            ret, frame = self.my_camera.read()
            if ret:
                drink_can_pos = self.detect_drink_can(frame)
                if drink_can_pos:
                    print("Drink can found at coordinates:", drink_can_pos)
                else:
                    print("Drink can not found in the frame.")
                
                cv2.imshow('Frame', frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        
        self.my_camera.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    perception = Perception()
    perception.main_loop()
