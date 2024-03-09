#!/usr/bin/python3
import cv2
import numpy as np

class Perception:
    def __init__(self):
        self.size = (640, 480)
        self.my_camera = cv2.VideoCapture(0)  # Assuming camera index 0
        # Define a wider range to capture various shades of white
        self.white_color_range = {'lower': np.array([0, 0, 200]), 'upper': np.array([180, 50, 255])}

    def detect_white_object(self, frame):
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frame_hsv, self.white_color_range['lower'], self.white_color_range['upper'])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            contour_area_max = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour_area_max)
            return (x, y, x + w, y + h)  # Return coordinates of the bounding box
        else:
            return None

    def main_loop(self):
        while True:
            ret, frame = self.my_camera.read()
            if ret:
                white_obj_bbox = self.detect_white_object(frame)
                if white_obj_bbox:
                    x1, y1, x2, y2 = white_obj_bbox
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw a green bounding box
                    print("White object found at coordinates:", (x1 + x2) // 2, (y1 + y2) // 2)
                else:
                    print("White object not found in the frame.")
                
                cv2.imshow('Frame', frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        
        self.my_camera.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    perception = Perception()
    perception.main_loop()
