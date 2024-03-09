#!/usr/bin/python3
import cv2
import numpy as np

class Perception:
    def __init__(self):
        self.size = (640, 480)
        self.my_camera = cv2.VideoCapture(0)  # Assuming camera index 0
        # Define parameters for detecting circles
        self.circle_detection_params = dict(dp=1, minDist=20, param1=50, param2=30, minRadius=10, maxRadius=100)

    def detect_black_circle(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(frame_gray, cv2.HOUGH_GRADIENT, **self.circle_detection_params)
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle
                return (x - r, y - r, x + r, y + r)  # Return coordinates of the bounding box of the circle
        else:
            return None

    def main_loop(self):
        while True:
            ret, frame = self.my_camera.read()
            if ret:
                black_circle_bbox = self.detect_black_circle(frame)
                if black_circle_bbox:
                    x1, y1, x2, y2 = black_circle_bbox
                    cv2.circle(frame, ((x1 + x2) // 2, (y1 + y2) // 2), (x2 - x1) // 2, (0, 255, 0), 2)  # Draw a green circle
                    print("Black circle found at center coordinates:", (x1 + x2) // 2, (y1 + y2) // 2)
                else:
                    print("Black circle not found in the frame.")
                
                cv2.imshow('Frame', frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        
        self.my_camera.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    perception = Perception()
    perception.main_loop()
