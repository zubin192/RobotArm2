#!/usr/bin/python3
import cv2
import numpy as np

class Perception:
    def __init__(self):
        self.size = (320, 240)  # Smaller resolution for faster processing
        self.cap = cv2.VideoCapture(0)  # Open the default camera
        self.scale_factor = 0.5  # Scale factor for resizing

    def detect_black_circle(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(frame_gray, cv2.HOUGH_GRADIENT, dp=1, minDist=40, param1=50, param2=30, minRadius=10, maxRadius=100)
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle
                return (x - r, y - r, x + r, y + r)  # Return coordinates of the bounding box of the circle
        else:
            return None

    def run(self, frame):
        frame_resized = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
        black_circle_bbox = self.detect_black_circle(frame_resized)
        
        if black_circle_bbox:
            x1, y1, x2, y2 = black_circle_bbox
            x1 = int(x1 / self.scale_factor)
            y1 = int(y1 / self.scale_factor)
            x2 = int(x2 / self.scale_factor)
            y2 = int(y2 / self.scale_factor)
            cv2.circle(frame, ((x1 + x2) // 2, (y1 + y2) // 2), (x2 - x1) // 2, (0, 255, 0), 2)  # Draw a green circle

        return frame

    def main_loop(self):
        while True:
            ret, frame = self.cap.read()
            if ret:
                frame = self.run(frame)
                cv2.imshow('Frame', frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    perception = Perception()
    perception.main_loop()
