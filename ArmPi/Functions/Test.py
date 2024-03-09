#!/usr/bin/python3
import cv2
import numpy as np
import threading
import time

class Perception:
    def __init__(self):
        self.size = (160, 120)  # Further reduced resolution
        self.cap = cv2.VideoCapture(0)  # Open the default camera
        self.scale_factor = 0.25  # Scale factor for resizing
        self.skip_frames = 2  # Skip every other frame
        self.frame_processed = True
        self.lock = threading.Lock()

    def preprocess_frame(self, frame):
        frame_resized = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
        frame_gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)
        _, frame_threshold = cv2.threshold(frame_gray, 30, 255, cv2.THRESH_BINARY)  # Threshold to detect the black dot
        frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))  # Morphological closing to fill gaps
        return frame_resized, frame_threshold

    def detect_black_circle(self, frame):
        circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, dp=1, minDist=30, param1=50, param2=30, minRadius=5, maxRadius=50)
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle
                return (x - r, y - r, x + r, y + r)  # Return coordinates of the bounding box of the circle
        else:
            return None

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if ret:
                frame_resized, frame_threshold = self.preprocess_frame(frame)
                black_circle_bbox = self.detect_black_circle(frame_threshold)
                
                if black_circle_bbox:
                    x1, y1, x2, y2 = black_circle_bbox
                    x1 = int(x1 / self.scale_factor)
                    y1 = int(y1 / self.scale_factor)
                    x2 = int(x2 / self.scale_factor)
                    y2 = int(y2 / self.scale_factor)
                    with self.lock:
                        cv2.circle(frame_resized, ((x1 + x2) // 2, (y1 + y2) // 2), (x2 - x1) // 2, (0, 255, 0), 2)  # Draw a green circle
                
                cv2.imshow('Frame', frame_resized)
                
                key = cv2.waitKey(1)
                if key == 27:
                    break
        self.cap.release()
        cv2.destroyAllWindows()

    def main_loop(self):
        while True:
            if self.frame_processed:
                ret, frame = self.cap.read()
                if ret:
                    self.frame_processed = False
                    t = threading.Thread(target=self.process_frame, args=(frame,))
                    t.start()
            else:
                time.sleep(0.001)  # Sleep briefly to reduce CPU usage

    def process_frame(self, frame):
        frame_resized, frame_threshold = self.preprocess_frame(frame)
        black_circle_bbox = self.detect_black_circle(frame_threshold)
        
        if black_circle_bbox:
            x1, y1, x2, y2 = black_circle_bbox
            x1 = int(x1 / self.scale_factor)
            y1 = int(y1 / self.scale_factor)
            x2 = int(x2 / self.scale_factor)
            y2 = int(y2 / self.scale_factor)
            with self.lock:
                cv2.circle(frame_resized, ((x1 + x2) // 2, (y1 + y2) // 2), (x2 - x1) // 2, (0, 255, 0), 2)  # Draw a green circle
        self.frame_processed = True

if __name__ == '__main__':
    perception = Perception()
    t = threading.Thread(target=perception.run)
    t.start()
    perception.main_loop()
