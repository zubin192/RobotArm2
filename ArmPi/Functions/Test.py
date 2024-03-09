#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import numpy as np
import Camera
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

class Perception:
    def __init__(self):
        self.size = (640, 480)
        self.my_camera = Camera.Camera()
        self.my_camera.camera_open()
        self.locations = {'black_circle': None}

    def detect_black_circle(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(frame_gray, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=10, maxRadius=100)
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = circle
                return (x - r, y - r, x + r, y + r)  # Return coordinates of the bounding box of the circle
        else:
            return None

    def run(self, img):
        self.locations = {'black_circle': None}
        
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        black_circle_bbox = self.detect_black_circle(frame_resize)
        
        if black_circle_bbox:
            x1, y1, x2, y2 = black_circle_bbox
            cv2.circle(img, ((x1 + x2) // 2, (y1 + y2) // 2), (x2 - x1) // 2, (0, 255, 0), 2)  # Draw a green circle
            world_x, world_y = convertCoordinate((x1 + x2) // 2, (y1 + y2) // 2, self.size)
            self.locations['black_circle'] = (world_x, world_y)

        return img

    def main_loop(self):
        while True:
            img = self.my_camera.frame
            if img is not None:
                frame = img.copy()
                Frame = self.run(frame)
                cv2.imshow('Frame', Frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        self.my_camera.camera_close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    perception = Perception()
    perception.main_loop()
