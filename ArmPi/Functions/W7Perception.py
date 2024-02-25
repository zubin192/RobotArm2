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
from CameraCalibration.CalibrationConfig import *
import numpy as np

class Perception:
    def __init__(self):
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        self.__target_color = ('red',)
        self.__isRunning = False
        self.rect = None
        self.size = (640, 480)
        self.rotation_angle = 0
        self.unreachable = False
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.t1 = 0
        self.roi = ()
        self.get_roi = False
        self.last_x, self.last_y = 0, 0

    def setTargetColor(self, target_color):
        self.__target_color = target_color
        return (True, ())

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # Iterate through all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # Only contours with an area greater than 300 are considered valid to filter out noise
                    area_max_contour = c

        return area_max_contour, contour_area_max  # Return the largest contour

    def start(self):
        self.__isRunning = True
        print("ColorTracking Start")

    def stop(self):
        self.__isRunning = False
        print("ColorTracking Stop")

    def exit(self):
        self.__isRunning = False
        print("ColorTracking Exit")

    def run(self, img):
        org_frame = img.copy()  # Copy the image

        frame_resize = cv2.resize(org_frame, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        # Convert image to HSV
        frame_hsv = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2HSV)

        color_lower = np.array([self.range_rgb[self.__target_color][0], 43, 46])
        color_upper = np.array([self.range_rgb[self.__target_color][1], 255, 255])
        frame_mask = cv2.inRange(frame_hsv, color_lower, color_upper)
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        areaMaxContour, area_max = self.getAreaMaxContour(contours)

        if areaMaxContour is not None:
            if area_max > 2500:  # Have found the largest area
                rect = cv2.minAreaRect(areaMaxContour)
                self.rect = np.int0(cv2.boxPoints(rect))

                roi = getROI([self.rect[0], self.rect[2]], org_frame)
                self.roi = roi
                self.get_roi = True

                img_centerx, img_centery = getCenter([self.rect[0], self.rect[2]], org_frame, square_length)
                self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, org_frame)

        return img

if __name__ == '__main__':
    perception = Perception()
    perception.start()
    perception.setTargetColor(('red', ))
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = perception.run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()