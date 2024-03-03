#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import threading
import math
import numpy as np
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

class Perception:
    def __init__(self):
        self.__target_color = ('red', 'blue', 'green')
        self.__isRunning = False
        self.rect = None
        self.size = (640, 480)
        self.my_camera = Camera.Camera()
        self.my_camera.camera_open()

    def setTargetColor(self, target_color):
        self.__target_color = target_color
        return (True, ())

    def getAreaMaxContour(self, contours):
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c

        return area_max_contour, contour_area_max

    def start(self):
        self.__isRunning = True
        print("ColorTracking Start")

    def run(self, img):
        positions = {'red': None, 'blue': None, 'green': None}
        locations = {'red': None, 'blue': None, 'green': None}
        rois = {'red': None, 'blue': None, 'green': None}
        get_rois = {'red': False, 'blue': False, 'green': False}

        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        if not self.__isRunning:
            return img

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        for i in color_range:
            if i in self.__target_color:
                detect_color = i
                if get_rois[detect_color]:
                    get_rois[detect_color] = False
                    frame_gb = getMaskROI(frame_gb, rois[detect_color], self.size)

                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.getAreaMaxContour(contours)
                if area_max > 2500:
                    self.rect = cv2.minAreaRect(areaMaxContour)
                    box = np.int0(cv2.boxPoints(self.rect))

                    rois[detect_color] = getROI(box)
                    get_rois[detect_color] = True

                    img_centerx, img_centery = getCenter(self.rect, rois[detect_color], self.size, square_length)
                    world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)

                    positions[detect_color] = (img_centerx, img_centery)
                    locations[detect_color] = (world_x, world_y)

                    cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
                    cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)

                    # Drawing a line across the detected block object along the X-axis
                    cv2.line(img, (min(box[:, 0]), img_centery), (max(box[:, 0]), img_centery), (255, 255, 255), 2)

        print('Positions:', positions)
        print('Locations:', locations)

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
    perception.start()
    perception.main_loop()
