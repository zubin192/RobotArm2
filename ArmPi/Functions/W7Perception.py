#!/usr/bin/python3
# coding=utf8

import cv2
import numpy as np
import time
import math
from Camera import Camera
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board


class ColorTracker:
    def __init__(self):
        self.__target_color = ('red',)
        self.__isRunning = False
        self.__roi = ()
        self.__rect = None
        self.__size = (640, 480)
        self.__world_x, self.__world_y = 0, 0
        self.__last_x, self.__last_y = 0, 0
        self.__center_list = []
        self.__count = 0
        self.__start_count_t1 = True
        self.__t1 = 0

    def set_target_color(self, target_color):
        self.__target_color = target_color

    def start(self):
        self.__isRunning = True

    def stop(self):
        self.__isRunning = False

    def run(self, img):
        if not self.__isRunning:
            return img

        frame_resize = cv2.resize(img.copy(), self.__size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        area_max = 0
        areaMaxContour = 0
        for color in color_range:
            if color in self.__target_color:
                detect_color = color
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.getAreaMaxContour(contours)

        if area_max > 2500:
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))
            roi = self.getROI(box)

            img_centerx, img_centery = self.getCenter(rect, roi, self.__size, square_length)
            world_x, world_y = self.convertCoordinate(img_centerx, img_centery, self.__size)

            cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')',
                        (min(box[0, 0], box[2, 0]), box[2, 1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, range_rgb[detect_color], 1)

            distance = math.sqrt(pow(world_x - self.__last_x, 2) + pow(world_y - self.__last_y, 2))
            self.__last_x, self.__last_y = world_x, world_y

            if distance < 0.3:
                self.__center_list.extend((world_x, world_y))
                self.__count += 1
                if self.__start_count_t1:
                    self.__start_count_t1 = False
                    self.__t1 = time.time()
                if time.time() - self.__t1 > 1.5:
                    rotation_angle = rect[2]
                    self.__start_count_t1 = True
                    self.__world_x, self.__world_y = np.mean(np.array(self.__center_list).reshape(self.__count, 2), axis=0)
                    self.__count = 0
                    self.__center_list = []
            else:
                self.__t1 = time.time()
                self.__start_count_t1 = True
                self.__count = 0
                self.__center_list = []

        return img

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c

        return area_max_contour, contour_area_max

    def getROI(self, box):
        # Implement your getROI function here
        pass

    def getCenter(self, rect, roi, size, square_length):
        # Implement your getCenter function here
        pass

    def convertCoordinate(self, img_centerx, img_centery, size):
        # Implement your convertCoordinate function here
        pass


def main():
    tracker = ColorTracker()
    tracker.start()

    my_camera = Camera()
    my_camera.camera_open()

    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame = tracker.run(frame)
            cv2.imshow('Frame', frame)
            key = cv2.waitKey(1)
            if key == 27:
                break

    my_camera.camera_close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
