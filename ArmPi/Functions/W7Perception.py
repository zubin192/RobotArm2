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


class ObjectTracker:
    def __init__(self):
        self.__isRunning = False
        self.__target_color = ('red',)
        self.rect = None
        self.size = (640, 480)
        self.rotation_angle = 0
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.get_roi = False
        self.last_x, self.last_y = 0, 0

    def start(self):
        self.__isRunning = True
        print("ColorTracking Start")

    def stop(self):
        self.__isRunning = False
        print("ColorTracking Stop")

    def exit(self):
        self.__isRunning = False
        print("ColorTracking Exit")

    def setTargetColor(self, target_color):
        self.__target_color = target_color
        return True

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

    def getMaskROI(self, img, roi, size):
        mask = np.zeros(size, dtype=np.uint8)
        cv2.fillPoly(mask, [roi], (255, 255, 255))
        masked_img = cv2.bitwise_and(img, img, mask=mask)
        return masked_img

    def getROI(self, box):
        x_min = min(box[:, 0])
        x_max = max(box[:, 0])
        y_min = min(box[:, 1])
        y_max = max(box[:, 1])
        roi = ((x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max))
        return np.array(roi)

    def getCenter(self, rect, roi, size, square_length):
        centerx = int((rect[0][0] + rect[1][0] + rect[2][0] + rect[3][0]) / 4)
        centery = int((rect[0][1] + rect[1][1] + rect[2][1] + rect[3][1]) / 4)
        return centerx, centery

    def convertCoordinate(self, img_centerx, img_centery, size):
        # Implement coordinate transformation here
        world_x = img_centerx
        world_y = img_centery
        return world_x, world_y

    def run(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        if not self.__isRunning:
            return img

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        if self.get_roi:
            self.get_roi = False
            frame_gb = self.getMaskROI(frame_gb, self.roi, self.size)

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        area_max = 0
        areaMaxContour = 0
        for i in self.color_range:
            if i in self.__target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, self.color_range[detect_color][0], self.color_range[detect_color][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.getAreaMaxContour(contours)
            if area_max > 2500:
                self.rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(self.rect))

                self.roi = self.getROI(box)
                self.get_roi = True

                img_centerx, img_centery = self.getCenter(self.rect, self.roi, self.size, square_length)
                self.world_x, self.world_y = self.convertCoordinate(img_centerx, img_centery, self.size)

                cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
                cv2.putText(img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')',
                            (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)

                distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))
                self.last_x, self.last_y = self.world_x, self.world_y

        return img


if __name__ == '__main__':
    tracker = ObjectTracker()
    tracker.start()
    tracker.setTargetColor(('red',))

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        processed_frame = tracker.run(frame)
        cv2.imshow('Frame', processed_frame)

        key = cv2.waitKey(1)
        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
