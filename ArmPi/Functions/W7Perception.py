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

class ColorTracker:
    range_rgb = {
        'red': (0, 0, 255),
        'blue': (255, 0, 0),
        'green': (0, 255, 0),
        'black': (0, 0, 0),
        'white': (255, 255, 255),
    }

    def __init__(self, target_color='red'):
        self.__target_color = target_color
        self.__is_running = False
        self.__roi = ()
        self.__rect = None
        self.__size = (640, 480)
        self.__world_x, self.__world_y = 0, 0
        self.__last_x, self.__last_y = 0, 0
        self.__center_list = []
        self.__count = 0
        self.__track = False
        self.__start_count_t1 = False
        self.__rotation_angle = 0
        self.__start_pick_up = False
        self.__camera = cv2.VideoCapture(0)
        
    def start(self):
        self.__is_running = True
        print("ColorTracking Start")

    def stop(self):
        self.__is_running = False
        print("ColorTracking Stop")

    def exit(self):
        self.__is_running = False
        print("ColorTracking Exit")

    def set_target_color(self, target_color):
        self.__target_color = target_color

    def get_area_max_contour(self, contours):
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c

        return area_max_contour, contour_area_max

    def run(self):
        while True:
            ret, frame = self.__camera.read()
            if not ret:
                print("Error: Unable to capture frame")
                break

            if not self.__is_running:
                break

            frame_resize = cv2.resize(frame, self.__size, interpolation=cv2.INTER_NEAREST)
            frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

            frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

            area_max = 0
            area_max_contour = None
            for i in self.range_rgb:
                if i in self.__target_color:
                    detect_color = i
                    frame_mask = cv2.inRange(frame_lab, self.range_rgb[detect_color][0], self.range_rgb[detect_color][1])
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                    area_max_contour, area_max = self.get_area_max_contour(contours)

                if area_max > 2500:
                    self.__rect = cv2.minAreaRect(area_max_contour)
                    box = np.int0(cv2.boxPoints(self.__rect))

                    roi = self.get_roi(box)
                    img_centerx, img_centery = self.get_center(self.__rect, roi, self.__size, square_length)
                    self.__world_x, self.__world_y = self.convert_coordinate(img_centerx, img_centery, self.__size)

                    cv2.drawContours(frame, [box], -1, self.range_rgb[detect_color], 2)
                    cv2.putText(frame, '(' + str(self.__world_x) + ',' + str(self.__world_y) + ')',
                                (min(box[0, 0], box[2, 0]), box[2, 1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                self.range_rgb[detect_color], 1)
                    distance = math.sqrt(pow(self.__world_x - self.__last_x, 2) + pow(self.__world_y - self.__last_y, 2))
                    self.__last_x, self.__last_y = self.__world_x, self.__world_y
                    self.__track = True
                    if distance < 0.3:
                        self.__center_list.extend((self.__world_x, self.__world_y))
                        self.__count += 1
                        if self.__start_count_t1:
                            self.__start_count_t1 = False
                            t1 = time.time()
                        if time.time() - t1 > 1.5:
                            self.__rotation_angle = self.__rect[2]
                            self.__start_count_t1 = True
                            self.__world_X, self.__world_Y = np.mean(np.array(self.__center_list).reshape(self.__count, 2), axis=0)
                            self.__count = 0
                            self.__center_list = []
                            self.__start_pick_up = True
                    else:
                        t1 = time.time()
                        self.__start_count_t1 = True
                        self.__count = 0
                        self.__center_list = []

            cv2.imshow('Frame', frame)
            key = cv2.waitKey(1)
            if key == 27:
                break

        self.__camera.release()
        cv2.destroyAllWindows()

    def get_roi(self, box):
        return cv2.boundingRect(np.array([box]))

    def get_center(self, rect, roi, size, square_length):
        box = np.int0(cv2.boxPoints(rect))
        roi_img = cv2.bitwise_and(frame_resize, frame_resize, mask=roi)
        roi_gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(roi_gray, 1, 255, cv2.THRESH_BINARY)
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                img_centerx = cX + roi[0]
                img_centery = cY + roi[1]
                cv2.circle(frame_resize, (img_centerx, img_centery), 7, (255, 255, 255), -1)
                cv2.putText(frame_resize, "center", (img_centerx - 20, img_centery - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                return img_centerx, img_centery
        return None, None

    def convert_coordinate(self, img_centerx, img_centery, size):
        world_x = (img_centerx - size[0] / 2) * square_length
        world_y = (size[1] / 2 - img_centery) * square_length
        return world_x, world_y

if __name__ == '__main__':
    tracker = ColorTracker()
    tracker.start()
    tracker.run()
