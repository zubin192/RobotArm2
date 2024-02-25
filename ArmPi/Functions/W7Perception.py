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
        self.__camera = Camera.Camera()  # Initialize camera

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

    def run(self, frame):
        self.start()
        self.__target_color = ('red',)  # Set default target color
        self.__camera.camera_open()  # Open camera
        img = frame.copy()
        img = self.detect_color(img)
        cv2.imshow('Frame', img)
        key = cv2.waitKey(1)
        if key == 27:
            self.__camera.camera_close()  # Close camera
            cv2.destroyAllWindows()

    def detect_color(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        if not self.__is_running:
            return img

        frame_resize = cv2.resize(img_copy, self.__size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        area_max = 0
        area_max_contour = None
        for i in range_rgb:
            if i in self.__target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                area_max_contour, area_max = self.get_area_max_contour(contours)

            if area_max > 2500:
                rect = cv2.minAreaRect(area_max_contour)
                box = np.int0(cv2.boxPoints(rect))

                roi = self.get_roi(box)
                get_roi = True

                img_centerx, img_centery = self.get_center(rect, roi, self.__size, square_length)
                world_x, world_y = self.convert_coordinate(img_centerx, img_centery, self.__size)

                cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
                cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')',
                            (min(box[0, 0], box[2, 0]), box[2, 1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            range_rgb[detect_color], 1)
                distance = math.sqrt(pow(world_x - self.__last_x, 2) + pow(world_y - self.__last_y, 2))
                self.__last_x, self.__last_y = world_x, world_y
                self.__track = True
                if distance < 0.3:
                    self.__center_list.extend((world_x, world_y))
                    self.__count += 1
                    if self.__start_count_t1:
                        self.__start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 1.5:
                        self.__rotation_angle = rect[2]
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
        return img

if __name__ == '__main__':
    tracker = ColorTracker()  # Instantiate the ColorTracker class
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            tracker.run(frame)  # Call the run method of the ColorTracker instance
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
