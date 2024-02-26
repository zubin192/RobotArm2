#!/usr/bin/python3
# coding=utf8

import cv2
import numpy as np
import math
import time

class ColorPerception:
    def __init__(self):
        self.__isRunning = False
        self.__target_color = ('red',)
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        self.color_range = {
            'red': [(0, 151, 100), (255, 255, 255)], 
            'green': [(0, 0, 0), (255, 115, 255)], 
            'blue': [(0, 0, 0), (255, 255, 110)], 
            'black': [(0, 0, 0), (56, 255, 255)], 
            'white': [(193, 0, 0), (255, 250, 255)], 
        }
        self.rect = None
        self.size = (640, 480)
        self.rotation_angle = 0
        self.unreachable = False
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.get_roi = False
        self.last_x, self.last_y = 0, 0
        self.start_count_t1 = 0
        self.t1 = 0
        self.roi = ()
        self.center_list = []
        self.count = 0
        self.track = False
        self.detect_color = None

    def setTargetColor(self, target_color):
        self.__target_color = target_color

    def start(self):
        self.__isRunning = True
        print("ColorTracking Start")

    def stop(self):
        self.__isRunning = False
        print("ColorTracking Stop")

    def exit(self):
        self.__isRunning = False
        print("ColorTracking Exit")

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
        for i in self.range_rgb:
            if i in self.__target_color:
                self.detect_color = i
                frame_mask = cv2.inRange(frame_lab, self.color_range[self.detect_color][0], self.color_range[self.detect_color][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.getAreaMaxContour(contours)

            if area_max > 2500:
                self.rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(self.rect))

                self.roi = self.getROI(box)
                self.get_roi = True

                img_centerx, img_centery = self.getCenter(self.rect, self.roi, self.size)
                self.world_x, self.world_y = self.convertCoordinate(img_centerx, img_centery, self.size)

                cv2.drawContours(img, [box], -1, self.range_rgb[self.detect_color], 2)
                cv2.putText(img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')',
                            (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[self.detect_color], 1)

                distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))
                self.last_x, self.last_y = self.world_x, self.world_y
                self.track = True

                if distance < 0.3:
                    self.center_list.extend((self.world_x, self.world_y))
                    self.count += 1
                    if self.start_count_t1:
                        self.start_count_t1 = False
                        self.t1 = time.time()
                    if time.time() - self.t1 > 1.5:
                        self.rotation_angle = self.rect[2]
                        self.start_count_t1 = True
                        self.world_X, self.world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                        self.count = 0
                        self.center_list = []
                        self.start_pick_up = True
                else:
                    self.t1 = time.time()
                    self.start_count_t1 = True
                    self.count = 0
                    self.center_list = []

        return img

    def getMaskROI(self, frame, roi, size):
        x1, y1, x2, y2 = roi
        mask = np.zeros_like(frame)
        cv2.rectangle(mask, (x1, y1), (x2, y2), (255, 255, 255), thickness=cv2.FILLED)
        masked_frame = cv2.bitwise_and(frame, mask)
        return masked_frame

    def getROI(self, box):
        x_min = min(box[:, 0])
        x_max = max(box[:, 0])
        y_min = min(box[:, 1])
        y_max = max(box[:, 1])
        return (x_min, y_min, x_max, y_max)

    def getCenter(self, rect, roi, size):
        box = np.int0(cv2.boxPoints(rect))
        x_min, y_min, x_max, y_max = roi
        img_centerx = int((x_min + x_max) / 2)
        img_centery = int((y_min + y_max) / 2)
        return img_centerx, img_centery

    def convertCoordinate(self, img_centerx, img_centery, size):
        world_x = img_centerx - size[0] / 2
        world_y = size[1] / 2 - img_centery
        return world_x, world_y

if __name__ == '__main__':
    color_perception = ColorPerception()
    color_perception.start()

    while True:
        # Here you would capture the frame from your camera
        # Replace this line with your camera capture code
        img = np.zeros((480, 640, 3), dtype=np.uint8)  # Placeholder for captured frame

        frame = img.copy()
        processed_frame = color_perception.run(frame)
        cv2.imshow('Processed Frame', processed_frame)
        key = cv2.waitKey(1)
        if key == 27:
            break

    cv2.destroyAllWindows()
