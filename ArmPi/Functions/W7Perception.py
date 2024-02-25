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

class ImageProcessor:
    def __init__(self, target_color, size):
        self.target_color = target_color
        self.size = size
        self.get_roi = False
        self.start_pick_up = False
        self.last_x, self.last_y = 0, 0
        self.center_list = []
        self.count = 0
        self.start_count_t1 = True
        self.t1 = time.time()

    def resize_and_blur(self, img):
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        return frame_gb

    def convert_to_lab(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    def detect_color(self, img):
        frame_mask = cv2.inRange(img, color_range[self.target_color][0], color_range[self.target_color][1])
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
        return closed

    def getAreaMaxContour(contours):
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
    
    def draw_and_label(self, img, areaMaxContour):
        rect = cv2.minAreaRect(areaMaxContour)
        box = np.int0(cv2.boxPoints(rect))
        roi = getROI(box)
        self.get_roi = True
        img_centerx, img_centery = getCenter(rect, roi, self.size, square_length)
        world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)
        cv2.drawContours(img, [box], -1, range_rgb[self.target_color], 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[self.target_color], 1)
        return img, world_x, world_y

    def process_image(self, img):
        img_copy = self.resize_and_blur(img)
        img_lab = self.convert_to_lab(img_copy)
        detected = self.detect_color(img_lab)
        areaMaxContour, area_max = self.find_largest_contour(detected)
        if area_max > 2500:
            img, world_x, world_y = self.draw_and_label(img, areaMaxContour)
            distance = math.sqrt(pow(world_x - self.last_x, 2) + pow(world_y - self.last_y, 2))
            self.last_x, self.last_y = world_x, world_y
            if action_finish:
                if distance < 0.3:
                    self.center_list.extend((world_x, world_y))
                    self.count += 1
                    if self.start_count_t1:
                        self.start_count_t1 = False
                        self.t1 = time.time()
                    if time.time() - self.t1 > 1.5:
                        rotation_angle = rect[2]
                        self.start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                        self.count = 0
                        self.center_list = []
                        self.start_pick_up = True
                else:
                    self.t1 = time.time()
                    self.start_count_t1 = True
                    self.count = 0
                    self.center_list = []
        return img

def main():
    # Initialize the camera
    my_camera = Camera.Camera()
    my_camera.camera_open()

    # Initialize the image processor with the target color and size
    my_processor = ImageProcessor('red', (640, 480))

    while True:
        img = my_camera.frame
        if img is not None:
            result = my_processor.process_image(img)
            cv2.imshow('Result', result)
            if cv2.waitKey(1) == 27:  # Press 'Esc' to exit
                break

    cv2.destroyAllWindows()
    my_camera.camera_close()

if __name__ == "__main__":
    main()