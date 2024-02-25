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

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__target_color = ('red',)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Function to set the target color
def setTargetColor(target_color):
    global __target_color
    __target_color = target_color
    return (True, ())

# Find the contour with the maximum area
def getAreaMaxContour(contours):
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = cv2.contourArea(c)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:
                area_max_contour = c

    return area_max_contour, contour_area_max

def run(img, roi):
    global __target_color
    global range_rgb

    frame = img.copy()
    frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    area_max = 0
    areaMaxContour = 0
    for i in color_range:
        if i in __target_color:
            detect_color = i
            frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)
    if area_max > 2500:
        rect = cv2.minAreaRect(areaMaxContour)
        box = np.int0(cv2.boxPoints(rect))

        img_centerx, img_centery = getCenter(rect, roi, size, square_length)
        world_x, world_y = convertCoordinate(img_centerx, img_centery, size)

        cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)

    return img

if __name__ == '__main__':
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            roi = None  # You need to define ROI or pass it here
            Frame = run(frame, roi)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
