#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import threading
import numpy as np
import ColorTracking
import Camera
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

# Initialize ArmIK and Camera
AK = ArmIK()
my_camera = Camera.Camera()
my_camera.camera_open()

# Set colors and parameters for perception
color_range = {
    'red':   [np.array([0, 160, 50]), np.array([10, 255, 255])],
    'green': [np.array([35, 100, 50]), np.array([90, 255, 255])],
    'blue':  [np.array([100, 100, 50]), np.array([140, 255, 255])],
}

__target_color = ('red',)

# Function to find the contour with maximum area
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

# Function to set the target color
def setTargetColor(target_color):
    global __target_color
    __target_color = target_color
    return (True, ())

# Function to run perception and arm movement
def run():
    global __target_color

    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

            area_max = 0
            areaMaxContour = 0

            for color in color_range:
                if color in __target_color:
                    frame_mask = cv2.inRange(frame_lab, color_range[color][0], color_range[color][1])
                    contours = cv2.findContours(frame_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                    areaMaxContour, area_max = getAreaMaxContour(contours)

            # Arm movement logic based on perception goes here

            cv2.imshow('Frame', frame)
            key = cv2.waitKey(1)
            if key == 27:
                break

if __name__ == '__main__':

    __target_color = ('red', )
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()