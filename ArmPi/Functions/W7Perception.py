#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import numpy as np

__target_color = ('red',)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

def setTargetColor(target_color):
    global __target_color
    __target_color = target_color
    return True

def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = abs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:
                area_max_contour = c

    return area_max_contour, contour_area_max

def run_color_tracking(img):
    global __target_color

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_gb = cv2.GaussianBlur(img_copy, (11, 11), 11)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

    area_max = 0
    areaMaxContour = 0
    for i in range_rgb:
        if i in __target_color:
            detect_color = i
            frame_mask = cv2.inRange(frame_lab, range_rgb[detect_color][0], range_rgb[detect_color][1])
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)

    if area_max > 2500:
        rect = cv2.minAreaRect(areaMaxContour)
        box = np.int0(cv2.boxPoints(rect))
        cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)

    return img

if __name__ == '__main__':
    my_camera = cv2.VideoCapture(0)
    if not my_camera.isOpened():
        print("Error: Couldn't open the camera.")
        sys.exit()

    while True:
        ret, frame = my_camera.read()
        if not ret:
            print("Error: Couldn't read frame.")
            break

        frame_processed = run_color_tracking(frame)
        cv2.imshow('Color Tracking', frame_processed)

        key = cv2.waitKey(1)
        if key == 27:
            break

    my_camera.release()
    cv2.destroyAllWindows()
