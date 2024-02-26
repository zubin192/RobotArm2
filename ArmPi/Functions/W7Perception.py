#!/usr/bin/python3
# coding=utf8
import cv2
import numpy as np

class Perception:
    def __init__(self):
        self.color_range = {
            'red': ([0, 120, 120], [10, 255, 255]),
            'blue': ([110, 50, 50], [130, 255, 255]),
            'green': ([50, 50, 50], [70, 255, 255])
        }

    def _getAreaMaxContour(self, contours):
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area_temp = cv2.contourArea(c)
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c

        return area_max_contour

    def _getMaskROI(self, frame, roi, size):
        x, y, w, h = roi
        mask_roi = np.zeros(size, dtype=np.uint8)
        mask_roi[y:y+h, x:x+w] = frame[y:y+h, x:x+w]
        return mask_roi

    def colorTracking(self, img):
        img_h, img_w = img.shape[:2]
        size = (640, 480)
        frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        detected_colors = {}

        for color, (lower, upper) in self.color_range.items():
            mask = cv2.inRange(frame_lab, np.array(lower), np.array(upper))
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            areaMaxContour = self._getAreaMaxContour(contours)
            
            if areaMaxContour is not None:
                rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(rect))
                x, y, w, h = cv2.boundingRect(areaMaxContour)
                roi = (x, y, w, h)
                mask_roi = self._getMaskROI(mask, roi, size)
                detected_colors[color] = {
                    'contour': areaMaxContour,
                    'rect': rect,
                    'roi': roi,
                    'mask_roi': mask_roi
                }

        return detected_colors

if __name__ == '__main__':
    perception = Perception()
    my_camera = cv2.VideoCapture(0)
    if not my_camera.isOpened():
        print("Error: Unable to open camera.")
        exit()
    
    while True:
        ret, frame = my_camera.read()
        if not ret:
            print("Error: Unable to capture frame.")
            break
        
        detected_colors = perception.colorTracking(frame)

        for color, data in detected_colors.items():
            contour = data['contour']
            rect = data['rect']
            roi = data['roi']
            mask_roi = data['mask_roi']
            # Process detected colors as needed
            
        cv2.imshow('Frame', frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
    
    my_camera.release()
    cv2.destroyAllWindows()
