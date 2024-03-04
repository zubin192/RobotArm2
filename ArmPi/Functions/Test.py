#!/usr/bin/python3
# coding=utf8

import sys
import threading
import time
import cv2
import numpy as np
import HiwonderSDK.Board as Board
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *

class RoboticArm:
    def __init__(self):
        self.servo1 = 500
        self.AK = ArmIK()

    def init_move(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def move_arm(self, target_position, pitch, roll, yaw):
        return self.AK.setPitchRangeMoving(target_position, pitch, roll, yaw, 1500)

    def open_gripper(self):
        Board.setBusServoPulse(1, self.servo1 - 280, 500)

    def close_gripper(self):
        Board.setBusServoPulse(1, self.servo1, 500)

class RoboticArmMotionControl:
    def __init__(self):
        self._stop = False
        self._is_running = False
        self._thread = threading.Thread(target=self._move)
        self._thread.setDaemon(True)

        # Initialize other necessary variables for motion control
        self._target_coordinates = None
        self._target_location = None
        self._action_finish = True
        self.robotic_arm = RoboticArm()

    def start(self):
        self._stop = False
        self._is_running = True
        self._thread.start()

    def stop(self):
        self._stop = True
        self._is_running = False

    def set_target_coordinates(self, coordinates, target_location=None):
        self._target_coordinates = coordinates
        self._target_location = target_location

    def _move(self):
        while True:
            if self._is_running:
                if self._target_coordinates and self._action_finish:
                    self._action_finish = False
                    x, y, z = self._target_coordinates
                    # Open gripper before moving to pick up the object
                    self.robotic_arm.open_gripper()
                    result = self.robotic_arm.move_arm((x, y, z), -90, -90, 0)
                    if result is not None:
                        time.sleep(result[2] / 1000)
                    # Close gripper after picking up the object
                    self.robotic_arm.close_gripper()
                    self._target_coordinates = None
                    self._action_finish = True
                elif self._target_location and self._action_finish:
                    self._action_finish = False
                    x, y, z = self._target_location
                    result = self.robotic_arm.move_arm((x, y, z), -90, -90, 0)
                    if result is not None:
                        time.sleep(result[2] / 1000)
                    # Open gripper to release the object
                    self.robotic_arm.open_gripper()
                    time.sleep(1)  # Delay to ensure object is released
                    # Return to initial position after placing the object
                    self.robotic_arm.init_move()
                    time.sleep(2)  # Wait for the arm to reach the initial position
                    self._target_location = None
                    self._action_finish = True
            else:
                if self._stop:
                    self._stop = False
                    self._is_running = False
                time.sleep(0.01)

class Perception:
    def __init__(self):
        self.__target_color = ('red', 'blue', 'green')
        self.__isRunning = False
        self.rect = None
        self.size = (640, 480)
        self.my_camera = Camera.Camera()
        self.my_camera.camera_open()

    def setTargetColor(self, target_color):
        self.__target_color = target_color
        return (True, ())

    def getAreaMaxContour(self, contours):
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c

        return area_max_contour, contour_area_max

    def start(self):
        self.__isRunning = True
        print("ColorTracking Start")

    def run(self, img):
        positions = {'red': None, 'blue': None, 'green': None}
        locations = {'red': None, 'blue': None, 'green': None}
        rois = {'red': None, 'blue': None, 'green': None}
        get_rois = {'red': False, 'blue': False, 'green': False}

        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        if not self.__isRunning:
            return img

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        for i in color_range:
            if i in self.__target_color:
                detect_color = i
                if get_rois[detect_color]:
                    get_rois[detect_color] = False
                    frame_gb = getMaskROI(frame_gb, rois[detect_color], self.size)

                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.getAreaMaxContour(contours)
                if area_max > 2500:
                    self.rect = cv2.minAreaRect(areaMaxContour)
                    box = np.int0(cv2.boxPoints(self.rect))

                    rois[detect_color] = getROI(box)
                    get_rois[detect_color] = True

                    img_centerx, img_centery = getCenter(self.rect, rois[detect_color], self.size, square_length)
                    world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)

                    positions[detect_color] = (img_centerx, img_centery)
                    locations[detect_color] = (world_x, world_y)

                    cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
                    cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)

        print('Positions:', positions)
        print('Locations:', locations)

        return img

    def main_loop(self):
        while True:
            img = self.my_camera.frame
            if img is not None:
                frame = img.copy()
                Frame = self.run(frame)
                cv2.imshow('Frame', Frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        self.my_camera.camera_close()
        cv2.destroyAllWindows()

def main():
    # Create robotic arm motion control instance
    motion_controller = RoboticArmMotionControl()

    # Initialize arm movement
    motion_controller.robotic_arm.init_move()
    time.sleep(2)  # Wait for the arm to reach the initial position

    # Start the motion controller
    motion_controller.start()

    # Create perception instance
    perception = Perception()
    perception.start()
    perception.main_loop()

if __name__ == '__main__':
    main()
