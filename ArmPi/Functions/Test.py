import sys
import cv2
import time
import threading
import numpy as np
import math
from ArmPi import Camera
from ArmPi import LABConfig
from ArmPi.ArmIK import Transform, ArmMoveIK
from ArmPi import HiwonderSDK

class RoboticArm:
    def __init__(self):
        self.servo1 = 500
        self.AK = ArmMoveIK.ArmIK()

    def init_move(self):
        HiwonderSDK.Board.setBusServoPulse(1, self.servo1 - 50, 300)
        HiwonderSDK.Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def move_arm(self, target_position, pitch, roll, yaw):
        return self.AK.setPitchRangeMoving(target_position, pitch, roll, yaw, 1500)

    def open_gripper(self):
        HiwonderSDK.Board.setBusServoPulse(1, self.servo1 - 280, 500)

    def close_gripper(self):
        HiwonderSDK.Board.setBusServoPulse(1, self.servo1, 500)

class PerceptionAndMotion:
    def __init__(self):
        self._stop = False
        self._is_running = False
        self._thread = threading.Thread(target=self._main_loop)
        self._thread.setDaemon(True)

        self.robotic_arm = RoboticArm()
        self.perception = Perception()

    def start(self):
        self._stop = False
        self._is_running = True
        self._thread.start()

    def stop(self):
        self._stop = True
        self._is_running = False

    def _main_loop(self):
        self.perception.start()
        while True:
            img = self.perception.my_camera.frame
            if img is not None:
                frame = img.copy()
                frame_with_detection = self.perception.run(frame)
                cv2.imshow('Frame', frame_with_detection)
                key = cv2.waitKey(1)
                if key == 27:
                    break
                elif key == ord('p'):
                    target_position, _ = self.perception.get_target_coordinates()
                    self.robotic_arm.open_gripper()
                    result = self.robotic_arm.move_arm(target_position, -90, -90, 0)
                    if result is not None:
                        time.sleep(result[2] / 1000)
                    self.robotic_arm.close_gripper()
                    self.robotic_arm.init_move()
                    time.sleep(2)
        self.perception.my_camera.camera_close()
        cv2.destroyAllWindows()

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

        for i in LABConfig.color_range:
            if i in self.__target_color:
                detect_color = i
                if get_rois[detect_color]:
                    get_rois[detect_color] = False
                    frame_gb = self.getMaskROI(frame_gb, rois[detect_color], self.size)

                frame_mask = cv2.inRange(frame_lab, LABConfig.color_range[detect_color][0], LABConfig.color_range[detect_color][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.getAreaMaxContour(contours)
                if area_max > 2500:
                    self.rect = cv2.minAreaRect(areaMaxContour)
                    box = np.int0(cv2.boxPoints(self.rect))

                    rois[detect_color] = self.getROI(box)
                    get_rois[detect_color] = True

                    img_centerx, img_centery = self.getCenter(self.rect, rois[detect_color], self.size, LABConfig.square_length)
                    world_x, world_y = self.convertCoordinate(img_centerx, img_centery, self.size)

                    positions[detect_color] = (img_centerx, img_centery)
                    locations[detect_color] = (world_x, world_y)

                    cv2.drawContours(img, [box], -1, LABConfig.range_rgb[detect_color], 2)
                    cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, LABConfig.range_rgb[detect_color], 1)

        print('Positions:', positions)
        print('Locations:', locations)

        return img

    def get_target_coordinates(self):
        positions = {'red': None, 'blue': None, 'green': None}
        locations = {'red': None, 'blue': None, 'green': None}

        for i in self.__target_color:
            if positions[i] is not None:
                return locations[i], positions[i]
        return None, None

    def getMaskROI(self, frame_gb, roi, size):
        mask = np.zeros(size, np.uint8)
        cv2.drawContours(mask, [roi], -1, (255, 255, 255), -1)
        res = cv2.bitwise_and(frame_gb, frame_gb, mask=mask)
        return res

    def getROI(self, box):
        roi = np.array([[min(box[:, 0]), min(box[:, 1])],
                        [max(box[:, 0]), min(box[:, 1])],
                        [max(box[:, 0]), max(box[:, 1])],
                        [min(box[:, 0]), max(box[:, 1])]])
        return roi

    def getCenter(self, rect, roi, size, square_length):
        center = (rect[0][0], rect[0][1])
        return center[0], center[1]

    def convertCoordinate(self, img_centerx, img_centery, size):
        return img_centerx, img_centery

def main():
    perception_and_motion = PerceptionAndMotion()
    perception_and_motion.start()
    time.sleep(2)
    perception_and_motion.stop()

if __name__ == '__main__':
    main()
