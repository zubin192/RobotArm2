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
 
AK = ArmIK()
 
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}
 
__target_color = ('red',)
# Set the detection color
def setTargetColor(target_color):
    global __target_color
    __target_color = target_color
    return (True, ())
 
# Find the contour with the maximum area
# Parameter is a list of contours to compare
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
 

def reset():
    global count
    global track
    global _stop
    global get_roi
    global first_move
    global center_list
    global __isRunning
    global detect_color
    global action_finish
    global start_pick_up
    global __target_color
    global start_count_t1
 
    count = 0
    _stop = False
    track = False
    get_roi = False
    center_list = []
    first_move = True
    __target_color = ()
    detect_color = 'None'
    action_finish = True
    start_pick_up = False
    start_count_t1 = True
 
# App initialization call
def init():
    print("ColorTracking Init")
    initMove()
 
# App start call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")
 
# App stop call
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Stop")
 
# App exit call
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Exit")
 
rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
world_x, world_y = 0, 0
# Mechanical arm movement thread
def move():
    global rect
    global track
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_color
    global action_finish
    global rotation_angle
    global world_X, world_Y
    global world_x, world_y
    global center_list, count
    global start_pick_up, first_move
 
    # Coordinates for placing different color blocks (x, y, z)
    coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
    }
    while True:
        if __isRunning:
            if first_move and start_pick_up:  # When the object is first detected
                action_finish = False
                set_rgb(detect_color)
                setBuzzer(0.1)
                result = AK.setPitchRangeMoving((world_X, world_Y - 2, 5), -90, -90, 0)  # Do not fill in the running time parameter, adapt the running time automatically
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                time.sleep(result[2] / 1000)  # The third item in the returned parameters is the time
                start_pick_up = False
                first_move = False
                action_finish = True
            elif not first_move and not unreachable:  # If it is not the first time the object is detected
                set_rgb(detect_color)
                if track:  # If it is the tracking phase
                    if not __isRunning:  # Check for stop and exit flags
                        continue
                    AK.setPitchRangeMoving((world_x, world_y - 2, 5), -90, -90, 0, 20)
                    time.sleep(0.02)
                    track = False
                if start_pick_up:  # If the object has not moved for a while, start picking up
                    action_finish = False
                    if not __isRunning:  # Check for stop and exit flags
                        continue
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # Open the gripper
                    # Calculate the angle the gripper needs to rotate
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.8)
 
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)  # Lower the height
                    time.sleep(2)
 
                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1, 500)  # Close the gripper
                    time.sleep(1)
 
                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  # Lift the mechanical arm
                    time.sleep(1)
 
                    if not __isRunning:
                        continue
                    # Classify and place blocks of different colors
                    result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)
                    time.sleep(result[2] / 1000)
 
                    if not __isRunning:
                        continue
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)
 
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
 
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)
 
                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 200, 500)  # Open the gripper, put down the object
                    time.sleep(0.8)
 
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)
 
                    initMove()  # Return to the initial position
                    time.sleep(1.5)
 
                    detect_color = 'None'
                    first_move = True
                    get_roi = False
                    action_finish = True
                    start_pick_up = False
                    set_rgb(detect_color)
                else:
                    time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)
 
# Run the sub-thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()
 
t1 = 0
roi = ()
last_x, last_y = 0, 0
def run(img):
    global roi
    global rect
    global count
    global track
    global get_roi
    global center_list
    global __isRunning
    global unreachable
    global detect_color
    global action_finish
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global world_x, world_y
    global start_count_t1, t1
    global start_pick_up, first_move
 
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
 
    if not __isRunning:
        return img
 
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    # If a certain area is recognized, keep detecting that area until there is none
    if get_roi and start_pick_up:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)
 
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert the image to LAB space
 
    area_max = 0
    areaMaxContour = 0
    if not start_pick_up:
        for i in color_range:
            if i in __target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # Bitwise operation on the original image and mask
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Opening operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # Closing operation
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find contours
                areaMaxContour, area_max = getAreaMaxContour(contours)  # Find the largest contour
        if area_max > 2500:  # If the largest area is found
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))
 
            roi = getROI(box)  # Get the ROI area
            get_roi = True
 
            img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # Get the center coordinates of the block
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size)  # Convert to real-world coordinates
 
            cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)  # Draw the center point
            distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2))  # Check if it has moved by comparing with the previous coordinates
            last_x, last_y = world_x, world_y
            track = True
            # Accumulate judgment
            if action_finish:
                if distance < 0.3:
                    center_list.extend((world_x, world_y))
                    count += 1
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 1.5:
                        rotation_angle = rect[2]
                        start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                        count = 0
                        center_list = []
                        start_pick_up = True
                else:
                    t1 = time.time()
                    start_count_t1 = True
                    count = 0
                    center_list = []
    return img
 
if __name__ == '__main__':
    init()
    start()
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