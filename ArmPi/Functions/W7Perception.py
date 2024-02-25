import cv2
import numpy as np
import math
import time
from typing import Dict, List, Tuple

# Constants
MIN_CONTOUR_AREA = 300
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.5
FONT_COLOR = (0, 255, 0)
FONT_THICKNESS = 2
RECTANGLE_THICKNESS = 2

class ObjectPerception:
    def __init__(self, color_ranges: Dict[str, Tuple[np.array, np.array]]):
        self.color_ranges = color_ranges
        self.__target_color = ('red',)

    def detect_objects(self, frame: np.array) -> Dict[str, List[Tuple[int, int, int, int]]]:
        frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

        detected_objects = {}
        for color, color_range in self.color_ranges.items():
            mask = cv2.inRange(frame_lab, color_range[0], color_range[1])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > MIN_CONTOUR_AREA]
            bounding_boxes = [cv2.boundingRect(cnt) for cnt in valid_contours]
            detected_objects[color] = bounding_boxes

        return detected_objects

    def draw_objects(self, frame: np.array, detected_objects: Dict[str, List[Tuple[int, int, int, int]]]) -> np.array:
        for color, bounding_boxes in detected_objects.items():
            for (x, y, w, h) in bounding_boxes:
                cv2.rectangle(frame, (x, y), (x + w, y + h), FONT_COLOR, RECTANGLE_THICKNESS)
                cv2.putText(frame, color, (x, y - 10), FONT, FONT_SCALE, FONT_COLOR, FONT_THICKNESS)

        return frame

    def set_target_color(self, target_color: str) -> Tuple[bool, Tuple]:
        self.__target_color = (target_color,)
        return True, ()

    def get_target_color(self) -> Tuple[str]:
        return self.__target_color

class ColorTracking:
    def __init__(self, perception: ObjectPerception):
        self.perception = perception
        self.__isRunning = False
        self._stop = False

    def init(self):
        pass

    def start(self):
        self.__isRunning = True

    def stop(self):
        self.__isRunning = False

    def exit(self):
        self._stop = True
        self.__isRunning = False

    def run(self, frame: np.array) -> np.array:
        if self.__isRunning:
            detected_objects = self.perception.detect_objects(frame)
            target_color = self.perception.get_target_color()[0]
            if target_color in detected_objects:
                for (x, y, w, h) in detected_objects[target_color]:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), FONT_COLOR, RECTANGLE_THICKNESS)
                    cv2.putText(frame, target_color, (x, y - 10), FONT, FONT_SCALE, FONT_COLOR, FONT_THICKNESS)
            return frame
        else:
            return frame

def main():
    color_ranges = {
        'red': (np.array([0, 150, 150]), np.array([10, 255, 255])),
        'green': (np.array([40, 50, 50]), np.array([80, 255, 255])),
        'blue': (np.array([100, 100, 100]), np.array([140, 255, 255]))
    }

    perception = ObjectPerception(color_ranges)
    perception.set_target_color('red')  # Set initial target color

    color_tracker = ColorTracking(perception)
    color_tracker.init()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error opening video capture")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error reading frame")
            break

        frame_with_objects = color_tracker.run(frame)

        cv2.imshow('Frame with Objects', frame_with_objects)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
