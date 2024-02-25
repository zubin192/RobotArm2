import cv2
import numpy as np
import math

class ObjectPerception:
    def __init__(self, color_ranges, range_rgb, size, square_length):
        self.color_ranges = color_ranges
        self.range_rgb = range_rgb
        self.size = size
        self.square_length = square_length

    def detect_and_label_objects(self, frame, __target_color):
        img_copy = frame.copy()
        img_h, img_w = frame.shape[:2]
        cv2.line(frame, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(frame, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        area_max = 0
        areaMaxContour = 0
        detect_color = None

        for color, color_range in self.color_ranges.items():
            if color in __target_color:
                detect_color = color
                frame_mask = cv2.inRange(frame_lab, color_range[0], color_range[1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = self.getAreaMaxContour(contours)

        if area_max > 2500:
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            roi = self.getROI(box)
            img_centerx, img_centery = self.getCenter(rect, roi, self.size, self.square_length)
            world_x, world_y = self.convertCoordinate(img_centerx, img_centery, self.size)

            cv2.drawContours(frame, [box], -1, self.range_rgb[detect_color], 2)
            cv2.putText(frame, '(' + str(world_x) + ',' + str(world_y) + ')',
                        (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[detect_color], 1)

            return frame, detect_color, world_x, world_y

        return frame, None, None, None

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

    def getROI(self, box):
        return None  # Implement your logic here

    def getCenter(self, rect, roi, size, square_length):
        return None, None  # Implement your logic here

    def convertCoordinate(self, img_centerx, img_centery, size):
        return None, None  # Implement your logic here

# Example usage:
if __name__ == "__main__":
    color_ranges = {
        'red': (np.array([0, 120, 80]), np.array([20, 255, 255])),
        'green': (np.array([40, 40, 40]), np.array([80, 255, 255])),
        'blue': (np.array([100, 100, 100]), np.array([140, 255, 255]))
    }

    range_rgb = {
        'red': (0, 0, 255),
        'blue': (255, 0, 0),
        'green': (0, 255, 0),
        'black': (0, 0, 0),
        'white': (255, 255, 255),
    }

    size = (640, 480)
    square_length = 0  # Set the appropriate value

    perception = ObjectPerception(color_ranges, range_rgb, size, square_length)

    # Capture video
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame, detect_color, world_x, world_y = perception.detect_and_label_objects(frame, ('red',))
        if detect_color:
            print(f"Detected {detect_color} block at ({world_x}, {world_y})")

        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
