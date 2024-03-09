import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import Camera
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

class CircleDetector:
    def __init__(self):
        self.target_color = 'black'
        self.camera = Camera.Camera()
        self.camera.camera_open()
        self.size = (640, 480)

    def process_frame(self, frame):
        frame_resize = cv2.resize(frame, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        return frame_lab

    def draw_circles(self, frame, frame_lab):
        frame_mask = cv2.inRange(frame_lab, color_range[self.target_color][0], color_range[self.target_color][1])
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
        circles = cv2.HoughCircles(closed, cv2.HOUGH_GRADIENT, dp=1, minDist=100, param1=70, param2=20, minRadius=50, maxRadius=400)


        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center_x, center_y = convertCoordinate(i[0], i[1], self.size)
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
                cv2.putText(frame, self.target_color, (i[0], i[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 1)
                cv2.putText(frame, f'x: {center_x:.2f}, y: {center_y:.2f}', (i[0], i[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 1)

    def run(self):
        while True:
            img = self.camera.frame
            if img is not None:
                frame = img.copy()
                frame_lab = self.process_frame(frame)
                self.draw_circles(frame, frame_lab)
                cv2.imshow('Frame', frame)
                key = cv2.waitKey(1)
                if key == 27:  # ESC key to break
                    break

        self.camera.camera_close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = CircleDetector()
    detector.run()