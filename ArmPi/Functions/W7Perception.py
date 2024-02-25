import cv2
import numpy as np

class ObjectPerception:
    def __init__(self, color_ranges):
        self.color_ranges = color_ranges

    def detect_objects(self, frame):
        # Convert frame to LAB color space
        frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

        detected_objects = {}
        for color, color_range in self.color_ranges.items():
            # Apply color thresholding
            mask = cv2.inRange(frame_lab, color_range[0], color_range[1])

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Filter contours by area
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 300]

            # Find bounding boxes for valid contours
            bounding_boxes = [cv2.boundingRect(cnt) for cnt in valid_contours]

            detected_objects[color] = bounding_boxes

        return detected_objects

    def draw_objects(self, frame, detected_objects):
        for color, bounding_boxes in detected_objects.items():
            for (x, y, w, h) in bounding_boxes:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return frame

# Example usage
if __name__ == "__main__":
    color_ranges = {
        'red': (np.array([0, 120, 80]), np.array([20, 255, 255])),
        'green': (np.array([40, 40, 40]), np.array([80, 255, 255])),
        'blue': (np.array([100, 100, 100]), np.array([140, 255, 255]))
    }

    perception = ObjectPerception(color_ranges)

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detected_objects = perception.detect_objects(frame)
        frame_with_objects = perception.draw_objects(frame, detected_objects)

        cv2.imshow('Frame with Objects', frame_with_objects)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
