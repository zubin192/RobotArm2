import cv2
import numpy as np
from tensorflow.keras.models import load_model

class Perception:
    def __init__(self):
        self.my_camera = Camera.Camera()
        self.my_camera.camera_open()
        self.model = load_model('path_to_your_model.h5')  # Load your trained model

    def run(self, img):
        # Preprocess your image for your model
        img_preprocessed = preprocess(img)

        # Predict the bounding box of the can
        bounding_box = self.model.predict(img_preprocessed)

        # Draw the bounding box on the image
        cv2.rectangle(img, (bounding_box[0], bounding_box[1]), (bounding_box[2], bounding_box[3]), (0, 255, 0), 2)

        # Estimate the distance based on the size of the bounding box
        distance = estimate_distance(bounding_box)

        print('Distance:', distance)

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

if __name__ == '__main__':
    perception = Perception()
    perception.main_loop()