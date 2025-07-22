import cv2
from .model import load_model

class YOLOOBB:
    def __init__(self, weights_path):
        self.model = load_model(weights_path)

    def detect(self, image):
        results = self.model.predict(image, verbose=False)
        return results[0]
