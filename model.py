from ultralytics import YOLO

def load_model(weights_path="weights/yolov8_obb.pt"):
    model = YOLO(weights_path)
    return model
