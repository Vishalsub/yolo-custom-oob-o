import cv2
import time
from yolov8_obb.inference import YOLOOBB
from realsense.camera import RealSenseCamera
from realsense.serial_comm import SerialConnection

def draw_obb(result, frame):
    for obb in result.obb:
        # OBB format: (xc, yc, w, h, angle), draw as rotated rect
        rect = ((obb[0], obb[1]), (obb[2], obb[3]), obb[4])
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
    return frame


def main():
    cam = RealSenseCamera()
    model = YOLOOBB(weights_path="weights/yolov8_obb.pt")
    serial = SerialConnection("/dev/ttyUSB0")

    try:
        while True:
            color_image, _ = cam.get_frame()
            if color_image is None:
                continue

            result = model.detect(color_image)

            if result.obb:
                obb = result.obb[0]  # example: first detection
                x, y = int(obb[0]), int(obb[1])
                serial.send(f"DETECTED,{x},{y}")
                print(f"[Serial] Sent coordinates: {x}, {y}")

            cv2.imshow("YOLO-OBB Detection", color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        serial.close()
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
