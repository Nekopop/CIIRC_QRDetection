from qrdet import QRDetector
import cv2
import glob
from pathlib import Path


def process_image(input_path):
    image = cv2.imread(filename=input_path)
    if image is None:
        print(f"Failed to load image: {input_path}")
        return None

    detections = detector.detect(image=image, is_bgr=True)

    # Draw the detections
    for detection in detections:
        x1, y1, x2, y2 = detection['bbox_xyxy']
        confidence = detection['confidence']
        segmenation_xy = detection['quad_xy']
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), color=(0, 255, 0), thickness=2)
        cv2.putText(image, f'{confidence:.2f}', (int(x1), int(y1) - 10), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1, color=(0, 255, 0), thickness=2)
    return image



if __name__ == "__main__":

    detector = QRDetector(model_size='s')
    images_root = "yolo-qr-detector/data"

    paths = []
    for file in glob.glob(f'{images_root}/rgb*.png'):
        ff = Path(file)
        fname = ff.name
        parent = ff.parent.name

        input_path = str(file)
        paths.append(input_path)

    # paths = ["data/rgb20.png", "data/rgb21.png", "data/rgb22.png", "data/rgb23.png"]

    for input_path in paths:
        print("Processing:", input_path)
        image = process_image(input_path)

        # Save the results
        cv2.imshow('QR Detection', image)
        cv2.waitKey()    
