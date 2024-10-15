from qrdet import QRDetector
import cv2
import glob
from pathlib import Path
import time
import numpy as np  # Used for calculating statistical information

# Process image and detect QR codes
def process_image(input_path):
    image = cv2.imread(filename=input_path)
    if image is None:
        print(f"Failed to load image: {input_path}")
        return None, 0.0  # Return 0 if processing fails

    start_time = time.perf_counter()  # Record start time of processing

    detections = detector.detect(image=image, is_bgr=True)

    end_time = time.perf_counter()  # Record end time of processing
    processing_time = end_time - start_time  # Calculate processing time

    # Draw detection results
    for detection in detections:
        x1, y1, x2, y2 = detection['bbox_xyxy']
        confidence = detection['confidence']
        segmentation_xy = detection['quad_xy']
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), color=(0, 255, 0), thickness=2)
        cv2.putText(image, f'{confidence:.2f}', (int(x1), int(y1) - 10), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1, color=(0, 255, 0), thickness=2)

    return image, processing_time  # Return both processed image and time taken


if __name__ == "__main__":
    detector = QRDetector(model_size='l')
    images_root = "dataset\evaluation-dataset"

    paths = []
    for file in glob.glob(f'{images_root}/*'):
        ff = Path(file)
        fname = ff.name
        parent = ff.parent.name

        input_path = str(file)
        paths.append(input_path)

    processing_times = []  # List to save processing times for each image

    # Measure and output processing time for each image
    for input_path in paths:
        print("Processing:", input_path)

        # Get processed image and processing time
        image, processing_time = process_image(input_path)
        processing_times.append(processing_time)  # Add processing time to list
        print(f"Processing time for {input_path}: {processing_time:.4f} seconds")

    cv2.destroyAllWindows()

    # Calculate statistics for processing times
    processing_times = np.array(processing_times)
    avg_time = np.mean(processing_times)  # Average processing time
    max_time = np.max(processing_times)   # Maximum processing time
    min_time = np.min(processing_times)   # Minimum processing time
    total_time = np.sum(processing_times)  # Total processing time

    # Display statistical information
    print("\nProcessing Time Statistics:")
    print(f"Total number of images processed: {len(processing_times)}")
    print(f"Total processing time: {total_time:.4f} seconds")
    print(f"Average processing time: {avg_time:.4f} seconds")
    print(f"Max processing time: {max_time:.4f} seconds")
    print(f"Min processing time: {min_time:.4f} seconds")
