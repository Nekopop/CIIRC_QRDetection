import json
import os
from pyzxing import BarCodeReader

# Function to load COCO-style annotation data
def load_coco_annotations(json_file):
    with open(json_file, 'r') as f:
        coco_data = json.load(f)
    return coco_data

# Function to detect QR codes in images and count the number detected
def count_detected_qr_codes(image_dir):
    reader = BarCodeReader()
    total_detected = 0

    # Check all files in the image directory
    for filename in os.listdir(image_dir):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            image_file = os.path.join(image_dir, filename)
            detection_results = reader.decode(image_file)

            # Count the number of detected QR codes
            detected_count = sum(1 for result in detection_results if result.get('format') == b'QR_CODE' and 'points' in result)
            total_detected += detected_count

            print(f"Image: {filename}, Detected QR Codes: {detected_count}")

    return total_detected

# Function to get the number of ground truth QR codes from the JSON file
def count_ground_truth_qr_codes(annotation_file):
    coco_data = load_coco_annotations(annotation_file)
    total_ground_truth = len(coco_data['annotations'])
    return total_ground_truth

# Example usage
image_directory = 'dataset\evaluation-dataset'  # Specify the directory where images are stored
annotation_file = 'qr_coordinates_coco.json'  # Specify the annotation file

# Count the number of detected QR codes
total_detected = count_detected_qr_codes(image_directory)

# Count the number of ground truth QR codes
total_ground_truth = count_ground_truth_qr_codes(annotation_file)

# Output the results
print(f"Total detected QR Codes in dataset: {total_detected}")
print(f"Total ground truth QR Codes in annotations: {total_ground_truth}")
