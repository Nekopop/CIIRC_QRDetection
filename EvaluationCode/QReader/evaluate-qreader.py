import cv2
import json
import os
import numpy as np
from qreader import QReader  # Import the QReader library
from pathlib import Path
import time

# Load COCO-style ground truth data
def load_coco_annotations(json_file):
    with open(json_file, 'r') as f:
        return json.load(f)

# Calculate bounding box from four points
def points_to_bounding_box(points):
    x_coordinates = [p[0] for p in points]
    y_coordinates = [p[1] for p in points]
    return (min(x_coordinates), min(y_coordinates), max(x_coordinates), max(y_coordinates))

# Calculate Intersection over Union (IoU)
def calculate_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interArea = max(0, xB - xA) * max(0, yB - yA)
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

    return interArea / float(boxAArea + boxBArea - interArea + 1e-6)

# Match detected QR codes with ground truth using IoU
def match_qr_codes(detected_points_list, ground_truth_points_list, iou_threshold=0.5):
    matches = []
    for i, detected_points in enumerate(detected_points_list):
        detected_box = points_to_bounding_box(detected_points)
        for j, ground_truth_points in enumerate(ground_truth_points_list):
            ground_truth_box = points_to_bounding_box(ground_truth_points)
            iou = calculate_iou(detected_box, ground_truth_box)
            if iou > iou_threshold:
                matches.append((i, j))
    return matches

# Evaluate the QReader detector and measure detection and decoding success
def evaluate_qreader_detector(image_dir, annotation_file):
    qreader = QReader(model_size="l")
    coco_data = load_coco_annotations(annotation_file)

    total_ground_truths, detected_count, decoded_count = 0, 0, 0
    false_positives, false_negatives = 0, 0
    total_time = 0  # Initialize total time for processing
    image_count = 0  # Initialize count of processed images

    # Map image IDs to their annotations
    image_annotations = {}
    for annotation in coco_data['annotations']:
        image_id = annotation['image_id']
        image_annotations.setdefault(image_id, []).append(annotation)

    # Process each image in the dataset
    for image_info in coco_data['images']:
        image_id = image_info['id']
        annotations = image_annotations.get(image_id, [])

        image_file = os.path.join(image_dir, image_info['file_name'])
        ground_truth_points_list = []

        # Collect ground truth QR code points (segmentation) from annotations
        for annotation in annotations:
            if 'segmentation' in annotation:
                for segmentation in annotation['segmentation']:
                    if len(segmentation) == 8:
                        points = [[segmentation[i], segmentation[i + 1]] for i in range(0, len(segmentation), 2)]
                        ground_truth_points_list.append(points)

        # Read the image and convert it to RGB format
        image = cv2.cvtColor(cv2.imread(image_file), cv2.COLOR_BGR2RGB)
        print(f"Processing {image_info['file_name']}:")

        # Measure the start time for QR code detection and decoding
        start_time = time.perf_counter()

        # Perform QR code detection and decoding
        detection_results = qreader.detect_and_decode(image=image, return_detections=True)

        # Measure the end time
        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        total_time += elapsed_time  # Add elapsed time to the total processing time
        image_count += 1  # Increment the count of processed images

        print(f"Time taken for detection & decoding: {elapsed_time:.6f} seconds")

        detected_points_list = []
        decoded_contents = []
        decode_success_count = 0

        # Process detection results
        for result in detection_results:
            if isinstance(result, tuple):
                decoded_content = result  # QR code decoded content
                decoded_contents.append(decoded_content)
                for t in decoded_content:
                    if t is not None and t != '':
                        decode_success_count += 1  # Count successful decodings

            if isinstance(result, list):
                # Store detected QR code points (quad_xy)
                t = 0
                while t < len(result):
                    detected_points_list.append(result[t]['quad_xy'])
                    t = t + 1

        # Print the decoded QR code content
        for content in decoded_contents:
            print(f"Detected QR code content: {content}")

        # Match detected QR codes with ground truth
        matches = match_qr_codes(detected_points_list, ground_truth_points_list)

        detected_count += len(matches)
        decoded_count += decode_success_count
        false_negatives += len(ground_truth_points_list) - len(matches)
        false_positives += len(detected_points_list) - len(matches)
        total_ground_truths += len(ground_truth_points_list)

        # Display progress for each image
        print(f"{len(ground_truth_points_list)} ground truths, {len(detected_points_list)} detected, {len(matches)} matched, {decode_success_count} decoded\n")

    # Display final results
    print(f"\nTotal ground truth QR codes: {total_ground_truths}")
    print(f"Total detected QR codes: {detected_count}")
    print(f"Total decoded QR codes: {decoded_count}")
    print(f"False positives: {false_positives}")
    print(f"False negatives: {false_negatives}")

    # Calculate and display detection success rate
    if total_ground_truths > 0:
        detection_success_rate = (detected_count / total_ground_truths) * 100
        print(f"Detection Success Rate: {detection_success_rate:.2f}%")
        print(f"Calculation: {detected_count} / {total_ground_truths} * 100 = {detection_success_rate:.2f}%")

    # Calculate and display decoding success rate
    if detected_count > 0:
        decoding_success_rate = (decoded_count / detected_count) * 100
        print(f"Decoding Success Rate: {decoding_success_rate:.2f}%")
        print(f"Calculation: {decoded_count} / {detected_count} * 100 = {decoding_success_rate:.2f}%")

    # Calculate and display the ratio of decoded QR codes to total ground truth QR codes
    if total_ground_truths > 0:
        decoded_to_ground_truth_ratio = (decoded_count / total_ground_truths) * 100
        print(f"Decoded-to-Ground-Truth Ratio: {decoded_to_ground_truth_ratio:.2f}%")
        print(f"Calculation: {decoded_count} / {total_ground_truths} * 100 = {decoded_to_ground_truth_ratio:.2f}%")

    # Calculate Precision, Recall, and F1 Score
    if detected_count > 0:
        precision = detected_count / (detected_count + false_positives)
        recall = detected_count / (detected_count + false_negatives)
        if precision + recall > 0:
            f1_score = 2 * (precision * recall) / (precision + recall)
        else:
            f1_score = 0.0
        print(f"Precision: {precision:.4f}")
        print(f"Recall: {recall:.4f}")
        print(f"F1 Score: {f1_score:.4f}")

    # Calculate and display the average processing time per image
    if image_count > 0:
        avg_time = total_time / image_count
        print(f"Average detection & decoding time per image: {avg_time:.6f} seconds")

# Example usage
image_directory = 'dataset\evaluation-dataset'
annotation_file = 'qr_coordinates_coco.json'
evaluate_qreader_detector(image_directory, annotation_file)
