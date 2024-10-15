import json
import numpy as np
import os
from pyzxing import BarCodeReader
import time  # Import time module for measuring elapsed time

# Load COCO-style ground truth data
def load_coco_annotations(json_file):
    with open(json_file, 'r') as f:
        coco_data = json.load(f)
    return coco_data

# Calculate bounding box from four corner points (x, y)
def points_to_bounding_box(points):
    x_coordinates = [p[0] for p in points]
    y_coordinates = [p[1] for p in points]
    x_min = min(x_coordinates)
    y_min = min(y_coordinates)
    x_max = max(x_coordinates)
    y_max = max(y_coordinates)
    return (x_min, y_min, x_max, y_max)

# Calculate IoU (Intersection over Union) for two bounding boxes
def calculate_iou(boxA, boxB):
    xA_min, yA_min, xA_max, yA_max = boxA
    xB_min, yB_min, xB_max, yB_max = boxB

    xA = max(xA_min, xB_min)
    yA = max(yA_min, yB_min)
    xB = min(xA_max, xB_max)
    yB = min(yA_max, yB_max)

    intersection = max(0, xB - xA) * max(0, yB - yA)

    boxAArea = (xA_max - xA_min) * (yA_max - yA_min)
    boxBArea = (xB_max - xB_min) * (yB_max - yB_min)

    iou = intersection / float(boxAArea + boxBArea - intersection + 1e-6)
    return iou

# Match detected QR codes with ground truth QR codes using IoU-based matching algorithm
def match_qr_codes(detected_points_list, ground_truth_points_list, iou_threshold=0.5):
    matches = []
    unmatched_detections = set(range(len(detected_points_list)))  # Initialize all detections as unmatched
    unmatched_ground_truths = set(range(len(ground_truth_points_list)))  # Initialize all ground truths as unmatched
    
    for i, detected_points in enumerate(detected_points_list):
        detected_box = points_to_bounding_box(detected_points)
        for j, ground_truth_points in enumerate(ground_truth_points_list):
            ground_truth_box = points_to_bounding_box(ground_truth_points)
            iou = calculate_iou(detected_box, ground_truth_box)
            if iou > iou_threshold:
                matches.append((i, j))
                unmatched_detections.discard(i)  # Remove matched detection
                unmatched_ground_truths.discard(j)  # Remove matched ground truth
                
    return matches, len(unmatched_detections), len(unmatched_ground_truths)

# Use pyzxing library to read QR codes and evaluate them
def evaluate_barcode_reader(image_dir, annotation_file):
    reader = BarCodeReader()
    coco_data = load_coco_annotations(annotation_file)
    
    total = 0
    failed = 0
    detected_count = 0
    false_positives = 0  # Initialize false positive count
    false_negatives = 0  # Initialize false negative count
    processed_images = set()
    
    # Start measuring total elapsed time
    total_elapsed_time = 0  # Initialize total elapsed time

    # Create a mapping of image_id to annotations
    image_annotations = {}
    for annotation in coco_data['annotations']:
        image_id = annotation['image_id']
        if image_id not in image_annotations:
            image_annotations[image_id] = []
        image_annotations[image_id].append(annotation)

    # Iterate through all images in the dataset
    for image_info in coco_data['images']:
        image_id = image_info['id']
        annotations = image_annotations.get(image_id, [])

        if not annotations:
            continue

        image_file = os.path.join(image_dir, image_info['file_name'])
        
        # Skip already processed images
        if image_file in processed_images:
            continue
        
        processed_images.add(image_file)

        ground_truth_points_list = []
        
        # Extract ground truth segmentation points (if available)
        for annotation in annotations:
            if 'segmentation' in annotation:
                for segmentation in annotation['segmentation']:
                    if len(segmentation) == 8:
                        points = [
                            [segmentation[i], segmentation[i + 1]] for i in range(0, len(segmentation), 2)
                        ]
                        ground_truth_points_list.append(points)

        print(f"Processing image: {image_info['file_name']}")

        # Display ground truth QR code coordinates
        if ground_truth_points_list:
            print(f"Ground truth QR code coordinates: {ground_truth_points_list}")
        else:
            print("No ground truth QR codes found.")

        try:
            # Measure the start time for QR code detection
            start_time = time.perf_counter()

            # Detect QR codes in the image using pyzxing
            detection_results = reader.decode(image_file)

            # Measure the end time
            end_time = time.perf_counter()
            elapsed_time = end_time - start_time  # Calculate elapsed time
            total_elapsed_time += elapsed_time  # Add to total elapsed time

            detected_points_list = []
            # Store detected QR code points (if valid)
            for result in detection_results:
                if 'points' in result and len(result['points']) == 4:
                    detected_points_list.append([[point[0], point[1]] for point in result['points']])

            # Display detected QR code coordinates
            if detected_points_list:
                print(f"Detected QR code coordinates: {detected_points_list}")
            else:
                print("No QR codes detected.")

            # Print the time taken for detection
            print(f"Time taken for QR code detection: {elapsed_time:.6f} seconds\n")

            # Match detected QR codes with all ground truth QR codes
            matches, unmatched_detections, unmatched_ground_truths = match_qr_codes(detected_points_list, ground_truth_points_list)

            # Count successful matches and failures
            detected_count += len(matches)
            false_negatives += unmatched_ground_truths  # Unmatched ground truths are false negatives
            false_positives += len(detected_points_list) - len(matches)  # Calculate false positives

        except FileNotFoundError:
            # If the image file is not found, count all ground truth QR codes as failed
            failed += len(ground_truth_points_list)

        total += len(ground_truth_points_list)

    # Calculate average time per processed image
    average_time = total_elapsed_time / len(processed_images) if processed_images else 0

    # Calculate Precision, Recall, and F1-score
    precision = detected_count / (detected_count + false_positives) if detected_count + false_positives > 0 else 0
    recall = detected_count / total if total > 0 else 0
    f1_score = (2 * precision * recall) / (precision + recall) if precision + recall > 0 else 0

    # Display results
    print(f"\nTotal ground truth QR codes: {total}")
    print(f"Total detected valid QR codes: {detected_count}")
    print(f"False positives: {false_positives}")
    print(f"False negatives: {false_negatives}")
    print(f"Success Rate: {100 * detected_count / total:.2f}%")
    print(f"Precision: {precision:.2f}")
    print(f"Recall: {recall:.2f}")
    print(f"F1 Score: {f1_score:.2f}")
    print(f"Average time per image: {average_time:.6f} seconds")

# Example usage
image_directory = 'dataset\evaluation-dataset'
annotation_file = 'qr_coordinates_coco.json'
evaluate_barcode_reader(image_directory, annotation_file)
