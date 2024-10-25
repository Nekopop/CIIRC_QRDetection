import cv2
import zxingcpp
import os
import time
import json


# Load COCO format annotation data
def load_coco_annotations(json_file):
    with open(json_file, 'r') as f:
        coco_data = json.load(f)
    return coco_data

# Calculate bounding box from four corner points
def points_to_bounding_box(points):
    x_coordinates = [p[0] for p in points]
    y_coordinates = [p[1] for p in points]
    x_min = min(x_coordinates)
    y_min = min(y_coordinates)
    x_max = max(x_coordinates)
    y_max = max(y_coordinates)
    return (x_min, y_min, x_max, y_max)

# Convert zxingcpp.Position object to a list of points
def position_to_points(position):
    return [
        [position.top_left.x, position.top_left.y],
        [position.top_right.x, position.top_right.y],
        [position.bottom_right.x, position.bottom_right.y],
        [position.bottom_left.x, position.bottom_left.y]
    ]

# Calculate Intersection over Union (IoU) between two bounding boxes
def calculate_iou(boxA, boxB):
    xA_min, yA_min, xA_max, yA_max = boxA
    xB_min, yB_min, xB_max, yB_max = boxB

    # Calculate the coordinates of the intersection rectangle
    xA = max(xA_min, xB_min)
    yA = max(yA_min, yB_min)
    xB = min(xA_max, xB_max)
    yB = min(yA_max, yB_max)

    # Calculate the area of intersection rectangle
    interArea = max(0, xB - xA) * max(0, yB - yA)

    # Calculate the area of both bounding boxes
    boxAArea = (xA_max - xA_min) * (yA_max - yA_min)
    boxBArea = (xB_max - xB_min) * (yB_max - yB_min)

    # Calculate the IoU
    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou

# Evaluate QRDetector and measure detection success rate
def evaluate_qr_detector(image_dir, annotation_file):
    # Load COCO format annotation data
    coco_data = load_coco_annotations(annotation_file)

    # Initialize counters and timers
    total_ground_truths, detected_count = 0, 0
    false_positives, false_negatives = 0, 0
    total_time = 0  # Total processing time
    image_count = 0  # Number of processed images
    decode_success_count = 0  # Number of successful decodes
    true_positives = 0  # Number of true positives

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

        # Collect QR code points from annotations
        for annotation in annotations:
            if 'segmentation' in annotation:
                for segmentation in annotation['segmentation']:
                    if len(segmentation) == 8:
                        points = [[segmentation[i], segmentation[i + 1]] for i in range(0, len(segmentation), 2)]
                        ground_truth_points_list.append(points)

        # Load the image and convert to RGB format
        image = cv2.cvtColor(cv2.imread(image_file), cv2.COLOR_BGR2RGB)
        print(f"Processing {image_info['file_name']}:")

        # Start timing the QR code detection
        start_time = time.perf_counter()

        barcodes = zxingcpp.read_barcodes(image, return_errors=True, binarizer=zxingcpp.LocalAverage)

        # Stop timing the QR code detection
        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        total_time += elapsed_time  # Add to total processing time
        image_count += 1  # Increment the number of processed images

        print(f"Time taken for detection: {elapsed_time:.6f} seconds")

        # Evaluate only QR codes
        qr_barcodes = [barcode for barcode in barcodes if barcode.format == zxingcpp.BarcodeFormat.QRCode]
        detected_points_list = [position_to_points(barcode.position) for barcode in qr_barcodes]

        # Match detected QR codes with ground truth
        matches = match_qr_codes(detected_points_list, ground_truth_points_list)

        # Count successful decodes and print decoded text
        for i, barcode in enumerate(qr_barcodes):
            if barcode.text and any(i == match[0] for match in matches):
                decode_success_count += 1
                print(f"Decoded text: {barcode.text}")

        true_positives += len(matches)  # Count only matched QR codes
        false_negatives += len(ground_truth_points_list) - len(matches)
        false_positives += len(detected_points_list) - len(matches)
        total_ground_truths += len(ground_truth_points_list)

        # Display progress for each image
        print(f"{len(ground_truth_points_list)} ground truths, {len(detected_points_list)} detected, {len(matches)} matched\n")

    # Display final results
    print(f"\nTotal ground truth QR codes: {total_ground_truths}")
    print(f"Total detected QR codes: {true_positives + false_positives}")
    print(f"False positives: {false_positives}")
    print(f"False negatives: {false_negatives}")

    # Calculate and display detection success rate
    if total_ground_truths > 0:
        detection_success_rate = (true_positives / total_ground_truths) * 100
        print(f"Detection Success Rate: {detection_success_rate:.2f}%")
        print(f"Calculation: {true_positives} / {total_ground_truths} * 100 = {detection_success_rate:.2f}%")

    # Calculate and display decoding success rate
    if true_positives > 0:
        decoding_success_rate = (decode_success_count / true_positives) * 100
        print(f"Decoding Success Rate: {decoding_success_rate:.2f}%")
        print(f"Calculation: {decode_success_count} / {true_positives} * 100 = {decoding_success_rate:.2f}%")

    # Calculate and display the ratio of decoded QR codes to total ground truth QR codes
    if total_ground_truths > 0:
        decoded_to_ground_truth_ratio = (decode_success_count / total_ground_truths) * 100
        print(f"Decoded-to-Ground-Truth Ratio: {decoded_to_ground_truth_ratio:.2f}%")
        print(f"Calculation: {decode_success_count} / {total_ground_truths} * 100 = {decoded_to_ground_truth_ratio:.2f}%")

    # Calculate Precision, Recall, and F1 Score
    if true_positives + false_positives > 0 and true_positives + false_negatives > 0:
        precision = true_positives / (true_positives + false_positives)
        recall = true_positives / (true_positives + false_negatives)
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

# Example usage
image_directory = 'dataset/evaluation-dataset'
annotation_file = 'qr_coordinates_coco.json'
evaluate_qr_detector(image_directory, annotation_file)