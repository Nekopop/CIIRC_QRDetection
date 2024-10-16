import cv2
import json
import os
import time
from qrdet import QRDetector 

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
    
    for i, detected_points in enumerate(detected_points_list):
        detected_box = points_to_bounding_box(detected_points)
        for j, ground_truth_points in enumerate(ground_truth_points_list):
            ground_truth_box = points_to_bounding_box(ground_truth_points)
            iou = calculate_iou(detected_box, ground_truth_box)
            if iou > iou_threshold:
                matches.append((i, j))
                
    return matches

# Use QRDet library to detect QR codes and evaluate them
def evaluate_qrdet_detector(image_dir, annotation_file):
    detector = QRDetector(model_size='s')  
    coco_data = load_coco_annotations(annotation_file)
    
    total = 0
    failed = 0
    detected_count = 0
    false_positives = 0
    false_negatives = 0
    processed_images = set()
    total_time = 0  # Initialize total processing time
    image_count = 0  # Initialize image count

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
            # Detect QR codes in the image using QRDet
            image = cv2.imread(image_file)

            # Measure the start time for QR code detection
            start_time = time.perf_counter()

            detection_results = detector.detect(image=image, is_bgr=True)

            # Measure the end time
            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            total_time += elapsed_time  # Add elapsed time to the total processing time
            image_count += 1  # Increment the count of processed images

            print(f"Time taken for detection in {image_info['file_name']}: {elapsed_time:.6f} seconds")

            detected_points_list = []
            # Store detected QR code points (if valid)
            for result in detection_results:
                if 'quad_xy' in result and len(result['quad_xy']) == 4:
                    detected_points_list.append([[point[0], point[1]] for point in result['quad_xy']])

            # Display detected QR code coordinates
            if detected_points_list:
                print(f"Detected QR code coordinates: {detected_points_list}\n")
            else:
                print("No QR codes detected.\n")

            # Match detected QR codes with all ground truth QR codes
            matches = match_qr_codes(detected_points_list, ground_truth_points_list)

            # Count successful matches and failures
            detected_count += len(matches)
            false_negatives += len(ground_truth_points_list) - len(matches)
            false_positives += len(detected_points_list) - len(matches)

        except FileNotFoundError:
            # If the image file is not found, count all ground truth QR codes as failed
            failed += len(ground_truth_points_list)

        total += len(ground_truth_points_list)

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
    # Calculate and display the average processing time per image
    if image_count > 0:
        avg_time = total_time / image_count
        print(f"Average detection time per image: {avg_time:.6f} seconds")

# Example usage
image_directory = 'dataset\evaluation-dataset'
annotation_file = 'qr_coordinates_coco.json'
evaluate_qrdet_detector(image_directory, annotation_file)
