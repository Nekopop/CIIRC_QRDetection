import json

# Load input JSON data
with open('qr_coordinates.json', 'r') as f:
    input_data = json.load(f)

# Initialize the COCO format structure
coco_format = {
    "images": [],
    "annotations": [],
    "categories": [
        {
            "supercategory": "QR_code",
            "id": 1,
            "name": "QR_code"
        }
    ]
}

# Initialize image and annotation IDs
image_id = 1
annotation_id = 1  # Initialize annotation ID

for file_name, coords_list in input_data.items():
    # Append image information to the COCO format
    coco_format["images"].append({
        "file_name": file_name,
        "id": image_id
    })

    for coords in coords_list:
        # Create the segmentation list from coordinates
        segmentation = [coord for point in coords for coord in point]
        
        # Calculate bounding box (xmin, ymin, width, height)
        xmin = min(point[0] for point in coords)
        ymin = min(point[1] for point in coords)
        xmax = max(point[0] for point in coords)
        ymax = max(point[1] for point in coords)
        width = xmax - xmin
        height = ymax - ymin
        area = width * height

        # Add the annotation to the COCO format with a unique ID
        coco_format["annotations"].append({
            "segmentation": [segmentation],
            "area": area,
            "iscrowd": 0,
            "image_id": image_id,
            "bbox": [xmin, ymin, width, height],
            "category_id": 1,
            "id": annotation_id
        })
        
        annotation_id += 1  # Increment annotation ID for the next annotation

    image_id += 1  # Increment image ID for the next image

# Save the converted COCO format JSON to a file
output_file = 'qr_coordinates_coco.json'
with open(output_file, 'w') as f:
    json.dump(coco_format, f, indent=4)

print(f'Saved COCO format JSON file: {output_file}')
