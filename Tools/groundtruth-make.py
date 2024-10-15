import os
import json

# Set the folder path containing the images
folder_path = 'evaluation-dataset'  # Change this to your folder path
output_file = 'qr_coordinates.json'

# Initialize a dictionary to hold the QR code data
qr_data = {}

# Check if the output file already exists to load existing data
if os.path.exists(output_file):
    with open(output_file, 'r') as json_file:
        qr_data = json.load(json_file)

# Iterate through each image in the specified folder
for filename in os.listdir(folder_path):
    if filename.endswith(('.png', '.jpg', '.jpeg')):  # Check for image files
        if filename in qr_data:  # Skip if data for this image already exists
            print(f'Skipping {filename}, data already exists.')
            continue
        
        image_path = os.path.join(folder_path, filename)
        
        # Input the number of QR codes in the image
        num_qr_codes = int(input(f'Enter the number of QR codes in {filename}: '))
        
        qr_corners = []
        
        # Loop through each QR code to input its corner coordinates
        for i in range(num_qr_codes):
            print(f'Input coordinates for QR code {i + 1} in {filename}:')
            corners = []
            for j in range(4):  # Get 4 corners
                x = float(input(f'Enter x coordinate for corner {j + 1}: '))
                y = float(input(f'Enter y coordinate for corner {j + 1}: '))
                corners.append((x, y))
            qr_corners.append(corners)
        
        # Save the data for this image to the dictionary
        qr_data[filename] = qr_corners
        
        # Write the updated data to a JSON file after each image
        with open(output_file, 'w') as json_file:
            json.dump(qr_data, json_file, indent=4)

        print(f'Data for {filename} saved to {output_file}')

print(f'All data saved to {output_file}')
