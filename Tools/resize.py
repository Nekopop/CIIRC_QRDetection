import os
from PIL import Image

# Path to the folder containing images
folder_path = 'dataset'
resize_factor = 4  # Change this value to adjust the resizing factor

# Loop through all files in the folder
for filename in os.listdir(folder_path):
    if 'PXL' in filename and filename.endswith(('.png', '.jpg', '.jpeg')):  # Check for image files
        image_path = os.path.join(folder_path, filename)
        img = Image.open(image_path)
        
        # Resize the image based on the resizing factor
        new_size = (img.width // resize_factor, img.height // resize_factor)
        img_resized = img.resize(new_size)

        # Save the resized image with a new filename
        img_resized.save(os.path.join(folder_path, f'resized_{filename}'))
