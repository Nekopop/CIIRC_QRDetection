import cv2
import os
from qreader import QReader  

# Use QReader to detect and decode QR codes
def detect_qr_codes(image_dir):
    qreader = QReader()

    # Process all images in the directory
    for image_file in os.listdir(image_dir):
        image_path = os.path.join(image_dir, image_file)
        image = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)
        detection_results = qreader.detect_and_decode(image=image, return_detections=True)

        # Display detected QR code contents and detections
        print(f"Processing {image_file}:")
        for result in detection_results:
            print(type(result))
            #print(result)
            #print(f"Result: {result}")  # Print the full result to inspect
            #if isinstance(result, tuple) and len(result) > 0:
                #for s in result:
                    #print(f"Detected QR code content: {s}")
            if len(result) > 0 and isinstance(result, list):
                t = 0
                while t < len(result):
                    print(f"Detection info: {result[t]['quad_xy']}")
                    t = t + 1
            else:
                print("No detection info available.")



# Example usage
image_directory = 'dataset\evaluation-dataset'
detect_qr_codes(image_directory)
