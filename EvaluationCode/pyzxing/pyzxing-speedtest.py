import time
import numpy as np
from pyzxing import BarCodeReader
import glob

def process_image(input_path):
    start_time = time.time()  # Start processing timer
    results = reader.decode(input_path)  # Decode QR code
    end_time = time.time()  # End processing timer
    processing_time = end_time - start_time
    return results, processing_time

if __name__ == "__main__":
    reader = BarCodeReader()
    images_root = "dataset\evaluation-dataset"

    paths = [str(file) for file in glob.glob(f'{images_root}/*.*')]
    processing_times = []

    for input_path in paths:
        print("Processing:", input_path)
        results, processing_time = process_image(input_path)
        processing_times.append(processing_time)
        print(f"Processing time for {input_path}: {processing_time:.4f} seconds")

    # Calculate statistics
    processing_times = np.array(processing_times)
    avg_time = np.mean(processing_times)
    max_time = np.max(processing_times)
    min_time = np.min(processing_times)
    total_time = np.sum(processing_times)

    print("\nProcessing Time Statistics:")
    print(f"Total images processed: {len(processing_times)}")
    print(f"Total processing time: {total_time:.4f} seconds")
    print(f"Average processing time: {avg_time:.4f} seconds")
    print(f"Max processing time: {max_time:.4f} seconds")
    print(f"Min processing time: {min_time:.4f} seconds")
