import glob
import subprocess
import time
import argparse

def count_total_images(image_pattern):
    return len(glob.glob(image_pattern))

def calculate_average_time(total_time, image_count):
    if image_count == 0:
        return 0.0
    return total_time / image_count

def main(input_folder, qrcode_file):
    image_pattern = f'{input_folder}/*.*'

    # Measure the execution time of the command
    start_time = time.perf_counter()
    subprocess.run(['java', '-jar', 'applications.jar', 'BatchScanQrCodes', '-i', f'glob:{image_pattern}', '-o', qrcode_file])
    end_time = time.perf_counter()

    total_time = end_time - start_time  # Convert to seconds

    total_images = count_total_images(image_pattern)
    average_time_per_image = calculate_average_time(total_time, total_images)

    print(f'Total Images: {total_images}')
    print(f'Total Processing Time: {total_time:.6f} seconds')
    print(f'Average Time per Image: {average_time_per_image:.6f} seconds')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process QR codes in images.')
    parser.add_argument('-i', '--input', required=True, help='Input folder containing images')
    parser.add_argument('-q', '--qrcode_file', required=True, help='File to save QR code detection results')
    args = parser.parse_args()

    main(args.input, args.qrcode_file)