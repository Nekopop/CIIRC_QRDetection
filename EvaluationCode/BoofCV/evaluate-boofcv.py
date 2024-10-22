import json
import argparse

def count_detected_qrcodes(qrcode_file):
    detected_count = 0

    with open(qrcode_file, 'r', encoding='utf-8') as file:
        lines = file.readlines()

        for line in lines:
            parts = line.split()
            if parts and parts[0].isdigit():  # Check if the first part is a digit
                detected_count += int(parts[0])

    return detected_count

def count_total_qrcodes(json_file):
    with open(json_file, 'r', encoding='utf-8') as file:
        data = json.load(file)

    total_qrcodes = 0
    for annotation in data['annotations']:
        if 'segmentation' in annotation and len(annotation['segmentation']) > 0:
            total_qrcodes += 1

    return total_qrcodes

def calculate_detection_rate(detected_count, total_qrcodes):
    if total_qrcodes == 0:
        return 0.0
    return (detected_count / total_qrcodes) * 100

def main(qrcode_file, json_file):
    detected_count = count_detected_qrcodes(qrcode_file)
    total_qrcodes = count_total_qrcodes(json_file)
    detection_rate = calculate_detection_rate(detected_count, total_qrcodes)

    print(f'Detected QR Codes: {detected_count}')
    print(f'Total QR Codes: {total_qrcodes}')
    print(f'Detection Rate: {detection_rate:.2f}%')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Evaluate QR code detection results.')
    parser.add_argument('-q', '--qrcode_file', required=True, help='File containing QR code detection results')
    parser.add_argument('-j', '--json_file', required=True, help='JSON file containing ground truth annotations')
    args = parser.parse_args()

    main(args.qrcode_file, args.json_file)