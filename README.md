# CIIRC_QRDetection

## References

* [ZXING](https://github.com/zxing/zxing) - ZXing library GitHub repository
* [Pyzxing](https://github.com/ChenjieXu/pyzxing) - Python wrapper for ZXing library
* [QRDet](https://github.com/Eric-Canas/qrdet) - Robust QR Detector based on YOLOv8
* [QReader](https://github.com/Eric-Canas/QReader) - Reading difficult and tricky QR codes within images in Python, uses QRDet
* [QR Code Reading Benchmark and Comparison](https://www.dynamsoft.com/codepool/qr-code-reading-benchmark-and-comparison.html)
* [Study of QR Code Scanning Performance in Different Environments](https://boofcv.org/index.php?title=Performance:QrCode)
* [BoofCV](https://github.com/lessthanoptimal/BoofCV) - Another library usable for QR detection

## Overview

This repository contains programs to compare the performance of multiple QR code libraries based on F1 score and execution time (elapsed time, not CPU time). For installation of each model, please refer to their respective repositories. OpenCV and QRDet can be installed via pip.

## Installation

To install OpenCV and QRDet, use the following commands:

```bash
pip install opencv-python
pip install qrdet
```

For other models, please refer to their respective repositories.

## Evaluation

1. **Evaluate Models**:
   To evaluate each model, run the evaluation scripts located in the `EvaluationCode` folder. These scripts will display detection accuracy and execution time.
   
   Example:
   ```bash
   python EvaluationCode\OpenCV\evaluate-opencv.py
   ```

   Ensure you have the image dataset and the corresponding ground truth JSON file prepared and specify the paths in the script. The qr_coordinates_coco.json file contains the ground truth coordinates of the QR codes.

2. **Tools**:
   The Tools folder contains scripts used to create ground truth data.

   - **groundtruth-make.py**:
     This script allows you to manually input the coordinates of QR codes in each image of a dataset. It saves the input data in a JSON file.
     ```bash
     python Tools\groundtruth-make.py
     ```

   - **json2coco.py**:
     This script converts the created qr_coordinates.json file to COCO format.
     ```bash
     python Tools\json2coco.py
     ```

   - **resize.py**:
     This script resizes images in a folder to a specified size while maintaining the aspect ratio.
     ```bash
     python Tools\resize.py
     ```

## Example Usage

### Evaluating a Model

1. Prepare your image dataset and ground truth JSON file.
2. Specify the paths to the dataset and JSON file in the evaluation script.
3. Run the evaluation script:
   ```bash
   python EvaluationCode\OpenCV\evaluate-opencv.py
   ```

### Creating Ground Truth Data

1. Run the `groundtruth-make.py` script to manually input QR code coordinates:
   ```bash
   python Tools\groundtruth-make.py
   ```

2. Convert the JSON file to COCO format using `json2coco.py`:
   ```bash
   python Tools\json2coco.py
   ```

3. Resize images using `resize.py`:
   ```bash
   python Tools\resize.py
   ```

## Notes

- Ensure that the paths to the image dataset and ground truth JSON file are correctly specified in the scripts.
- Refer to the respective repositories for installation instructions of other QR code libraries.