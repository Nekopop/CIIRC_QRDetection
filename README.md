# CIIRC_QRDetection

## Overview

This repository contains programs to compare the performance of multiple QR code libraries based on F1 score and execution time (elapsed time, not CPU time). For installation of each model, please refer to their respective repositories. OpenCV, QRDet, ZXing-C++ and QReader can be installed via pip.

## Installation

To install the necessary dependencies, use the following commands:

```bash
pip install opencv-python
pip install qrdet
pip install zxingcpp
pip install qreader
```

Ensure you have Java 11 or later installed.

To install `pyzxing`, run the following commands:

```bash
git clone https://github.com/ChenjieXu/pyzxing.git
cd pyzxing
python setup.py install
```

## Evaluation

1. **Evaluate Models**:
   To evaluate each model, run the evaluation scripts located in the `EvaluationCode` folder. These scripts will display detection accuracy and execution time.
   
   Example:
   ```bash
   python EvaluationCode/OpenCV/evaluate-opencv.py 
   ```
   ```bash
   python EvaluationCode/pyzxing/evaluate-pyzxing.py 
   ```
   ```bash
   python EvaluationCode/QRDet/evaluate-qrdet.py 
   ```
   ```bash
   python EvaluationCode/QReader/evaluate-qreader.py 
   ```
   ```bash
   python EvaluationCode/zxing-cpp/evaluate-zxingcpp.py 
   ```
   

   Ensure you have the image dataset and the corresponding ground truth JSON file prepared and specify the paths in the codes. The qr_coordinates_coco.json file contains the ground truth coordinates of the QR codes.

2. **Evaluate BoofCV**:
   To try the QRCode Detection by using BoofCV model, run the following command:
   ```bash
   java -jar applications.jar BatchScanQrCodes -i "glob:<input_folder>/*.*" -o <output_file>
   ```

   Example:
   ```bash
   java -jar applications.jar BatchScanQrCodes -i "glob:../dataset/evaluation-dataset/*.*" -o myresults.txt
   ```

   Ensure you have the image dataset prepared and specify the input folder and output file paths in the codes.

   Next, Use the scripts in the BoofCV folder to evaluate BoofCV. These scripts allow you to specify input and output paths for detail evaluations.

   Example:
   ```bash
   python EvaluationCode/BoofCV/speedtest-boofcv.py -i dataset/evaluation-dataset -q myresults.txt
   ```
   ```bash
   python EvaluationCode/BoofCV/evaluate-boofcv.py -q myresults.txt -j qr_coordinates_coco.json
   ```
   Ensure you have the image dataset and the corresponding ground truth JSON file prepared and specify the paths in the script.

## **Tools**:
   The Tools folder contains scripts used to create ground truth data.

   - **groundtruth-make.py**:
     This script allows you to manually input the coordinates of QR codes in each image of a dataset. It saves the input data in a JSON file.
     ```bash
     python Tools/groundtruth-make.py
     ```

   - **json2coco.py**:
     This script converts the created qr_coordinates.json file to COCO format.
     ```bash
     python Tools/json2coco.py
     ```

   - **resize.py**:
     This script resizes images in a folder to a specified size while maintaining the aspect ratio.
     ```bash
     python Tools/resize.py
     ```

## Notes

- Ensure that the paths to the image dataset and ground truth JSON file are correctly specified in the scripts.
- Refer to the respective repositories for installation instructions of other QR code libraries.

## References

* [ZXing](https://github.com/zxing/zxing) - ZXing library GitHub repository
* [Pyzxing](https://github.com/ChenjieXu/pyzxing) - Python wrapper for ZXing library
* [ZXing-C++](https://github.com/zxing-cpp/zxing-cpp) - ZXing implemented in C++ with improved performance.
* [QRDet](https://github.com/Eric-Canas/qrdet) - Robust QR Detector based on YOLOv8
* [QReader](https://github.com/Eric-Canas/QReader) - Reading difficult and tricky QR codes within images in Python, uses QRDet
* [QR Code Reading Benchmark and Comparison](https://www.dynamsoft.com/codepool/qr-code-reading-benchmark-and-comparison.html)
* [Study of QR Code Scanning Performance in Different Environments](https://boofcv.org/index.php?title=Performance:QrCode)
* [BoofCV](https://github.com/lessthanoptimal/BoofCV) - Another library usable for QR detection