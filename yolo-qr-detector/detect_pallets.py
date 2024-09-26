import cv2
import os
import numpy as np
import glob
from pathlib import Path

# https://www.dynamsoft.com/codepool/qr-code-detect-decode-yolo-opencv.html


def postprocess(frame, outs, threshold, thickness):
    frameHeight, frameWidth = frame.shape[:2]

    classIds = []
    confidences = []
    boxes = []

    for out in outs:
        for detection in out:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > threshold:
                x, y, width, height = detection[:4] * np.array([frameWidth, frameHeight, frameWidth, frameHeight])
                left = int(x - width / 2)
                top = int(y - height / 2)
                classIds.append(classId)
                confidences.append(float(confidence))
                boxes.append([left, top, int(width), int(height)])

    indices = cv2.dnn.NMSBoxes(boxes, confidences, threshold, threshold - 0.1)
    print(indices)
    for i in indices:            
        #i = i[0]
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]

        # Draw bounding box for objects
        cv2.rectangle(frame, (left, top), (left + width, top + height), (0, 0, 255), thickness)

        # Draw class name and confidence
        label = '%s:%.2f' % (classes[classIds[i]], confidences[i])
        cv2.putText(frame, label, (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))


def process_image(input_path, name, ln):
    frame = cv2.imread(input_path)
    threshold = 0.6
    maxWidth = 1280; maxHeight = 720
    imgHeight, imgWidth = frame.shape[:2]
    hScale = 1; wScale = 1
    thickness = 1
    if imgHeight > maxHeight:
        hScale = imgHeight / maxHeight
        thickness = 6
    if imgWidth > maxWidth:
        wScale = imgWidth / maxWidth
        thickness = 6

    img_size = (416, 416)
    blob = cv2.dnn.blobFromImage(frame, 1/255, img_size, swapRB=True, crop=False)

    net.setInput(blob)
    outs = net.forward(ln)

    # print("outs:", outs)

    postprocess(frame, outs, threshold, thickness)

    if hScale > wScale:
        frame = cv2.resize(frame, (int(imgWidth / hScale), maxHeight))
    elif hScale < wScale:
        frame = cv2.resize(frame, (maxWidth, int(imgHeight / wScale)))
    cv2.imshow('QR Detection', frame)
    cv2.waitKey()    


if __name__ == "__main__":

    root_dir = 'e:/Work/yolo-qr-detector'

    # https://stackoverflow.com/questions/55792523/failed-to-parse-netparameter-file
    classes = open(root_dir + '/models/yolov3-tiny-qr/qrcode.names').read().strip().split('\n')
    net = cv2.dnn.readNetFromDarknet(root_dir + '/models/yolov3-tiny-qr/qrcode-yolov3-tiny.cfg', root_dir + '/models/yolov3-tiny-qr/qrcode-yolov3-tiny_last.weights')
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)

    ln = net.getLayerNames()
    print("Layer names:", len(ln), ln)
    uol = net.getUnconnectedOutLayers()
    print("Unconnected out layers:", uol)
    ln = [ln[i - 1] for i in uol]
    print("Unconnected out layers names:", ln)

    images_root = "../rovozci_datasets/rosbag2_2024_03_07-15_00_42_0"

    for file in glob.glob(f'{images_root}/*/rgb.png'):
        ff = Path(file)
        fname = ff.name
        parent = ff.parent.name
        print(fname, parent, str(ff))

        process_image(str(file), parent, ln)

