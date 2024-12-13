import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class DepthDataChecker:
    def __init__(self, bag_file, topic_name):
        self.bag_file = bag_file
        self.topic_name = topic_name

    def check_depth_scale(self):
        # Initialize the bag reader
        storage_options = StorageOptions(uri=self.bag_file, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        # Retrieve topic types
        topic_type_pairs = reader.get_all_topics_and_types()
        topic_type_map = {topic.name: topic.type for topic in topic_type_pairs}

        if self.topic_name not in topic_type_map:
            print(f"[ERROR] Topic '{self.topic_name}' not found in the bag file.")
            return

        if topic_type_map[self.topic_name] != 'sensor_msgs/msg/CompressedImage':
            print(f"[ERROR] Topic '{self.topic_name}' is not of type CompressedImage.")
            return

        # Read messages from the topic
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == self.topic_name:
                msg = deserialize_message(data, CompressedImage)
                depth_scale = self.analyze_image(msg)
                if depth_scale is not None:
                    print(f"[INFO] Detected depth scale: {depth_scale}")
                else:
                    print("[WARN] Unable to determine depth scale from the current image.")
                break

    def analyze_image(self, msg):
        # Decode the compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # Use IMREAD_UNCHANGED to preserve original depth format

        if image is None:
            print("[ERROR] Failed to decode the image.")
            return None

        if image.dtype == np.uint16:
            print(f"[INFO] Image detected with dtype: {image.dtype}, likely in millimeters.")
            return "millimeters (uint16)"
        elif image.dtype == np.float32:
            print(f"[INFO] Image detected with dtype: {image.dtype}, likely in meters.")
            return "meters (float32)"
        else:
            print(f"[INFO] Image detected with dtype: {image.dtype}, unable to infer scale.")
            return "unknown scale"

def main():
    rclpy.init()
    bag_file = '/home/myrousz/rovozci_datasets/2024-03-07-Strancice-RGBD/rosbag2_2024_03_07-14_28_39/rosbag2_2024_03_07-14_28_39_0.db3'  # Replace with your bag file path
    topic_name = '/oak/stereo/image_raw/compressed'  # Replace with your depth topic name

    checker = DepthDataChecker(bag_file, topic_name)
    checker.check_depth_scale()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
