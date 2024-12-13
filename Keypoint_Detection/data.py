import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage
import cv2
import numpy as np
import yaml
import os


class BagReader:
    def __init__(self, bag_file, output_base_dir):
        self.bag_file = bag_file
        self.output_base_dir = output_base_dir

        # Extract the base name of the DB file (without extension)
        bag_file_name = os.path.splitext(os.path.basename(bag_file))[0]
        self.output_dir = os.path.join(output_base_dir, f"extracted_data_{bag_file_name}")

        self.image_counter = 0  # Counter for unique image filenames

        # Create subdirectories for different data types
        self.rgb_dir = os.path.join(self.output_dir, 'rgb_images')
        self.depth_dir = os.path.join(self.output_dir, 'depth_images')
        self.camera_info_dir = os.path.join(self.output_dir, 'camera_info')
        
        for dir_path in [self.rgb_dir, self.depth_dir, self.camera_info_dir]:
            os.makedirs(dir_path, exist_ok=True)

        # Mapping of ROS 2 message types
        self.message_types = {
            'sensor_msgs/msg/CameraInfo': CameraInfo,
            'sensor_msgs/msg/CompressedImage': CompressedImage,
        }

    def extract_data(self):
        # Initialize the bag reader
        storage_options = StorageOptions(uri=self.bag_file, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        # Retrieve topic types
        topic_type_pairs = reader.get_all_topics_and_types()
        topic_type_map = {topic.name: topic.type for topic in topic_type_pairs}

        # Process messages in the bag file
        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic in topic_type_map:
                message_type_name = topic_type_map[topic]
                if message_type_name in self.message_types:
                    msg_type = self.message_types[message_type_name]
                    msg = deserialize_message(data, msg_type)

                    # Process CameraInfo or CompressedImage
                    if isinstance(msg, CameraInfo):
                        self.save_camera_info(msg, topic, timestamp)
                    elif isinstance(msg, CompressedImage):
                        self.save_image(msg, topic, timestamp)

    def save_camera_info(self, msg, topic, timestamp):
        # Determine subdirectory based on topic
        subdir = 'rgb' if 'rgb' in topic.lower() else 'stereo'
        output_path = os.path.join(self.camera_info_dir, subdir)
        os.makedirs(output_path, exist_ok=True)

        # Save CameraInfo as YAML
        file_name = f"{topic.split('/')[-2]}_{timestamp}_camera_info.yaml"
        file_path = os.path.join(output_path, file_name)
        data = {field: getattr(msg, field) for field in msg.get_fields_and_field_types()}
        with open(file_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        print(f"[INFO] Camera info saved to: {file_path}")

    def save_image(self, msg, topic, timestamp):
        # Determine subdirectory based on topic
        subdir = self.rgb_dir if 'rgb' in topic.lower() else self.depth_dir
        os.makedirs(subdir, exist_ok=True)

        # Save image
        self.image_counter += 1  # Increment counter for unique filenames
        file_name = f"{topic.split('/')[-2]}_{timestamp}_{self.image_counter:04d}_image.png"
        file_path = os.path.join(subdir, file_name)

        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            cv2.imwrite(file_path, image)
            print(f"[INFO] Image saved to: {file_path}")
        else:
            print(f"[WARN] Failed to decode image from topic: {topic}")

def main():
    rclpy.init()  # Initialize ROS 2
    bag_file = '/home/myrousz/rovozci_datasets/2024-03-07-Strancice-RGBD/rosbag2_2024_03_07-14_28_39/rosbag2_2024_03_07-14_28_39_0.db3'
    output_base_dir = 'extracted_data'

    reader = BagReader(bag_file, output_base_dir)
    try:
        reader.extract_data()
    except Exception as e:
        print(f"[ERROR] Failed to process bag file: {e}")
    finally:
        rclpy.shutdown()  # Ensure proper shutdown

if __name__ == '__main__':
    main()