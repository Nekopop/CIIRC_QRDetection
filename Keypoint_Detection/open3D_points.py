import os
import rosbag2_py
import rclpy.serialization
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import open3d as o3d

def read_single_frame(reader, depth_topic, rgb_topic):
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == depth_topic:
            depth_image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
        elif topic == rgb_topic:
            rgb_image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
        if 'depth_image' in locals() and 'rgb_image' in locals():
            return depth_image, rgb_image
    return None, None

def read_all_frames(reader, depth_topic, rgb_topic):
    frames = []
    depth_image = None
    rgb_image = None
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == depth_topic:
            msg = rclpy.serialization.deserialize_message(data, CompressedImage)
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        elif topic == rgb_topic:
            msg = rclpy.serialization.deserialize_message(data, CompressedImage)
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        if depth_image is not None and rgb_image is not None:
            frames.append((depth_image, rgb_image))
            depth_image = None
            rgb_image = None
    return frames

def create_point_cloud(rgb, depth, K, voxel_size=0.05):
    rgb_resized = cv2.resize(rgb, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_AREA)
    color_raw = o3d.geometry.Image(rgb_resized.astype(np.uint8))
    depth_raw = o3d.geometry.Image(depth.astype(np.uint16))
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_raw,
        depth_raw,
        depth_scale=1000,
        depth_trunc=65535,
        convert_rgb_to_intensity=False
    )
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width=depth.shape[1],
        height=depth.shape[0],
        fx=K[0, 0],
        fy=K[1, 1],
        cx=K[0, 2],
        cy=K[1, 2]
    )
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return pcd
    
def main():
    bag_path = '/home/myrousz/rovozci_datasets/2024-03-07-Strancice-RGBD/rosbag2_2024_03_07-15_29_58/rosbag2_2024_03_07-15_29_58_0.db3'
    depth_topic = '/oak/stereo/image_raw/compressed'
    rgb_topic = '/oak/rgb/image_raw/compressed'

    # Create folder name from input db3 file name
    bag_filename = os.path.basename(bag_path)  # Get the file name
    bag_name_without_ext = os.path.splitext(bag_filename)[0]  # Remove the extension

    # Specify the folder to save results (ensure unique folder names)
    output_folder = f'/home/myrousz/qr-workspace/output-pointclouds_{bag_name_without_ext}'
    os.makedirs(output_folder, exist_ok=True)

    # Specify the folder to save images
    image_folder = os.path.join(output_folder, 'images')
    os.makedirs(image_folder, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    K = np.array([
        [2067.088623046875 * 0.5, 0.0, 1235.701171875 * 0.5],
        [0.0, 2065.724365234375 * 0.5, 727.046630859375 * 0.5],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)

    depth_image = None
    rgb_image = None
    frame_idx = 0
    processed_frames = 0

    print("Starting processing...")

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == depth_topic:
            msg = rclpy.serialization.deserialize_message(data, CompressedImage)
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        elif topic == rgb_topic:
            msg = rclpy.serialization.deserialize_message(data, CompressedImage)
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        if depth_image is not None and rgb_image is not None:
            if frame_idx % 3 == 0:
                pcd = create_point_cloud(rgb_image, depth_image, K)
                # Save point cloud data
                output_pcd_file = os.path.join(output_folder, f"pointcloud_frame_{frame_idx:04d}.ply")
                o3d.io.write_point_cloud(output_pcd_file, pcd)
                # Save images
                rgb_image_file = os.path.join(image_folder, f"rgb_frame_{frame_idx:04d}.png")
                depth_image_file = os.path.join(image_folder, f"depth_frame_{frame_idx:04d}.png")
                cv2.imwrite(rgb_image_file, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
                cv2.imwrite(depth_image_file, depth_image)
                print(f"Processed frame {frame_idx}: Saved point cloud and images")
                processed_frames += 1
            # Reset
            depth_image = None
            rgb_image = None
            frame_idx += 1

    print(f"Processing completed. Total frames processed: {processed_frames} / {frame_idx}")

if __name__ == '__main__':
    main()
