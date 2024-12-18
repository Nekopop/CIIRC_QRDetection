# CIIRC_QRDetection
Written by Kojiro Sato,TUAT
This repository contains scripts and tools for QR code detection and point cloud generation using ROS 2 bag files.

## Contents

- **`open3D_points2.py`**

  - **Description**:  
    Reads RGB and depth images from a ROS 2 bag file and generates corresponding point clouds. Processes frames at specified intervals and saves the point clouds along with the images.

- **`data.py`**

  - **Description**:  
    Extracts images and camera information from a ROS 2 bag file. Saves RGB and depth images to separate directories.

## Usage

### Requirements

- Python 3.x
- ROS 2 installed and sourced
- Python packages:
  - `numpy`
  - `opencv-python`
  - `open3d`
  - `rosbag2_py`
  - `rclpy`
  - `PyYAML` (for `data.py`)

### `open3D_points2.py`

1. **Set the bag file path**  
   Modify the `bag_path` variable to point to your ROS 2 bag file:

   ```python
   bag_path = '/path/to/your/rosbag.db3'
   ```

2. **Configure topics**  
   Set the depth and RGB image topics:

   ```python
   depth_topic = '/your/depth/topic'
   rgb_topic = '/your/rgb/topic'
   ```

3. **Run the script**

   ```bash
   python open3D_points2.py
   ```

4. **Results**  
   Point clouds and images will be saved in the output directory, which is automatically named based on the bag file.

### `data.py`

1. **Set the bag file path**  
   Modify the `bag_file` variable:

   ```python
   bag_file = '/path/to/your/rosbag.db3'
   ```

2. **Run the script**

   ```bash
   python data.py
   ```

3. **Results**  
   Images and camera information are extracted and saved in the `extracted_data` directory.

## Notes

- Ensure that you have the necessary permissions to read the bag files and write to the output directories.
- If you encounter issues with topics or message types, use `ros2 bag info` to inspect the bag file and adjust the script parameters accordingly.

