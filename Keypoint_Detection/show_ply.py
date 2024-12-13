import open3d as o3d

# Load the PLY file
ply_file_path = "/home/myrousz/qr-workspace/ros2-data/1206/output-pointclouds_rosbag2_2024_03_07-15_29_58_0/pointcloud_frame_0051.ply"  # Replace with your .ply file path
point_cloud = o3d.io.read_point_cloud(ply_file_path)

# Check if the point cloud is loaded correctly
if point_cloud.is_empty():
    print(f"Failed to load {ply_file_path}")
else:
    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])
