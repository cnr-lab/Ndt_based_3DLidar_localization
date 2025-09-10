# NDT-based 3D LiDAR Localization

A ROS2 package for 3D LiDAR-based localization using NDT/GICP algorithms with point cloud maps.

## Features

- **NDT/GICP Registration**: High-accuracy point cloud registration
- **EKF Sensor Fusion**: Integration with odometry and IMU data
- **PCL Integration**: Full PCL (Point Cloud Library) support
- **ROS2 Compatible**: Built for ROS2 Humble
- **Real-time Processing**: Optimized for real-time localization

## Packages

- `lidar_localization_ros2`: Main localization package
- `ndt_omp_ros2`: NDT OpenMP implementation for ROS2

## Requirements

- ROS2 Humble
- PCL (Point Cloud Library)
- OpenMP
- Eigen3

## Installation

```bash
# Clone the repository
git clone https://github.com/cnr-lab/Ndt_based_3DLidar_localization.git

# Build the workspace
cd Ndt_based_3DLidar_localization
colcon build --packages-select ndt_omp_ros2 lidar_localization_ros2
source install/setup.bash
```

## Build Instructions

### Prerequisites
- ROS2 Humble
- PCL (Point Cloud Library)
- OpenMP
- Eigen3
- robot_localization (for EKF)

### Build Commands

```bash
# Clean build (recommended for first time)
cd Ndt_based_3DLidar_localization
rm -rf build/ install/ log/
colcon build --packages-select ndt_omp_ros2 lidar_localization_ros2

# Symlink build (for development)
colcon build --packages-select ndt_omp_ros2 lidar_localization_ros2 --symlink-install

# Source the workspace
source install/setup.bash
```

### Build Troubleshooting
- If you encounter CMake cache issues, clean the build directory first
- Ensure all dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`

## Usage

### Basic Usage

```bash
# Launch the localization system
ros2 launch lidar_localization_ros2 pcl_localization.launch.py

# With custom map
ros2 launch lidar_localization_ros2 pcl_localization.launch.py map_path:=/path/to/your/map.pcd
```

### Complete System Setup

1. **Start Simulation (if using simulation)**
```bash
# In terminal 1
cd /path/to/your/simulation/workspace
source install/setup.bash
ros2 launch your_simulator_package simulator.launch.py
```

2. **Launch Localization System**
```bash
# In terminal 2
cd Ndt_based_3DLidar_localization
source install/setup.bash
ros2 launch lidar_localization_ros2 pcl_localization.launch.py
```

3. **Set Initial Pose (in RViz)**
- Use RViz's "2D Pose Estimate" tool
- Click on the map to set the robot's initial position

### Launch Options

```bash
# Disable RViz
ros2 launch lidar_localization_ros2 pcl_localization.launch.py rviz:=false

# Use different map
ros2 launch lidar_localization_ros2 pcl_localization.launch.py map_path:=/path/to/map.pcd
```

### Verification

Check if the system is working:
```bash
# Check topics
ros2 topic list

# Check TF frames
ros2 run tf2_tools view_frames

# Monitor localization pose
ros2 topic echo /pcl_pose
```

## Topics

### Input
- `/cloud` (sensor_msgs/PointCloud2): LiDAR point cloud
- `/map` (sensor_msgs/PointCloud2): Point cloud map
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): Initial pose
- `/odom` (nav_msgs/Odometry): Odometry data (optional)
- `/imu` (sensor_msgs/Imu): IMU data (optional)

### Output
- `/pcl_pose` (geometry_msgs/PoseWithCovarianceStamped): Localized pose
- `/path` (nav_msgs/Path): Robot path
- `/odom_filtered` (nav_msgs/Odometry): Filtered odometry

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `registration_method` | string | "NDT_OMP" | Registration algorithm |
| `score_threshold` | double | 5.0 | Convergence threshold |
| `ndt_resolution` | double | 2.0 | Voxel resolution [m] |
| `use_odom` | bool | true | Enable odometry fusion |
| `use_imu` | bool | false | Enable IMU fusion |

## License

BSD-2-Clause License

## Acknowledgments

Based on the original work by [rsasaki0109/lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)
