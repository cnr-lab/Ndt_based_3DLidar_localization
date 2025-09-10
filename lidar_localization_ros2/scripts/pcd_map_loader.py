#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import threading
import time

class PcdMapLoader(Node):
    def __init__(self):
        super().__init__('pcd_map_loader')
        
        # Parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        if not self.has_parameter('map_path'):
            self.declare_parameter('map_path', '/root/fast_lio_slam_ws/maps/custom_map_20250909_120724.pcd')
        if not self.has_parameter('publish_rate'):
            self.declare_parameter('publish_rate', 0.2)  # 5Hz
        if not self.has_parameter('frame_id'):
            self.declare_parameter('frame_id', 'map')
        
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.map_path = self.get_parameter('map_path').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Load PCD map
        self.map_cloud = self.load_pcd_map()
        if self.map_cloud is None:
            self.get_logger().error(f'Failed to load map from {self.map_path}')
            return
        
        # Publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(PointCloud2, '/map', qos_profile)
        
        # Timer for publishing map
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_map)
        
        self.get_logger().info(f'PcdMapLoader initialized. Map: {self.map_path}')
        self.get_logger().info(f'Map points: {len(self.map_cloud.points)}')
    
    def load_pcd_map(self):
        """Load PCD map file"""
        try:
            self.get_logger().info(f'Loading PCD map from: {self.map_path}')
            pcd = o3d.io.read_point_cloud(self.map_path)
            
            if len(pcd.points) == 0:
                self.get_logger().error('Loaded PCD file is empty!')
                return None
            
            self.get_logger().info(f'Successfully loaded PCD map with {len(pcd.points)} points')
            return pcd
            
        except Exception as e:
            self.get_logger().error(f'Error loading PCD map: {str(e)}')
            return None
    
    def publish_map(self):
        """Publish the map as PointCloud2"""
        if self.map_cloud is None:
            return
        
        try:
            # Convert Open3D point cloud to ROS PointCloud2
            points = np.asarray(self.map_cloud.points)
            
            # Create PointCloud2 message with XYZI format
            msg = PointCloud2()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            # Define fields for XYZI
            msg.fields = [
                point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='intensity', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1)
            ]
            
            # Add intensity field (set to 0.0 for all points)
            intensities = np.zeros((points.shape[0], 1), dtype=np.float32)
            points_with_intensity = np.hstack([points, intensities])
            
            # Set point cloud data
            msg.height = 1
            msg.width = points_with_intensity.shape[0]
            msg.point_step = 16  # 4 bytes per field * 4 fields
            msg.row_step = msg.point_step * msg.width
            msg.data = points_with_intensity.astype(np.float32).tobytes()
            msg.is_bigendian = False
            msg.is_dense = True
            
            # Publish
            self.map_pub.publish(msg)
            
            self.get_logger().info(f'Published map with {len(points)} points', throttle_duration_sec=5.0)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing map: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PcdMapLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
