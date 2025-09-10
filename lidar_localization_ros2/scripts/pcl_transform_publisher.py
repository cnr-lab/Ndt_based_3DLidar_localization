#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Path
import threading
import time

class PclTransformPublisher(Node):
    def __init__(self):
        super().__init__('pcl_transform_publisher')
        
        # Parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.use_sim_time = self.get_parameter('use_sim_time').value
        
        # State variables
        self.latest_pose = None
        self.pose_mutex = threading.Lock()
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            PoseWithCovarianceStamped, 
            '/pcl_pose',  # PCL localization output topic (original name)
            self.pose_callback, 
            qos_profile
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/localization_path', 10)
        
        # Path storage
        self.path = Path()
        self.path.header.frame_id = 'map'
        
        # Timer for TF publishing
        self.timer = self.create_timer(0.01, self.publish_tf)  # 100Hz
        
        self.get_logger().info('PclTransformPublisher node initialized.')
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback for PCL localization pose"""
        with self.pose_mutex:
            # Convert PoseWithCovarianceStamped to PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            self.latest_pose = pose_stamped
            
            # Add to path
            self.path.header.stamp = msg.header.stamp
            self.path.poses.append(pose_stamped)
            
            # Keep only last 1000 poses
            if len(self.path.poses) > 1000:
                self.path.poses.pop(0)
            
            # Publish path
            self.path_pub.publish(self.path)
    
    def publish_tf(self):
        """Publish map -> base_footprint transform"""
        with self.pose_mutex:
            if self.latest_pose is None:
                return
            
            # Create transform from pose
            transform = TransformStamped()
            transform.header.stamp = self.latest_pose.header.stamp
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'base_footprint'
            
            # Position
            transform.transform.translation.x = self.latest_pose.pose.position.x
            transform.transform.translation.y = self.latest_pose.pose.position.y
            transform.transform.translation.z = self.latest_pose.pose.position.z
            
            # Orientation
            transform.transform.rotation = self.latest_pose.pose.orientation
            
            # Broadcast transform
            self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = PclTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
