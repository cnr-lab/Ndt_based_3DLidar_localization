import os

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.actions
import launch_ros.events

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()
    
    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # Get launch configurations
    rviz = LaunchConfiguration('rviz')
    
    # PointCloud Converter (XYZ -> XYZI)
    pointcloud_converter = launch_ros.actions.Node(
        name='pointcloud_converter',
        package='pcl_localization_ros2',
        executable='pointcloud_converter',
        output='screen',
        parameters=[{'use_sim_time': True}]
        )
    
    # Map Loader (PCD -> PointCloud2) - Not needed, pcl_localization loads map directly
    # map_loader = launch_ros.actions.Node(
    #     name='pcd_map_loader',
    #     package='pcl_localization_ros2',
    #     executable='pcd_map_loader.py',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    #     )
    
    # PCL Transform Publisher (pose -> TF)
    pcl_transform_publisher = launch_ros.actions.Node(
        name='pcl_transform_publisher',
        package='pcl_localization_ros2',
        executable='pcl_transform_publisher.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
        )
    
    # EKF Filter for sensor fusion
    ekf_filter = launch_ros.actions.Node(
        name='ekf_filter',
        package='robot_localization',
        executable='ekf_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('pcl_localization_ros2'),
            'param',
            'robot_localization_ekf.yaml'
        )],

        )

    # RViz Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('pcl_localization_ros2'),
            'rviz',
            'pcl_localization.rviz'
        )],
        output='screen',
        condition=IfCondition(rviz)
    )

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        'localization_param_dir',
        default=os.path.join(
            get_package_share_directory('pcl_localization_ros2'),
            'param',
            'localization.yaml'))

    pcl_localization = launch_ros.actions.LifecycleNode(
        name='pcl_localization',
        namespace='',
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        parameters=[localization_param_dir],
        output='screen')

    to_inactive = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    from_unconfigured_to_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            goal_state='unconfigured',
            entities=[
                launch.actions.LogInfo(msg="-- Unconfigured --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    from_inactive_to_active = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=pcl_localization,
            start_state = 'configuring',
            goal_state='inactive',
            entities=[
                launch.actions.LogInfo(msg="-- Inactive --"),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(pcl_localization),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    ld.add_action(from_unconfigured_to_inactive)
    ld.add_action(from_inactive_to_active)

    # Add all nodes
    ld.add_action(rviz_arg)  # Launch argument
    ld.add_action(pointcloud_converter)  # PointCloud converter
    # ld.add_action(map_loader)  # Map loader
    ld.add_action(ekf_filter)  # EKF filter
    ld.add_action(pcl_localization)
    # ld.add_action(pcl_transform_publisher)  # TF 변환을 위해 필요
    ld.add_action(rviz_node)  # RViz visualization
    ld.add_action(to_inactive)

    return ld