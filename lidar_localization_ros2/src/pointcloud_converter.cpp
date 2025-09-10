#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstring>

// Use PCL's built-in PointXYZI

class PointCloudConverter : public rclcpp::Node
{
public:
    PointCloudConverter() : Node("pointcloud_converter")
    {
        // Create publisher and subscriber
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points/points", 10,
            std::bind(&PointCloudConverter::pointcloud_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "PointCloud converter initialized");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert to XYZI format
        pcl::PointCloud<pcl::PointXYZI> xyz_cloud;
        xyz_cloud.points.reserve(msg->width * msg->height);
        
        for (size_t i = 0; i < msg->width * msg->height; ++i) {
            // Extract point data directly from message
            float x = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + 0]);
            float y = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + 4]);
            float z = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + 8]);
            float intensity = 0.0; // Default intensity
            
            // Check if intensity field exists
            if (msg->point_step >= 16) {
                intensity = *reinterpret_cast<const float*>(&msg->data[i * msg->point_step + 12]);
            }
            
            // Filter out invalid points
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
                (x != 0.0 || y != 0.0 || z != 0.0)) {
                
                pcl::PointXYZI point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = intensity;
                
                xyz_cloud.points.push_back(point);
            }
        }
        
        if (xyz_cloud.empty()) {
            return;
        }
        
        // Create PointCloud2 message manually with intensity field
        sensor_msgs::msg::PointCloud2 output_msg;
        output_msg.header = msg->header;
        output_msg.header.frame_id = "gpu_lidar";
        
        // Define fields for XYZI
        output_msg.fields.resize(4);
        output_msg.fields[0].name = "x";
        output_msg.fields[0].offset = 0;
        output_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_msg.fields[0].count = 1;
        
        output_msg.fields[1].name = "y";
        output_msg.fields[1].offset = 4;
        output_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_msg.fields[1].count = 1;
        
        output_msg.fields[2].name = "z";
        output_msg.fields[2].offset = 8;
        output_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_msg.fields[2].count = 1;
        
        output_msg.fields[3].name = "intensity";
        output_msg.fields[3].offset = 12;
        output_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_msg.fields[3].count = 1;
        
        // Set point cloud data
        output_msg.height = 1;
        output_msg.width = xyz_cloud.points.size();
        output_msg.point_step = 16; // 4 bytes per field * 4 fields
        output_msg.row_step = output_msg.point_step * output_msg.width;
        output_msg.is_bigendian = false;
        output_msg.is_dense = true;
        
        // Copy point data
        output_msg.data.resize(xyz_cloud.points.size() * output_msg.point_step);
        for (size_t i = 0; i < xyz_cloud.points.size(); ++i) {
            const auto& point = xyz_cloud.points[i];
            memcpy(&output_msg.data[i * output_msg.point_step + 0], &point.x, sizeof(float));
            memcpy(&output_msg.data[i * output_msg.point_step + 4], &point.y, sizeof(float));
            memcpy(&output_msg.data[i * output_msg.point_step + 8], &point.z, sizeof(float));
            memcpy(&output_msg.data[i * output_msg.point_step + 12], &point.intensity, sizeof(float));
        }
        
        // Publish
        publisher_->publish(output_msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudConverter>());
    rclcpp::shutdown();
    return 0;
}
