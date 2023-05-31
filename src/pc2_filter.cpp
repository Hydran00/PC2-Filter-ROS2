#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include "pcl/conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/passthrough.h"
using namespace std;
class PC2Filter : public rclcpp::Node
{
public:
  PC2Filter() : Node("pc2_filter")
  {
    // defining input topic name
    this->declare_parameter("input_topic_name", "/camera/depth/color/points");
    std::string input_topic_name = this->get_parameter("input_topic_name").as_string();

    // defining output topic name
    this->declare_parameter("output_topic_name", "/filtered_pc2");
    std::string output_topic_name = this->get_parameter("output_topic_name").as_string();

    subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_name, 10, std::bind(&PC2Filter::filter, this, std::placeholders::_1));
    publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_name, 10);
    RCLCPP_INFO(this->get_logger(), "Filtering node activated, publishing filtered point cloud under /filtered_pc2");

    this->declare_parameter("voxel_grid_size", 0.01);
    voxel_grid_size = (float)this->get_parameter("voxel_grid_size").as_double();

    this->declare_parameter("segment_distance_max", 1.0);
    segment_distance_max = (float)this->get_parameter("segment_distance_max").as_double();

    this->declare_parameter("segment_distance_min", 0.0);
    segment_distance_min = (float)this->get_parameter("segment_distance_min").as_double();

  }

  void filter(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Convert ROS2 PointCloud2 to PCL format
    pcl::fromROSMsg(*msg, *pcl_cloud);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(segment_distance_min, segment_distance_max); // Keep points between 0 and 1 meter from the sensor
    pass.filter(*segmented_cloud);




    // Apply VoxelGrid filter
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(segmented_cloud);
    voxel_filter.setLeafSize(voxel_grid_size,voxel_grid_size,voxel_grid_size); // Adjust the leaf size as per your requirements
    voxel_filter.filter(*filtered_cloud);

    // Apply Passthrough filter for segmentation

    // Convert the pcl::PointCloud to sensor_msgs::msg::PointCloud2
    sensor_msgs::msg::PointCloud2::SharedPtr processed_msg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*filtered_cloud, *processed_msg);
    processed_msg->header = msg->header;
    // Publish the filtered point cloud
    publisher->publish(*processed_msg);
    std::cout << "Filtered point cloud published." << std::endl;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
  float voxel_grid_size;
  float segment_distance_max;
  float segment_distance_min;
};
int main(int argc, char **argv)
{

  // pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
  // pcl::PCLPointCloud2::Ptr voxel_cloud (new pcl::PCLPointCloud2());
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PC2Filter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  // pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
  // pcl::PCLPointCloud2::Ptr voxel_cloud(new pcl::PCLPointCloud2());
  // pcl::PCDReader cloud_reader;
  // pcl::PCDWriter cloud_writer;

  // std::string path = "/home/luqman/ros2_ws/src/ros2_learners/point_cloud_perception/point_clouds/";
  // // Reading the cloud
  // cloud_reader.read(path + std::string("cloud.pcd"), *cloud);

  // std::cout << "Source Cloud Points " << cloud->width * cloud->height << std::endl;
  // // Voxel
  // pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
  // voxel_filter.setInputCloud(cloud);
  // voxel_filter.setLeafSize(0.05, 0.05, 0.05);
  // voxel_filter.filter(*voxel_cloud);

  // std::cout << "Voxel Cloud Points " << voxel_cloud->width * voxel_cloud->height << std::endl;

  // // Write
  // cloud_writer.write(path + std::string("voxelized.pcd"), *voxel_cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

  // return (0);
}
