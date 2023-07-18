#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>

class PointCloudSubscriber
{
public:
  bool saved;

  PointCloudSubscriber();

private:
  ros::Subscriber point_cloud_sub_;
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr_;

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  

};
