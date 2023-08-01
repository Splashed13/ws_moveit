#include "../include/panda_moveit/PCDWriter.hpp"

PCDWriter::PCDWriter() : nh("pc")
{
    point_cloud_sub_ = nh.subscribe("/D435_camera1/depth/color/points", 1, &PCDWriter::pointCloudCallback, this);
    saved = false;
    // The following line initializes the point cloud pointer to an empty point cloud created with () constructor.
    point_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

// Receives a reference to a shared pointer to the point cloud message, allowing efficient access to the message without copying it.
void PCDWriter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!saved)
    {
        // *point_cloud_ptr_ dereferences the pointer to the point cloud object.
        pcl::fromROSMsg(*msg, *point_cloud_ptr_);

        // take the negative of all data points in the point cloud to flip it
        for (size_t i = 0; i < point_cloud_ptr_->points.size(); ++i)
        {
            point_cloud_ptr_->points[i].x = -point_cloud_ptr_->points[i].x;
            point_cloud_ptr_->points[i].y = -point_cloud_ptr_->points[i].y;
            point_cloud_ptr_->points[i].z = -point_cloud_ptr_->points[i].z;
        }

        pcl::io::savePCDFileASCII("/home/tho868/ws_moveit/src/panda_moveit/PointCloudData/test_pcd.pcd", *point_cloud_ptr_);
        ROS_INFO_NAMED("PC","Saved PCD file.");

        saved = true;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "PCD_writer");

    PCDWriter point_cloud_subscriber;

    ros::Rate loop_rate(10); // 10Hz

    while (ros::ok())
    {
        if (point_cloud_subscriber.saved)
            break;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
