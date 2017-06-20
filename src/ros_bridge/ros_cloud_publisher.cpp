#include "ros_cloud_publisher.h"
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>

void RosCloudPublisher::OnNewObjectReceived(const Cloud &cloud, const int id)
{
    sensor_msgs::PointCloud pub_cloud;
    pub_cloud.points.reserve(cloud.size());
    for (int i = 0; i < cloud.size())
    {
        
    }
    pub_cloud.header.stamp = ros::Time::now();
    pub_cloud.header.frame = ""; //must be changed for everysensor
    pub_cloud.header.blabla =
}