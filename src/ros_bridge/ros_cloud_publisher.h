#ifndef _ROS_PUBLISHER_
#define _ROS_PUBLISHER_

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud.h>

namespace depth_clustering
{
/** 
    at IMR for Debugging
**/

class RosCloudPublisher : public AbstractClient<Cloud>
{
    using ClientT = AbstractClient<Cloud>;

  public:
    RosCloudPublisher(ros::Nodehandle &nh) : nh_(nh)
    {
        AbstractClient<Cloud>();
        pub = nh_.advertise<sensor_msgs::PointCloud>("debug_cloud", 1);
    }

    ~RosCloudPublisher()
    {
        ROS_INFO("RosCloudPublisher destructed");
    }

    void OnNewObjectReceived(const Cloud &cloud, const int id) override;

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub;
}

} //
