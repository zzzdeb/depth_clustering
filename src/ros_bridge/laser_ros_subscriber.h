// Copyright (C) 2017  E. Zolboo, RWTH Aachen

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SRC_ROS_BRIDGE_LASER_ROS_SUBSCRIBER_H_
#define SRC_ROS_BRIDGE_LASER_ROS_SUBSCRIBER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string>
#include <map>

#include "communication/abstract_sender.h"
#include "utils/pose.h"
#include "utils/cloud.h"
#include "utils/useful_typedefs.h"

namespace depth_clustering
{

/**
 * @brief      Class for cloud odom ros subscriber.
 */
class LaserRosSubscriber : public AbstractSender<Cloud>
{
      using PointCloudT = sensor_msgs::PointCloud2;
    using LaserT = sensor_msgs::LaserScan;
    using OdometryT = nav_msgs::Odometry;
    using ApproximateTimePolicy =
        // message_filters::sync_policies::ApproximateTime<PointCloudT, OdometryT>;
        message_filters::sync_policies::ApproximateTime<LaserT, OdometryT>;

  public:
    LaserRosSubscriber(ros::NodeHandle *node_handle,
                       const ProjectionParams &params,
                       const std::string &topic_scan,
                       const std::string &topic_odom = "");
    virtual ~LaserRosSubscriber()
    {
        delete _subscriber_odom;
        delete _subscriber_scan;
        delete _sync;
    }

    /**
   * @brief      Get synchronized odometry and cloud
   *
   * @param[in]  msg_cloud  The message cloud
   * @param[in]  msg_odom   The message odom
   */
    void Callback(const LaserT::ConstPtr &msg_cloud,
                  const OdometryT::ConstPtr &msg_odom);

    /**
   * @brief      Get point cloud from ROS
   *
   * @param[in]  msg_cloud  The message cloud
   */
    void CallbackVelodyne(const LaserT::ConstPtr &msg_cloud);

    /**
   * @brief      Starts listening to ros.
   */
    void StartListeningToRos();

  protected:
    Pose RosOdomToPose(const OdometryT::ConstPtr &msg);
    ////
    Cloud::Ptr RosScanToCloud(const LaserT::ConstPtr &msg);
    // Cloud::Ptr RosScanOdomToCloud(const LaserT::ConstPtr &scan_msg, const OdometryT::ConstPtr &odom_msg);
    Cloud::Ptr RosScanToCloudTF(const LaserT::ConstPtr &msg);
    int num_of_scans=0;
    bool first_scan = true;//for debuging
    ros::Time first_scan_time;
    Cloud cloud;
    

    ros::NodeHandle *_node_handle;

    message_filters::Subscriber<LaserT> *_subscriber_scan;
    message_filters::Subscriber<OdometryT> *_subscriber_odom;
    message_filters::Synchronizer<ApproximateTimePolicy> *_sync;
    std::string _topic_scan;
    std::string _topic_odom;

    ProjectionParams _params;

    int _msg_queue_size;
};

} // namespace depth_clustering

#endif // SRC_ROS_BRIDGE_CLOUD_ODOM_ROS_SUBSCRIBER_H_
