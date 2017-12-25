// Copyright (C) 2017  E. Zolboo, I. Bogoslavskyi, C. Stachniss, University of Bonn

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

#ifndef SRC_ROS_BRIDGE_IMAGE_ODOM_ROS_SUBSCRIBER_H_
#define SRC_ROS_BRIDGE_IMAGE_ODOM_ROS_SUBSCRIBER_H_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <map>
#include <string>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "communication/abstract_sender.h"
#include "utils/cloud.h"
#include "utils/pose.h"
#include "utils/useful_typedefs.h"


namespace depth_clustering {

/**
 * @brief      Class for cloud odom ros subscriber.
 */
class ImageOdomRosSubscriber : public AbstractSender<Cloud> {
  using ImageP = sensor_msgs::ImageConstPtr;
  using OdometryT = nav_msgs::Odometry;
  using ApproximateTimePolicy =
      message_filters::sync_policies::ApproximateTime<ImageP, OdometryT>;

 public:
  ImageOdomRosSubscriber(ros::NodeHandle* node_handle,
                         const ProjectionParams& params,
                         const std::string& topic_image,
                         const std::string& topic_odom = "");
  virtual ~ImageOdomRosSubscriber() {
    delete _subscriber_odom;
    delete _subscriber_image;
    delete _sync;
  }

  /**
   * @brief      Get synchronized odometry and image
   *
   * @param[in]  msg_image  The message image
   * @param[in]  msg_odom   The message odom
   */
  void Callback(const sensor_msgs::ImageConstPtr& msg_image,
                const OdometryT::ConstPtr& msg_odom);

  /**
   * @brief      Get image from ROS
   *
   * @param[in]  msg_cloud  The message image
   */
  void CallbackImage(const sensor_msgs::ImageConstPtr& msg_image);

  /**
   * @brief      Starts listening to ros.
   */
  void StartListeningToRos();

 protected:
  Pose RosOdomToPose(const OdometryT::ConstPtr& msg);
  Cloud::Ptr ImageToCloud(const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle* _node_handle;

  message_filters::Subscriber<sensor_msgs::Image>* _subscriber_image;
  message_filters::Subscriber<OdometryT>* _subscriber_odom;
  message_filters::Synchronizer<ApproximateTimePolicy>* _sync;
  std::string _topic_image;
  std::string _topic_odom;

  ProjectionParams _params;

  int _msg_queue_size;
};

}  // namespace depth_clustering

#endif  // SRC_ROS_BRIDGE_CLOUD_ODOM_ROS_SUBSCRIBER_H_
