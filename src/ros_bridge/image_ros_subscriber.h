// Copyright (C) 2017  E.Zolboo, RWTH Aachen

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

#ifndef SRC_ROS_BRIDGE_IMAGE_ROS_SUBSCRIBER_H_
#define SRC_ROS_BRIDGE_IMAGE_ROS_SUBSCRIBER_H_

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
class ImageRosSubscriber : public AbstractSender<Cloud> {
  using ImageP = sensor_msgs::ImageConstPtr;

 public:
  ImageRosSubscriber(const ros::NodeHandle& nh,
                         const ProjectionParams& params,
                         const std::string& topic_image
                        );
  virtual ~ImageRosSubscriber() {
  }

  /**
   * @brief      Get image from ROS
   *
   * @param[in]  msg_cloud  The message image
   */
  void CallbackImage(const sensor_msgs::ImageConstPtr& msg_image);

 protected:
  Cloud::Ptr ImageToCloud(const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle _nh;

  image_transport::ImageTransport _it;
  image_transport::Subscriber _sub;

  std::string _topic_image;

  ProjectionParams _params;

  int _msg_queue_size;
};

}  // namespace depth_clustering

#endif  // SRC_ROS_BRIDGE_CLOUD_ODOM_ROS_SUBSCRIBER_H_
