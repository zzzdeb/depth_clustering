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

#include "ros_bridge/image_ros_subscriber.h"
#include <eigen_conversions/eigen_msg.h>

#include <algorithm>
#include <string>
#include <vector>

#include "utils/pose.h"

namespace depth_clustering {

using ros::NodeHandle;
using sensor_msgs::Image;

using std::vector;
using std::string;
using std::map;

ImageRosSubscriber::ImageRosSubscriber(const NodeHandle& nh,
                                       const ProjectionParams& params,
                                       const string& topic_image)
    : AbstractSender{SenderType::STREAMER},
      _nh{nh},
      _it{nh},
      _topic_image{topic_image},
      _params{params},
      _msg_queue_size{100} 
{
  _sub = _it.subscribe(topic_image, _msg_queue_size,
                       &ImageRosSubscriber::CallbackImage, this);
}

void ImageRosSubscriber::CallbackImage(
    const sensor_msgs::ImageConstPtr& msg_image) {
  // PrintMsgStats(msg_image);
  Cloud::Ptr cloud_ptr = ImageToCloud(msg_image);
  ShareDataWithAllClients(*cloud_ptr);
}

Cloud::Ptr ImageRosSubscriber::ImageToCloud(
    const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cvimg =
      cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
  cv::Mat depth_image;
  cvimg->image.convertTo(depth_image, CV_32F);
  depth_image /= 500;
  return Cloud().FromImage(depth_image, _params);
}

}  // namespace depth_clustering
