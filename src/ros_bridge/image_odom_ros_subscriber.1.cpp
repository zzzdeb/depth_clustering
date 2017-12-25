// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

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

#include "ros_bridge/image_odom_ros_subscriber.h"
#include <eigen_conversions/eigen_msg.h>

#include <algorithm>
#include <string>
#include <vector>

#include "utils/pose.h"

namespace depth_clustering {

using ros::NodeHandle;
using message_filters::Subscriber;
using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;
using nav_msgs::Odometry;
using sensor_msgs::Image;

using std::vector;
using std::string;
using std::map;


// void PrintMsgStats(const sensor_msgs::PointCloud2ConstPtr& msg) {
//   fprintf(stderr, "<<<<<<<<<<<<<<< new cloud >>>>>>>>>>>>>>>\n");
//   fprintf(stderr, "received msg   %d\n", msg->header.seq);
//   fprintf(stderr, "height:        %d\n", msg->height);
//   fprintf(stderr, "width:         %d\n", msg->width);
//   fprintf(stderr, "num of fields: %lu\n", msg->fields.size());
//   fprintf(stderr, "fields of each point:\n");
//   for (auto const& pointField : msg->fields) {
//     fprintf(stderr, "\tname:     %s\n", pointField.name.c_str());
//     fprintf(stderr, "\toffset:   %d\n", pointField.offset);
//     fprintf(stderr, "\tdatatype: %d\n", pointField.datatype);
//     fprintf(stderr, "\tcount:    %d\n", pointField.count);
//     fprintf(stderr, "\n");
//   }
//   fprintf(stderr, "is bigendian:  %s\n", msg->is_bigendian ? "true" : "false");
//   fprintf(stderr, "point step:    %d\n", msg->point_step);
//   fprintf(stderr, "row step:      %d\n", msg->row_step);
//   fprintf(stderr, "data size:     %lu\n", msg->data.size() * sizeof(msg->data));
//   fprintf(stderr, "is dense:      %s\n", msg->is_dense ? "true" : "false");
//   fprintf(stderr, "=========================================\n");
// }

ImageOdomRosSubscriber::ImageOdomRosSubscriber(NodeHandle* node_handle,
                                               const ProjectionParams& params,
                                               const string& topic_image,
                                               const string& topic_odom)
    : AbstractSender{SenderType::STREAMER}, _params{params} {
  _node_handle = node_handle;
  _topic_image = topic_image;
  _topic_odom = topic_odom;
  _msg_queue_size = 100;

  _subscriber_image = nullptr;
  _subscriber_odom = nullptr;
  _sync = nullptr;
}

void ImageOdomRosSubscriber::StartListeningToRos() {
  if (!_topic_odom.empty()) {
    _subscriber_image = new Subscriber<sensor_msgs::ImageConstPtr>(
        *_node_handle, _topic_image, _msg_queue_size);
    _subscriber_odom =
        new Subscriber<Odometry>(*_node_handle, _topic_odom, _msg_queue_size);
    _sync = new Synchronizer<ApproximateTimePolicy>(
        ApproximateTimePolicy(100), *_subscriber_image, *_subscriber_odom);
    _sync->registerCallback(
        boost::bind(&ImageOdomRosSubscriber::Callback, this, _1, _2));
  } else {
    _subscriber_image = new Subscriber<sensor_msgs::ImageConstPtr>(
        *_node_handle, _topic_image, _msg_queue_size);
    _subscriber_image->registerCallback(
        &ImageOdomRosSubscriber::CallbackImage, this);
  }
}

void ImageOdomRosSubscriber::Callback(const sensor_msgs::ImageConstPtr& msg_image,
                                      const Odometry::ConstPtr& msg_odom) {
  // PrintMsgStats(msg_image);
  Cloud::Ptr cloud_ptr = ImageToCloud(msg_image);
  cloud_ptr->SetPose(RosOdomToPose(msg_odom));
  ShareDataWithAllClients(*cloud_ptr);
}

void ImageOdomRosSubscriber::CallbackImage(
    const sensor_msgs::ImageConstPtr& msg_image) {
  // PrintMsgStats(msg_image);
  Cloud::Ptr cloud_ptr = ImageToCloud(msg_image);
  ShareDataWithAllClients(*cloud_ptr);
}

Pose ImageOdomRosSubscriber::RosOdomToPose(const Odometry::ConstPtr& msg) {
  Pose pose;
  // we want float, so some casting is needed
  Eigen::Affine3d pose_double;
  tf::poseMsgToEigen(msg->pose.pose, pose_double);
  pose = pose_double.cast<float>();
  return pose;
}

Cloud::Ptr ImageOdomRosSubscriber::ImageToCloud(
    const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cvimg = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  return Cloud().FromImage(cvimg->image, _params);
}

}  // namespace depth_clustering
