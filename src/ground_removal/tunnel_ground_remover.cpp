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

#include "./tunnel_ground_remover.h"

#include <opencv2/highgui/highgui.hpp>

#include <algorithm>
#include <memory>

#include "image_labelers/diff_helpers/angle_diff.h"
#include "image_labelers/diff_helpers/simple_diff.h"
#include "image_labelers/linear_image_labeler.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"

#include <ros/console.h>

namespace depth_clustering {

using cv::Mat;
using cv::DataType;
using std::to_string;
using time_utils::Timer;

const cv::Point ANCHOR_CENTER = cv::Point(-1, -1);
const int SAME_OUTPUT_TYPE = -1;

void TunnelGroundRemover::OnNewObjectReceived(const Cloud& cloud,
                                              const int sender_id) {
  // this can be done even faster if we switch to column-major implementation
  // thus allowing us to load whole row in L1 cache
  if (!cloud.projection_ptr()) {
    ROS_ERROR("No projection in cloud..");
  }

  Cloud cloud_copy(cloud);

  Timer total_timer;
  PointCloudT::Ptr pcl_cloud_p = cloud.ToPcl();
  PointCloudT gl_pcl_p;
  if (_use_obb)
    RemoveGroundOBB(pcl_cloud_p, gl_pcl_p);
  else
    RemoveGroundAABB(pcl_cloud_p, gl_pcl_p);

  uint64_t end = total_timer.measure();
  ROS_INFO("Tunnel Ground removed in %lu us", end);

  Cloud::Ptr groundless_cloud_p = cloud.FromPcl(gl_pcl_p);
  groundless_cloud_p->InitProjection(_params);

  // DepthGroundRemover dgr(_params, 5_deg);
  // const cv::Mat& depth_image =
  //     dgr.RepairDepth(groundless_cloud_p->projection_ptr()->depth_image(), 5,
  //     1.0f);
  // auto angle_image = dgr.CreateAngleImage(depth_image);
  // auto smoothed_image = dgr.ApplySavitskyGolaySmoothing(angle_image,
  // _window_size);

  // cloud_copy.projection_ptr()->depth_image() = smoothed_image;
  cloud_copy.projection_ptr()->depth_image() =
      groundless_cloud_p->projection_ptr()->depth_image();

  if (end > 2000000)  //!!! must be provided
  {
    _use_obb = false;
    ROS_INFO("ground_removal set to without OBB");
  } else if (end < 1500000)  //!!! must be provided
  {
    _use_obb = true;
    ROS_INFO("ground_removal set to use OBB");
  }

  this->ShareDataWithAllClients(cloud_copy);
  _counter++;
}

void TunnelGroundRemover::RemoveGroundOBB(const PointCloudT::Ptr& cloud_p,
                                          PointCloudT& gl_cloud) {
  pcl::PointXYZL min_point_OBB;
  pcl::PointXYZL max_point_OBB;
  pcl::PointXYZL position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;

  OrientedBoundingBox<pcl::PointXYZL> feature_extractor;
  feature_extractor.setAngleStep(20);  //!!! must be tuned
  feature_extractor.setInputCloud(cloud_p);
  Timer timer;
  feature_extractor.compute();
  ROS_INFO("computing OBB took %lu us ", timer.measure());
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
                           rotational_matrix_OBB);

  for (auto point : *cloud_p) {
    if (point.z > -1) {
      PointT p(point);
      gl_cloud.push_back(p);
    }
  }
  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(gl_cloud, cloud2);
  cloud2.header.frame_id = "velodyne"; //!!!
  cloud2.header.stamp = ros::Time::now();
  _cloud_pub.publish(cloud2);

  geometry_msgs::Vector3 scale;
  scale.x = min_point_OBB.x;
  scale.y = min_point_OBB.y;
  scale.z = min_point_OBB.z;
  geometry_msgs::Pose pose;
  pose.position.x = position_OBB.x;
  pose.position.y = position_OBB.y;
  pose.position.z = position_OBB.z;
  Eigen::Quaternionf q(rotational_matrix_OBB);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  visualization_msgs::MarkerArray obj_markers;  //!!! Type
  visualization_msgs::Marker marker;
  int id = 0;
  ros::Time stamp = ros::Time::now();  //!!! is it really now?
                                       // for (const auto &object: objects)
                                       // {

  marker.header.frame_id = "velodyne";  //!!!
  marker.header.stamp = stamp;
  marker.ns = "";
  marker.id = id++;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale = scale;
  marker.color.a = 1;  // Don't forget to set the alpha!
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0;
  // only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  obj_markers.markers.push_back(marker);
  // }
  _marker_pub.publish(obj_markers);
}

void TunnelGroundRemover::RemoveGroundAABB(const PointCloudT::Ptr& cloud_p,
                                           PointCloudT& gl_cloud) {
  // pcl::PointXYZL min_point_AABB;
  // pcl::PointXYZL max_point_AABB;
  // pcl::PointXYZL position_AABB;
  // Eigen::Matrix3f rotational_matrix_OBB;

  // pcl::MomentOfInertiaEstimation<pcl::PointXYZL> feature_extractor;
  // feature_extractor.setInputCloud(cloud_p);
  // feature_extractor.compute();  //!!! is it necessarily to compute all feateres?
  // feature_extractor.getAABB(min_point_OBB, max_point_OBB, position_OBB,
  //                          rotational_matrix_OBB);
}

}  // namespace depth_clustering
