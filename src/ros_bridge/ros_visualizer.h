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

#ifndef _ROS_VISUALIZER_
#define _ROS_VISUALIZER_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <map>
#include <utility>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "communication/abstract_client.h"
#include "utils/cloud.h"
#include "utils/useful_typedefs.h"

#include <pcl/point_cloud.h>

namespace depth_clustering
{
  typedef pcl::PointXYZL PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

class RosVisualizer : public AbstractClient<std::pair<Cloud::Ptr, cv::Mat> >
{

public:
  explicit RosVisualizer();
  explicit RosVisualizer(ros::NodeHandle& nh);
  virtual ~RosVisualizer();

  void initNode(ros::NodeHandle &nh);
  void OnNewObjectReceived(const std::pair<Cloud::Ptr, cv::Mat> &cloud_pair, const int id) override;
  void LabelPCL(PointCloudT::Ptr pcl_cloud, const cv::Mat& label_image, const Cloud& cloud);
  void set_frame_id(const std::string &frame_id) { _frame_id = frame_id; }
  void fromCloudToPCL(PointCloudT::Ptr pcl_cloud, const Cloud &cloud);

protected:
  ros::NodeHandle _nh;
  ros::Publisher _depth_image_pub;
  ros::Publisher _label_image_pub;
  ros::Publisher _cloud_pub;
  
private:
 void PubImages(const cv::Mat &depth_image, const cv::Mat &label_image);
 void PubCloud(const PointCloudT &cloud);

 std::string _frame_id;
 int _min_cluster_size;
 int _max_cluster_size;
};

} // namespace depth_clustering
#endif