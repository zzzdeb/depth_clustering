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

#ifndef SRC_OBJECTS_PUBLISHER_
#define SRC_OBJECTS_PUBLISHER_

#include <map>
#include <string>
#include <unordered_map>
#include <vector>
#include <string>

#include "communication/abstract_client.h"
#include "utils/cloud.h"
#include "utils/useful_typedefs.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/point_cloud.h>

#include <opencv2/imgproc/imgproc.hpp>

// #include <Eigen/Geometry> 

namespace depth_clustering
{

class ObjectsPublisher
    : public AbstractClient<std::unordered_map<uint16_t, Cloud>>
{
public:
  ObjectsPublisher() : AbstractClient<std::unordered_map<uint16_t, Cloud>>(), 
                      _use_abb{true} { }
  ObjectsPublisher(ros::NodeHandle& nh);
  
  void OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud> &clouds,
                           const int id) override;
  
  virtual ~ObjectsPublisher() {}

  void MinBB(const pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_ptr,
            std::pair<geometry_msgs::Pose, geometry_msgs::Vector3>& transfromation);

  void MinRecArea(const pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_ptr,
            std::pair<geometry_msgs::Pose, geometry_msgs::Vector3>& transfromation);

  void AxisAlignedBB(const Cloud& cloud,
            std::pair<geometry_msgs::Pose, geometry_msgs::Vector3>& transfromation);

  void PublishObjects(const std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Vector3> >& object);

  void SetUseABB(bool use_abb) {_use_abb = use_abb;}

protected:
    ros::NodeHandle _nh;
    ros::Publisher _objects_pub;
    std::string _frame_id;

private:
  bool _use_abb;
};

} // namespace depth_clustering

#endif // SRC_VISUALIZATION_VISUALIZER_H_
