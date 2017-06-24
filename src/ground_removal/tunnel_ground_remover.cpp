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

#include <memory>
#include <algorithm>

#include "utils/velodyne_utils.h"
#include "image_labelers/linear_image_labeler.h"
#include "image_labelers/diff_helpers/angle_diff.h"
#include "image_labelers/diff_helpers/simple_diff.h"
#include "utils/timer.h"


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
    fprintf(stderr, "No projection in cloud..\n");
  }

  Cloud cloud_copy(cloud);

  Timer total_timer;
  PointCloudT::Ptr pcl_cloud_p = cloud.ToPcl();
  if (_use_mbb)
    RemoveGroundOBB(pcl_cloud_p);
  else
    RemoveGroundAABB(pcl_cloud_p);
  
  uint64_t end = total_timer.measure();
  fprintf(stderr, "INFO: Ground removed in %lu us\n", end);
  
  Cloud::Ptr groundless_cloud_p = cloud.FromPcl(*pcl_cloud_p);
  groundless_cloud_p->SetProjectionPtr(cloud_copy.projection_ptr()->Clone());

  cloud_copy.projection_ptr()->depth_image() = groundless_cloud_p->projection_ptr()->depth_image();
  
  if(end>2000000) //!!! must be provided
  {
    _use_mbb = false;
    fprintf(stderr, "INFO: ground_removal set to without MinBB");
  }
  else
    if (end<1500000) //!!! must be provided
    {
      _use_mbb = true;
      fprintf(stderr, "INFO: ground_removal set to use MinBB");
    }

  this->ShareDataWithAllClients(cloud_copy);
  _counter++;
}

void TunnelGroundRemover::RemoveGroundOBB(const PointCloudT::Ptr& cloud_p)
{
  pcl::PointXYZL min_point_OBB;
  pcl::PointXYZL max_point_OBB;
  pcl::PointXYZL position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;

  pcl::MomentOfInertiaEstimation<pcl::PointXYZL> feature_extractor; 
  feature_extractor.setInputCloud (cloud_p);
  feature_extractor.compute (); //!!! is it necessarily to compute all feateres?
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
}

void TunnelGroundRemover::RemoveGroundAABB(const PointCloudT::Ptr& cloud_p)
{
  pcl::PointXYZL min_point_OBB;
  pcl::PointXYZL max_point_OBB;
  pcl::PointXYZL position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;

  pcl::MomentOfInertiaEstimation <pcl::PointXYZL> feature_extractor; 
  feature_extractor.setInputCloud (cloud_p);
  feature_extractor.compute (); //!!! is it necessarily to compute all feateres?
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
}


}  // namespace depth_clustering
