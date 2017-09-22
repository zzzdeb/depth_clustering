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

#include "./tunnel_ground_remover.h"

#include <ros/console.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

#include "utils/timer.h"
#include "utils/velodyne_utils.h"

namespace depth_clustering {

using std::abs;

using cv::Mat;
using cv::DataType;
using time_utils::Timer;

const cv::Point ANCHOR_CENTER = cv::Point(-1, -1);
const int SAME_OUTPUT_TYPE = -1;

void TunnelGroundRemover::OnNewObjectReceived(const Cloud& cloud,
                                              const int sender_id) {

  if (!cloud.projection_ptr()) {
    ROS_INFO("No projection in cloud..");
  }

  Cloud cloud_copy(cloud);

  const cv::Mat& depth_image =
      _smoother.RepairDepth(cloud.projection_ptr()->depth_image(), 5, 1.0f);

  Timer total_timer;

  auto no_ground_image = Remove(cloud, depth_image);

  ROS_INFO("Tunnel Ground removed in %lu us", total_timer.measure());

  cloud_copy.projection_ptr()->depth_image() = no_ground_image;

  this->ShareDataWithAllClients(cloud_copy);
  _counter++;
}

Mat TunnelGroundRemover::Remove(const Cloud& cloud,
                                const cv::Mat& image) const {
  Timer timer;
  Mat res = image;

  Mat z_projected = image.mul(_z_projector);
  // debug!!
  cv::FileStorage file("/home/zzz/test/z_projector.ext",
                       cv::FileStorage::WRITE);
  file << "z_projector " << _z_projector;
  file << "z_projected " << z_projected;
  // for simple height ground removal, projecting on z axis is enough.
  Mat projected = z_projected;

  // using pca
  if (_use_pca) {
    PointCloudT::Ptr pcl_cloud = cloud.ToPcl();
    pcl::PCA<pcl::PointXYZL> pca;
    // feature_extractor.setAngleStep(20);  //!!! must be tuned
    pca.setInputCloud(pcl_cloud);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

    ROS_INFO("executing PCA took %lu us ", timer.measure());

    float v1z = abs(eigen_vectors(0, 2));
    float v2z = abs(eigen_vectors(1, 2));
    float v3z = abs(eigen_vectors(2, 2));

    // eigenVector
    float x, y, z;
    int i;
    if (v1z > v2z)
      if (v1z > v3z)
        i = 0;
      else
        i = 2;
    else if (v2z > v3z)
      i = 1;
    else
      i = 2;
    x = eigen_vectors(i, 0);
    y = eigen_vectors(i, 1);
    z = eigen_vectors(i, 2);

    Mat x_projected = image.mul(_x_projector, x);
    Mat y_projected = image.mul(_y_projector, y);
    projected = z_projected * z + x_projected + y_projected;
  }

  for (int i = 0; i < projected.rows; i++)
    //!!!
    for (int j = 0; j < projected.cols; j++)
      if (projected.at<float>(i, j) < _height - _sensor_h)
        res.at<float>(i, j) = 0;
  return res;
}

void TunnelGroundRemover::InitProjectors() {
  // Declare what you need
  cv::FileStorage file("/home/zzz/test/some_name.ext", cv::FileStorage::WRITE);

  _z_projector = Mat(_params.rows(), _params.cols(), DataType<float>::type);

  file << "_z_projector " << _z_projector;
  for (unsigned int i = 0; i < _params.rows(); i++) {
    //!!!
    for (unsigned int j = 0; j < _params.cols(); j++)
      _z_projector.at<float>(i, j) = _params.RowAngleSines().at(i);
  }

  // Write to file!
  file << "_z_projector after " << _z_projector;
  _x_projector = Mat(_params.rows(), _params.cols(), DataType<float>::type);
  //!!! finde heraus welche welche ist

  file << "_x_projector" << _x_projector;
  for (unsigned int i = 0; i < _params.rows(); i++) {
    for (unsigned int j = 0; j < _params.cols(); j++)
      _x_projector.at<float>(i, j) =
          _params.RowAngleCosines().at(i) * _params.ColAngleCosines().at(j);
  }
  file << "_x_projector after " << _x_projector;

  _y_projector = Mat(_params.rows(), _params.cols(), DataType<float>::type);
  file << "_y_projector " << _y_projector;
  for (unsigned int i = 0; i < _params.rows(); i++) {
    for (unsigned int j = 0; j < _params.cols(); j++)
      _y_projector.at<float>(i, j) =
          _params.RowAngleCosines().at(i) * _params.ColAngleSines().at(j);
  }
  file << "_y_projector after " << _y_projector;
}

}  // namespace depth_clustering
