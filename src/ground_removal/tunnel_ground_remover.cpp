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

 #include <tf/LinearMath/Quaternion.h>

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
  // for simple height ground removal, projecting on z axis is enough.
  Mat projected = z_projected;

  // using pca
  if (_use_pca) {
    PointCloudT::Ptr pcl_cloud = cloud.ToPcl();
    // ROS_ERROR("cloud has %i points", pcl_cloud->size());
    // //downsampling
    PointCloudT::Ptr cloud_filtered(new PointCloudT());
  
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZL> sor;
    sor.setInputCloud (pcl_cloud);
    sor.setLeafSize (0.3f, 0.3f, 0.3f);
    sor.filter (*cloud_filtered);

    // ROS_ERROR("filtered has %i points", cloud_filtered->size());
    // ROS_INFO("Filtering took %lu us ", timer.measure());

    pcl::PCA<pcl::PointXYZL> pca;
    pca.setInputCloud(cloud_filtered);//!!!
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

    ROS_INFO("executing PCA took %lu us ", timer.measure());

    auto pca1 = eigen_vectors.col(0);
    Eigen::Vector3f rot_v = pca1.cross(Eigen::Vector3f(0, 0, 1));


    Eigen::Matrix3f rot_m = Eigen::AngleAxisf(0.5 * M_PI, rot_v).toRotationMatrix();
    Eigen::Vector3f v_axis = rot_m*pca1;

    double x = v_axis(0);
    double y = v_axis(1);
    double z = v_axis(2);
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
    // Write to file!
  // cv::FileStorage file("/home/zzz/test/some_name.ext", cv::FileStorage::WRITE);

  _z_projector = Mat(_params.rows(), _params.cols(), DataType<float>::type);

  // file << "_z_projector " << _z_projector;
  for (unsigned int i = 0; i < _params.rows(); i++) {
    //!!!
    for (unsigned int j = 0; j < _params.cols(); j++)
      _z_projector.at<float>(i, j) = _params.RowAngleSines().at(i);
  }


  // file << "_z_projector after " << _z_projector;
  _x_projector = Mat(_params.rows(), _params.cols(), DataType<float>::type);
  //!!! finde heraus welche welche ist

  // file << "_x_projector" << _x_projector;
  for (unsigned int i = 0; i < _params.rows(); i++) {
    for (unsigned int j = 0; j < _params.cols(); j++)
      _x_projector.at<float>(i, j) =
          _params.RowAngleCosines().at(i) * _params.ColAngleCosines().at(j);
  }
  // file << "_x_projector after " << _x_projector;

  _y_projector = Mat(_params.rows(), _params.cols(), DataType<float>::type);
  // file << "_y_projector " << _y_projector;
  for (unsigned int i = 0; i < _params.rows(); i++) {
    for (unsigned int j = 0; j < _params.cols(); j++)
      _y_projector.at<float>(i, j) =
          _params.RowAngleCosines().at(i) * _params.ColAngleSines().at(j);
  }
  // file << "_y_projector after " << _y_projector;
}

}  // namespace depth_clustering
