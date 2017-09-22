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

#include "./depth_ground_remover.h"

#include <opencv2/highgui/highgui.hpp>

#include <algorithm>

#include "utils/velodyne_utils.h"
#include "image_labelers/linear_image_labeler.h"
#include "image_labelers/diff_helpers/angle_diff.h"
#include "image_labelers/diff_helpers/simple_diff.h"
#include "image_labelers/diff_helpers/ground_diff.h"
#include "utils/timer.h"

#include <ros/console.h>

namespace depth_clustering {

using cv::Mat;
using cv::DataType;
using std::to_string;
using time_utils::Timer;

const cv::Point ANCHOR_CENTER = cv::Point(-1, -1);
const int SAME_OUTPUT_TYPE = -1;

void DepthGroundRemover::OnNewObjectReceived(const Cloud& cloud,
                                             const int sender_id) {
  // this can be done even faster if we switch to column-major implementation
  // thus allowing us to load whole row in L1 cache
  if (!cloud.projection_ptr()) {
    ROS_ERROR("No projection in cloud. Skipping ground removal.");
    return;
  }
  Cloud cloud_copy(cloud);
  const cv::Mat& depth_image =
      _smoother.RepairDepth(cloud.projection_ptr()->depth_image(), 5, 1.0f);
  Timer total_timer;
  auto angle_image = _smoother.CreateAngleImage(depth_image);
  auto smoothed_image = _smoother.ApplySavitskyGolaySmoothing(angle_image, _window_size);
  auto no_ground_image = ZeroOutGroundBFS(depth_image, smoothed_image,
                                          _ground_remove_angle, _window_size);
  ROS_INFO("Ground removed in %lu us", total_timer.measure());
  cloud_copy.projection_ptr()->depth_image() = no_ground_image;
  this->ShareDataWithAllClients(cloud_copy);
  _counter++;
}

Mat DepthGroundRemover::ZeroOutGround(const cv::Mat& image,
                                      const cv::Mat& angle_image,
                                      const Radians& threshold) const {
  // TODO(igor): test if its enough to remove only values starting from the
  // botom pixel. I don't like removing all values based on a threshold.
  // But that's a start, so let's stick with it for now.
  Mat res = cv::Mat::zeros(image.size(), CV_32F);
  for (int r = 0; r < image.rows; ++r) {
    for (int c = 0; c < image.cols; ++c) {
      if (angle_image.at<float>(r, c) > threshold.val()) {
        res.at<float>(r, c) = image.at<float>(r, c);
      }
    }
  }
  return res;
}

Mat DepthGroundRemover::ZeroOutGroundBFS(const cv::Mat& image,
                                         const cv::Mat& angle_image,
                                         const Radians& threshold,
                                         int kernel_size) const {
  Mat res = cv::Mat::zeros(image.size(), CV_32F);
  LinearImageLabeler<> image_labeler(image, _params, threshold);
  // SimpleDiff simple_diff_helper(&angle_image); !!!
  GroundDiff ground_diff_helper(&angle_image);
  Radians start_thresh = 30_deg;
  for (int c = 0; c < image.cols; ++c) {
    // start at bottom pixels and do bfs
    int r = image.rows - 1;
    while (r > 0 && image.at<float>(r, c) < 0.001f) {
      --r;
    }
    auto current_coord = PixelCoord(r, c);
    uint16_t current_label = image_labeler.LabelAt(current_coord);
    if (current_label > 0) {
      // this coord was already labeled, skip
      continue;
    }
    // TODO(igor): this is a test. Maybe switch it on, maybe off.
    if (angle_image.at<float>(r, c) > start_thresh.val()) {
      continue;
    }
    // image_labeler.LabelOneComponent(1, current_coord, &simple_diff_helper); !!!
    image_labeler.LabelOneComponent(1, current_coord, &ground_diff_helper);
  }
  auto label_image_ptr = image_labeler.GetLabelImage();
  if (label_image_ptr->rows != res.rows || label_image_ptr->cols != res.cols) {
    ROS_ERROR("label image and res do not correspond.");
    return res;
  }
  kernel_size = std::max(kernel_size - 2, 3);
  Mat kernel = _smoother.GetUniformKernel(kernel_size, CV_8U);
  Mat dilated = Mat::zeros(label_image_ptr->size(), label_image_ptr->type());
  cv::dilate(*label_image_ptr, dilated, kernel);
  for (int r = 0; r < dilated.rows; ++r) {
    for (int c = 0; c < dilated.cols; ++c) {
      if (dilated.at<uint16_t>(r, c) == 0) {
        // all unlabeled points are non-ground
        res.at<float>(r, c) = image.at<float>(r, c);
      }
    }
  }
  return res;
}



}  // namespace depth_clustering
