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

#ifndef _TUNNEL_GROUND_REMOVER_
#define _TUNNEL_GROUND_REMOVER_

#include <opencv2/opencv.hpp>

#include <string>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "projections/projection_params.h"
#include "utils/cloud.h"

#include "utils/radians.h"

#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "./depth_ground_remover.h"

#include "ground_removal/abstract_ground_remover.h"
#include "utils/savitsky_golay_smoothing.h"

namespace depth_clustering {

typedef pcl::PointXYZL PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/**
 * @brief      A class to remove ground based upon pca axis
 * @details     A class to remove ground based upon pca axis
 *
 *
 * @param      params  projection params
 */
class TunnelGroundRemover : public AbstractGroundRemover {
  using ClientT = AbstractClient<Cloud>;
  using SenderT = AbstractSender<Cloud>;

 public:
  explicit TunnelGroundRemover(const ProjectionParams& params, double height,
                               double sensor_h, int window_size = 5,
                               bool use_pca = true)
      : AbstractGroundRemover(),
        _params{params},
        _smoother{params, window_size},
        _height{height},
        _sensor_h{sensor_h},
        _use_pca{use_pca},
        _window_size{window_size} {
    InitProjectors();
  }
  virtual ~TunnelGroundRemover() {}

  /**
   * @brief      when someone sends us an object we process it
   * @details    receiving a depth image we remove ground from it and send to
   *             next recepient
   *
   * @param      cloud        Cloud
   * @param      sender_id    id of the sender
   */
  void OnNewObjectReceived(const Cloud& cloud, const int sender_id) override;
  /**
 * @brief      removes ground by height on vertical axis, calculated through
 *              PCA.
 * @details    removes ground by height on vertical axis, calculated through
 *              PCA.
 *
 * @param[in]      cloud        Cloud
 * @param[in]      image        DepthImage of the cloud (smoothed)
 *
 * @return         depth image where ground depth set to zero.
 */
  cv::Mat Remove(const Cloud& cloud, const cv::Mat& image) const;

 private:
  /**
    * @brief      Initialize 3 projector Images of each axis based on
    *                parameters
    * @details    Initialize 3 projector Images of each axis based on
    *                parameters
    *
    */
  void InitProjectors();

 protected:

  ProjectionParams _params;
  float _eps = 0.001f;
  SavitskyGolaySmoothing _smoother;

 private:
  double _height;
  double _sensor_h;
  bool _use_pca;
  int _window_size = 5;
  mutable int _counter = 0;

  cv::Mat _x_projector;
  cv::Mat _y_projector;
  cv::Mat _z_projector;
};

}  // namespace depth_clustering

#endif  // SRC_GROUND_REMOVAL_DEPTH_GROUND_REMOVER_H_
