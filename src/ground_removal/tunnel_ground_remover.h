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

#ifndef _TUNNEL_GROUND_REMOVER_
#define _TUNNEL_GROUND_REMOVER_0

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <string>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/oriented_bounding_box.h"
#include "utils/radians.h"

#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "./depth_ground_remover.h"

#include "ground_removal/abstract_ground_remover.h"

namespace depth_clustering {

typedef pcl::PointXYZL PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/**
 * @brief      A class to remove ground based upon depth image
 * @details    Given a depth image and image config this class should remove the
 *             ground and send the new depth image with no ground further
 *             through the pipeline to its clients.
 *
 * @param      params  projection params
 */
class TunnelGroundRemover : public AbstractGroundRemover {
  using ClientT = AbstractClient<Cloud>;
  using SenderT = AbstractSender<Cloud>;

 public:
  explicit TunnelGroundRemover(ros::NodeHandle& nh,
                               const ProjectionParams& params, double height,
                               double sensor_h, int window_size = 5,
                               bool use_obb = true)
      : AbstractGroundRemover(),
        _nh{nh},
        _marker_pub{
            _nh.advertise<visualization_msgs::MarkerArray>("tunnel", 1)},
        _cloud_pub{
            _nh.advertise<sensor_msgs::PointCloud2>("ground_remover", 1)},
        _params{params},
        _height{height},
        _sensor_h{sensor_h},
        _smoother{params, window_size},
        _window_size{window_size},
        _use_obb{use_obb} {
    _nh.getParam("node/laser_frame", _frame_id);
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

  void RemoveGroundOBB(const PointCloudT::Ptr& cloud_p, PointCloudT& gl_cloud);
  void RemoveGroundByHeight(const PointCloudT::Ptr& cloud_p,
                            PointCloudT& gl_cloud);

  void PublishInfo(const PointCloudT& gl_cloud, const PointT& min_point_OBB,
                   const PointT& max_point_OBB, const Eigen::Matrix3f& rot_M,
                   const PointT& position_OBB = PointT());

 protected:
  ros::NodeHandle _nh;
  ros::Publisher _marker_pub, _cloud_pub;
  std::string _frame_id;

  ProjectionParams _params;
  float _eps = 0.001f;
  SavitskyGolaySmoothing _smoother;

 private:
  double _height;
  double _sensor_h;
  bool _use_obb;
  int _window_size = 5;
  mutable int _counter = 0;
};

}  // namespace depth_clustering

#endif  // SRC_GROUND_REMOVAL_DEPTH_GROUND_REMOVER_H_
