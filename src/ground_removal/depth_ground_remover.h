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

#ifndef SRC_GROUND_REMOVAL_DEPTH_GROUND_REMOVER_H_
#define SRC_GROUND_REMOVAL_DEPTH_GROUND_REMOVER_H_

#include <opencv2/opencv.hpp>

#include <algorithm>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/radians.h"

#include "ground_removal/abstract_ground_remover.h"
#include "utils/savitsky_golay_smoothing.h"

namespace depth_clustering {

/**
 * @brief      A class to remove ground based upon depth image
 * @details    Given a depth image and image config this class should remove the
 *             ground and send the new depth image with no ground further
 *             through the pipeline to its clients.
 *
 * @param      params  projection params
 */
class DepthGroundRemover : public AbstractGroundRemover {
  using ClientT = AbstractClient<Cloud>;
  using SenderT = AbstractSender<Cloud>;

 public:
  explicit DepthGroundRemover(const ProjectionParams& params,
                              const Radians& ground_remove_angle,
                              int window_size = 5)
      : AbstractGroundRemover{},
        _params{params},
        _window_size{window_size},
        _ground_remove_angle{ground_remove_angle},
        _smoother{params, window_size} {}
  virtual ~DepthGroundRemover() {}

  /**
   * @brief      when someone sends us an object we process it
   * @details    receiving a depth image we remove ground from it and send to
   *             next recepient
   *
   * @param      depth_image  32 bit depth image
   * @param      sender_id    id of the sender
   */
  void OnNewObjectReceived(const Cloud& cloud, const int sender_id) override;

  /**
   * @brief      Zero out all pixels that belong to ground
   *
   * @param[in]  image        Input depth image
   * @param[in]  angle_image  The angle image
   * @param[in]  threshold    angle threshold
   *
   * @return     depth image with 0 instead of ground pixels
   */
  cv::Mat ZeroOutGround(const cv::Mat& image, const cv::Mat& angle_image,
                        const Radians& threshold) const;

  cv::Mat ZeroOutGroundBFS(const cv::Mat& image, const cv::Mat& angle_image,
                           const Radians& threshold, int kernel_size) const;

  /**
   * @brief      create a help image with angle in radians written for each
   *             pixel
   *
   * @param      depth_image  [input depth image]
   * @return     [32 bit float image with angle in radians written in every
   *             pixel]
   */

 protected:
  ProjectionParams _params;
  int _window_size = 5;
  Radians _ground_remove_angle = 5_deg;
  SavitskyGolaySmoothing _smoother;

  mutable int _counter = 0;
};

}  // namespace depth_clustering

#endif  // SRC_GROUND_REMOVAL_DEPTH_GROUND_REMOVER_H_
