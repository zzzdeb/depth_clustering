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

#ifndef _ABSTRACT_GROUND_REMOVER_
#define _ABSTRACT_GROUND_REMOVER_

#include <opencv2/opencv.hpp>

#include <algorithm>

#include "communication/abstract_client.h"
#include "communication/abstract_sender.h"
#include "projections/projection_params.h"

#include "utils/cloud.h"

namespace depth_clustering {

/**
 * @brief      A class to remove ground based upon depth image
 * @details    Given a depth image and image config this class should remove the
 *             ground and send the new depth image with no ground further
 *             through the pipeline to its clients.
 *
 * @param      params  projection params
 */
class AbstractGroundRemover : public AbstractClient<Cloud>,
                           public AbstractSender<Cloud> {
  using ClientT = AbstractClient<Cloud>;
  using SenderT = AbstractSender<Cloud>;

 public:
  explicit AbstractGroundRemover()
      : ClientT{},
        SenderT{SenderType::STREAMER}
        {}

  virtual ~AbstractGroundRemover() {}
  
};

}  // namespace depth_clustering

#endif  // _ABSTRACT_GROUND_REMOVER_
