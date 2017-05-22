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

#include <ros/ros.h>
#include <ros/console.h>

#include <qapplication.h>

#include <string>

#include "ros_bridge/cloud_odom_ros_subscriber.h"
#include "ros_bridge/laser_ros_subscriber.h"

#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "projections/ring_projection.h"
#include "projections/spherical_projection.h"
#include "utils/radians.h"
#include "visualization/cloud_saver.h"
#include <visualization/visualizer.h>

#include "tclap/CmdLine.h"

using std::string;

using namespace depth_clustering;

using ClustererT = ImageBasedClusterer<LinearImageLabeler<>>;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fast_segmentation");
  Radians angle_tollerance = Radians::FromDegrees(angle_arg);

  std::unique_ptr<ProjectionParams> proj_params_ptr = nullptr;

    proj_params_ptr = ProjectionParams::IMR_LaserScanner();

    proj_params_ptr = ProjectionParams::Husky_2d();

  QApplication application(argc, argv);

  ros::init(argc, argv, "show_objects_node");
  ros::NodeHandle nh;

  string topic_clouds = "/assembled_laser";
  string topic_laser = "/hokuyo/scan/raw";
  string topic_odom = "";

  // LaserRosSubscriber subscriber(&nh, *proj_params_ptr, topic_laser, topic_odom=topic_odom); //CloudOdomRosSubscriber
  CloudOdomRosSubscriber subscriber(&nh, *proj_params_ptr, topic_clouds);
  VisualizerRos visualizer;
  visualizer.show();
  // visualizer.addNode(&nh);

  int min_cluster_size = 50;     // 20
  int max_cluster_size = 100000; //100000

  int smooth_window_size = 7;
  Radians ground_remove_angle = 7_deg;

  auto depth_ground_remover = DepthGroundRemover(
      *proj_params_ptr, ground_remove_angle, smooth_window_size);

  ClustererT clusterer(angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

  subscriber.AddClient(&depth_ground_remover);
  depth_ground_remover.AddClient(&clusterer);
  depth_ground_remover.AddClient(&visualizer); ///
  // subscriber.AddClient(&clusterer);
  clusterer.AddClient(visualizer.object_clouds_client());
  ///
  // clusterer.AddClient(&visualizer);

  fprintf(stderr, "INFO: Running with angle tollerance: %f degrees\n",
          angle_tollerance.ToDegrees());

  subscriber.StartListeningToRos();
  ros::AsyncSpinner spinner(1);
  spinner.start();

  auto exit_code = application.exec();

  // if we close application, still wait for ros to shutdown
  ros::waitForShutdown();
  return exit_code;
}
