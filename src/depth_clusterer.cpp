#include <string>
#include <thread>

#include <ros/console.h>
#include <ros/ros.h>

#include "clusterers/image_based_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"

#include "ground_removal/abstract_ground_remover.h"
#include "ground_removal/depth_ground_remover.h"
#include "ground_removal/tunnel_ground_remover.h"

#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/folder_reader.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"

#include "ros_bridge/cloud_odom_ros_subscriber.h"
#include "ros_bridge/objects_publisher.h"
#include "ros_bridge/ros_visualizer.h"

using std::string;
using std::vector;

using namespace depth_clustering;

using ClustererT = ImageBasedClusterer<LinearImageLabeler<>>;

void ReadData(const Radians& angle_tollerance, const string& in_path,
              RosVisualizer* visualizer, ObjectsPublisher* objects_publisher,
              ros::NodeHandle* nh) {
  // delay reading for one second to allow GUI to load
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // now load the data
  ROS_INFO("running on Moosman data");

  int min_cluster_size;
  if (nh->getParam("clusterer/min_cluster_size", min_cluster_size))
    ROS_INFO("Minimum cluster size: %i", min_cluster_size);
  else
    ROS_ERROR("Could not find ~/clusterer/min_cluster_size");

  int max_cluster_size;
  if (nh->getParam("clusterer/max_cluster_size", max_cluster_size))
    ROS_INFO("Minimum cluster size: %i", max_cluster_size);
  else
    ROS_ERROR("Could not find ~/clusterer/max_cluster_size");

  auto image_reader =
      FolderReader(in_path, ".png", FolderReader::Order::SORTED);
  auto config_reader = FolderReader(in_path, "img.cfg");
  auto proj_params_ptr =
      ProjectionParams::FromConfigFile(config_reader.GetNextFilePath());

  // Ground_remover
  AbstractGroundRemover* ground_remover;
  string remover;
  if (!nh->getParam("ground_remover/remover", remover)) {
    ROS_ERROR(
        "Could not find ~/ground_remover/remover. Go on without "
        "ground_remover");
  } else {
    if (remover == "depth_ground_remover") {
      int smooth_window_size;
      if (nh->getParam("ground_remover/smooth_window_size", smooth_window_size))
        ROS_INFO("ground_remover/smooth_window_size: %i", smooth_window_size);
      else
        ROS_ERROR("Could not find ~/ground_remover/smooth_window_size");
      float angle;
      if (nh->getParam("ground_remover/angle", angle))
        ROS_INFO("ground_remover/angle: %f", angle);
      else
        ROS_ERROR("Could not find ~/ground_remover/angle");
      Radians ground_remove_angle = Radians::FromDegrees(angle);

      ground_remover = new DepthGroundRemover(
          *proj_params_ptr, ground_remove_angle, smooth_window_size);
    }
    if (remover == "tunnel_ground_remover") {
      double height;
      if (nh->getParam("ground_remover/height", height))
        ROS_INFO("ground_remover/height: %d", height);
      else
        ROS_ERROR("Could not find ~/ground_remover/height");
      double sensor_height;
      if (nh->getParam("ground_remover/sensor_height", sensor_height))
        ROS_INFO("ground_remover/sensor_height: %d", sensor_height);
      else
        ROS_INFO("Could not find ~/ground_remover/sensor_height");
      ground_remover =
          new TunnelGroundRemover(*nh, *proj_params_ptr, height, sensor_height);
    }
  }

  ImageBasedClusterer<LinearImageLabeler<>> clusterer(
      angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);
  clusterer.SetCloudClient(visualizer);

  ground_remover->AddClient(&clusterer);
  // tunnel_ground_remover.AddClient(&clusterer);
  clusterer.AddClient(objects_publisher);

  for (const auto& path : image_reader.GetAllFilePaths()) {
    if (!ros::ok()) break;
    auto depth_image = MatFromDepthPng(path);
    auto cloud_ptr = Cloud::FromImage(depth_image, *proj_params_ptr);
    time_utils::Timer timer;
    // visualizer->OnNewObjectReceived(*cloud_ptr, 0);
    ground_remover->OnNewObjectReceived(*cloud_ptr, 0);
    // tunnel_ground_remover.OnNewObjectReceived(*cloud_ptr, 0);
    auto current_millis = timer.measure(time_utils::Timer::Units::Milli);
    ROS_INFO("It took %lu ms to process and show everything.", current_millis);
    uint max_wait_time = 100;
    if (current_millis > max_wait_time) {
      continue;
    }
    auto time_to_wait = max_wait_time - current_millis;
    ROS_INFO("Waiting another %lu ms.", time_to_wait);
    std::this_thread::sleep_for(std::chrono::milliseconds(time_to_wait));
  }
}

void run_from_path(ros::NodeHandle& nh_p, ros::NodeHandle& nh) {
  string in_path;
  nh_p.getParam("from_path", in_path);

  double angle_arg;
  nh_p.param("clusterer/angle_tollerance", angle_arg, 10.0);

  Radians angle_tollerance = Radians::FromDegrees(angle_arg);
  ROS_INFO("Reading from: %s ", in_path.c_str());

  RosVisualizer visualizer(nh_p);

  ObjectsPublisher objects_publisher(nh_p);

  // create and run loader thread
  std::thread loader_thread(ReadData, angle_tollerance, in_path, &visualizer,
                            &objects_publisher, &nh_p);

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // join thread after the application is dead
  loader_thread.join();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "depth_clusterer");

  ros::NodeHandle nh_p("~");
  ros::NodeHandle nh;

  string from_path;
  if (nh_p.getParam("from_path", from_path)) {
    run_from_path(nh_p, nh);
    return 0;
  }

  string lidar_config_link;
  if (!nh_p.getParam("lidar_config_link", lidar_config_link))
    ROS_ERROR("could not find lidar_config_link");
  auto proj_params_ptr = ProjectionParams::FromConfigFile(lidar_config_link);

  double angle_arg;
  nh_p.param("clusterer/angle_tollerance", angle_arg, 10.0);
  Radians angle_tollerance = Radians::FromDegrees(angle_arg);

  RosVisualizer visualizer(nh_p);

  ObjectsPublisher objects_publisher(nh_p);

  string topic_clouds;
  if (!nh_p.getParam("node/topic_clouds", topic_clouds))
    ROS_ERROR("could not find topic_clouds");

  CloudOdomRosSubscriber subscriber(&nh, *proj_params_ptr, topic_clouds);

  // Clusterer
  int min_cluster_size;
  if (nh_p.getParam("clusterer/min_cluster_size", min_cluster_size))
    ROS_INFO("Minimum cluster size: %i", min_cluster_size);
  else
    ROS_ERROR("Could not find ~/clusterer/min_cluster_size");

  int max_cluster_size;
  if (nh_p.getParam("clusterer/max_cluster_size", max_cluster_size))
    ROS_INFO("Minimum cluster size: %i", max_cluster_size);
  else
    ROS_ERROR("Could not find ~/clusterer/max_cluster_size");

  ClustererT clusterer(angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);
  clusterer.SetCloudClient(&visualizer);
  clusterer.AddClient(&objects_publisher);

  // Ground_remover
  AbstractGroundRemover* ground_remover;
  string remover;
  if (!nh_p.getParam("ground_remover/remover", remover)) {
    ROS_INFO(
        "Could not find ~/ground_remover/remover. Go on without "
        "ground_remover");
    subscriber.AddClient(&clusterer);
  } else {
    if (remover == "depth_ground_remover") {
      int smooth_window_size;
      if (nh_p.getParam("ground_remover/smooth_window_size",
                        smooth_window_size))
        ROS_INFO("ground_remover/smooth_window_size: %i", smooth_window_size);
      else
        ROS_ERROR("Could not find ~/ground_remover/smooth_window_size");
      float angle;
      if (nh_p.getParam("ground_remover/angle", angle))
        ROS_INFO("ground_remover/angle: %f", angle);
      else
        ROS_ERROR("Could not find ~/ground_remover/angle");
      Radians ground_remove_angle = Radians::FromDegrees(angle);

      ground_remover = new DepthGroundRemover(
          *proj_params_ptr, ground_remove_angle, smooth_window_size);
    }
    if (remover == "tunnel_ground_remover") {
      double height;
      if (nh_p.getParam("ground_remover/height", height))
        ROS_INFO("ground_remover/height: %d", height);
      else
        ROS_ERROR("Could not find ~/ground_remover/height");
      double sensor_height;
      if (nh_p.getParam("ground_remover/sensor_height", sensor_height))
        ROS_INFO("ground_remover/sensor_height: %d", sensor_height);
      else
        ROS_INFO("Could not find ~/ground_remover/sensor_height");
      ground_remover = new TunnelGroundRemover(nh_p, *proj_params_ptr, height,
                                               sensor_height);
      subscriber.WithoutProjection();
    }
    subscriber.AddClient(ground_remover);
    ground_remover->AddClient(&clusterer);
  }

  ROS_INFO("Running with angle tollerance: %f degrees",
           angle_tollerance.ToDegrees());

  subscriber.StartListeningToRos();
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::waitForShutdown();
  return 0;
}