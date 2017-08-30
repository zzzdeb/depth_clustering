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
#include "utils/init_from_ros_param.h"

#include "ros_bridge/cloud_odom_ros_subscriber.h"
#include "ros_bridge/objects_publisher.h"
#include "ros_bridge/ros_visualizer.h"

#include "../config/cmake_config.h"

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

  // Clusterer
  int min_cluster_size;
  InitFromRosParam<int>(*nh, "clusterer/min_cluster_size", min_cluster_size);

  int max_cluster_size;
  InitFromRosParam<int>(*nh, "clusterer/max_cluster_size", max_cluster_size);

  ClustererT clusterer(angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);
  clusterer.SetCloudClient(visualizer);
  clusterer.AddClient(objects_publisher);

  //ProjectionParam
  auto image_reader =
      FolderReader(in_path, ".png", FolderReader::Order::SORTED);
  auto config_reader = FolderReader(in_path, "img.cfg");
  auto proj_params_ptr =
      ProjectionParams::FromConfigFile(config_reader.GetNextFilePath());

  // Ground_remover
  AbstractGroundRemover* ground_remover;
  string remover;
  if (InitFromRosParam<string>(*nh, "ground_remover/remover", remover, "No Groundremover"))
  {
    if (remover == "depth_ground_remover") {
      int smooth_window_size;
      InitFromRosParam<int>(*nh, "ground_remover/smooth_window_size",
                            smooth_window_size);
      float angle;
      InitFromRosParam<float>(*nh, "ground_remover/angle", angle);
      Radians ground_remove_angle = Radians::FromDegrees(angle);

      ground_remover = new DepthGroundRemover(
          *proj_params_ptr, ground_remove_angle, smooth_window_size);
    }
    if (remover == "tunnel_ground_remover") {
      double height;
      InitFromRosParam<double>(*nh, "ground_remover/height", height);
      double sensor_height;
      InitFromRosParam<double>(*nh, "ground_remover/sensor_height", sensor_height);
      ground_remover = new TunnelGroundRemover(*nh, *proj_params_ptr, height,
                                                sensor_height);
    }
    ground_remover->AddClient(&clusterer);
  }

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

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "depth_clusterer");

  ros::NodeHandle nh_p("~");
  ros::NodeHandle nh;

  //Angle tollerance for segmentation. Default value: 10 Degree
  double angle_arg;
  InitFromRosParam<double>(nh_p, "clusterer/angle_tollerance", angle_arg, 10.0);
  Radians angle_tollerance = Radians::FromDegrees(angle_arg);

  RosVisualizer visualizer(nh_p);
    
  ObjectsPublisher objects_publisher(nh_p);


  //If it is running from Moosmanns data?
  string from_path;
  if (InitFromRosParam<string>(nh_p, "from_path", from_path)) {
    // create and run loader thread
    std::thread loader_thread(ReadData, angle_tollerance, from_path, &visualizer,
    &objects_publisher, &nh_p);

    // join thread after the application is dead
    loader_thread.join();
    return 0;
  }

  // Topic to subscribe
  string topic_clouds;
  InitFromRosParam<string>(nh_p, "node/topic_clouds", topic_clouds);

  //ProjectionParam from file
  string lidar_config_link;
  InitFromRosParam<string>(nh_p, "lidar_config_link", lidar_config_link);
  lidar_config_link = SOURCE_PATH + lidar_config_link;
  ROS_INFO_STREAM("Absolute lidar_config link: "<< lidar_config_link);
  auto proj_params_ptr = ProjectionParams::FromConfigFile(lidar_config_link);

  CloudOdomRosSubscriber subscriber(&nh, *proj_params_ptr, topic_clouds);

  // Clusterer
  int min_cluster_size;
  InitFromRosParam<int>(nh_p, "clusterer/min_cluster_size", min_cluster_size);

  int max_cluster_size;
  InitFromRosParam<int>(nh_p, "clusterer/max_cluster_size", max_cluster_size);

  ClustererT clusterer(angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);
  clusterer.SetCloudClient(&visualizer);
  clusterer.AddClient(&objects_publisher);

  // Ground_remover
  AbstractGroundRemover* ground_remover;
  string remover;
  if (!InitFromRosParam<string>(nh_p, "ground_remover/remover", remover, "No Groundremover"))
    subscriber.AddClient(&clusterer);
  else {
    if (remover == "depth_ground_remover") {
      int smooth_window_size;
      InitFromRosParam<int>(nh_p, "ground_remover/smooth_window_size",
                            smooth_window_size);
      float angle;
      InitFromRosParam<float>(nh_p, "ground_remover/angle", angle);
      Radians ground_remove_angle = Radians::FromDegrees(angle);

      ground_remover = new DepthGroundRemover(
          *proj_params_ptr, ground_remove_angle, smooth_window_size);
    }
    if (remover == "tunnel_ground_remover") {
      double height;
      InitFromRosParam<double>(nh_p, "ground_remover/height", height);
      double sensor_height;
      InitFromRosParam<double>(nh_p, "ground_remover/sensor_height", sensor_height);
      ground_remover = new TunnelGroundRemover(nh_p, *proj_params_ptr, height,
                                                sensor_height);
      subscriber.WithoutProjection(); //!!!
    }
    subscriber.AddClient(ground_remover);
    ground_remover->AddClient(&clusterer);
  }

  ROS_INFO("Running with angle tollerance: %f degrees",
            angle_tollerance.ToDegrees());

  subscriber.StartListeningToRos();
  int num_thread;
  InitFromRosParam<int>(nh_p, "node/num_thread", num_thread);
  ros::AsyncSpinner spinner(num_thread);
  spinner.start();

  ros::waitForShutdown();
  return 0;
}