
#include <string>
#include <thread>

#include <ros/console.h>
#include <ros/ros.h>

#include "clusterers/image_based_clusterer.h"
#include "image_labelers/diff_helpers/diff_factory.h"

#include "ground_removal/depth_ground_remover.h"
#include "ground_removal/tunnel_ground_remover.h"

#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/folder_reader.h"
#include "utils/init_from_ros_param.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"

#include "ros_bridge/cloud_odom_ros_subscriber.h"
#include "ros_bridge/objects_publisher.h"
#include "ros_bridge/ros_visualizer.h"

#include "../config/cmake_config.h"

using std::string;
using std::vector;

using namespace depth_clustering;
using ClustererT = ImageBasedClusterer<LinearImageLabeler<>>;

void TestImages(const string& in_path, RosVisualizer* visualizer, ObjectsPublisher* objects_publisher,
              ros::NodeHandle* nh) {
  // delay reading for one second to allow GUI to load
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // now load the data
  ROS_INFO("running on TestData");

  // ProjectionParam
  auto image_reader =
      FolderReader(in_path, ".png", FolderReader::Order::SORTED);
  auto config_reader = FolderReader(in_path, "img.cfg");
  auto proj_params_ptr =
      ProjectionParams::FromConfigFile(config_reader.GetNextFilePath());

  for (const auto& path : image_reader.GetAllFilePaths()) {
    // if (ros::isShuttingDown()) {
    //   while (ros::ok()) {
    //     continue;      
    //   }
    //   delete ground_remover;
    //   return;
    // }
    auto depth_image = MatFromDepthPng(path);
    auto cloud_ptr = Cloud::FromImage(depth_image, *proj_params_ptr);
    time_utils::Timer timer;

    auto foo = std::make_pair(make_shared<Cloud>(*cloud_ptr),*labels_ptr);
    ROS_INFO("Created pair in: %lu us",
          timer.measure());
    _cloud_client->OnNewObjectReceived(foo, this->id());
    ROS_INFO("Shared with cloud clients in: %lu us",
          timer.measure());
    
  }
}

int main(int argc, char* argv[]){

  ros::init(argc, argv, "depth_clusterer");
    
  ros::NodeHandle nh_p("~");
  ros::NodeHandle nh;
  
  // Angle tollerance for segmentation. Default value: 10 Degree
  double angle_arg;
  InitFromRosParam<double>(nh_p, "clusterer/angle_tollerance", angle_arg, 10.0);
  Radians angle_tollerance = Radians::FromDegrees(angle_arg);

  RosVisualizer visualizer(nh_p);
  
  ObjectsPublisher objects_publisher(nh_p);

  // Clusterer
  int min_cluster_size;
  InitFromRosParam<int>(nh_p, "clusterer/min_cluster_size", min_cluster_size);
  
  int max_cluster_size;
  InitFromRosParam<int>(nh_p, "clusterer/max_cluster_size", max_cluster_size);
  
  ClustererT clusterer(angle_tollerance, min_cluster_size, max_cluster_size);
  clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);
  clusterer.SetCloudClient(&visualizer);
  clusterer.AddClient(&objects_publisher);
  
  // If it is running from Moosmanns data?
  string from_path;
  if (InitFromRosParam<string>(nh_p, "from_path", from_path, "")) {
      // create and run loader thread
      std::thread loader_thread(ReadData, from_path, &clusterer,
        &visualizer, &objects_publisher, &nh_p);
        
        // join thread after the application is dead
        loader_thread.join();
        return 0;
    }
}