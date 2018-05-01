#include <string>

#include <ros/console.h>
#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


#include "projections/projection_params.h"
#include "utils/cloud.h"
#include "utils/folder_reader.h"
#include "utils/init_from_ros_param.h"
#include "utils/radians.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

#include "ground_removal/depth_ground_remover.h"

#include "../config/cmake_config.h"

typedef pcl::PointXYZL PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace depth_clustering;

int main(int argc, char** argv) {
    if (argc!=5)
    {
      ROS_ERROR("argc=5 path image cloud groundremove");
      return -1;
    }
    const string in_path = argv[1];
    ros::init(argc, argv, "speed_Test");
    ROS_INFO("from_path %s", in_path.c_str());
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("depth_image", 1);

    ros::Publisher cloud_pub =
        nh.advertise<sensor_msgs::PointCloud2>("speed_test_cloud", 1);

    // ProjectionParam
    auto image_reader =
        FolderReader(in_path, ".png", FolderReader::Order::SORTED);
    auto config_reader = FolderReader(in_path, "img.cfg");
    auto proj_params_ptr =
        ProjectionParams::FromConfigFile(config_reader.GetNextFilePath());

    ros::Rate loop_rate(20);
    for (const auto& path : image_reader.GetAllFilePaths()) {
      if (!ros::ok()) {
        break;
    }

    auto depth_image = MatFromDepthPng(path);
    auto cloud_ptr = Cloud::FromImage(depth_image, *proj_params_ptr);
    if (std::atoi(argv[3]))
    {
    PointCloudT::Ptr pcl_cloud = cloud_ptr->ToPcl();

    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(*pcl_cloud, cloud2);
    cloud2.header.stamp = ros::Time::now();
    cloud2.header.frame_id = "world";
    cloud_pub.publish(cloud2);
    }

    if (std::atoi(argv[2])){
    if (std::atoi(argv[4])){
      auto ground_remove_angle = Radians::FromDegrees(7.0);
      int smooth_window_size = 5;
      auto ground_remover = DepthGroundRemover(
          *proj_params_ptr, ground_remove_angle, smooth_window_size);

      auto smoother =
          SavitskyGolaySmoothing(*proj_params_ptr, smooth_window_size);
      const cv::Mat& depth_image_ =
          smoother.RepairDepth(cloud_ptr->projection_ptr()->depth_image(), 5, 1.0f);
      auto angle_image = smoother.CreateAngleImage(depth_image_);
      auto smoothed_image =
          smoother.ApplySavitskyGolaySmoothing(angle_image, smooth_window_size);
      depth_image = ground_remover.ZeroOutGroundBFS(
          depth_image_, smoothed_image, ground_remove_angle, smooth_window_size);
    }
    // cv::Mat depth_image = cv::imread(path, CV_LOAD_IMAGE_ANYDEPTH);
    depth_image *= 500.;
    depth_image.convertTo(depth_image, CV_16U);
    sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_image).toImageMsg();

    image_pub.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}