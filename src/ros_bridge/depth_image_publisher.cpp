#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include "utils/folder_reader.h"
#include "utils/velodyne_utils.h"

using namespace depth_clustering;

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("depth_image", 1);

  auto image_reader =
      FolderReader(argv[1], ".png", FolderReader::Order::SORTED);

  ros::Rate loop_rate(10);
  for (const auto& path : image_reader.GetAllFilePaths()) {
    if (!ros::ok()) {
      break;
    }
    // auto depth_image = MatFromDepthPng(path);
    cv::Mat depth_image = cv::imread(path, CV_LOAD_IMAGE_ANYDEPTH);
    sensor_msgs::ImagePtr msg =
    cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_image).toImageMsg();

    pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
  }
}