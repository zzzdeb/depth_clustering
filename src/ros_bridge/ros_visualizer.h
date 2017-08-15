#ifndef _ROS_VISUALIZER_
#define _ROS_VISUALIZER_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <map>
#include <utility>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "communication/abstract_client.h"
#include "utils/cloud.h"
#include "utils/useful_typedefs.h"

#include <pcl/point_cloud.h>

namespace depth_clustering
{
  typedef pcl::PointXYZL PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

class RosVisualizer : public AbstractClient<std::pair<Cloud::Ptr, cv::Mat> >
{

public:
  explicit RosVisualizer();
  explicit RosVisualizer(ros::NodeHandle& nh);
  virtual ~RosVisualizer();

  void initNode(ros::NodeHandle &nh);
  void OnNewObjectReceived(const std::pair<Cloud::Ptr, cv::Mat> &cloud_pair, const int id) override;
  void LabelPCL(PointCloudT::Ptr pcl_cloud, const cv::Mat& label_image, const Cloud& cloud);
  void set_frame_id(const std::string &frame_id) { _frame_id = frame_id; }
  void fromCloudToPCL(PointCloudT::Ptr pcl_cloud, const Cloud &cloud);

protected:
  void draw();

private:
  void PubImage(const cv::Mat& label_image);
  void PubCloud(const PointCloudT &cloud);

  std::string _frame_id;

protected:
  ros::NodeHandle _nh;
  ros::Publisher _image_pub;
  ros::Publisher _cloud_pub;
};

} // namespace depth_clustering
#endif