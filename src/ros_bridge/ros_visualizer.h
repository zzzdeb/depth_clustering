#ifndef _ROS_VISUALIZE_
#define _ROS_VISUALIZE_

#include "visualization/visualizer.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <utility>
#include <vector>
#include <string>

namespace depth_clustering
{

class LabelClient
    : public AbstractClient<cv::Mat>
{
public:
  LabelClient() : AbstractClient<cv::Mat>() {}

  void OnNewObjectReceived(const cv::Mat &label_image,
                           const int id) override;

  void SetUpdateListener(IUpdateListener *update_listener)
  {
    _update_listener = update_listener;
  }

  virtual ~LabelClient() {}

  cv::Mat label_image() const;

private:
  cv::Mat label_image_;
  IUpdateListener *_update_listener;
  mutable std::mutex _cluster_mutex;
};

class RosVisualizer : public AbstractClient<Cloud>,
                      public IUpdateListener
{

  typedef pcl::PointXYZL PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

public:
  explicit RosVisualizer();
  virtual ~RosVisualizer();

  void initNode(ros::NodeHandle &nh);
  void OnNewObjectReceived(const Cloud &cloud, const int id) override;

  void onUpdate() override;

  ObjectPtrStorer *object_clouds_client() { return &_cloud_obj_storer; }
  LabelClient *label_client() { return &label_client_; }

  void set_frame_id(const std::string &frame_id) { frame_id_ = frame_id; }

protected:
  void draw();
  // void init();
  void fromCloudToPCL(PointCloudT::Ptr pcl_cloud, const Cloud &cloud);
  void LabelPCL(PointCloudT::Ptr pcl_cloud);

private:
  void PubCloud(const PointCloudT &cloud);
  void PubCubes(const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &cent_exts);

  bool _updated;
  ObjectPtrStorer _cloud_obj_storer;
  LabelClient label_client_;

  PointCloudT::Ptr pcl_cloud_;
  Cloud _cloud;
  mutable std::mutex _cloud_mutex;

  std::string frame_id_;

protected:
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub, marker_pub;
};

} // namespace depth_clustering
#endif