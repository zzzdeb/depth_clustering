#ifndef _ROS_VISUALIZE_
#define _ROS_VISUALIZE_

#include "visualization/visualizer.h"

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


#include <utility>
#include <vector>
#include <string>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace depth_clustering
{

class LabelClient
    : public AbstractClient<cv::Mat>
{
public:
  LabelClient() : AbstractClient<cv::Mat>() {}
  LabelClient(ros::NodeHandle& nh) : AbstractClient<cv::Mat>()
  {
    initNode(nh);
  }

  void initNode(ros::NodeHandle& nh);

  void OnNewObjectReceived(const cv::Mat &label_image,
                           const int id) override;

  void SetUpdateListener(IUpdateListener *update_listener)
  {
    _update_listener = update_listener;
  }

  virtual ~LabelClient() {}
  
  cv::Mat label_image() const;


private:
  void PubImage();

  cv::Mat _label_image;
  IUpdateListener *_update_listener;
  mutable std::mutex _cluster_mutex;

  ros::Publisher _image_pub;
  ros::NodeHandle* _nh_ptr;
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
  LabelClient *label_client() { return &_label_client; }

  void set_frame_id(const std::string &frame_id) { _frame_id = frame_id; }

protected:
  void draw();
  // void init();
  void fromCloudToPCL(PointCloudT::Ptr pcl_cloud, const Cloud &cloud);
  void LabelPCL(PointCloudT::Ptr pcl_cloud);

private:
  void PubCloud(const PointCloudT &cloud);
  void PubMarkers(const std::unordered_map<uint16_t, Cloud>& object_clouds);

  bool _updated;
  ObjectPtrStorer _cloud_obj_storer;
  LabelClient _label_client;

  PointCloudT::Ptr _pcl_cloud;
  Cloud _cloud;
  mutable std::mutex _cloud_mutex;

  std::string _frame_id;

protected:
  ros::NodeHandle _nh;
  ros::Publisher _cloud_pub, _marker_pub;
};

} // namespace depth_clustering
#endif