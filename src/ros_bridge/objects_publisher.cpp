#include "objects_publisher.h"
#include "../utils/init_from_ros_param.h"

using std::pair;
using std::unordered_map;
using std::vector;

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace depth_clustering {

ObjectsPublisher::ObjectsPublisher(ros::NodeHandle& nh)
{
  _nh = nh;
  _objects_pub = _nh.advertise<MarkerArray>("segmented_objects", 1);
  InitFromRosParam(nh, "node/laser_frame_id", _frame_id);
  InitFromRosParam(nh, "objects_publisher/use_axial_bounding_box", _use_abb);
}

void ObjectsPublisher::OnNewObjectReceived(
    const unordered_map<uint16_t, Cloud>& clouds, const int id) {
  vector<pair<geometry_msgs::Pose, geometry_msgs::Vector3> > objects;
  for (const auto& cloud : clouds) {
    pair<geometry_msgs::Pose, geometry_msgs::Vector3> transformation;
    if (_use_abb)
      MinRecArea(cloud.second.ToPcl(), transformation); //!!!
    else
      AxisAlignedBB(cloud.second, transformation);
    objects.push_back(transformation);
  }
  PublishObjects(objects);
}

void ObjectsPublisher::MinBB(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_ptr,
    pair<geometry_msgs::Pose, geometry_msgs::Vector3>& transformation) {
  // pcl::PointXYZL min_point_OBB;
  // pcl::PointXYZL max_point_OBB;
  // pcl::PointXYZL position_OBB;
  // Eigen::Matrix3f rotational_matrix_OBB;

  // pcl::MomentOfInertiaEstimation<pcl::PointXYZL> feature_extractor;
  // feature_extractor.setInputCloud(cloud_ptr);
  // feature_extractor.setAngleStep(30);
  // feature_extractor.compute();  //!!! is it necessarily to compute all feateres?
  //                               //! computeOBB is private create yourown for
  // //! computeOBB.
  // feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
  //                          rotational_matrix_OBB);
  // float factor = 1.5;
  // geometry_msgs::Vector3 scale;
  // scale.x = min_point_OBB.x * factor;
  // scale.y = min_point_OBB.y * factor;
  // scale.z = min_point_OBB.z * factor;
  // geometry_msgs::Pose pose;
  // pose.position.x = position_OBB.x;
  // pose.position.y = position_OBB.y;
  // pose.position.z = position_OBB.z;

  // Eigen::Quaternionf q(rotational_matrix_OBB);
  // pose.orientation.x = q.x();
  // pose.orientation.y = q.y();
  // pose.orientation.z = q.z();
  // pose.orientation.w = q.w();
  // transformation = make_pair(pose, scale);  //!!!
}

void ObjectsPublisher::MinRecArea(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_ptr,
    std::pair<geometry_msgs::Pose, geometry_msgs::Vector3>& transformation) {
  std::vector<cv::Point2f> points_2d;
  float zmin, zmax, zpos;
  zpos = 0;
  zmin = cloud_ptr->at(0).z;
  zmax = cloud_ptr->at(0).z;
  for (const auto& point : *cloud_ptr) {
    zmin = std::min(zmin, point.z);
    zmax = std::max(zmax, point.z);
    zpos += point.z;
    
    cv::Point2f cvpoint(point.x, point.y);
    points_2d.push_back(cvpoint);
  }

  zpos /= cloud_ptr->size();
  float zscale = 0;
  if (zmin < zmax) {
    zscale = zmax - zmin;
  }
  // Find the minimum area enclosing bounding box
  cv::RotatedRect box = cv::minAreaRect(points_2d);

  geometry_msgs::Vector3 scale;
  geometry_msgs::Pose pose;
  scale.x = box.size.width;
  scale.y = box.size.height;
  scale.z = zscale;
  pose.position.x = box.center.x;
  pose.position.y = box.center.y;
  pose.position.z = zpos;
  Eigen::Quaternionf q(Eigen::AngleAxisf(box.angle*3.14/180.0, Eigen::Vector3f::UnitZ()));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  transformation = make_pair(pose, scale);
}

  void ObjectsPublisher::AxisAlignedBB(
        const Cloud& cloud,
        pair<geometry_msgs::Pose, geometry_msgs::Vector3>& transformation) {
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f extent = Eigen::Vector3f::Zero();
  Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest());
  Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max());
  for (const auto& point : cloud.points()) {
    center = center + point.AsEigenVector();
    min_point << std::min(min_point.x(), point.x()),
        std::min(min_point.y(), point.y()), std::min(min_point.z(), point.z());
    max_point << std::max(max_point.x(), point.x()),
        std::max(max_point.y(), point.y()), std::max(max_point.z(), point.z());
  }
  center /= cloud.size();

  if (min_point.x() < max_point.x()) {
    extent = max_point - min_point;
  }

  geometry_msgs::Vector3 scale;
  scale.x = extent.x();
  scale.y = extent.y();
  scale.z = extent.z();
  geometry_msgs::Pose pose;
  pose.position.x = center.x();
  pose.position.y = center.y();
  pose.position.z = center.z();

  transformation = make_pair(pose, scale);
}

void ObjectsPublisher::PublishObjects(
    const vector<pair<geometry_msgs::Pose, geometry_msgs::Vector3> >& objects) {
  visualization_msgs::MarkerArray obj_markers;
  visualization_msgs::Marker marker;
  int id = 0;
  ros::Time stamp = ros::Time::now();  //!!! is it really now?
  for (const auto& object : objects) {
    marker.header.frame_id = _frame_id;
    marker.header.stamp = stamp;
    marker.ns = _frame_id;
    marker.id = id++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = object.first;
    marker.scale = object.second;
    marker.color.a = 0.3;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g =
        static_cast<float>(id) / objects.size(); 
    marker.color.b = 1 - marker.color.g;
    marker.lifetime = ros::Duration(0.5);  //!!!
    // only if using a MESH_RESOURCE marker type:
    // marker.mesh_resource =
    // "package://pr2_description/meshes/base_v0/base.dae";
    obj_markers.markers.push_back(marker);
  }
  _objects_pub.publish(obj_markers);
}

}  // namespace depth_clustering