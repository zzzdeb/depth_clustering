#include "objects_publisher.h"

using std::pair;
using std::unordered_map;
using std::vector;

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace depth_clustering{

ObjectsPublisher::ObjectsPublisher(ros::NodeHandle& nh, bool use_mbb)
: _use_mbb {use_mbb}
{
    ObjectsPublisher();
    _nh = nh;
    _objects_pub = _nh.advertise<MarkerArray>("segmented_objects", 1);
    _frame_id = "world"; //!!! must be changed
}

void ObjectsPublisher::OnNewObjectReceived(const unordered_map<uint16_t, Cloud> &clouds,
                           const int id)
{
      std::lock_guard<std::mutex> guard(_objects_mutex);
      _objects.clear();
      for (const auto &cloud : clouds)
      {
        pair<geometry_msgs::Pose, geometry_msgs::Vector3> transformation;
        if(_use_mbb)
          MinBB(cloud.second.ToPcl(), transformation);
        else
          AxisAlignedBB(cloud.second, transformation);
        _objects.push_back(transformation);
      }
      PublishObjects(_objects);
}

void ObjectsPublisher::MinBB(const pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_ptr,
                            pair<geometry_msgs::Pose, geometry_msgs::Vector3>& transformation)
{
  pcl::PointXYZL min_point_OBB;
  pcl::PointXYZL max_point_OBB;
  pcl::PointXYZL position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;

  pcl::MomentOfInertiaEstimation <pcl::PointXYZL> feature_extractor; 
  feature_extractor.setInputCloud (cloud_ptr);
  feature_extractor.compute (); //!!! is it necessarily to compute all feateres? computeOBB is private create yourown for computeOBB.
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  
  geometry_msgs::Vector3 scale;
  scale.x = min_point_OBB.x; scale.y = min_point_OBB.y; scale.z = min_point_OBB.z;
  geometry_msgs::Pose pose;
  pose.position.x = position_OBB.x; pose.position.y = position_OBB.y; pose.position.z = position_OBB.z;
  
  Eigen::Quaternionf q(rotational_matrix_OBB);
  pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
  transformation = make_pair(pose, scale);
}

void ObjectsPublisher::AxisAlignedBB(const Cloud& cloud,
                            pair<geometry_msgs::Pose, geometry_msgs::Vector3>& transformation)
{
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        Eigen::Vector3f extent = Eigen::Vector3f::Zero();
        Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                                  std::numeric_limits<float>::lowest(),
                                  std::numeric_limits<float>::lowest());
        Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max());
        for (const auto &point : cloud.points())
        {
            center = center + point.AsEigenVector();
            min_point << std::min(min_point.x(), point.x()),
                std::min(min_point.y(), point.y()),
                std::min(min_point.z(), point.z());
            max_point << std::max(max_point.x(), point.x()),
                std::max(max_point.y(), point.y()),
                std::max(max_point.z(), point.z());
        }
        center /= cloud.size();
        
        if (min_point.x() < max_point.x())
        {
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
  
  // pcl::PointXYZL min_point_AABB;
  // pcl::PointXYZL max_point_AABB;
  // Eigen::Vector3f position_AABB;

  // pcl::MomentOfInertiaEstimation <pcl::PointXYZL> feature_extractor; 
  // feature_extractor.setInputCloud (cloud_ptr);
  // feature_extractor.compute (); //!!! look up;
  // feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  // feature_extractor.getMassCenter (position_AABB);
  
  // geometry_msgs::Vector3 scale;
  // scale.x = min_point_AABB.x; scale.y = min_point_AABB.y; scale.z = min_point_AABB.z;
  // geometry_msgs::Pose pose;
  // pose.position.x = position_AABB[0]; pose.position.y = position_AABB[1]; pose.position.z = position_AABB[2];
  // transformation = make_pair(pose, scale);

}

void ObjectsPublisher::PublishObjects(const vector<pair<geometry_msgs::Pose, geometry_msgs::Vector3> >& objects)
{
  visualization_msgs::MarkerArray obj_markers; //!!! Type
  visualization_msgs::Marker marker;
  int id = 0;
  ros::Time stamp = ros::Time::now(); //!!! is it really now?
  for (const auto &object: objects)
  {
    
        marker.header.frame_id = _frame_id;
        marker.header.stamp = stamp;
        marker.ns = _frame_id;
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = object.first;
        marker.scale = object.second;
        marker.color.a = 0.3; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = static_cast<float>(id)/objects.size(); // !!!changeable color
        marker.color.b = 1-marker.color.g;
        //only if using a MESH_RESOURCE marker type:
        // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        obj_markers.markers.push_back(marker);
  }
  _objects_pub.publish(obj_markers);
}

} //namespace depth_clustering