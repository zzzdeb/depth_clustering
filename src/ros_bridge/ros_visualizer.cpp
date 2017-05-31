#include "./ros_visualizer.h"

namespace depth_clustering
{

using std::string;
using std::vector;
using std::array;
using std::to_string;

using std::map;
using std::mutex;
using std::string;
using std::vector;
using std::thread;
using std::lock_guard;
using std::unordered_map;

typedef pcl::PointXYZL PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

RosVisualizer::RosVisualizer()
{
    _cloud_obj_storer.SetUpdateListener(this);
    label_client_.SetUpdateListener(this);
    pcl_cloud_ = PointCloudT().makeShared();
}
RosVisualizer::~RosVisualizer() {}

void RosVisualizer::initNode(ros::NodeHandle &nh)
{
    nh_ = nh;
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
}

void RosVisualizer::LabelPCL(PointCloudT::Ptr pcl_cloud)
{
    // label cloud from image labels
    vector<vector<int>> labels(0);
    cv::Mat a = label_client_.label_image();
    for (int row = 0; row < label_client_.label_image().rows; ++row)
    {
        for (int col = 0; col < label_client_.label_image().cols; ++col)
        {
            const auto &point_container = _cloud.projection_ptr()->at(row, col);
            if (point_container.IsEmpty())
            {
                // this is ok, just continue, nothing interesting here, no points.
                continue;
            }
            uint16_t label = label_client_.label_image().at<uint16_t>(row, col);
            if (label < 1)
            {
                // this is a default label, skip
                continue;
            }

            for (const auto &point_idx : point_container.points())
            {
                if (label > labels.size())
                    labels.resize(label, vector<int>());
                labels.at(label - 1).push_back(point_idx);
            }
        }
    }
    // filter out unfitting clusters
    for (unsigned int i = 0; i < labels.size(); i++)
        if (labels[i].size() > 20 && labels[i].size() < 50000)
        {
            for (auto ind : labels[i])
                pcl_cloud_->at(ind).label = i;
        }
}

void RosVisualizer::draw()
{
    vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> cent_exts;

    lock_guard<mutex> guard(_cloud_mutex);
    LabelPCL(pcl_cloud_);
    PubCloud(*pcl_cloud_);
    for (const auto &kv : _cloud_obj_storer.object_clouds())
    {
        const auto &cluster = kv.second;
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        Eigen::Vector3f extent = Eigen::Vector3f::Zero();
        Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                                  std::numeric_limits<float>::lowest(),
                                  std::numeric_limits<float>::lowest());
        Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max(),
                                  std::numeric_limits<float>::max());
        for (const auto &point : cluster.points())
        {
            center = center + point.AsEigenVector();
            min_point << std::min(min_point.x(), point.x()),
                std::min(min_point.y(), point.y()),
                std::min(min_point.z(), point.z());
            max_point << std::max(max_point.x(), point.x()),
                std::max(max_point.y(), point.y()),
                std::max(max_point.z(), point.z());
        }
        center /= cluster.size();
        if (min_point.x() < max_point.x())
        {
            extent = max_point - min_point;
        }
        cent_exts.push_back(std::make_pair(center, extent));
    }
    // PubCubes(cent_exts);
}

// void RosVisualizer::init()
// {
//     // setSceneRadius(100.0);
//     // camera()->showEntireScene();
//     // glDisable(GL_LIGHTING);
// }

void RosVisualizer::PubCloud(const PointCloudT &pcl_cloud)
{
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    cloud2.header.stamp = ros::Time::now();
    cloud2.header.frame_id = frame_id_;
    cloud_pub.publish(cloud2);
}

void RosVisualizer::PubCubes(const vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &cent_exts)
{
    visualization_msgs::MarkerArray markers;
    for (const auto cent_ext : cent_exts)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cent_ext.first.x();
        marker.pose.position.y = cent_ext.first.y();
        marker.pose.position.z = cent_ext.first.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = cent_ext.second.x();
        marker.scale.y = cent_ext.second.y();
        marker.scale.z = cent_ext.second.z();
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        markers.markers.push_back(marker);
    }
    marker_pub.publish(markers);
}

void RosVisualizer::fromCloudToPCL(PointCloudT::Ptr pcl_cloud, const Cloud &cloud)
{
    pcl_cloud->clear();
    pcl_cloud->reserve(cloud.size());
    PointT p;
    for (const auto &point : cloud.points())
    {
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        pcl_cloud->push_back(p);
    }
}

void RosVisualizer::OnNewObjectReceived(const Cloud &cloud, const int id)
{
    lock_guard<mutex> guard(_cloud_mutex);
    _cloud = cloud;
    fromCloudToPCL(pcl_cloud_, cloud);
}

void RosVisualizer::onUpdate()
{
    this->draw();
}

cv::Mat LabelClient::label_image() const
{
    lock_guard<mutex> guard(_cluster_mutex);
    return label_image_;
}

void LabelClient::OnNewObjectReceived(
    const cv::Mat &label_image, const int id)
{
    std::unique_lock<mutex> locker(_cluster_mutex);
    label_image_ = label_image;
    locker.unlock();

    if (_update_listener)
    {
        _update_listener->onUpdate();
    }
}

} //namespace depth_clustering
