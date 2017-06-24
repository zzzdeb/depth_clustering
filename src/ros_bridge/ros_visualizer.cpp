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
    _label_client.SetUpdateListener(this);
    _pcl_cloud = PointCloudT().makeShared();
}
RosVisualizer::~RosVisualizer() {}

void RosVisualizer::initNode(ros::NodeHandle &nh)
{
    _nh = nh;
    _cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    _marker_pub = _nh.advertise<visualization_msgs::MarkerArray>("segmented_objects", 0);
    _label_client.initNode(nh);
}

void RosVisualizer::LabelPCL(PointCloudT::Ptr pcl_cloud)
{
    // label cloud from image labels
    vector<vector<int>> labels(0);
    cv::Mat a = _label_client.label_image();
    for (int row = 0; row < _label_client.label_image().rows; ++row)
    {
        for (int col = 0; col < _label_client.label_image().cols; ++col)
        {
            const auto &point_container = _cloud.projection_ptr()->at(row, col);
            if (point_container.IsEmpty())
            {
                // this is ok, just continue, nothing interesting here, no points.
                continue;
            }
            uint16_t label = _label_client.label_image().at<uint16_t>(row, col);
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
                _pcl_cloud->at(ind).label = i;
        }
}

void RosVisualizer::draw()
{
    lock_guard<mutex> guard(_cloud_mutex);
    LabelPCL(_pcl_cloud);
    PubCloud(*_pcl_cloud);
    PubMarkers(_cloud_obj_storer.object_clouds());
}

void RosVisualizer::PubCloud(const PointCloudT &pcl_cloud)
{
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    cloud2.header.stamp = ros::Time::now();
    cloud2.header.frame_id = _frame_id;
    _cloud_pub.publish(cloud2);
}

void RosVisualizer::PubMarkers(const unordered_map<uint16_t, Cloud>& object_clouds)
{
    vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> cent_exts;
    for (const auto &kv : object_clouds)
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

    visualization_msgs::MarkerArray markers;
    int id = 0;
    ros::Time stamp = ros::Time::now();
    for (const auto cent_ext : cent_exts)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = _frame_id;
        marker.header.stamp = stamp;
        marker.ns = _frame_id;
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;
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
        marker.color.a = 0.3; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = static_cast<float>(id)/cent_exts.size(); // jede objekt hat eigene Farbe
        marker.color.b = 1-marker.color.g;
        //only if using a MESH_RESOURCE marker type:
        // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        markers.markers.push_back(marker);
    }
    _marker_pub.publish(markers);
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
    fromCloudToPCL(_pcl_cloud, cloud);
}

void RosVisualizer::onUpdate()
{
    this->draw();
}

cv::Mat LabelClient::label_image() const
{
    lock_guard<mutex> guard(_cluster_mutex);
    return _label_image;
}

void LabelClient::OnNewObjectReceived(
    const cv::Mat &label_image, const int id)
{
    std::unique_lock<mutex> locker(_cluster_mutex);
    this->_label_image = label_image;

    if (_nh_ptr)
    {
        this->PubImage();
    }
    locker.unlock();

    // if (_update_listener)
    // {
    //     _update_listener->onUpdate();
    //     // _update_listener->
    // }
}

void LabelClient::PubImage()
{
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image ros_image;
    std_msgs::Header header;
    // header.seq = counter; // user defined counter
    header.stamp = ros::Time::now();

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, _label_image);
    img_bridge.toImageMsg(ros_image);
    _image_pub.publish(ros_image);
}
void LabelClient::initNode(ros::NodeHandle& nh)
{    
    _nh_ptr = &nh;
    _image_pub = _nh_ptr->advertise<sensor_msgs::Image>("depth_image", 1);
}
} //namespace depth_clustering
