#include "./ros_visualizer.h"

namespace depth_clustering
{

using std::string;
using std::vector;
using std::array;
using std::to_string;

using std::map;
using std::pair;
using std::mutex;
using std::string;
using std::vector;
using std::thread;
using std::lock_guard;
using std::unordered_map;

typedef pcl::PointXYZL PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

RosVisualizer::RosVisualizer(): 
AbstractClient<std::pair<Cloud::Ptr, cv::Mat> >(){}

RosVisualizer::RosVisualizer(ros::NodeHandle& nh)
: AbstractClient<std::pair<Cloud::Ptr, cv::Mat> >(), _nh{nh}
{
    _cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    _image_pub = _nh.advertise<sensor_msgs::Image>("depth_image", 1);
    if(!nh.getParam("objects_publisher/frame_id", _frame_id)) 
        ROS_ERROR("couldnt find objects_publisher/frame_id");
}

RosVisualizer::~RosVisualizer() {}

void RosVisualizer::initNode(ros::NodeHandle &nh)
{
    _nh = nh;
    _cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    _image_pub = _nh.advertise<sensor_msgs::Image>("depth_image", 1);
}

void RosVisualizer::LabelPCL(PointCloudT::Ptr pcl_cloud, const cv::Mat& label_image, const Cloud& cloud)
{
    // label cloud from image labels
    vector<vector<int>> labels(0);
    cv::Mat a = label_image;
    for (int row = 0; row < label_image.rows; ++row)
    {
        for (int col = 0; col < label_image.cols; ++col)
        {
            const auto &point_container = cloud.projection_ptr()->at(row, col);
            if (point_container.IsEmpty())
            {
                // this is ok, just continue, nothing interesting here, no points.
                continue;
            }
            uint16_t label = label_image.at<uint16_t>(row, col);
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
                pcl_cloud->at(ind).label = i;
        }
}


void RosVisualizer::PubCloud(const PointCloudT &pcl_cloud)
{
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(pcl_cloud, cloud2);
    cloud2.header.stamp = ros::Time::now();
    cloud2.header.frame_id = _frame_id;
    _cloud_pub.publish(cloud2);
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

void RosVisualizer::OnNewObjectReceived(const std::pair<Cloud::Ptr, cv::Mat> &cloud_pair, const int id)
{
    PointCloudT::Ptr cloud = cloud_pair.first->ToPcl(); 
    LabelPCL(cloud, cloud_pair.second, *cloud_pair.first);
    PubCloud(*cloud);
    PubImage(cloud_pair.second);
}

void RosVisualizer::PubImage(const cv::Mat& label_image)
{
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image ros_image;
    std_msgs::Header header;
    // header.seq = counter; // user defined counter
    header.stamp = ros::Time::now();//!!!

    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, label_image);
    img_bridge.toImageMsg(ros_image);
    _image_pub.publish(ros_image);
}

} //namespace depth_clustering
