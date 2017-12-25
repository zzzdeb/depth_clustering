// Copyright (C) 2017  E. Zolboo, RWTH Aachen

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "./ros_visualizer.h"

#include <stdlib.h>

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
: AbstractClient<std::pair<Cloud::Ptr, cv::Mat> >()
{
  initNode(nh);
  nh.getParam("node/laser_frame_id", _frame_id);
  nh.getParam("clusterer/min_cluster_size", _min_cluster_size);
  nh.getParam("clusterer/max_cluster_size", _max_cluster_size);
}

RosVisualizer::~RosVisualizer() {}

void RosVisualizer::initNode(ros::NodeHandle &nh)
{
    _nh = nh;
    _cloud_pub = _nh.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
    _depth_image_pub = _nh.advertise<sensor_msgs::Image>("depth_image", 1);
    _label_image_pub = _nh.advertise<sensor_msgs::Image>("label_image", 1);
}

void RosVisualizer::LabelPCL(PointCloudT::Ptr pcl_cloud, const cv::Mat& label_image, const Cloud& cloud)
{
    // label cloud from image labels
    vector<vector<int>> labels(0);
    // cv::Mat a = label_image;
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
                //!!! debug
                // it shows all objects labeled with 0
                for (const auto &point_idx : point_container.points())
                {
                //    pcl_cloud->at(point_idx).z -= 1000;
                   pcl_cloud->at(point_idx).label = 0;
                
                // pcl_cloud-
                }
                continue;
            }

            for (const auto &point_idx : point_container.points())
            {
                if (label > labels.size())
                    labels.resize(label, vector<int>());
                labels.at(label - 1).push_back(point_idx);
                // pcl_cloud->at(point_idx).z -= 1000;
            }
        }
    }
    
    // filter out unfitting clusters
    for (unsigned int i = 0; i < labels.size(); i++)
    {
        // int label = std::rand() % 20+10;
        if (labels[i].size() > _min_cluster_size &&
        labels[i].size() < _max_cluster_size) {
            for (auto ind : labels[i])
            pcl_cloud->at(ind).label = i;
            // pcl_cloud->at(ind).label = 10;
        }
        else{
            for (auto ind : labels[i])
            pcl_cloud->at(ind).label = 10;
        }
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
    PubImages(cloud_pair.first->projection_ptr()->depth_image(),
              cloud_pair.second);
}

void RosVisualizer::PubImages(const cv::Mat &depth_image, const cv::Mat &label_image) {
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image ros_label_image;
  sensor_msgs::Image ros_depth_image;
  std_msgs::Header header;
  // header.seq = counter; // user defined counter
  header.stamp = ros::Time::now();  //!!!

  // converting to ros msg
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16,
                                  label_image);
  img_bridge.toImageMsg(ros_label_image);

  cv::Mat depth_image_to_publish;
  depth_image.convertTo(depth_image_to_publish, CV_16U);
  
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16,
                                  depth_image_to_publish);
  img_bridge.toImageMsg(ros_depth_image);

  //publishing
  _depth_image_pub.publish(ros_depth_image);
  _label_image_pub.publish(ros_label_image);
}

} //namespace depth_clustering
