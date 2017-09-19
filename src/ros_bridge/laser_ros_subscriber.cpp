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

#include "ros_bridge/laser_ros_subscriber.h"
#include <eigen_conversions/eigen_msg.h>

#include <vector>
#include <string>
#include <algorithm>

#include "utils/pose.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>

namespace depth_clustering
{

using ros::NodeHandle;
using message_filters::Subscriber;
using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;
using nav_msgs::Odometry;
// using sensor_msgs::PointCloud2;
// using sensor_msgs::PointCloud2ConstPtr;
using sensor_msgs::LaserScan;

using std::vector;
using std::string;
using std::map;

// template <class T>
// T BytesTo(const vector<uint8_t>& data, uint32_t start_idx) {
//   const size_t kNumberOfBytes = sizeof(T);
//   uint8_t byte_array[kNumberOfBytes];
//   // forward bit order (it is a HACK. We do not account for bigendianes)
//   for (size_t i = 0; i < kNumberOfBytes; ++i) {
//     byte_array[i] = data[start_idx + i];
//   }
//   T result;
//   std::copy(reinterpret_cast<const uint8_t*>(&byte_array[0]),
//             reinterpret_cast<const uint8_t*>(&byte_array[kNumberOfBytes]),
//             reinterpret_cast<uint8_t*>(&result));
//   return result;
// }

// void PrintMsgStats(const sensor_msgs::PointCloud2ConstPtr& msg) {
//   fprintf(stderr, "<<<<<<<<<<<<<<< new cloud >>>>>>>>>>>>>>>\n");
//   fprintf(stderr, "received msg   %d\n", msg->header.seq);
//   fprintf(stderr, "height:        %d\n", msg->height);
//   fprintf(stderr, "width:         %d\n", msg->width);
//   fprintf(stderr, "num of fields: %lu\n", msg->fields.size());
//   fprintf(stderr, "fields of each point:\n");
//   for (auto const& pointField : msg->fields) {
//     fprintf(stderr, "\tname:     %s\n", pointField.name.c_str());
//     fprintf(stderr, "\toffset:   %d\n", pointField.offset);
//     fprintf(stderr, "\tdatatype: %d\n", pointField.datatype);
//     fprintf(stderr, "\tcount:    %d\n", pointField.count);
//     fprintf(stderr, "\n");
//   }
//   fprintf(stderr, "is bigendian:  %s\n", msg->is_bigendian ? "true" : "false");
//   fprintf(stderr, "point step:    %d\n", msg->point_step);
//   fprintf(stderr, "row step:      %d\n", msg->row_step);
//   fprintf(stderr, "data size:     %lu\n", msg->data.size() * sizeof(msg->data));
//   fprintf(stderr, "is dense:      %s\n", msg->is_dense ? "true" : "false");
//   fprintf(stderr, "=========================================\n");
// }

LaserRosSubscriber::LaserRosSubscriber(NodeHandle *node_handle,
                                       const ProjectionParams &params,
                                       const string &topic_scan,
                                       const string &topic_odom)
    : AbstractSender{SenderType::STREAMER}, _params{params}
{
    _node_handle = node_handle;
    _topic_scan = topic_scan;
    _topic_odom = topic_odom;
    _msg_queue_size = 100;

    _subscriber_scan = nullptr;
    _subscriber_odom = nullptr;
    _sync = nullptr;
}

void LaserRosSubscriber::StartListeningToRos()
{
    if (!_topic_odom.empty())
    {
        _subscriber_scan = new Subscriber<LaserScan>(
            *_node_handle, _topic_scan, _msg_queue_size);
        _subscriber_odom =
            new Subscriber<Odometry>(*_node_handle, _topic_odom, _msg_queue_size);
        _sync = new Synchronizer<ApproximateTimePolicy>(
            ApproximateTimePolicy(100), *_subscriber_scan, *_subscriber_odom);
        _sync->registerCallback(
            boost::bind(&LaserRosSubscriber::Callback, this, _1, _2));
    }
    else
    {
        _subscriber_scan = new Subscriber<LaserScan>(
            *_node_handle, _topic_scan, _msg_queue_size);
        _subscriber_scan->registerCallback(
            &LaserRosSubscriber::CallbackVelodyne, this);
    }
}

void LaserRosSubscriber::Callback(const LaserScan::ConstPtr &msg_scan,
                                  const Odometry::ConstPtr &msg_odom)
{
    // PrintMsgStats(msg_cloud);
    Cloud::Ptr cloud_ptr = RosScanToCloud(msg_scan);
    cloud_ptr->SetPose(RosOdomToPose(msg_odom));
    cloud_ptr->InitProjection(_params);
    ShareDataWithAllClients(*cloud_ptr);
}

void LaserRosSubscriber::CallbackVelodyne(
    const LaserScan::ConstPtr &msg_scan)
{
    // PrintMsgStats(msg_cloud);
    if(first_scan)
    {
        first_scan = false;
        first_scan_time = msg_scan->header.stamp;
    }
    Cloud::Ptr cloud_ptr = RosScanToCloudTF(msg_scan);
    cloud_ptr->InitProjection(_params);
    if (++num_of_scans >=80)
    {
        ShareDataWithAllClients(*cloud_ptr);
        cloud = Cloud();
        num_of_scans = 0;
    }
}

Pose LaserRosSubscriber::RosOdomToPose(const Odometry::ConstPtr &msg)
{
    Pose pose;
    // we want float, so some casting is needed
    Eigen::Affine3d pose_double;
    tf::poseMsgToEigen(msg->pose.pose, pose_double);
    pose = pose_double.cast<float>();
    return pose;
}

Cloud::Ptr LaserRosSubscriber::RosScanToCloud(
    const LaserScan::ConstPtr &msg)
{

    Cloud cloud;

    for (unsigned int i = 0; i < msg->ranges.size(); ++i)
    {
        RichPoint p;
        float range = msg->ranges[i];
        if (range > msg->range_min && range < msg->range_max)
        {
            float angle = msg->angle_min + i * msg->angle_increment;

            p.x() = range * cos(angle);
            p.y() = range * sin(angle);
            p.z() = 0;
            cloud.push_back(p);
        }
        // else
        //     p = invalid_point_;
    }

    return make_shared<Cloud>(cloud);
}

Cloud::Ptr LaserRosSubscriber::RosScanToCloudTF(
    const LaserScan::ConstPtr &msg)
{

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        ros::Time now = ros::Time::now();
        // transformStamped = tfBuffer.lookupTransform("platform_laser_rotation/base/rotation_center", "platform_laser_rotation/carrier/laser_mount", now,
        //                                             ros::Duration(3.0));
        transformStamped.header = msg->header;
        transformStamped.header.frame_id = "platform_laser_rotation/base/rotation_center";
        transformStamped.child_frame_id = "platform_laser_rotation/carrier/laser_mount";
        transformStamped.transform.translation.x = -0.043;
        transformStamped.transform.translation.y = 0.066;
        transformStamped.transform.translation.z = 0.130;
        
        double dur = (msg->header.stamp-first_scan_time).toSec();
        tf2::Quaternion rot((dur/0.025)*0.0785, 0, -1.570);
        transformStamped.transform.rotation.x = rot.x();
        transformStamped.transform.rotation.y = rot.y();
        transformStamped.transform.rotation.z = rot.z();
        transformStamped.transform.rotation.w = rot.w();
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could NOT transform a to b: %s", ex.what());
    }
    for (unsigned int i = 0; i < msg->ranges.size(); ++i)
    {
        RichPoint p;
        Eigen::Vector3d p_in;
        Eigen::Vector3d p_out;
        float range = msg->ranges[i];
        if (range > msg->range_min && range < msg->range_max)
        {
            float angle = msg->angle_min + i * msg->angle_increment;

            p_in.x() = range * sin(angle);
            p_in.y() = range * cos(angle);
            p_in.z() = 0;

            tf2::doTransform(p_in, p_out, transformStamped);
            p.AsEigenVector() = p_out.cast<float>();
            cloud.push_back(p);
        }
        // else
        //     p = invalid_point_;
    }

    return make_shared<Cloud>(cloud);
}

} // namespace depth_clustering
