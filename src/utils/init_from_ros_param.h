#ifndef _UTILS_INIT_FROM_ROS_PARAM_
#define _UTILS_INIT_FROM_ROS_PARAM_

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

using std::string;

namespace depth_clustering {

//Init variables from ros parameter
template <class A>
bool InitFromRosParam(const ros::NodeHandle& nh, const string& param, A& a) {
  bool res = nh.getParam(param, a);
  if (res)
    ROS_INFO_STREAM(param << " : " << a);
  else
    ROS_ERROR("could not find %s", param);
  return res;
}

template <class A>
bool InitFromRosParam(const ros::NodeHandle& nh, const string& param, A& a, const A& b) {
  bool res = nh.param(param, a, b);
  if (res)
    ROS_INFO_STREAM(param << " : " << a);
  else 
    ROS_INFO_STREAM("couldnt find "<< param << "\t Default : " << b);
  return res;
}

} // namespace depth_clustering

#endif