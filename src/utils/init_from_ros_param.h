#ifndef _UTILS_INIT_FROM_ROS_PARAM_
#define _UTILS_INIT_FROM_ROS_PARAM_

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

using std::string;

namespace depth_clustering {

  /**
   * @brief      Init variable from ros parameter
   *
   * @param[in]  nh     Nodehandler to call parameter
   * @param[in]  param  Name of parameter
   * @param[in]  a      Variable
   *
   * @return            if variable is initialized successfully
   */
template <class A>
bool InitFromRosParam(const ros::NodeHandle& nh, const string& param, A& a) {
  bool res = nh.getParam(param, a);
  if (res)
    ROS_INFO_STREAM(param << " : " << a);
  else
    ROS_ERROR_STREAM("could not find " << param);
  return res;
}

  /**
   * @brief      Init variable from ros parameter
   *
   * @param[in]  nh     Nodehandler to call parameter
   * @param[in]  param  Name of parameter
   * @param[in]  a      Variable
   * @param[in]  b      Default value
   *
   * @return            if variable is initialized successfully
   */
template <class A>
bool InitFromRosParam(const ros::NodeHandle& nh, const string& param, A& a,
                      const A& b) {
  a = b;
  bool res = nh.getParam(param, a);
  if (res)
    ROS_INFO_STREAM(param << " : " << a);
  else
    ROS_INFO_STREAM("couldnt find " << param << "\t Default : " << a);
  return res;
}

}  // namespace depth_clustering

#endif