#include <ros/ros.h>
#include <ros/console.h>

#include <visualization_msgs/MarkerArray.h>

#include "utils/timer.h"

class Listener 
{
public:
  Listener(ros::NodeHandle& nh):_nh{nh}{
    _sub = _nh.subscribe("/depth_clusterer/segmented_objects", 1,
                         &Listener::CallBack, this);
  }
  ~Listener(){}
  void CallBack(const visualization_msgs::MarkerArray& msg) { ROS_INFO("called %lu us", _timer.measure()); }
private:
 depth_clustering::time_utils::Timer _timer;
 ros::NodeHandle _nh;
 ros::Subscriber _sub;
};

int main(int argc, char* argv[]) 
{ 
    ros::init(argc, argv, "dc_listener");
    ros::NodeHandle nh;
    Listener listen = Listener(nh);
    ros::spin();
    return 0;
}