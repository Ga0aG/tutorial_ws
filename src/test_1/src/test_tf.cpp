// #include <ros/ros.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/TransformStamped.h>
#include "learning_pluginlib/polygon_base.h"
#include <ros/ros.h>
int main(int argc, char** argv){
  ros::init(argc, argv, "learning_dump");
  ros::NodeHandle node;
  system("cd ~/yaowei_ws/src/map_service/maps;rosparam dump testParam.yaml /virtualWalls");
  return 0;
}