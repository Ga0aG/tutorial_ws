#include <boost/bind.hpp>
#include <ros/ros.h>
int f(int a, int b)
{
    return a + b;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "param_test");
  int a = boost::bind(f,1,2)();
  ROS_INFO("%d",a);
}