#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <learning_dynamic_param/tutorialConfig.h>

void callback(learning_dynamic_param::tutorialConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_tutorials");

  dynamic_reconfigure::Server<learning_dynamic_param::tutorialConfig> server;
  dynamic_reconfigure::Server<learning_dynamic_param::tutorialConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}