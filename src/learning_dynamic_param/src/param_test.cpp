#include <ros/ros.h>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <learning_dynamic_param/tutorialConfig.h>

class tutorial{
    public:
        int int_param;
        double double_param;
        std::string str_param;
        bool bool_param;
        tutorial(void){
            int_param = 1;
            double_param = 0.01;
            str_param = "Hello_World";
            bool_param = true;
            std::cout << "init success" << std::endl; 
        }
        void callback(learning_dynamic_param::tutorialConfig &config, uint32_t level);
        void print();
};

void tutorial::callback(learning_dynamic_param::tutorialConfig &config, uint32_t level) {
    int_param = config.int_param;
    double_param = config.double_param;
    str_param = config.str_param;
    bool_param = config.bool_param;
    ROS_INFO("Reconfigure Request: %d %f %s %s %d",
    int_param,double_param,str_param.c_str(),bool_param?"True":"False",config.size);
}
void tutorial::print(){
    ROS_INFO("initial param: %d %f %s %s",
    int_param,double_param,str_param.c_str(),bool_param?"True":"False");
}

int main(int argc, char **argv) {
  tutorial test{};
  ros::init(argc, argv, "param_test");
  dynamic_reconfigure::Server<learning_dynamic_param::tutorialConfig> server;
  dynamic_reconfigure::Server<learning_dynamic_param::tutorialConfig>::CallbackType f;

  f = boost::bind(&tutorial::callback,test, _1, _2);
  server.setCallback(f);
  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
  }