#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <learning_dynamic_param/tutorialConfig.h>

class configClass{
    public:
    int int_param;
    double double_param;
    std::string str_param;
    bool bool_param;
    int size;

    static configClass& getInstance();
    #define CONFIGCLASS configClass::getInstance()

    void reconfigure(learning_dynamic_param::tutorialConfig &config);
    boost::mutex& configMutex() {return config_mutex_;}

    private:
    configClass(){
        int_param = 1;
        double_param = 1.0;
        str_param = "1";
        bool_param = true;
        size = 1;
    }
    boost::mutex config_mutex_; 
    static configClass *instance;
};

configClass* configClass::instance = NULL;
configClass& configClass::getInstance(){
    if (instance == NULL) instance = new configClass();
    return *instance;
}

void configClass::reconfigure(learning_dynamic_param::tutorialConfig &config) {
    boost::mutex::scoped_lock l(config_mutex_);
    int_param = config.int_param;
    double_param = config.int_param;
    str_param = config.int_param;
    bool_param = config.int_param;
    size = config.int_param;
}

class testClass{
    public:
        testClass(){};
        void onInit(){
            ros::NodeHandle nh("");
            f = boost::bind(&testClass::reconfigureCB, this, _1, _2);
            server.setCallback(f);
        } 
        void reconfigureCB(learning_dynamic_param::tutorialConfig &config, uint32_t level){
            CONFIGCLASS.reconfigure(config);
            ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
        }
    private:
        // 放在onInit里参数服务器里看不到参数
        dynamic_reconfigure::Server<learning_dynamic_param::tutorialConfig> server;
        dynamic_reconfigure::Server<learning_dynamic_param::tutorialConfig>::CallbackType f;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_tutorials");
    testClass t{};
    t.onInit();
    ros::spin();
    return 0;
}