#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <learning_nodelet/NodeletClass.h>


namespace learning_nodelet
{
    void NodeletClass::onInit()
    {
        NODELET_DEBUG("Initializing nodelet...");
        ROS_INFO("Nodelet is Ok for test!!");
    }
}

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(learning_nodelet, NodeletClass, learning_nodelet::NodeletClass, nodelet::Nodelet)