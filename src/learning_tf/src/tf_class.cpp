#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
int main(int argc, char** argv){
    ros::init(argc, argv, "tf_class");
    ros::NodeHandle node;
    tf::Quaternion tf_q(1,0,0,0);
    geometry_msgs::PoseStamped oscillation_pose_;
    ROS_INFO("%d",(int)tf_q.length());
    ROS_INFO("%d",(int)oscillation_pose_.pose.position.x);
    int a;
    ROS_INFO("%d",a);
    return 0;
}