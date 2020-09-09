#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PosePublisher");
    ros::Time::init();
    ros::Time t = ros::Time::now();
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    tf::Quaternion q;
    tf::Transform Transform;
    q.setRPY(0,0,0);
    Transform.setRotation(q);
    Transform.setOrigin(tf::Vector3(0,0,0));
    //frame_id, child_frame_id
    br.sendTransform(tf::StampedTransform(Transform,t,"map","tf1"));

    q.setRPY(M_PI/4,0,M_PI);
    Transform.setRotation(q);
    Transform.setOrigin(tf::Vector3(1,0,1.5));
    br.sendTransform(tf::StampedTransform(Transform,t,"tf1","tf2"));
    q.setRPY(0,M_PI/3,M_PI/2);
    Transform.setRotation(q);
    Transform.setOrigin(tf::Vector3(0,2,0.5));
    br.sendTransform(tf::StampedTransform(Transform,t,"tf2","tf3"));

    tf::StampedTransform transformStamped;
    try{
        //target_frame, source_frame
        listener.waitForTransform("tf2", "tf1", ros::Time(0), ros::Duration(0.01));
        listener.lookupTransform("tf2", "tf1", ros::Time(0), transformStamped);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("tf transform error: %s",ex.what());
    }
    ROS_INFO("test1:%f",transformStamped.getOrigin().z());//-1.060660

    try{
        listener.waitForTransform("tf1", "tf2", ros::Time(0), ros::Duration(0.01));
        listener.lookupTransform("tf1", "tf2", ros::Time(0), transformStamped);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("tf transform error: %s",ex.what());
    }
    ROS_INFO("test2:%f",transformStamped.getOrigin().z());//1.500000

    tf::StampedTransform transformStamped2;
    try{
        listener.waitForTransform("tf2", "tf3", ros::Time(0), ros::Duration(0.01));
        listener.lookupTransform("tf2", "tf3", ros::Time(0), transformStamped2);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("tf transform error: %s",ex.what());
    }
    ROS_INFO("test3:%f",transformStamped2.getOrigin().z());//0.500000



    tf::Transform Transform3 = transformStamped * transformStamped2;
    //x:1.000000,y:-1.060660,z:3.267767
    ROS_INFO("test4: x:%f,y:%f,z:%f",Transform3.getOrigin().x(),Transform3.getOrigin().y(),Transform3.getOrigin().z());
    // 0.000000,y:3.799038,z:0.383975
    Transform3 = transformStamped2 * transformStamped;
    ROS_INFO("test5: x:%f,y:%f,z:%f",Transform3.getOrigin().x(),Transform3.getOrigin().y(),Transform3.getOrigin().z());

    tf::StampedTransform transformStamped3;
    try{
        listener.waitForTransform("tf1", "tf3", ros::Time(0), ros::Duration(0.1));
        listener.lookupTransform("tf1", "tf3", ros::Time(0), transformStamped3);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("tf transform error: %s",ex.what());
    }
    // x:1.000000,y:-1.060660,z:3.267767
    ROS_INFO("test4: x:%f,y:%f,z:%f",transformStamped3.getOrigin().x(),transformStamped3.getOrigin().y(),transformStamped3.getOrigin().z());
    return 0;

    // Conclusion
    // lookupTransform 的说明文档大概错了， 应该是source_frame target_frame
    // transform13 = transform12*transform23
}
