#include <learning_actionlib/DoDishesAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<learning_actionlib::DoDishesAction> Server;

void execute(const learning_actionlib::DoDishesGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
    ros::Rate r(1);
    learning_actionlib::DoDishesFeedback feedback;
    ROS_INFO("Dishwasher %d is working.", goal->dishwasher_id);
    // 假设洗盘子的进度，并且按照1hz的频率发布进度feedback
    for(int i=1; i<=10; i++)
    {
        feedback.percent_complete = i * 10;
        as->publishFeedback(feedback);
        r.sleep();
    }
    // 当action完成后，向客户端返回结果
    ROS_INFO("Dishwasher %d finish working.", goal->dishwasher_id);
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_server");
  ros::NodeHandle n;
  Server server(n, "do_dishes", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}