#include <learning_actionlib/DoDishesAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<learning_actionlib::DoDishesAction> Client;

// 当action完成后会调用次回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const learning_actionlib::DoDishesResultConstPtr& result)
{
    ROS_INFO("Yay! The dishes are now clean");
    ros::shutdown();
}

// 当action激活后会调用次回调函数一次
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// 收到feedback后调用的回调函数
void feedbackCb(const learning_actionlib::DoDishesFeedbackConstPtr& feedback)
{
    ROS_INFO(" percent_complete : %f ", feedback->percent_complete);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "do_dishes_client");
  Client client("do_dishes", true); // true -> don't need ros::spin()
  
  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("Action server started, sending goal.");
  
  learning_actionlib::DoDishesGoal goal;
  goal.dishwasher_id = 1;
  client.sendGoal(goal,&doneCb, &activeCb, &feedbackCb);
  client.waitForResult(ros::Duration(5.0));
  // if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //   ROS_INFO("Yay! The dishes are now clean");
  // ROS_INFO("Current State: %s\n", client.getState().toString().c_str());//只出现一次而且比succeeded早
  ros::spin();//有这个会打印feedback
  return 0;
}