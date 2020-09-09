#include "ros/ros.h"
#include "learning_service/num_op.h"
// #include "beginner_tutorials/AddTwoInts.h"

bool send_cmd(learning_service::num_op::Request  &req,
         learning_service::num_op::Response &res)
{
  if(req.operation == 0){
      res.result = req.a + req.b;
      ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
      ROS_INFO("sending back response: [%ld]", (long int)res.result);
      return true;
  }
  else if(req.operation == 1){
      res.result = req.a - req.b;
      ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
      ROS_INFO("sending back response: [%ld]", (long int)res.result);
      return true;
  }
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "num_op_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("num_op", send_cmd);
  ROS_INFO("Ready to calculate.");
  ros::spin();

  return 0;
}
