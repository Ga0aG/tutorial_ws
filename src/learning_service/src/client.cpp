#include "ros/ros.h"
#include "learning_service/num_op.h"
#include <cstdlib>
//rosrun learning_service client 1 2 0

// rosservice call /num_op "a: 0
// b: 1
// operation: 0"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "num_op_client");
  if (argc != 4)
  {
    ROS_INFO("usage: num_op_client X Y Z");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<learning_service::num_op>("num_op");
  learning_service::num_op srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  srv.request.operation = atoll(argv[3]);
  if (client.call(srv))
  {
    ROS_INFO("result: %ld", (long int)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service num_op");
    return 1;
  }

  return 0;
}