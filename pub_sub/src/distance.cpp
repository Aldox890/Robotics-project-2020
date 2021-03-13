#include "ros/ros.h"
#include "pub_sub/distance.h"
//#include "math.h"

bool distanceFunc(pub_sub::distance::Request &req,pub_sub::distance::Response &res){
  res.distance = sqrt(pow((req.carPosX - req.obsPosX),2) + pow((req.carPosY - req.obsPosY),2) + pow((req.carPosZ - req.obsPosZ),2));
  //ROS_INFO("sending back response: [%g]", (double)res.distance);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("distance", distanceFunc);
  ROS_INFO("Distance service ready");
  ros::spin();
  return 0;
}
