#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "pub_sub/distance.h"
#include "pub_sub/CustomMessage.h"

double min;
double max;
ros::Publisher distancePub;

void callbackFunction(const nav_msgs::Odometry::ConstPtr& msg1,const nav_msgs::Odometry::ConstPtr& msg2)
{
	//ROS_INFO("Seq: [%d]", msg1->header.seq);
	//ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg1->pose.pose.position.x,msg1->pose.pose.position.y, msg1->pose.pose.position.z);
  pub_sub::distance srv;
	ros::NodeHandle n;
	ros::ServiceClient client;
	client = n.serviceClient<pub_sub::distance>("distance");

	srv.request.carPosX = msg1->pose.pose.position.x;
	srv.request.carPosY = msg1->pose.pose.position.y;
	srv.request.carPosZ = msg1->pose.pose.position.z;

	srv.request.obsPosX = msg2->pose.pose.position.x;
	srv.request.obsPosY = msg2->pose.pose.position.y;
	srv.request.obsPosZ = msg2->pose.pose.position.z;


	if (client.call(srv))
  {
    //ROS_INFO("distance: %g", /*(long int)*/(double)srv.response.distance);
		pub_sub::CustomMessage msg;
		msg.distance = (double)srv.response.distance;
		if((double)srv.response.distance < min){
			msg.status_flag = "Crash";
		}
		else if((double)srv.response.distance >= min && (double)srv.response.distance <= max){
			msg.status_flag = "Unsafe";
		}
		else if((double)srv.response.distance > max){
			msg.status_flag = "Safe";
		}
		distancePub.publish(msg);
  }
  else
  {
    ROS_ERROR("Failed to call service distance");
    return ;
  }
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pub");
	ros::NodeHandle n;
	n.getParam("/min", min);
  n.getParam("/max", max);

	ros::Subscriber car;
	ros::Subscriber obs;
	distancePub = n.advertise<pub_sub::CustomMessage>("distanceTopic", 1000);


	//ros::NodeHandle nCar = nCar.subscribe("/odomCar", 1000, carCallback);
	//ros::NodeHandle nObs = nObs.subscribe("/odomObs", 1000, obsCallback);
	message_filters::Subscriber<nav_msgs::Odometry> sub1(n, "/odomCar", 1000);
  message_filters::Subscriber<nav_msgs::Odometry> sub2(n, "/odomObs", 1000);
	message_filters::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(sub1, sub2, 10);
  sync.registerCallback(boost::bind(&callbackFunction, _1, _2));

	ros::spin();
	return 0;
}
