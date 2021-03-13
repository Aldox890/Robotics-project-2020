#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"


double longitudeZero;
double latitudeZero;
double altitudeZero;
ros::Publisher odom_pub;
std::string nodeName;

void carCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
   /*
   double enuLatitude = msg->latitude - latitudeZero;
   double enuLongitude = msg->longitude - longitudeZero;
   double enuAltitude = msg->altitude - altitudeZero;
   */

     double a = 6378137;
     double b = 6356752.3142;
     double f = (a - b) / a;
     double e_sq = f * (2-f);
     float deg_to_rad = 0.0174533;

     float latitude = msg->latitude;
     float longitude = msg->longitude;
     float h = msg->altitude;

     float latitude_init = latitudeZero;
     float longitude_init = longitudeZero;
     float h0 = altitudeZero;

     //lla to ecef
     float lamb = deg_to_rad*(latitude);
     float phi = deg_to_rad*(longitude);
     float s = sin(lamb);
     float N = a / sqrt(1 - e_sq * s * s);

     float sin_lambda = sin(lamb);
     float  cos_lambda = cos(lamb);
     float  sin_phi = sin(phi);
     float  cos_phi = cos(phi);

     float  x = (h + N) * cos_lambda * cos_phi;
     float  y = (h + N) * cos_lambda * sin_phi;
     float  z = (h + (1 - e_sq) * N) * sin_lambda;

     //ROS_INFO("ECEF position: [%f,%f, %f]", x, y,z);


     // ecef to enu

     lamb = deg_to_rad*(latitude_init);
     phi = deg_to_rad*(longitude_init);
     s = sin(lamb);
     N = a / sqrt(1 - e_sq * s * s);

     sin_lambda = sin(lamb);
     cos_lambda = cos(lamb);
     sin_phi = sin(phi);
     cos_phi = cos(phi);

     float  x0 = (h0 + N) * cos_lambda * cos_phi;
     float  y0 = (h0 + N) * cos_lambda * sin_phi;
     float  z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

     float xd = x - x0;
     float  yd = y - y0;
     float  zd = z - z0;

     float  enuLatitude = -sin_phi * xd + cos_phi * yd;
     float  enuLongitude = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
     float  enuAltitude = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;


   //ROS_INFO("%s Latitude: [%g]", nodeName.c_str(),enuLatitude);
   //ROS_INFO("%s Longitude: [%g]", nodeName.c_str(),enuLongitude);
   //ROS_INFO("%s Altitude: [%g]", nodeName.c_str(),enuAltitude);

   // publish TF
   tf::Transform transform;

   tf::Quaternion q;
   q.setRPY(0, 0, 0);
   transform.setRotation(q);

   static tf::TransformBroadcaster br;
   transform.setOrigin( tf::Vector3(enuLatitude, enuAltitude, enuLongitude) );
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", nodeName));

   //publish Odometry
   ros::Time current_time;
   nav_msgs::Odometry odom;
   ros::NodeHandle n;
   tf::TransformBroadcaster odom_broadcaster;
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

   /*
   geometry_msgs::TransformStamped odom_trans;
   odom_trans.header.stamp = current_time;
   odom_trans.header.frame_id = "odom";
   odom_trans.child_frame_id = "base_link";
   odom_trans.transform.translation.x = enuLatitude;
   odom_trans.transform.translation.y = enuAltitude;
   odom_trans.transform.translation.z = enuLongitude;
   odom_trans.transform.rotation = odom_quat;
   odom_broadcaster.sendTransform(odom_trans);
   */

   odom.header.stamp = current_time;
   odom.header.frame_id = "world";
   odom.child_frame_id = "car";
   odom.pose.pose.position.x = enuLatitude;
   odom.pose.pose.position.y = enuAltitude;
   odom.pose.pose.position.z = enuLongitude;
   odom.pose.pose.orientation = odom_quat;

   odom_pub.publish(odom);
   //ROS_INFO("%s", nodeName.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  nodeName = ros::this_node::getName();
  //ROS_INFO("name: [%s]",nodeName.c_str());
  n.getParam("/latitude", latitudeZero);
  n.getParam("/longitude", longitudeZero);
  n.getParam("/altitude", altitudeZero);
  //ROS_INFO("latitude: [%g]" , latitudeZero);
  //ROS_INFO("longitude: [%g]" , longitudeZero);
  //ROS_INFO("altitude: [%g]" , altitudeZero);

  //sensor_msgs::NavSatFix
  //std_msgs::String latitude;
  //n.getParam("/latitude", latitude.data);
  //ROS_INFO("Parameter: [%s]" + latitude.data);

  if(nodeName == "/car"){
     odom_pub = n.advertise<nav_msgs::Odometry>("odomCar", 1000);
  }
  else if (nodeName == "/obstacle"){
     odom_pub = n.advertise<nav_msgs::Odometry>("odomObs", 1000);
  }


	ros::NodeHandle nCar;
  ros::NodeHandle nObs;
  ros::Subscriber car;
  ros::Subscriber obs;

  if(nodeName == "/car"){
     car = nCar.subscribe("/swiftnav/front/gps_pose", 1000, carCallback,ros::TransportHints().tcpNoDelay());
  }
  else if (nodeName == "/obstacle"){
     obs = nObs.subscribe("/swiftnav/obs/gps_pose", 1000, carCallback,ros::TransportHints().tcpNoDelay());
  }
	ros::spin();
	return 0;
}
