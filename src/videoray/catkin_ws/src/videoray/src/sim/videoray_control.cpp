#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
     ros::init(argc, argv, "videoray_control");

     ros::NodeHandle n;

     ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/robot/motion", 1000);

     ros::Rate loop_rate(10);

     while (ros::ok())
     {
          geometry_msgs::Twist twist;
          geometry_msgs::Vector3 linear;
          geometry_msgs::Vector3 angular;

          linear.x = 0.1; //forward
          //linear.y = 0;
          //linear.z = 0;
          linear.z = 0.1; // up
          //linear.z = -0.1; // down
          
          angular.x = 0.1;
          angular.y = 0.0;
          //angular.z = 0.1; // rotate left
          angular.z = -0.1; // rotate right
          
          twist.linear = linear;
          twist.angular = angular;

          twist_pub.publish(twist);

          ros::spinOnce();

          loop_rate.sleep();
     }
     

     return 0;
}
