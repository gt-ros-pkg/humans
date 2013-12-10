#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "videoray/Throttle.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <sstream>
#include <cmath>

using std::cout;
using std::endl;

//
// Converts desired heading, desired velocity, and desired depth into
// throttle (left, right, vertical) commands
//
#define PI (3.14159265359)

double depth_ref = 0;
double speed_ref = 0;
double heading_ref = 0;

double roll_ = 0;
double pitch_ = 0;
double yaw_ = 0;

videoray::Throttle throttle_;

nav_msgs::Odometry odom_;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
     odom_ = *msg;
}

void desiredVelocityCallback(const std_msgs::Float32::ConstPtr& msg)
{
     speed_ref = msg->data;
}

void desiredHeadingCallback(const std_msgs::Float32::ConstPtr& msg)
{
     heading_ref = msg->data;
}

void desiredDepthCallback(const std_msgs::Float32::ConstPtr& msg)
{
     depth_ref = msg->data;
     ROS_INFO("Desired Depth: %f", depth_ref);
}

void quaternionToEuler(const double &q0, const double &q1, 
                       const double &q2, const double &q3,
                       double &roll, double &pitch, double &yaw)
{
     roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2) );
     pitch = asin(2*(q0*q2-q3*q1));
     yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3) );
}

double normDegrees(double input)
{
     if (input < 0) {
          input += 360;
     } else if(input >= 360) {
          input -= 360;
     }
     return input;
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "videoray_control_p");
     
     ros::NodeHandle n;

     ros::Publisher throttle_pub = 
          n.advertise<videoray::Throttle>("throttle_cmds", 1);
     
     ros::Subscriber desired_vel_sub = n.subscribe("desired_velocity", 
                                                   1, 
                                                   desiredVelocityCallback);

     ros::Subscriber desired_head_sub = n.subscribe("desired_heading", 
                                                    1, 
                                                    desiredHeadingCallback);

     ros::Subscriber desired_depth_sub = n.subscribe("desired_depth", 
                                                     1, 
                                                     desiredDepthCallback);

     ros::Subscriber odom_sub = n.subscribe("odometry", 
                                            1, 
                                            odomCallback);
     
     ros::Rate loop_rate(10);

     geometry_msgs::Quaternion quat;

     double depth_err = 0;
     double speed_err = 0;
     double heading_err = 0;
     double heading = 0;
     double heading_port, heading_star, speed_port, speed_star;
     double heading_weight = 0.5;
     double speed_weight = 0.5;
     double K_heading = 1;
     double K_speed = 10;
     double K_depth = 50;

     while (ros::ok())
     {          
          quat = odom_.pose.pose.orientation;
          quaternionToEuler(quat.w, quat.x, quat.y, quat.z,
                            roll_, pitch_, yaw_);
          
          //ROS_INFO("Desired heading: %f", heading_ref);
          
          
          depth_err = depth_ref - odom_.pose.pose.position.z;
          speed_err = speed_ref - odom_.twist.twist.linear.x;

          heading = normDegrees(yaw_*180/PI);
          ROS_INFO("Actual heading: %f", heading);
          heading_err = heading_ref - heading;
          
          if (abs(heading_err) < 180) {
               heading_port = -K_heading*heading_err;
               heading_star = K_heading*heading_err;
          } else  {
               heading_port = K_heading*heading_err;
               heading_star = -K_heading*heading_err;
          }
          
          speed_port = K_speed*speed_err;
          speed_star = K_speed*speed_err;

          throttle_.PortInput = heading_weight*heading_port + speed_weight*speed_port;
          throttle_.StarInput = heading_weight*heading_star + speed_weight*speed_star;
          throttle_.VertInput = K_depth*depth_err;         

          throttle_pub.publish(throttle_);

          ros::spinOnce();
          loop_rate.sleep();
     }
     return 0;
}
