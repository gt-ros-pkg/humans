#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "videoray/Throttle.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <sstream>

#include <fstream>


using std::cout;
using std::endl;

#define PI (3.14159265359)

double nav_x_ = 0;
double nav_y_ = 0;
double nav_heading_ = 0;
double nav_z_ = 0;
double roll_ = 0;
double pitch_ = 0;
double yaw_ = 0;

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

void eulerToQuaternion(const double &roll, const double &pitch, 
                       const double &yaw,
                       double &q0, double &q1, 
                       double &q2, double &q3)
{
     q0 = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
     q1 = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
     q2 = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
     q3 = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
}

void cb_nav_x(const std_msgs::Float32::ConstPtr& msg)
{

     nav_x_ = msg->data;
}
void cb_nav_y(const std_msgs::Float32::ConstPtr& msg)
{
     nav_y_ = msg->data;
}
void cb_nav_heading(const std_msgs::Float32::ConstPtr& msg)
{
     nav_heading_ = msg->data;
     nav_heading_ = normDegrees(90 - nav_heading_);
}

void cb_nav_z(const std_msgs::Float32::ConstPtr& msg)
{
     nav_z_ = msg->data;
     nav_z_ += 3.6;
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "videoray_moos");     
     ros::NodeHandle n;

     ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("motion",1);
     geometry_msgs::Pose pose_;
     
     ros::Subscriber sub_nav_x = n.subscribe("NAV_X", 1, cb_nav_x);
     ros::Subscriber sub_nav_y = n.subscribe("NAV_Y", 1, cb_nav_y);
     ros::Subscriber sub_nav_heading = n.subscribe("NAV_HEADING", 1, cb_nav_heading);
     ros::Subscriber sub_nav_z = n.subscribe("NAV_Z", 1, cb_nav_z);
     
     double rate = 10;
     ros::Rate loop_rate(rate);

     ros::Time begin = ros::Time::now();
     ros::Time curr_time = begin;
     ros::Time prev_time = begin;
     ros::Duration dt = curr_time - prev_time;
     
     geometry_msgs::Quaternion quat;

     std::string ns = ros::this_node::getNamespace();
     std::string filename = "/home/syllogismrxs/repos/syllo-moos/share/missions/uhri_sim/" + ns + "_log.txt";
     std::ofstream log;
     log.open (filename.c_str());     

     while (ros::ok())
     {
          curr_time = ros::Time::now();
          dt = curr_time - prev_time;
          prev_time = curr_time;

          //ROS_INFO("11: %f", x_[11]);
          pitch_ = 0; roll_ = 0;
          yaw_ = nav_heading_ * PI / 180.0;

          eulerToQuaternion(roll_, 
                            pitch_, 
                            yaw_,
                            quat.w, quat.x, quat.y, quat.z);          
          
          pose_.position.x = nav_x_;
          pose_.position.y = nav_y_;
          pose_.position.z = nav_z_;
          pose_.orientation.x = quat.x;
          pose_.orientation.y = quat.y;
          pose_.orientation.z = quat.z;
          pose_.orientation.w = quat.w;

          log << nav_x_ << "\t" << nav_y_ << endl;

          pose_pub.publish(pose_);

          ros::spinOnce();

          loop_rate.sleep();
     }
     
     log.close();
     
     return 0;
}
