#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <vector>

using namespace std;

int main(int argc, char *argv[])
{
     ros::init(argc, argv, "talker");
     ros::NodeHandle nh;
     
     //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

     ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/uwsim/joint_state_command", 1);

     ros::Rate loop_rate(10);

     int count = 0;
     while (ros::ok())
     {
          sensor_msgs::JointState joint_state;

          std::vector<std::string> name;
          name.push_back("Elbow");
          
          std::vector<double> position;
          //position.push_back(0.5);

          std::vector<double> velocity;
          velocity.push_back(0.1);

          std::vector<double> effort;
          //effort.push_back(0.5);


          joint_state.name = name;
          joint_state.position = position;
          joint_state.velocity = velocity;
          joint_state.effort = effort;
          
          joint_pub.publish(joint_state);

          ros::spinOnce();

          loop_rate.sleep();
          ++count;
     }


     return 0;
     
}
