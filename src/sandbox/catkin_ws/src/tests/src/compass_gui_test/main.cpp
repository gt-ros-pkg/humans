#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <sstream>

#include <syllo_common/SylloNode.h>
#include <syllo_common/Orientation.h>


#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>

using std::cout;
using std::endl;


//void MinRangeCallback(const std_msgs::Float32::ConstPtr& msg)
//{
//     sonar.set_min_range(msg->data);
//}

int main(int argc, char **argv)
{              
     ros::init(argc, argv, "rqt_compass_test");
     ros::NodeHandle nh;
     
     SylloNode node;
     node.init();          
     
     ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/rqt_compass/pose",1);

     double heading = 0;
     double pitch = 0;
     double roll = 0;
     double x,y,z,w;
     geometry_msgs::PoseStamped pose_;

     while (ros::ok()) {          

          eulerToQuaternion_xyzw(roll, pitch, heading, x,y,z,w);
          pose_.pose.orientation.x = x;
          pose_.pose.orientation.y = y;
          pose_.pose.orientation.z = z;
          pose_.pose.orientation.w = w;

          pose_pub.publish(pose_);

          heading += 0.01;
          if (heading >= 360) {
               heading = 0;
          }

          node.spin();
     }
     node.cleanup();
     return 0;
}
