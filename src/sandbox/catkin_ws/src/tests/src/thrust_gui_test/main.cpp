#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <sstream>

#include <syllo_common/SylloNode.h>
#include <syllo_common/Orientation.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <videoray/Throttle.h>

using std::cout;
using std::endl;

//void MinRangeCallback(const std_msgs::Float32::ConstPtr& msg)
//{
//     sonar.set_min_range(msg->data);
//}

float delta = 1;
void next_value(float &value, bool &direction)
{
     if (direction) {
          value += delta;
     } else {
          value -= delta;
     }     

     if (value > 100) {
          value = 100;
          direction = not(direction); 
     } else if (value < -100) {
          value = -100;
          direction = not(direction);
     }
}

int main(int argc, char **argv)
{              
     ros::init(argc, argv, "rqt_compass_test");
     ros::NodeHandle nh;
     
     SylloNode node;
     node.init();          
     
     ros::Publisher throttle_pub = nh.advertise<videoray::Throttle>("/rqt_thrust_monitor/throttle_cmd",1);
     videoray::Throttle throttle_;
     throttle_.PortInput = 0;
     throttle_.StarInput = 50;
     throttle_.VertInput = 100;

     bool port_dir = false, star_dir = true, vert_dir = false;

     while (ros::ok()) {          

          next_value(throttle_.PortInput, port_dir);
          next_value(throttle_.StarInput, star_dir);
          next_value(throttle_.VertInput, vert_dir);

          throttle_pub.publish(throttle_);

          node.spin();
     }
     node.cleanup();
     return 0;
}
