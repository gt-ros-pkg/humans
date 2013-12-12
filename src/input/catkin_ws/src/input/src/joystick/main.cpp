#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "Joystick.h"
#include <syllo_common/SylloNode.h>

#include <sstream>

using std::cout;
using std::endl;

int main(int argc, char **argv)
{
     ros::init(argc, argv, "joystick_node");
     ros::NodeHandle n_;

     // Handle generic parameters for a SylloNode
     SylloNode syllo_node_;
     syllo_node_.init();
    
     // Handle node specific parameters
     std::string joystick_dev;
     ros::param::param<std::string>("joystick_dev", joystick_dev, "/dev/input/js0");     
     
     // Set the joystick_dev param in case it wasn't set
     ros::param::set("joystick_dev", joystick_dev);

     // Notify roscore of joystick publication
     ros::Publisher joystick_pub_ = n_.advertise<sensor_msgs::Joy>("/joystick", 1);     

     // Create a joystick instance
     Joystick joy_;
     joy_.init(joystick_dev);

     // Create arrays to hold joystick values
     int *axis_values_ = new int[joy_.num_of_axes()];
     int *button_values_ = new int[joy_.num_of_buttons()];

     // Declare the joystick message and resize the lists accordingly
     sensor_msgs::Joy joy_msg_;
     joy_msg_.buttons.resize(joy_.num_of_buttons());
     joy_msg_.axes.resize(joy_.num_of_axes());

     while (ros::ok())
     {          
          joy_msg_.header.stamp = ros::Time().now();
          
          // Update time stamp
          joy_.update(&axis_values_, &button_values_);

          // Copy over button values
          for (int i = 0; i < joy_.num_of_buttons(); i++) {
               joy_msg_.buttons[i] = button_values_[i];
          }
          
          // Copy over axis values
          for (int i = 0; i < joy_.num_of_axes(); i++) {
               joy_msg_.axes[i] = axis_values_[i];
          }
          
          // Publish the joystick message
          joystick_pub_.publish(joy_msg_);

          syllo_node_.spin();
     }

     // Clean up syllo node
     syllo_node_.cleanup();

     // Clean up axis and buttons
     if (axis_values_) {
          delete[] axis_values_;
     }
     if (button_values_) {
          delete[] button_values_;
     }

     return 0;
}
