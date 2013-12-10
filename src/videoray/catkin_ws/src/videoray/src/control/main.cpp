#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "VideoRayComm.h"
#include <syllo_common/Filter.h>
#include <syllo_common/SylloNode.h>
#include <syllo_common/Orientation.h>
#include "sensor_msgs/Joy.h"
#include "videoray/Throttle.h"

#include <sstream>

using std::cout;
using std::endl;

int *axis_;
int *button_;
int num_of_buttons_ = -1;
int num_of_axes_ = -1;
bool joystick_enabled_ = false;

void callback_joystick(const sensor_msgs::JoyConstPtr& msg)
{
     // Check for change in number of buttons
     int num_of_buttons = msg->buttons.size();
     if (num_of_buttons != num_of_buttons_) {
          num_of_buttons_ = num_of_buttons;
          if (button_ != NULL) {
               delete[] button_;
          }
          button_ = new int[num_of_buttons_];
     }

     // Check for change in number of axes
     int num_of_axes = msg->axes.size();
     if (num_of_axes != num_of_axes_) {
          num_of_axes_ = num_of_axes;
          if (axis_ != NULL) {
               delete[] axis_;
          }
          axis_ = new int[num_of_axes_];
     }

     // Update button array
     for (int i = 0 ; i < num_of_buttons; i++) {
          button_[i] = msg->buttons[i];
     }

     // Update axis array
     for (int i = 0 ; i < num_of_axes; i++) {
          axis_[i] = msg->axes[i];
     }
     
     joystick_enabled_ = true;
}

int main(int argc, char **argv)
{     

     ros::init(argc, argv, "videoray_control");
     ros::NodeHandle n_;

     // Handle generic parameters for a SylloNode
     SylloNode syllo_node_;
     syllo_node_.init();    

     ros::Subscriber joystick_sub_ = n_.subscribe<>("/joystick", 1, callback_joystick);
     ros::Publisher pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/pose", 1);     
     ros::Publisher pose_only_pub_ = n_.advertise<geometry_msgs::Pose>("/pose_only", 1);     
     ros::Publisher throttle_pub_ = n_.advertise<videoray::Throttle>("/throttle_cmd", 1);
     
     geometry_msgs::PoseStamped pose_stamped_;
     
     VideoRayComm::Status_t status;
     VideoRayComm comm;
     
     int vert_thrust_ = 0;
     int port_thrust_ = 0;
     int star_thrust_ = 0;
     int turn_ = 0;
     int lights_ = 0;
     int tilt_ = 0;
     int focus_ = 0;
     
     while (ros::ok()) {                   

          if (joystick_enabled_) {

               //cout << "Vertical: " << axis_[1] << endl;
               vert_thrust_ = invert_sign(normalize(axis_[1], -32767, 32767, -99, 99));
               port_thrust_ = invert_sign(normalize(axis_[3], -32767, 32767, -89, 89));
               star_thrust_ = invert_sign(normalize(axis_[3], -32767, 32767, -89, 89));

               turn_ = invert_sign(normalize(axis_[2], -32767, 32767, -89, 89));
               port_thrust_ -= turn_;
               star_thrust_ += turn_;
          
               //cout << axis_[0] << axis_[1] << axis_[2] << axis_[3] << axis_[4] << axis_[5] << endl;

               tilt_ += invert_sign(normalize(axis_[5], -32767, 32767, -1, 1));
               tilt_ = saturate(tilt_, -80, 80);

               focus_ += invert_sign(normalize(axis_[4], -32767, 32767, -1, 1));
               focus_ = saturate(tilt_, -100, 100);

               if (button_[3]) {
                    lights_ += 1;
               } else if (button_[1]) {
                    lights_ -= 1;
               }
               lights_ = saturate(lights_, 0, 63);

               // Reset button
               if (button_[9]) {
                    tilt_ = 0;
                    focus_ = 0;
                    lights_ = 0;
                    port_thrust_ = 0;
                    star_thrust_ = 0;
                    vert_thrust_ = 0;               
               }

          }

          status = comm.set_vertical_thruster(vert_thrust_);
          status = comm.set_port_thruster(port_thrust_);
          status = comm.set_starboard_thruster(star_thrust_);
          status = comm.set_lights(lights_);
          status = comm.set_camera_tilt(tilt_);
          
          status = comm.send_control_command();
          if (status != VideoRayComm::Success) {
               cout << "Exec Transfer Error!" << endl;
          }
               
          status = comm.send_nav_data_command();
          if (status != VideoRayComm::Success) {
               cout << "Exec Transfer Error!" << endl;
          }

          // Time stamp the message
          pose_stamped_.header.stamp = ros::Time().now();

          // Populate x,y,z positions
          pose_stamped_.pose.position.x = 0;
          pose_stamped_.pose.position.y = 0;
          pose_stamped_.pose.position.z = comm.depth();
                    
          // Populate the orientation
          geometry_msgs::Quaternion quat;
          eulerToQuaternion_xyzw_deg(comm.roll(), comm.pitch(), comm.heading(),
                                 quat.x, quat.y, quat.z, quat.w);
          
          pose_stamped_.pose.orientation = quat;
          
          // Publish pose stamped and regular pose for rqt_pose_view
          pose_pub_.publish(pose_stamped_);
          pose_only_pub_.publish(pose_stamped_.pose);
          
          videoray::Throttle throttle;
          throttle.PortInput = port_thrust_;
          throttle.StarInput = star_thrust_;
          throttle.VertInput = vert_thrust_;
          throttle_pub_.publish(throttle);

          //cout << "------------------------------------" << endl;
          //cout << "Heading: " << comm.heading() << endl;
          //cout << "Pitch: " << comm.pitch() << endl;
          //cout << "Roll: " << comm.roll() << endl;
          //cout << "Depth: " << comm.depth() << endl;
          //cout << "Yaw Accel: " << comm.yaw_accel() << endl;
          //cout << "Pitch Accel: " << comm.pitch_accel() << endl;
          //cout << "Roll Accel: " << comm.roll_accel() << endl;
          //cout << "Surge Accel: " << comm.surge_accel() << endl;
          //cout << "Sway Accel: " << comm.sway_accel() << endl;
          //cout << "Heave Accel: " << comm.heave_accel() << endl;                    
          //}
          
          syllo_node_.spin();
     }

     // Clean up syllo node
     syllo_node_.cleanup();

     return 0;
}
