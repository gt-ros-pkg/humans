#include <iostream>
#include <uw_gazebo_ros_plugins/VelCmd.h>
#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>

#include <boost/bind.hpp>

using std::cout;
using std::endl;

namespace gazebo
{

     // Register this plugin with the simulator
     GZ_REGISTER_MODEL_PLUGIN(VelCmd);

     // Constructor
     VelCmd::VelCmd()
     {
          // Start up ROS
          std::string name = "velcmd_node";
          int argc = 0;
          ros::init(argc, NULL, name);
     }

     // Destructor
     VelCmd::~VelCmd()
     {
          delete this->nh_;
     }

     // Load the controller
     void VelCmd::Load( physics::ModelPtr _parent, 
                                   sdf::ElementPtr _sdf )
     {        
          //// Make sure the ROS node for Gazebo has already been initalized
          //if (!ros::isInitialized()) {
          //     ROS_FATAL_STREAM("A ROS node for Gazebo has not been " 
          //                      << "initialized, unable to load plugin. "
          //                      << "Load the Gazebo system plugin "
          //                      << "'libgazebo_ros_api_plugin.so' in the " 
          //                      << "gazebo_ros package)");
          //     return;
          //}          

          // Store the pointer to the model
          this->model_ = _parent;

          // ROS Nodehandle
          this->nh_ = new ros::NodeHandle("~");          
          
          //this->joint0_ = this->model_->GetJoint("right_back_wheel_joint");
          //this->joint1_ = this->model_->GetJoint("left_back_wheel_joint");
          //this->joint2_ = this->model_->GetJoint("left_axle_joint");
          //this->joint3_ = this->model_->GetJoint("right_axle_joint");
          //
          //this->joint4_ = this->model_->GetJoint("right_front_wheel_joint");
          //this->joint5_ = this->model_->GetJoint("left_front_wheel_joint");
          
          model_name_ = _sdf->GetParent()->Get<std::string>("name");
          robot_namespace_ = _sdf->Get<std::string>("robot_namespace");

          cout << "======================================" << endl;
          cout << "VelCmd Gazebo ROS Plugin Config " << endl;
          cout << "Model Name: " << model_name_ << endl;
          cout << "Robot Namespace: " << robot_namespace_ << endl;
          cout << "======================================" << endl;

          // ROS Subscriber
          this->sub_twist_ = this->nh_->subscribe<geometry_msgs::Twist>(
               "/" + robot_namespace_+std::string("/vel_cmd"), 1000, 
               &VelCmd::twist_cb, this );          

          this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
               boost::bind(&VelCmd::UpdateChild, this));
     }

     // Update the controller
     void VelCmd::UpdateChild()
     {
          this->model_->SetLinearVel(math::Vector3(linear_vel_.x, 
                                                   linear_vel_.y, 
                                                   linear_vel_.z));

          this->model_->SetAngularVel(math::Vector3(angular_vel_.x,
                                                    angular_vel_.y,
                                                    angular_vel_.z));
          //this->joint0_->SetForce(0, force_);
          //this->joint1_->SetForce(0, force_);
          //
          //this->joint4_->SetForce(0, force_);
          //this->joint5_->SetForce(0, force_);
          //
          //this->joint2_->SetAngle(0, gazebo::math::Angle(angle_));
          //this->joint3_->SetAngle(0, gazebo::math::Angle(angle_));
     }     

     void VelCmd::twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
     {
          linear_vel_ = msg->linear;
          angular_vel_ = msg->angular;
     }
}
