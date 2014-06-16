#ifndef _GRUVI_GAZEBO_ROS_
#define _GRUVI_GAZEBO_ROS_
 
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
 
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
 
namespace gazebo
{
 
     class VelCmd : public ModelPlugin
     {
     public: 
          VelCmd(); 
          virtual ~VelCmd(); 
          void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
          
          void twist_cb(const geometry_msgs::Twist::ConstPtr& msg);
 
     protected: 
          virtual void UpdateChild();
 
     private:
          event::ConnectionPtr updateConnection_;
          std::string model_name_;
          std::string robot_namespace_;
          physics::ModelPtr model_;
          ros::NodeHandle *nh_;
          
          gazebo::physics::JointPtr joint0_;
          gazebo::physics::JointPtr joint1_;
          gazebo::physics::JointPtr joint2_;
          gazebo::physics::JointPtr joint3_;

          gazebo::physics::JointPtr joint4_;
          gazebo::physics::JointPtr joint5_;

          geometry_msgs::Vector3 linear_vel_;
          geometry_msgs::Vector3 angular_vel_;

          // ROS Subscriber
          ros::Subscriber sub_twist_;
     };
}
 
#endif
 
