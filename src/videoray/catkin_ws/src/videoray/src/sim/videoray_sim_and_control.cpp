#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "videoray/Throttle.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <sstream>

#include <boost/numeric/odeint.hpp>

using std::cout;
using std::endl;

using namespace boost::numeric::odeint;

typedef boost::array< double , 12 > state_type;

#define PI (3.14159265359)

nav_msgs::Odometry odom_;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
     odom_ = *msg;
}

state_type x_ = {0,0,0,0,0,0,0,0,0,0,0,0};

// Linear and angular velocity states
double u;
double v;
double w;
double p;
double q;
double r;
          
// Added Mass Terms
double X_udot = 1.94; // inertia matrix M (m11)
double Y_vdot = 6.05; // inertia matrix M (m22)
double Z_wdot = 3.95; // m33
double N_rdot = 0.1;
//double N_rdot = 1.18e-2; // vehicle's motion of inertia about z-axis
// (6,6) entry of the vehicle inertia Matrix M

// Linear Drag Coefficients
double Xu = -0.95;
double Yv = -5.87;
double Nr = -0.023;
double Zw = -3.70;

// Quadratic Drag Coefficients
double Xuu = -6.04;
double Yvv = -30.73;
double Nrr = -0.45;
double Zww = -26.36;

double Ct_forw = 0.026667;
double Ct_back = 0.026667;
double Ct_vert_forw = 0.026667;
double Ct_vert_back = 0.026667;

double u_sat_low = -150;
double u_sat_high = 150;
     
// Control inputs
double X = 0;
double N = 0;
double Z = 0;

double u_port = 0;
double u_star = 0;
double u_vert = 0;

double xpos  = 0;
double ypos  = 0;
double zpos  = 0;
double phi   = 0;
double theta = 0;
double psi   = 0;

double c1 = 0;
double c2 = 0;
double c3 = 0;
double s1 = 0;
double s2 = 0;
double s3 = 0;
double t2 = 0;

videoray::Throttle throttle_;

void videoray_model( const state_type &x , state_type &dxdt , double t )
{
/// States: 
/// 0:  u     : surge velocity
/// 1:  v     : sway velocity
/// 2:  w     : heave velocity
/// 3:  p     : roll rate
/// 4:  q     : pitch rate
/// 5:  r     : yaw rate
/// 6:  xpos  : earth x-pos
/// 7:  ypos  : earth y-pos
/// 8:  zpos  : earth z-pos
/// 9:  phi   : roll angle
/// 10: theta : pitch angle
/// 11: psi   : yaw angle
     u = x[0];
     v = x[1];
     w = x[2];
     p = x[3];
     q = x[4];
     r = x[5];
     xpos  = x[6];
     ypos  = x[7];
     zpos  = x[8];
     phi   = x[9];
     theta = x[10];
     psi   = x[11];
          
     // Calculate fixed frame velocity rates
     dxdt[0] = (-Y_vdot*v*r + Xu*u + Xuu*u*abs(u) + X) / X_udot;
     dxdt[1] = (X_udot*u*r + Yv*v + Yvv*v*abs(v)) / Y_vdot;
     dxdt[2] = (Zw*w + Zww*w*abs(w) + Z) / Z_wdot;

     // Calculate fixed frame orientation rates
     dxdt[3] = 0;
     dxdt[4] = 0;
     dxdt[5] = (Nr*r + Nrr*r*abs(r) + N) / N_rdot;     

     c1 = cos(phi);
     c2 = cos(theta); 
     c3 = cos(psi); 
     s1 = sin(phi); 
     s2 = sin(theta); 
     s3 = sin(psi); 
     t2 = tan(theta);

     // Calculate inertial frame position
     dxdt[6] = c3*c2*u + (c3*s2*s1-s3*c1)*v + (s3*s1+c3*c1*s2)*w;
     dxdt[7] = s3*c2*u + (c1*c3+s1*s2*s3)*v + (c1*s2*s3-c3*s1)*w;
     dxdt[8] = -s2*u + c2*s1*v + c1*c2*w;

     // Calculate inertial frame orientations
     dxdt[9] = p + (q*s1 + r*c1)*t2;
     dxdt[10] = q*c1 - r*s1;
     dxdt[11] = (q*s1 + r*c1)* (1 / cos(theta));
}

//
// Converts input throttle commands to simulated linear and angular velocities.
//
geometry_msgs::Twist velocity_cmd_;
geometry_msgs::Vector3 velocity_linear_;
geometry_msgs::Vector3 velocity_angular_;

double saturate(double input, const double &min, const double &max)
{
     if (min > max) {
          ROS_INFO("saturate(): Invalid Min / Max Combo");
          return 0;
     } else if (input < min) {
          input = min;
     } else if(input > max) {
          input = max;
     }
     return input;
}

// Assumes that input has already been saturated within the in_min and in_max
// boundaries. Use the saturate() function on input before calling normalize
double normalize(double input, const double &in_min, const double &in_max,
                 const double &out_min, const double &out_max)
{
     if (in_min >= in_max || out_min >= out_max) {
          ROS_INFO("normalize(): Invalid Min / Max Combo");
          return 0;
     }

     double ratio = input / (in_max - in_min);
     return ratio * (out_max - out_min);

     return input;
}

double thrust_port = 0, thrust_star = 0;
void processThrottleCmds()
{
     u_port = saturate(throttle_.PortInput, u_sat_low, u_sat_high);
     u_star = saturate(throttle_.StarInput, u_sat_low, u_sat_high);
     u_vert = saturate(throttle_.VertInput, u_sat_low, u_sat_high);

     // Ct is different for reverse and forward
     if ( u_port >= 0 ) {
          thrust_port = u_port * Ct_forw;
     } else {
          thrust_port = u_port * Ct_back;
     }

     if ( u_star >= 0 ) {
          thrust_star = u_star * Ct_forw;
     } else {
          thrust_star = u_star * Ct_back;
     }

     X = thrust_port + thrust_star;
     N = thrust_star - thrust_port;

     // Ct is different for reverse and forward
     if ( u_vert >= 0 ) {
          Z = u_vert * Ct_vert_forw;
     } else {
          Z = u_vert * Ct_vert_back;
     }

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

void quaternionToEuler(const double &q0, const double &q1, 
                       const double &q2, const double &q3,
                       double &roll, double &pitch, double &yaw)
{
     roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2) );
     pitch = asin(2*(q0*q2-q3*q1));
     yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3) );
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

double depth_ref = 0;
double speed_ref = 0;
double heading_ref = 0;

void desiredVelocityCallback(const std_msgs::Float32::ConstPtr& msg)
{
     speed_ref = msg->data;
}

void desiredHeadingCallback(const std_msgs::Float32::ConstPtr& msg)
{
     heading_ref = normDegrees(msg->data - 90);
     //heading_ref = msg->data;
}

void desiredDepthCallback(const std_msgs::Float32::ConstPtr& msg)
{
     depth_ref = msg->data;
}

//geometry_msgs::Quaternion quat_;
double depth_err = 0;
double speed_err = 0;
double heading_err = 0;
double heading = 0;
double heading_port, heading_star, speed_port, speed_star;
double heading_weight = 0.5;
double speed_weight = 0.5;
//double K_heading = 0.25;
double K_heading = 0.01;
double K_speed = 10;
double K_depth = 50;
double roll_ = 0;
double pitch_ = 0;
double yaw_ = 0;

void execControlLaw()
{     
     //quat_ = odom_.pose.pose.orientation;
     //quaternionToEuler(quat_.w, quat_.x, quat_.y, quat_.z,
     //                  roll_, pitch_, yaw_);
          
     //depth_err = depth_ref - odom_.pose.pose.position.z;
     //speed_err = speed_ref - odom_.twist.twist.linear.x;
     
     roll_ = x_[9];
     pitch_ = x_[10];
     yaw_ = x_[11];
     
     depth_err = depth_ref - x_[8];
     speed_err = speed_ref - x_[0];
     
     heading = normDegrees(yaw_*180/PI);
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

     //throttle_.PortInput = 100;
     //throttle_.StarInput = 95;
     //throttle_.VertInput = 0;
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "videoray_sim_and_control");     
     ros::NodeHandle n;

     //ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("motion", 1);     
     //ros::Subscriber odom_sub = n.subscribe("odometry", 1, 
     //                                       odomCallback);

     ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("motion",1);
     geometry_msgs::Pose pose_;

     ros::Publisher pub_nav_x = n.advertise<std_msgs::Float32>("NAV_X",1);
     ros::Publisher pub_nav_y = n.advertise<std_msgs::Float32>("NAV_Y",1);
     ros::Publisher pub_nav_depth = n.advertise<std_msgs::Float32>("NAV_DEPTH",1);
     ros::Publisher pub_nav_heading = n.advertise<std_msgs::Float32>("NAV_HEADING",1);
     ros::Publisher pub_nav_speed = n.advertise<std_msgs::Float32>("NAV_SPEED",1);

     std_msgs::Float32 nav_x, nav_y, nav_depth, nav_heading, nav_speed;

     ros::Subscriber desired_vel_sub = n.subscribe("desired_velocity", 
                                                   1, 
                                                   desiredVelocityCallback);

     ros::Subscriber desired_head_sub = n.subscribe("desired_heading", 
                                                    1, 
                                                    desiredHeadingCallback);

     ros::Subscriber desired_depth_sub = n.subscribe("desired_depth", 
                                                     1, 
                                                     desiredDepthCallback);
     
     //double rate = 30;
     double rate = 10;
     ros::Rate loop_rate(rate);

     ros::Time begin = ros::Time::now();
     ros::Time curr_time = begin;
     ros::Time prev_time = begin;
     ros::Duration dt = curr_time - prev_time;
     
     geometry_msgs::Quaternion quat;

     runge_kutta4< state_type > stepper;
     //runge_kutta_dopri5< state_type > stepper;
     //adams_bashforth_moulton< 2 , state_type > stepper;

     while (ros::ok())
     {
          //cout << "*" << std::flush;
          
          curr_time = ros::Time::now();
          dt = curr_time - prev_time;
          prev_time = curr_time;

          ROS_INFO("dt: %f\n", dt.toSec());

          //cout << dt.toSec() << endl << std::flush;

          // Update state vector with odometry data from morse...
          //state_type x = {0,0,0,0,0,0,0,0,0,0,0,0};
          //x[0] = odom_.twist.twist.linear.x;
          //x[1] = odom_.twist.twist.linear.y;
          //x[2] = odom_.twist.twist.linear.z;
          //x[3] = odom_.twist.twist.angular.x;
          //x[4] = odom_.twist.twist.angular.y;
          //x[5] = odom_.twist.twist.angular.z;
          
          execControlLaw();

          processThrottleCmds();

          //geometry_msgs::Quaternion quat = odom_.pose.pose.orientation;
          //quaternionToEuler(quat.x, quat.y, quat.z, quat.w,
          //                  roll_, pitch_, yaw_);
          
          //boost::numeric::odeint::integrate(videoray_model, 
          //                                  x_, 
          //                                  curr_time.toSec() , 
          //                                  (curr_time + dt).toSec(), 
          //                                  dt.toSec());
          //boost::numeric::odeint::integrate(videoray_model, 
          //                                  x_, 
          //                                  curr_time.toSec() , 
          //                                  curr_time.toSec() + 1.0/rate, 
          //                                  1.0/rate);

          stepper.do_step(videoray_model, x_ , curr_time.toSec() , dt.toSec() );

          ROS_INFO("========================");
          ROS_INFO("Current: %f, \tdt: %f", curr_time.toSec(), dt.toSec());
          ROS_INFO("Surge: %f", x_[0]);
          ROS_INFO("Sway: %f", x_[1]);
          ROS_INFO("Heave: %f", x_[2]);
          
          ROS_INFO("3: %f", x_[3]);
          ROS_INFO("4: %f", x_[4]);
          ROS_INFO("5: %f", x_[5]);
          ROS_INFO("6: %f", x_[6]);
          ROS_INFO("7: %f", x_[7]);
          ROS_INFO("8: %f", x_[8]);
          ROS_INFO("9: %f", x_[9]);
          ROS_INFO("10: %f", x_[10]);
          ROS_INFO("11: %f", x_[11]);
          
          //velocity_linear_.x = x[0];
          //velocity_linear_.y = x[1];
          //velocity_linear_.z = x[2];
          //velocity_angular_.z = x[5];
          //
          //velocity_cmd_.linear = velocity_linear_;
          //velocity_cmd_.angular = velocity_angular_;
          //twist_pub.publish(velocity_cmd_);
          
          

          //quaternionToEuler(quat.w, quat.x, quat.y, quat.z,
          //                  roll_, pitch_, yaw_);               
          //
          //eulerToQuaternion(x_[9], 
          //                  x_[10], 
          //                  x_[11],
          //                  quat.w, quat.x, quat.y, quat.x);          

          //quat.w = 0.1; quat.x = 0.2; quat.y = 0.3 ; quat.z = 0.4;
          
          //quaternionToEuler(quat.w, quat.x, quat.y, quat.z,
          //                  roll_, pitch_, yaw_);               
          
          roll_  = x_[9];
          pitch_ = x_[10];
          yaw_   = x_[11];
                    
          eulerToQuaternion(roll_, 
                            pitch_, 
                            yaw_,
                            quat.w, quat.x, quat.y, quat.z);          
          
          pose_.position.x = x_[6];
          pose_.position.y = x_[7];
          pose_.position.z = x_[8];
          pose_.orientation.x = quat.x;
          pose_.orientation.y = quat.y;
          pose_.orientation.z = quat.z;
          pose_.orientation.w = quat.w;

          pose_pub.publish(pose_);

          nav_x.data = x_[6];
          nav_y.data = x_[7];
          nav_depth.data = x_[8];
          nav_speed.data = x_[0];
          nav_heading.data = normDegrees(x_[11]*180.0/PI + 90);

          pub_nav_x.publish(nav_x);
          pub_nav_y.publish(nav_y);
          pub_nav_depth.publish(nav_depth);
          pub_nav_heading.publish(nav_heading);
          pub_nav_speed.publish(nav_speed);

          ros::spinOnce();

          loop_rate.sleep();
     }
     return 0;
}
