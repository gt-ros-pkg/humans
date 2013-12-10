#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "videoray/Throttle.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <sstream>

#include <boost/numeric/odeint.hpp>

using std::cout;
using std::endl;

using namespace boost::numeric::odeint;

typedef boost::array< double , 6 > state_type;

#define PI (3.14159265359)

videoray::Throttle throttle_;
void throttleCallback(const videoray::Throttle::ConstPtr& msg)
{
     throttle_ = *msg;
     //// Left Throttle Conversion:
     //left_vel_ = saturate(msg->LeftThrottle, -100, 100);
     //left_vel_ = normalize(left_vel_, -100, 100, -1, 1);
     //
     //// Right Throttle Conversion:
     //right_vel_ = saturate(msg->RightThrottle, -100, 100);
     //right_vel_ = normalize(right_vel_, -100, 100, -1, 1);
     //
     //// Vertical Throttle Conversion:
     //vert_vel_ = saturate(msg->VerticalThrottle, -100, 100);
     //vert_vel_ = normalize(vert_vel_, -100, 100, -1, 1);
}

nav_msgs::Odometry odom_;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
     odom_ = *msg;
}

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
double N_rdot = 1.18e-2; // vehicle's motion of inertia about z-axis
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

void videoray_model( const state_type &x , state_type &dxdt , double t )
{
/// States: 
/// 0:  u     : surge velocity
/// 1:  v     : sway velocity
/// 2:  w     : heave velocity
/// 3:  p     : roll rate
/// 4:  q     : pitch rate
/// 5:  r     : yaw rate
     u = x[0];
     v = x[1];
     w = x[2];
     p = x[3];
     q = x[4];
     r = x[5];
          
     // Calculate fixed frame velocity rates
     dxdt[0] = (-Y_vdot*v*r + Xu*u + Xuu*u*abs(u) + X) / X_udot;
     dxdt[1] = (X_udot*u*r + Yv*v + Yvv*v*abs(v)) / Y_vdot;
     dxdt[2] = (Zw*w + Zww*w*abs(w) + Z) / Z_wdot;

     // Calculate fixed frame orientation rates
     dxdt[3] = 0;
     dxdt[4] = 0;
     dxdt[5] = (Nr*r + Nrr*r*abs(r) + N) / N_rdot;     
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

void quaternionToEuler(const double &q0, const double &q1, 
                       const double &q2, const double &q3,
                       double &roll, double &pitch, double &yaw)
{
     roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2) );
     pitch = asin(2*(q0*q2-q3*q1));
     yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3) );
}

runge_kutta4< state_type > stepper; 

int main(int argc, char **argv)
{
     ros::init(argc, argv, "videoray_sim");     
     ros::NodeHandle n;

     ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("motion", 1);
     ros::Subscriber throttle_sub = n.subscribe("throttle_cmds", 1, 
                                                throttleCallback);
     ros::Subscriber odom_sub = n.subscribe("odometry", 1, 
                                            odomCallback);
     
     double rate = 10;
     ros::Rate loop_rate(rate);

     ros::Time begin = ros::Time::now();
     ros::Time curr_time = begin;
     ros::Time prev_time = begin;
     ros::Duration dt = curr_time - prev_time;
     
     while (ros::ok())
     {
          //cout << "*" << std::flush;
          

          curr_time = ros::Time::now();
          dt = curr_time - prev_time;
          prev_time = curr_time;

          //cout << dt.toSec() << endl << std::flush;

          // Update state vector with odometry data from morse...
          state_type x = {0,0,0,0,0,0};
          x[0] = odom_.twist.twist.linear.x;
          x[1] = odom_.twist.twist.linear.y;
          x[2] = odom_.twist.twist.linear.z;
          x[3] = odom_.twist.twist.angular.x;
          x[4] = odom_.twist.twist.angular.y;
          x[5] = odom_.twist.twist.angular.z;

          processThrottleCmds();

          //geometry_msgs::Quaternion quat = odom_.pose.pose.orientation;
          //quaternionToEuler(quat.x, quat.y, quat.z, quat.w,
          //                  roll_, pitch_, yaw_);
          
          //boost::numeric::odeint::integrate(videoray_model, 
          //                                  x, 
          //                                  curr_time.toSec() , 
          //                                  (curr_time + dt).toSec(), 
          //                                  dt.toSec());
          boost::numeric::odeint::integrate(videoray_model, 
                                            x, 
                                            curr_time.toSec() , 
                                            curr_time.toSec() + 1.0/rate, 
                                            1.0/rate);

          //ROS_INFO("========================");
          //ROS_INFO("Current: %f, \tdt: %f", curr_time.toSec(), dt.toSec());
          //ROS_INFO("Surge: %f", x[0]);
          //ROS_INFO("Sway: %f", x[1]);
          //ROS_INFO("Heave: %f", x[2]);
          //
          //ROS_INFO("3: %f", x[3]);
          //ROS_INFO("4: %f", x[4]);
          //ROS_INFO("5: %f", x[5]);
          ////ROS_INFO("6: %f", x[6]);
          ////ROS_INFO("7: %f", x[7]);
          ////ROS_INFO("8: %f", x[8]);
          ////ROS_INFO("9: %f", x[9]);
          ////ROS_INFO("10: %f", x[10]);
          ////ROS_INFO("11: %f", x[11]);
          
          velocity_linear_.x = x[0];
          velocity_linear_.y = x[1];
          velocity_linear_.z = x[2];
          velocity_angular_.z = x[5];
          
          velocity_cmd_.linear = velocity_linear_;
          velocity_cmd_.angular = velocity_angular_;
          twist_pub.publish(velocity_cmd_);

          ros::spinOnce();

          loop_rate.sleep();
     }
     return 0;
}
