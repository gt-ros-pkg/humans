#ifndef VIDEORAYCOMM_H_
#define VIDEORAYCOMM_H_
/// ----------------------------------------------------------------------------
/// @file VideoRayComm.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2015-06-17 13:10:48 syllogismrxs>
///
/// @version 1.0
/// Created: 13 Aug 2013
///
/// ----------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a 
/// copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation 
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the 
/// Software is furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ----------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The VideoRayComm class ...
/// 
/// ----------------------------------------------------------------------------

#include "Packetizer.h"
#include <syllo_serial/serialib.h>

#define MANIP_CTRL_SIZE 0x8

class VideoRayComm {
public:
     
     enum Status_t
     {
          Success = 0,
          Failure
     };

     enum ManipState_t
     {
          Idle = 0,
          Opening,
          Closing          
     };

     enum CamCtrl_t
     {
          Enable = 0,
          Disable,
          Arrow_Up,
          Arrow_Right,
          Arrow_Down,
          Arrow_Left
     };

     VideoRayComm();
     ~VideoRayComm();
     
     Status_t set_desired_heading(int heading);
     Status_t set_desired_depth(int depth);
     Status_t set_focus(int focus);
     Status_t set_camera_tilt(int tilt);
     Status_t set_lights(int lights);
     Status_t set_vertical_thruster(int thrust);
     Status_t set_port_thruster(int thrust);
     Status_t set_starboard_thruster(int thrust);

     Status_t set_manipulator_state(ManipState_t state);

     Status_t send_control_command();
     Status_t set_depth_pid_parameters();     

     Status_t send_nav_data_command();
     Status_t request_status();

     double depth();
     double heading();
     double roll();
     double pitch();
     double water_temperature();
     double internal_temperature();
     double water_ingress();
     double humidity();
     double rov_voltage();

     double yaw_accel();
     double pitch_accel();
     double roll_accel();
     double surge_accel();
     double sway_accel();
     double heave_accel();

     Status_t set_cam_cmd(CamCtrl_t cam_ctrl);

protected:
private:
     double depth_;
     double heading_;
     double roll_;
     double pitch_;
     double water_temperature_;
     double humidity_;
     double internal_temperature_;
     double water_ingress_;
     double yaw_accel_;
     double pitch_accel_;
     double roll_accel_;
     double surge_accel_;
     double sway_accel_;
     double heave_accel_;
     
     double rov_voltage_;

     char tx_ctrl_data[15];
     //char tx_sensor_data[7];
     char servo_ctrl_data[MANIP_CTRL_SIZE];
     
     ManipState_t manip_state_;

     Packetizer packetizer_;
     Packetizer receiver_;
     serialib serial_;     

     short swap_bytes(char *in, int msb, int lsb);
};

#endif
