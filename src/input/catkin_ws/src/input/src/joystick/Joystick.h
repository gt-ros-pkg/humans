#ifndef JOYSTICK_H_
#define JOYSTICK_H_
/// ----------------------------------------------------------------------------
/// @file Joystick.h
/// @author syllogismrxs <syllogismrxs@syllogismrxs-HP-EliteBook-6930p>
///
/// Time-stamp: <2013-09-04 14:02:17 syllogismrxs>
///
/// @version 1.0
/// Created: 19 Feb 2013
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
/// The Joystick class ...
/// 
/// ----------------------------------------------------------------------------

#include <fcntl.h>
#include <linux/joystick.h>

class Joystick {
private:
protected:

     std::string inputDevice_;
  
     int servo_range_;
     int servo_center_;
     int axis_range_ ;
     int output_axis_max_;
     int right_elevator_axis_;
     int left_elevator_axis_;
     int top_rudder_axis_;
     int bottom_rudder_axis_;
     int motor_axis_;
     int joy_fd_;
  
     struct js_event js_;
     int num_of_buttons_;
     int num_of_axes_;
     int *axis_,*button_;

public:
     Joystick();
     int init(std::string inputDevice);
     int num_of_buttons();
     int num_of_axes();
     int update(int **axis, int **button);     
};

#endif
