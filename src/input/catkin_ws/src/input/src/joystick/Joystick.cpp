#include <iostream>
#include <stdio.h>
#include "Joystick.h"

using std::cout;
using std::endl;

Joystick::Joystick()
{
     servo_range_ = 255;
     servo_center_ = servo_range_ / 2;
     axis_range_ = 32767;
     output_axis_max_ = 100;
     right_elevator_axis_ = 1;
     left_elevator_axis_ = -1;
     top_rudder_axis_ = -1;
     bottom_rudder_axis_ = 1;
     motor_axis_ = -1;

     num_of_buttons_ = 0;
     num_of_axes_ = 0;
     
     inputDevice_ = "/dev/input/js1";
}

int Joystick::init(std::string inputDevice)
{
     inputDevice_ = inputDevice;

     if((joy_fd_ = open( inputDevice_.c_str(), O_RDONLY)) == -1) {
	  cout << "Unable to open joystick: "<< inputDevice << endl;
	  return -1;
     }
     
     ioctl( joy_fd_, JSIOCGAXES, &num_of_axes_);
     ioctl( joy_fd_, JSIOCGBUTTONS, &num_of_buttons_);
     
     fcntl( joy_fd_, F_SETFL, O_NONBLOCK );

     axis_ = new int[num_of_axes_];
     button_ =  new int[num_of_buttons_];
     
     // Force the read of the joystick ten times to 
     // flush the buffer
     for (int i = 0 ; i < 10; i++) {
          this->update(&axis_, &button_);
     }

     return 0;
}

int Joystick::update(int **axis, int **button)
{
     //Read the state of the joystick returned in JS_EVENT Struct
     read(joy_fd_, &js_, sizeof(struct js_event));
     switch (js_.type & ~JS_EVENT_INIT) {
     case JS_EVENT_AXIS:
     	  axis_ [js_.number] = js_.value;
	  break;	
     case JS_EVENT_BUTTON:
	  button_ [ js_.number] = js_.value;
	  break;
     default:
     	  cout << "Invalid joystick event" << endl;
     }
     
#if 0
     printf( "X: %6d  Y: %6d  ", axis_[0], axis_[1] );
     if( num_of_axes_ > 2 )
     	  printf("Z: %6d  ", axis_[2] );
     			
     if( num_of_axes_ > 3 )
     	  printf("R: %6d  ", axis_[3] );
     			
     for( int x=0 ; x<num_of_buttons_ ; ++x )
     	  printf("B%d: %d  ", x, button_[x] );
     
     printf("  \r");
     fflush(stdout);
#endif
     
     *axis = axis_;
     *button = button_;

     return 0;
}

int Joystick::num_of_buttons()
{
     return num_of_buttons_;
}
     
int Joystick::num_of_axes()
{
     return num_of_axes_;
}
