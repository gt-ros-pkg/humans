function [u_port u_star u_vert] = control_p(t,x)
%==============================================================================
% Title  : Simple proportional controller
% Author : Kevin DeMarco <demarco@gatech.edu> 
%        : http://www.kevindemarco.com
% Date   : June 7, 2013
% Notes  : 
%==============================================================================
global depth_ref;
global heading_ref;
global speed_ref;

depth_err = depth_ref - x(9);
speed_err = speed_ref - x(1);

heading = normDegrees(x(12)*180/pi);
heading_err = heading_ref - heading;

K_heading = 100;
if (abs(heading_err) < 180)
    heading_port = -K_heading*heading_err;
    heading_star = K_heading*heading_err;
else 
    heading_port = K_heading*heading_err;
    heading_star = -K_heading*heading_err;
end

K_speed = 100;
speed_port = K_speed*speed_err;
speed_star = K_speed*speed_err;

heading_weight = 0.5;
speed_weight = 0.5;

u_port = heading_weight*heading_port + speed_weight*speed_port;
u_star = heading_weight*heading_star + speed_weight*speed_star;

%if (heading_err*180/pi > 5) 
%    u_port = Kp*heading_err;
%    u_star = -Kp*heading_err;
%else
%    u_port = Kp*speed_err;
%    u_star = Kp*speed_err;
%end

Kp = 100;
u_vert = Kp*depth_err;


endfunction
