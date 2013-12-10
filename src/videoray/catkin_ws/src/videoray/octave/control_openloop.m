function [u_port u_star u_vert] = control_openloop(t,x)
%==============================================================================
% Title  : Simple open loop controller for dynamics demonstration
% Author : Kevin DeMarco <demarco@gatech.edu> 
%        : http://www.kevindemarco.com
% Date   : June 7, 2013
% Notes  : 
%==============================================================================

% Very simple control law that enables thrusters for 0.5 seconds, then
% disables all thrusters. This is just to test the system dynamics.

if t < 5
    u_port = 130; 
    u_star = 150;
    u_vert = -100;
else
    u_port = 0;
    u_star = 0;
    u_vert = 0;
end

endfunction
