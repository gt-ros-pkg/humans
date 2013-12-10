%==========================================================================
% Title  : Kinematic simulation of underwater vehicle
% Author : Kevin DeMarco <demarco@gatech.edu> 
%        : http://www.kevindemarco.com
% Date   : June 7, 2013
%==========================================================================

% Clear all variables and windows
close all;
clear;
clc;

% Initial State (see <name>_model.m file for state description)
x0 = [0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0 ; 0];

% Time variables
t0 = 0;       % initial time
tEnd = 30;     % end time
tStep = 0.01; % time step (if applicable)

% Always use VideoRay Pro 3 coefficients until we get them for Pro 4
videoray_pro_3_params

%model_id = menu("Select a model to simulate:", \
%"VideoRay Pro III", \
%"VideoRay Pro 4");
%
% Load model parameters / coefficients
%if model_id == 1
%    videoray_pro_3_params
%else
%    videoray_pro_4_params
%end

clc;
global controller_id = 1;
controller_id = menu("Select a controller:", \
"Demo - open loop control", \
"Proportional");

global DEMO = 1;
global PROPORTIONAL = 2;

if controller_id > DEMO
    global depth_ref;
    global heading_ref;
    global speed_ref;
    
    % Get user input on simulation length
    tEnd = input("Enter simulation time length (sec): ");    
    depth_ref = input("Enter desired depth: ");    
    heading_ref = input("Enter desired heading: ");    
    speed_ref = input("Enter desired speed: ");    
    %tEnd = 20;
    %depth_ref = -1;
    %heading_ref = 90;
    %speed_ref = 1;
end


%% ODE Solvers
% Currently, you have the option of two different ODE solvers. They provide
% similar results, but the trajectories do not produce exact matches. The
% lsode solver comes installed with octave, so you don't need to install
% additional software, but if you choose to use the ode23 solver, you will
% need to install the octave-odepkg package. I personally prefer the ode23
% solver, but I left it commented out at first since additional software is
% required.

%% 1.) ode23 ODE solver option...
% If you want to use ode23, you need to install odepkg. See the README.md
% file in this directory for instructions.
% ODE solver options (if using ode23)
%pkg load odepkg
%vopt = odeset ("RelTol", 1e-5, \
%"AbsTol", 1e-5, \
%"MaxStep", 1, \
%"InitialStep", 1, \
%"NormControl", "on");

%%Build time vector and run ODE solver
%tt = [t0, tEnd];
%[t,xa] = ode23(@videoray_model, tt, x0, vopt);

%% 2.) lsode ODE solver option...
%t = linspace (t0, tEnd, (tEnd-t0)/tStep);
t = t0:tStep:tEnd;
xa = lsode( @(x,t) videoray_model(t,x) , x0, t);

%% Plot Velocities
%figure('Position',[20,20,600,600]);
figure;
subplot(3,1,1);
plot(t,xa(:,1));
ylabel('Surge (m/s)');

subplot(3,1,2);
plot(t,xa(:,2));
ylabel('Sway (m/s)');

subplot(3,1,3);
plot(t,xa(:,3));
xlabel('Time (s)');
ylabel('Heave (m/s)');

%% Plot Orientation Rates
figure;
subplot(3,1,1);
plot(t,xa(:,4));
ylabel('Roll Rate');

subplot(3,1,2);
plot(t,xa(:,5));
ylabel('Pitch Rate');

subplot(3,1,3);
plot(t,xa(:,6));
xlabel('Time');
ylabel('Yaw Rate');

%% Plot Positions
figure;
subplot(3,1,1);
plot(t,xa(:,7));
ylabel('X-Pos (m)');

subplot(3,1,2);
plot(t,xa(:,8));
ylabel('Y-Pos (m)');

subplot(3,1,3);
plot(t,xa(:,9));
xlabel('Time (s)');
ylabel('Z-Pos (m)');

%% Plot Orientation
figure;
subplot(3,1,1);
plot(t,xa(:,10)*180/pi);
ylabel('Roll');

subplot(3,1,2);
plot(t,xa(:,11)*180/pi);
ylabel('Pitch');

subplot(3,1,3);
plot(t,xa(:,12)*180/pi);
xlabel('Time (s)');
ylabel('Yaw');

%% Plot Thrust Vectors
figure;
subplot(3,1,1);
plot(t(1:end-1),diff(xa(:,13))/tStep);
ylabel('Thrust-Forw');

subplot(3,1,2);
plot(t(1:end-1),diff(xa(:,14))/tStep);
ylabel('Thrust-Rotate');

subplot(3,1,3);
plot(t(1:end-1),diff(xa(:,15))/tStep);
xlabel('Time');
ylabel('Thrust-Vert');

%% Plot Control Input
figure;
subplot(3,1,1);
plot(t(1:end-1),diff(xa(:,16))/tStep);
ylabel('Port Input');

subplot(3,1,2);
plot(t(1:end-1),diff(xa(:,17))/tStep);
ylabel('Starboard Input');

subplot(3,1,3);
plot(t(1:end-1),diff(xa(:,18))/tStep);
xlabel('Time (s)');
ylabel('Vertical Input');


%% Plot X-Y Plane
figure;
plot(xa(:,7), xa(:,8));
title('X-Y Pos');
xlabel('X-Pos (m)');
ylabel('Y-Pos (m)');
