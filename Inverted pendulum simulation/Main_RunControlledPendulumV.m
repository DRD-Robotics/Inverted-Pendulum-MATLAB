% calculate Ardiuino simulation of inverted pendulum
% SFC of theta, thetaDot and position
% Lueberger observer only used to estimate theta, thetaDot
% estimate position made by directly integrating the velocity control signal
% with integral action on position
% nonlinear plant simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% all rights reserved
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 23/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 1. Load / Calculate Parameters
% This section will load the system parameters and constants, then it will
% design an observer just for angle and angular velocity states, and also
% design the State Feedback Controller

close all
clear all
clc

% using calculated parameters NOT demo values
wantDefault = 0;

% get parameters for rod pendulum
params = GetRodPendulumParams(wantDefault, 5);
params
% get state space coefficients
c = GetStateSpaceCoesffs(wantDefault, params);

% get state space model with thetaDot, theta
ssm = GetSSModel2x2V(wantDefault, params);

% get state space model with thetaDot, theta and  position of cart
% integral action on position
ssmPos = GetSSModel4x4V(params);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build observer just for angle and angular velocity states
% calculate observer gain L here
% [needs doing]
% put your code for calculating L here

PX=20*[-10 -11];
L = place(ssm.A', ssm.C', PX);
disp(' ')
disp('Luenberger Gain Values')
disp(L)

%L = place(ssm.A', ssm.C', [-100 -50]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for state space model with thetaDot, theta and  position of cart
% calculate SFC gain K here
% [needs doing]
% put your code for calculating K here
PX=8*[-10 -12 -14 -17];
K = place(ssmPos.A, ssmPos.B, PX);
disp(' ')
disp('State Feedback Controller Gain K')
disp(K)

%% 2. Initialise simulation parameters

% setup time points
dt =  0.0010;
Tfinal = 5;
t = 0:dt:Tfinal;

% Set controller saturation limit
maxSpeed = 10; %too fast and it will zoom away, too slow and it will fall

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
titleMessage = 'NLSimulateVelSFCLArduino4x4 partial observer Arduino';
disp(titleMessage)

% initialize arrays
tData = [];
uData = [];
xData = [];
xhatData = [];
kickFlag = [];

%% 3. Simulations

% every sub-loop randomly perturb intial condition
runs = 2; %c-sytle
for kick=1:runs
    % for each run randomly perturb intial condition
    %x0 = [0; 0.2*(rand - 0.5); 0; 0] %STABILITY
    x0 = [1;1;9;0];    
    % run Euler integration
    [xhat, t, x, u] = SFCVLIA4x4(@CBNLVCPend, c.a1, c.a2, c.b0, ssmPos.A,...
                      ssmPos.B, ssmPos.C, ssmPos.D, ssm.A, ssm.B, K, L,...
                      t,x0, maxSpeed);
    % get time
    newTime = (kick-1) * t(end) + t;
    
    % just show kick arrow for short time after kick
    frames = length(t);
    kickFlagK = zeros(1,frames);
    if(x0(2) > 0)
        % scale arrow to size of kick
        kickFlagK(1: floor(frames/4)) = -abs(x0(2));
    else
        % scale arrow to size of kick
        kickFlagK(1: floor(frames/4)) = abs(x0(2));
    end
    
    % concatenate data between runs
    tData = [tData newTime];
    uData = [uData u];
    xData = [xData x];
    xhatData = [xhatData xhat];
    kickFlag = [kickFlag kickFlagK];
end

%% 4. Plots

% plot out the state and control variables
PlotStateVariable2x2(xData, tData,titleMessage);
% for all time points animate the results
figure
range=1;

% cart is moving so set distance
%distance = zeros( size(xData(1, :)));      %NO CART MOVEMENT
distance = (xData(1,:));                     %WORKS BUT NO 0
%distance = sin( (20*pi)*0.0003*(1:999));   %IAN BACK & FORTH CRAZY

% use animate function
step = 3;
AnimatePendulumCart( (xData(1, :) + pi),  distance, 0.6, tData, range,...
                    kickFlag, step, titleMessage);



